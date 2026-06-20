/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *               2024-     Tom de Bree <Tom@voltinflux.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "leafinv.h"
#include "NissLeafMng.h"
#include "delay.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "stm32_can.h"
#include "utils.h"

/* Resolver-offset diagnostics for the Nissan traction-inverter ECU.
 * Request frames go to 0x784, responses come back on 0x78C, ISO-TP single-frame
 * (byte 0 = PCI/length). All exchanges below are synchronous: we send a frame
 * then busy-wait for the 0x78C reply (captured in DecodeCAN from the CAN RX
 * interrupt). Only valid while stationary, in Neutral, with no torque request.
 */
#define RESOLVER_REQ_ID 0x784
#define RESOLVER_RSP_ID 0x78C
#define RESOLVER_TIMEOUT_LOOPS 200 // x 500us busy-wait = ~100ms per exchange

/*Info on running Leaf Gen 2 PDM
IDs required :
0x1D4 VCM (10ms)
0x1DB LBC (10ms)
0x1DC LBC (10ms)
0x1F2 VCM (10ms)
0x50B VCM (100ms)
0x55B LBC (100ms)
0x59E LBC (500ms)
0x5BC LBC (100ms)
PDM sends:
0x390 (100ms)
0x393 (100ms)
0x679 on evse plug insert

*/

void LeafINV::SetCanInterface(CanHardware *c) {
  NissLeafMng::SetCanInterface(
      c); // set Leaf VCM messages on same bus as Inverter
  can = c;

  can->RegisterUserMessage(0x1DA);           // Leaf inv msg
  can->RegisterUserMessage(0x55A);           // Leaf inv msg
  can->RegisterUserMessage(RESOLVER_RSP_ID); // Resolver diag response (0x78C)
}

void LeafINV::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  if (id == 0x1DA) // THIS MSG CONTAINS INV VOLTAGE, MOTOR SPEED AND ERROR STATE
  {
    voltage = (bytes[0] * 2); // MEASURED VOLTAGE FROM LEAF INVERTER

    if (Param::GetInt(Param::ShuntType) == 0 &&
        voltage <
            420) // Only populate if no shunt is used and voltage is under 420
    {
      Param::SetFloat(Param::udc, voltage);
    }

    int16_t parsed_speed = (bytes[4] << 7) | bytes[5] >> 1;
    if (parsed_speed > 0x3fff)
      parsed_speed -= 0x7fff; // 15 bit signed conversion
    // speed = (parsed_speed == 0x7fff ? 0 : parsed_speed);//LEAF MOTOR RPM
    speed = parsed_speed;
    error = (bytes[6] & 0xb0) != 0x00; // INVERTER ERROR STATE

  } else if (id == 0x55A) // THIS MSG CONTAINS INV TEMP AND MOTOR TEMP
  {
    inv_temp = fahrenheit_to_celsius(bytes[2]);   // INVERTER TEMP
    motor_temp = fahrenheit_to_celsius(bytes[1]); // MOTOR TEMP
  } else if (id == RESOLVER_RSP_ID)               // Resolver diag reply (0x78C)
  {
    // Captured here from CAN RX interrupt; the resolver routines poll
    // diagRxValid.
    for (int i = 0; i < 8; i++)
      diagRxData[i] = bytes[i];
    diagRxValid = true;
  }
}

void LeafINV::SetTorque(float torquePercent) {
  final_torque_request = (torquePercent * 2047) / 100.0f;

  if (Param::GetInt(Param::reversemotor) == 1) {
    final_torque_request *= -1; // reverse torque request to flip motor rotation
  }

  Param::SetInt(Param::torque,
                final_torque_request); // post processed final torque value sent
                                       // to inv to web interface
}

void LeafINV::Task10Ms() // ONLY RAN IN RUNMODE
{

  NissLeafMng::Task10Ms(
      final_torque_request); // This is only called when in run mode, send
                             // torque request with it
}

void LeafINV::Task100Ms() // Always ran
{
  // Inverter calling of 100ms VCM task takes precedences over PDM
  NissLeafMng::Task100Ms();
}

int8_t LeafINV::fahrenheit_to_celsius(uint16_t fahrenheit) {
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if (result < -128)
    return -128;
  if (result > 127)
    return 127;
  return result;
}

/*===========================================================================
 * Resolver-offset diagnostics
 *===========================================================================*/

// Safety interlock: only allow resolver access when the car cannot lurch.
ResolverStatus LeafINV::InterlockCheck() {
  if (ABS(speed) >= 100)
    return ResolverStatus::Moving;
  if (final_torque_request != 0)
    return ResolverStatus::ThrottleApplied;
  if (Param::GetInt(Param::dir) != 0) // 0 = Neutral (see DIRS)
    return ResolverStatus::NotNeutral;
  return ResolverStatus::OK;
}

// Send one ISO-TP single frame on 0x784 and wait for the 0x78C reply.
// expectPos is the expected positive-response service byte (0x50/0x61/0x7B).
ResolverStatus LeafINV::DiagExchange(const uint8_t *tx, uint8_t txLen,
                                     uint8_t expectPos, uint8_t *rx) {
  if (!can)
    return ResolverStatus::NoResponse;

  uint8_t buf[8] = {0};
  for (uint8_t i = 0; i < txLen && i < 8; i++)
    buf[i] = tx[i];

  diagRxValid = false;
  can->Send(RESOLVER_REQ_ID, buf, 8);

  uint8_t pendingCount = 0;
  for (int i = 0; i < RESOLVER_TIMEOUT_LOOPS; i++) {
    if (diagRxValid) {
      for (int j = 0; j < 8; j++)
        rx[j] = diagRxData[j];

      // 7F <sid> 78 = responsePending: the ECU is busy and will send the real
      // reply shortly. Keep waiting (fresh window) instead of failing.
      if (rx[1] == 0x7F && rx[3] == 0x78 && pendingCount < 50) {
        pendingCount++;
        diagRxValid = false;
        i = 0;
        continue;
      }

      if (rx[1] == 0x7F) // negative response: 7F <sid> <nrc>
        return ResolverStatus::NegativeResponse;
      if (rx[1] != expectPos)
        return ResolverStatus::NegativeResponse;
      return ResolverStatus::OK;
    }
    uDelay(500);
  }
  return ResolverStatus::NoResponse;
}

// Debug helper: send one raw frame, return whatever comes back (no judgement).
ResolverStatus LeafINV::ResolverRaw(const uint8_t *tx, uint8_t txLen,
                                    uint8_t *rxOut) {
  ResolverStatus s = InterlockCheck();
  if (s != ResolverStatus::OK)
    return s;
  if (!can)
    return ResolverStatus::NoResponse;

  uint8_t buf[8] = {0};
  for (uint8_t i = 0; i < txLen && i < 8; i++)
    buf[i] = tx[i];

  diagRxValid = false;
  can->Send(RESOLVER_REQ_ID, buf, 8);

  for (int i = 0; i < RESOLVER_TIMEOUT_LOOPS && !diagRxValid; i++)
    uDelay(500);

  if (!diagRxValid)
    return ResolverStatus::NoResponse;

  for (int j = 0; j < 8; j++)
    rxOut[j] = diagRxData[j];
  return ResolverStatus::OK;
}

void LeafINV::GetLastDiagFrame(uint8_t *out8) const {
  for (int i = 0; i < 8; i++)
    out8[i] = diagRxData[i];
}

// Enter Nissan manufacturer session (10 AA) — unlocks the resolver records.
ResolverStatus LeafINV::DiagSession() {
  static const uint8_t req[] = {0x02, 0x10, 0xAA};
  uint8_t rx[8];
  return DiagExchange(req, sizeof(req), 0x50, rx);
}

// Return to default session (10 81). Best-effort — the session also times out
// on its own, so a missing/negative reply is ignored.
void LeafINV::EndDiagSession() {
  static const uint8_t req[] = {0x02, 0x10, 0x81};
  uint8_t rx[8];
  DiagExchange(req, sizeof(req), 0x50, rx);
}

// Probe records 01 (required), 03 and 04 (optional), packing whatever replies
// 61 contiguously into out.bytes. Assumes the diag session is already open.
ResolverStatus LeafINV::ReadRecords(ResolverCode &out) {
  uint8_t rx[8];
  out.recordMask = 0;
  out.length = 0;
  for (int i = 0; i < 5; i++)
    out.bytes[i] = 0;

  // Record 01 (1 byte) — must be present on every car.
  static const uint8_t req01[] = {0x02, 0x21, 0x01};
  ResolverStatus s = DiagExchange(req01, sizeof(req01), 0x61, rx);
  if (s != ResolverStatus::OK)
    return s;
  out.bytes[out.length++] = rx[3];
  out.recordMask |= RESOLVER_REC01;

  // Record 03 (2 bytes BE) — present on 10-digit cars only.
  static const uint8_t req03[] = {0x02, 0x21, 0x03};
  if (DiagExchange(req03, sizeof(req03), 0x61, rx) == ResolverStatus::OK) {
    out.bytes[out.length++] = rx[3];
    out.bytes[out.length++] = rx[4];
    out.recordMask |= RESOLVER_REC03;
  }

  // Record 04 (2 bytes BE) — present on 10-digit cars only.
  static const uint8_t req04[] = {0x02, 0x21, 0x04};
  if (DiagExchange(req04, sizeof(req04), 0x61, rx) == ResolverStatus::OK) {
    out.bytes[out.length++] = rx[3];
    out.bytes[out.length++] = rx[4];
    out.recordMask |= RESOLVER_REC04;
  }

  return ResolverStatus::OK;
}

ResolverStatus LeafINV::ResolverRead(ResolverCode &out) {
  ResolverStatus s = InterlockCheck();
  if (s != ResolverStatus::OK)
    return s;

  // Session is best-effort: some inverters (per the LeafSpy-era docs) require
  // 10 AA to unlock the records, others (e.g. 2-digit ZE1-style) reject it and
  // read/write fine without any session. Proceed regardless and let the actual
  // 61/7B responses be the real success signal.
  (void)DiagSession();

  s = ReadRecords(out);
  EndDiagSession();
  return s;
}

// Write the records named by mask (bytes packed contiguously in the same order
// as ReadRecords produces), then re-read and verify. The caller is expected to
// pass exactly the record set that a preceding ResolverRead reported present.
ResolverStatus LeafINV::ResolverWrite(const uint8_t *bytes, uint8_t mask,
                                      ResolverCode &verifyOut) {
  ResolverStatus s = InterlockCheck();
  if (s != ResolverStatus::OK)
    return s;

  (void)DiagSession(); // best-effort, see ResolverRead

  uint8_t rx[8];
  uint8_t idx = 0;

  if (mask & RESOLVER_REC01) {
    uint8_t req[] = {0x03, 0x3B, 0x01, bytes[idx]};
    s = DiagExchange(req, sizeof(req), 0x7B, rx);
    if (s != ResolverStatus::OK) {
      EndDiagSession();
      return s;
    }
    idx += 1;
  }
  if (mask & RESOLVER_REC03) {
    uint8_t req[] = {0x04, 0x3B, 0x03, bytes[idx], bytes[idx + 1]};
    s = DiagExchange(req, sizeof(req), 0x7B, rx);
    if (s != ResolverStatus::OK) {
      EndDiagSession();
      return s;
    }
    idx += 2;
  }
  if (mask & RESOLVER_REC04) {
    uint8_t req[] = {0x04, 0x3B, 0x04, bytes[idx], bytes[idx + 1]};
    s = DiagExchange(req, sizeof(req), 0x7B, rx);
    if (s != ResolverStatus::OK) {
      EndDiagSession();
      return s;
    }
    idx += 2;
  }

  // Re-read to verify. Some inverters apply the new offset immediately (the
  // re-read matches → OK). Others stage it and only make it active after a
  // power cycle, so an immediate re-read still shows the old value — that is
  // WriteStaged, not a failure (the 7B acks above confirm the write landed).
  s = ReadRecords(verifyOut);
  EndDiagSession();
  if (s != ResolverStatus::OK)
    return s;

  if (verifyOut.recordMask != mask || verifyOut.length != idx)
    return ResolverStatus::WriteStaged;
  for (uint8_t i = 0; i < idx; i++) {
    if (verifyOut.bytes[i] != bytes[i])
      return ResolverStatus::WriteStaged;
  }
  return ResolverStatus::OK;
}
