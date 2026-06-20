/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#ifndef LEAFINV_H
#define LEAFINV_H

#include "inverter.h"
#include "my_fp.h"
#include <stdint.h>

/* Resolver-offset diagnostics over raw CAN (req 0x784 / resp 0x78C), using the
 * Nissan KWP2000-style services 0x10/0x21/0x3B. The full code is the
 * concatenation of the records the inverter actually answers: record 01 (1
 * byte) plus, on older cars, records 03 and 04 (2 bytes big-endian each). It is
 * response-driven, not year-coded: ZE1/2018+ inverters answer only record 01
 * (2-digit code), earlier inverters answer 01+03+04 (10-digit code). */
struct ResolverCode {
  uint8_t
      bytes[5]; // rec01(1) + rec03(2) + rec04(2), in order, only present recs
  uint8_t recordMask; // bit0=rec01 present, bit1=rec03, bit2=rec04
  uint8_t length; // total bytes present (1 for 2-digit cars, 5 for 10-digit)
};

enum class ResolverStatus {
  OK,
  Moving,           // motor turning - refused
  ThrottleApplied,  // torque requested - refused
  NotNeutral,       // selector not in Neutral - refused
  NoResponse,       // inverter did not answer (unpowered / wrong bus)
  NegativeResponse, // inverter returned 7F
  WriteStaged       // write ack'd but not active yet (power-cycle to apply)
};

#define RESOLVER_REC01 0x01
#define RESOLVER_REC03 0x02
#define RESOLVER_REC04 0x04

class LeafINV : public Inverter {
public:
  void DecodeCAN(int id, uint32_t data[2]);
  void Task10Ms();
  void Task100Ms();
  static bool ControlCharge(bool RunCh);
  void SetTorque(float torque);
  float GetMotorTemperature() { return motor_temp; }
  float GetInverterTemperature() { return inv_temp; }
  float GetInverterVoltage() { return voltage; }
  float GetMotorSpeed() { return speed; }
  int GetInverterState() { return error; }
  void SetCanInterface(CanHardware *c);

  // Resolver offset diagnostics. Both self-gate on the safety interlock
  // (stationary + no throttle + Neutral) and always exit the diag session.
  ResolverStatus ResolverRead(ResolverCode &out);
  ResolverStatus ResolverWrite(const uint8_t *bytes, uint8_t mask,
                               ResolverCode &verifyOut);

  // Debug: send one raw single frame to 0x784 and capture the 0x78C reply
  // (no session handling - caller drives the full sequence). Self-gates on the
  // interlock. Returns OK with rxOut filled if any frame came back, NoResponse
  // on timeout, or the interlock status.
  ResolverStatus ResolverRaw(const uint8_t *tx, uint8_t txLen, uint8_t *rxOut);
  // Copy the last 0x78C frame captured (8 bytes) - for negative-response
  // detail.
  void GetLastDiagFrame(uint8_t *out8) const;

private:
  uint8_t nissan_crc(uint8_t *data);
  static int8_t fahrenheit_to_celsius(uint16_t fahrenheit);

  // ---- Resolver diag helpers ----
  ResolverStatus InterlockCheck();
  ResolverStatus DiagSession(); // enter Nissan session (10 AA)
  void EndDiagSession(); // return to default session (10 81), best-effort
  // Send one single-frame request on 0x784 and wait for the 0x78C reply.
  // Returns NoResponse on timeout, NegativeResponse on 7F, OK otherwise (rx
  // filled).
  ResolverStatus DiagExchange(const uint8_t *tx, uint8_t txLen,
                              uint8_t expectPos, uint8_t *rx);
  ResolverStatus ReadRecords(ResolverCode &out);

  uint32_t lastRecv;
  int16_t speed;
  int16_t inv_temp;
  int16_t motor_temp;
  bool error;
  uint16_t voltage;
  int16_t final_torque_request;

  // Diagnostic RX capture: written from CAN RX interrupt, polled in main loop.
  volatile bool diagRxValid;
  volatile uint8_t diagRxData[8];
};

#endif // LEAFINV_H
