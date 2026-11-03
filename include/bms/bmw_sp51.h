/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
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

#ifndef BMW_SP51_H
#define BMW_SP51_H
#include "bms/bms.h"

/* BMW Gen5 PHEV battery (SP51, Preh EA443 SME), classic CAN 500k.
 * The pack has its own contactors and precharge. The ZombieVerter takes
 * the EME role: it sends the close request (0x10B, 0x53A) and, when
 * SP51Supp is on, the few car frames the pack needs to stay awake.
 * See docs/bmw-f-series/sp51-battery.md.
 */
class BMW_SP51 : public BMS {
public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint8_t *data) override;
  float MaxChargeCurrent() override;
  void Task10Ms() override;
  void Task100Ms() override;

  // Steps of the close sequence, shown as SP51State.
  enum State {
    WAKE = 0,    // sending, the pack has not answered yet
    STANDBY = 1, // pack awake, waiting for it to settle
    READY = 2,   // pack settled and open, a close may start
    REQUEST = 3, // 0x53A state B for 2 s before the close
    CLOSING = 4, // close sent, waiting for the pack to report closed
    CLOSED = 5,  // pack reports closed (HV ready)
    FAULT = 6,   // aborted, open sent, waiting for the ZV outputs off
    LOST = 7     // the pack was awake but 0x112 has stopped
  };

  // Why the last close was aborted, shown as SP51Fault.
  enum Fault {
    NONE = 0,
    PACK_LOST = 1,     // 0x112 stopped while closing or closed
    OPEN_REQ = 2,      // the pack asked to open (0x112 b5 or b6)
    PRECHECK_FAIL = 3, // pack not ready when the close was due
    CLOSE_TIMEOUT = 4, // no "closed" within 5 s of the close
    LEFT_CLOSED = 5    // pack left "closed" for more than 1 s
  };

private:
  bool CloseRequested();
  bool PackValid();
  bool PackSettled();
  bool PackClosed();
  bool LimitsAllowed();
  bool PackOpenRequest();
  void RunStateMachine();
  void SendFrames();
  void Abort(Fault why);
  void Restart();

  // Received from the pack (written in the CAN interrupt).
  volatile uint16_t age112 = 0xFFFF; // 10 ms ticks since the last 0x112
  volatile bool seen112 = false;
  volatile bool seenPack = false;   // any pack frame since start up
  volatile uint8_t n45 = 0;         // 0x1FA b1 == 0x45 in a row
  volatile uint8_t last1FA = 0;     // 0x1FA b1, high nibble = HV state
  volatile uint8_t openReq = 0;     // 0x112 b5 top two bits, 1 = normal
  volatile uint8_t openInstant = 0; // 0x112 b6 bits 0-1, 1 = normal
  volatile uint8_t openFast = 0;    // 0x112 b6 bits 2-3, 1 = normal
  volatile float packVolts = 0;
  volatile float packAmps = 0;
  volatile float linkVolts = 0;
  volatile float tempMin = 0, tempMax = 0;
  volatile float uChargeMax = 0, uDischargeMin = 0;
  volatile float iChargeMax = 0, iDischargeMax = 0;
  volatile float pChargeKw = 0, pDischargeKw = 0;
  volatile uint8_t soc = 0xFF;

  // Our side of the conversation.
  uint32_t tick = 0;      // 10 ms ticks since sending started
  uint16_t steady112 = 0; // 10 ms ticks of 0x112 without a gap
  uint8_t alive10B = 0, alive12F = 0, alive1A1 = 0;
  uint32_t seconds328 = 0;
  uint16_t days328 = 0;
  uint8_t closeCmd = 0; // 0x10B command, 0 = open, 1 = close
  uint8_t stage53A = 0; // 0x53A state A, B, C, D = 0, 1, 2, 3
  uint8_t framesC = 0;  // 0x53A state C frames sent
  uint8_t state = WAKE;
  uint8_t fault = NONE;
  uint16_t stepTicks = 0; // 10 ms ticks spent in the current step
  uint16_t notClosedTicks = 0;
  uint16_t silentTicks = 0; // wake retry, counts down while silent
  bool wakeRetried = false;
};

#endif // BMW_SP51_H
