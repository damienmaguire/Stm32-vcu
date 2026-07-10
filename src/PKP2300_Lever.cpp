/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2024 Damien Maguire
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
 *
 * Support for the Blink Marine PKP-2300-SI CANOpen button panel.
 * Spec: https://www.murcal.com/pdf%20folder/15.blink_PKP-2300-SI_CANOpen.pdf
 *
 * CANOpen node ID 1 (default):
 *   TPDO1 (panel → VCU): 0x181 – button states in byte 0, one bit per key
 *   RPDO1 (VCU → panel): 0x201 – LED colors, one byte per LED (K1–K6)
 *
 * LED color codes:
 *   0=off, 1=red, 2=green, 3=blue, 4=cyan, 5=magenta, 6=orange, 7=white
 */

#include "PKP2300_Lever.h"
#include "params.h"
#include "throttle.h"

// CANOpen IDs for node 1
#define PKP_TPDO1 0x181 // button states from panel
#define PKP_RPDO1 0x201 // LED control to panel

// LED color codes
#define LED_OFF    0
#define LED_CYAN   4
#define LED_ORANGE 6

// Button bit positions in TPDO1 byte 0
#define BTN_K1 (1 << 0)
#define BTN_K2 (1 << 1)
#define BTN_K3 (1 << 2)
#define BTN_K4 (1 << 3)
#define BTN_K5 (1 << 4)
#define BTN_K6 (1 << 5)

// SoC threshold (%) for each LED (K1=16.7%, K2=33.3%, ..., K6=100%)
static const float socThreshold[6] = {
    100.0f / 6.0f * 1,
    100.0f / 6.0f * 2,
    100.0f / 6.0f * 3,
    100.0f / 6.0f * 4,
    100.0f / 6.0f * 5,
    100.0f / 6.0f * 6,
};

void PKP2300_Lever::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(PKP_TPDO1);
}

void PKP2300_Lever::DecodeCAN(int id, uint32_t *data) {
  if (id != PKP_TPDO1)
    return;

  uint8_t *bytes = (uint8_t *)data;
  uint8_t buttons = bytes[0];

  // Detect rising edges (button just pressed) for toggles and gear commands
  uint8_t pressed = buttons & ~prevButtonState;

  // K3 = Forward/Drive
  if (pressed & BTN_K3)
    gear = DRIVE;

  // K6 = Reverse
  if (pressed & BTN_K6)
    gear = REVERSE;

  // K2 = Neutral
  if (pressed & BTN_K2)
    gear = NEUTRAL;

  // K5 = Park
  if (pressed & BTN_K5)
    gear = PARK;

  // K1 = toggle regen disable
  if (pressed & BTN_K1) {
    regenDisabled = !regenDisabled;
    Throttle::noregenreq = regenDisabled;
  }

  // K4 = toggle heater
  if (pressed & BTN_K4) {
    heaterOn = !heaterOn;
    Param::SetInt(Param::HeatReq, heaterOn ? 1 : 0);
  }

  prevButtonState = buttons;
}

void PKP2300_Lever::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);
  if (opmode == MOD_OFF)
    gear = NEUTRAL;

  // Alternate blink state every 100 ms call
  blinkState = !blinkState;

  SendLEDs();
}

void PKP2300_Lever::SendLEDs() {
  float soc = Param::GetFloat(Param::SOC);
  int opmode = Param::GetInt(Param::opmode);
  bool charging = (opmode == MOD_CHARGE);

  // Find the LED index that is just above the current SoC level.
  // This LED blinks orange in charge mode.
  int blinkLed = -1; // no blink LED
  if (charging) {
    for (int i = 0; i < 6; i++) {
      if (soc < socThreshold[i]) {
        blinkLed = i;
        break;
      }
    }
  }

  uint8_t ledBytes[8] = {0};
  for (int i = 0; i < 6; i++) {
    if (i == blinkLed) {
      // The LED just above current SoC blinks orange in charge mode
      ledBytes[i] = blinkState ? LED_ORANGE : LED_OFF;
    } else if (soc >= socThreshold[i]) {
      // SoC has reached this LED's threshold – show orange
      ledBytes[i] = LED_ORANGE;
    } else {
      // Below threshold – show default cyan background
      ledBytes[i] = LED_CYAN;
    }
  }

  can->Send(PKP_RPDO1, (uint32_t *)ledBytes, 8);
}

bool PKP2300_Lever::GetGear(Shifter::Sgear &outGear) {
  outGear = gear;
  return true;
}
