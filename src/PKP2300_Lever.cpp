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
#define PKP_TPDO1 0x195 // button states from panel
#define PKP_RPDO1 0x215 // LED control to panel

// LED bytes
#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE   2

// Button (and LED) bit positions in TPDO1 byte 0
#define BTN_DRIVE   (1 << 0)
#define BTN_NEUTRAL (1 << 1)
#define BTN_REVERSE (1 << 2)
#define BTN_PARK    (1 << 3)
#define BTN_REGEN   (1 << 4)
#define BTN_HEAT    (1 << 5)

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
  //Only allow changing away from PARK if brake pedal is pressed
  bool allowGearChange = gear != PARK || Param::GetBool(Param::din_brake);

  // Detect rising edges (button just pressed) for toggles and gear commands
  uint8_t pressed = buttons & ~prevButtonState;

  // K3 = Forward/Drive
  if ((pressed & BTN_DRIVE) && allowGearChange)
    gear = DRIVE;

  // K6 = Reverse
  if ((pressed & BTN_REVERSE) && allowGearChange)
    gear = REVERSE;

  // K2 = Neutral
  if ((pressed & BTN_NEUTRAL) && allowGearChange)
    gear = NEUTRAL;

  // K5 = Park
  if (pressed & BTN_PARK)
    gear = PARK;

  // K1 = toggle regen disable
  if (pressed & BTN_REGEN) {
    regenDisabled = !regenDisabled;
    Throttle::noregenreq = regenDisabled;
  }

  // K4 = toggle heater
  if (pressed & BTN_HEAT) {
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
  uint8_t ledBytes[8] = {0};

  if (soc > 16.6f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] = BTN_HEAT;
  if (soc > 32.2f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_REVERSE;
  if (soc > 49.8f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_REGEN;
  if (soc > 66.4f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_NEUTRAL;
  if (soc > 83.0f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_PARK;
  if (soc > 99.0f)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_DRIVE;

  if (charging && blinkState) { //turn on one above current SoC if blinkstate is on
    if (soc < 16.6f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] = BTN_HEAT;
    else if (soc < 32.2f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_REVERSE;
    else if (soc < 49.8f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_REGEN;
    else if (soc < 66.4f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_NEUTRAL;
    else if (soc < 83.0f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_PARK;
    else if (soc < 99.0f)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= BTN_DRIVE;

  }

  //Drive mode always takes precedence over SoC display
  if (opmode == MOD_RUN) {
    switch (gear)
    {
    case DRIVE:
      ledBytes[LED_BLUE] = BTN_DRIVE;
      ledBytes[LED_RED] &= ~BTN_DRIVE;
      ledBytes[LED_GREEN] &= ~BTN_DRIVE;
      break;
    case REVERSE:
      ledBytes[LED_BLUE] = BTN_REVERSE;
      ledBytes[LED_RED] &= ~BTN_REVERSE;
      ledBytes[LED_GREEN] &= ~BTN_REVERSE;
      break;
    case NEUTRAL:
      ledBytes[LED_BLUE] = BTN_NEUTRAL;
      ledBytes[LED_RED] &= ~BTN_NEUTRAL;
      ledBytes[LED_GREEN] &= ~BTN_NEUTRAL;
      break;
    case PARK:
      ledBytes[LED_BLUE] = BTN_PARK;
      ledBytes[LED_RED] &= ~BTN_PARK;
      ledBytes[LED_GREEN] &= ~BTN_PARK;
      break;
    }

    if (regenDisabled) {
      ledBytes[LED_BLUE] = BTN_REGEN;
      ledBytes[LED_RED] &= ~BTN_REGEN;
      ledBytes[LED_GREEN] &= ~BTN_REGEN;
    }
  }

  if (heaterOn) {
    ledBytes[LED_BLUE] = BTN_HEAT;
    ledBytes[LED_RED] &= ~BTN_HEAT;
    ledBytes[LED_GREEN] &= ~BTN_HEAT;
  }

  can->Send(PKP_RPDO1, (uint32_t *)ledBytes, 8);
}

bool PKP2300_Lever::GetGear(Shifter::Sgear &outGear) {
  outGear = gear;
  return true;
}
