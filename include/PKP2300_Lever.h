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
 * Button mapping:
 *   K3 = Forward/Drive, K6 = Reverse, K2 = Neutral, K5 = Park
 *   K1 = Disable regen (toggle), K4 = Heater on/off (toggle)
 *
 * LED display: SoC bar graph, each of K1-K6 represents 16.7%
 *   Default LED color: cyan
 *   When SoC threshold reached: orange
 *   In charge mode: LED above current SoC level blinks orange
 */

#ifndef PKP2300_LEVER_H
#define PKP2300_LEVER_H

#include "shifter.h"
#include <stdint.h>

class PKP2300_Lever : public Shifter {
public:
  void Task100Ms();
  void DecodeCAN(int id, uint32_t *data);
  bool GetGear(Shifter::Sgear &outGear);
  void SetCanInterface(CanHardware *c);

private:
  void UpdateLEDs();
  void SendLEDs();

  Shifter::Sgear gear = NEUTRAL;

  // Previous button state for edge detection
  uint8_t prevButtonState = 0;

  // Toggleable states
  bool regenDisabled = false;
  bool heaterOn = false;

  // Blink state for charge mode LED
  bool blinkState = false;
};

#endif // PKP2300_LEVER_H
