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

#include "my_math.h"
#include "params.h"
#include "utils.h"

namespace utils {

void SelectDirection(Vehicle *vehicle, Shifter *shifter) {
  Vehicle::gear gear;
  Shifter::Sgear gearS;
  int8_t selectedDir = Param::GetInt(Param::dir);
  int8_t userDirSelection = 0;
  static int8_t prevValidDir = 0;
  int8_t dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;
  uint8_t ChangeLim =
      Param::GetInt(Param::DirChange); //"0=None, 1=Speed Thres, 2=Speed+Brake"

  // Forget the last valid direction at key-off, so the first forward/reverse
  // selection of each drive cycle goes through the change limit checks again
  if (Param::GetInt(Param::opmode) == MOD_OFF)
    prevValidDir = 0;

  // Collecting Inputs//
  if (vehicle->GetGear(gear)) {
    // if the vehicle class supplies gear selection then use that
    switch (gear) {
    case Vehicle::PARK:
      userDirSelection = 2; // Park
      break;
    case Vehicle::REVERSE:
      userDirSelection = -1; // Reverse
      break;
    case Vehicle::NEUTRAL:
      userDirSelection = 0; // Neutral
      break;
    case Vehicle::DRIVE:
      userDirSelection = 1; // Drive
      break;
    }
  } else if (shifter->GetGear(gearS)) {
    // if the shifter class supplies gear selection then use that
    switch (gearS) {
    case Shifter::PARK:
      userDirSelection = 2; // Park
      break;
    case Shifter::REVERSE:
      userDirSelection = -1; // Reverse
      break;
    case Shifter::NEUTRAL:
      userDirSelection = 0; // Neutral
      break;
    case Shifter::DRIVE:
      userDirSelection = 1; // Drive
      break;
    }
  } else {
    // otherwise use the traditional inputs
    if (Param::GetInt(Param::dirmode) == DIR_DEFAULTFORWARD) {
      if (Param::GetBool(Param::din_forward) &&
          Param::GetBool(Param::din_reverse))
        selectedDir = 0;
      else if (Param::GetBool(Param::din_reverse))
        userDirSelection = -1;
      else
        userDirSelection = 1;
    } else if ((Param::GetInt(Param::dirmode) & 1) == DIR_BUTTON) {
      /* if forward AND reverse selected, force neutral, because it's charge
       * mode */
      if (Param::GetBool(Param::din_forward) &&
          Param::GetBool(Param::din_reverse))
        selectedDir = 0;
      else if (Param::GetBool(Param::din_forward))
        userDirSelection = 1 * dirSign;
      else if (Param::GetBool(Param::din_reverse))
        userDirSelection = -1 * dirSign;
      else
        userDirSelection = selectedDir;
    } else {
      /* neither forward nor reverse or both forward and reverse -> neutral */
      if (!(Param::GetBool(Param::din_forward) ^
            Param::GetBool(Param::din_reverse)))
        selectedDir = 0;
      else if (Param::GetBool(Param::din_forward))
        userDirSelection = 1 * dirSign;
      else if (Param::GetBool(Param::din_reverse))
        userDirSelection = -1 * dirSign;
    }
    if (Param::GetInt(Param::DriveInhibit) ==
        1) // drive inhibit selected, check for plug
    {
      if (Param::GetInt(Param::PlugDet) ==
          1) // If charge socket is not empty. Force Neutral
      {
        userDirSelection = 0;
      }
    }
  }
  // Change Allowed Logic//

  if (selectedDir != userDirSelection) // only run if requested change
  {
    if (ChangeLim == 0) // No limits on requesting new direction
    {
      selectedDir = userDirSelection; // direct pass through
    } else if (ChangeLim == 1 ||
               ChangeLim == 2) // speed limit only when changing F to R or R to
                               // F, note last selected direction is valid,
                               // ignore neutral
    {
      if (userDirSelection != 0 &&
          userDirSelection !=
              prevValidDir) // neutral changes always allowed, check if we are
                            // not going back into same mode as before neutral
      {
        if (Param::GetInt(Param::DirChangeRpm) >
            (ABS(Param::GetInt(Param::speed)))) {
          if (Param::GetBool(Param::din_brake) && ChangeLim == 2) {
            selectedDir = userDirSelection;
            prevValidDir =
                selectedDir; // update only when its a valid forward or
                             // reverse action under speed threshold
          } else if (ChangeLim == 1) {
            selectedDir = userDirSelection;
            prevValidDir =
                selectedDir; // update only when its a valid forward or
                             // reverse action under speed threshold
          }
        }
      } else {
        selectedDir = userDirSelection; // direct pass through
      }
    }
  }

  // Shiftlock out control
  if (ChangeLim == 0) // No limits on requesting new direction
  {
    SetInt(Param::ShiftLock, 0); // No shift lock out ever
  } else if (Param::GetInt(Param::DirChangeRpm) >
             (ABS(Param::GetInt(Param::speed)))) // speed based limit
  {
    if (Param::GetBool(Param::din_brake) &&
        ChangeLim == 2) // brake input required
    {
      SetInt(Param::ShiftLock,
             1);               // Solenoid pull out brake pressed and speed low
    } else if (ChangeLim == 1) // if only speed
    {
      SetInt(Param::ShiftLock, 1); // Solenoid pull out speed low
    } else {
      SetInt(Param::ShiftLock, 0); // No shift lock out speed too high
    }
  } else {
    SetInt(Param::ShiftLock, 0); // No shift lock out speed too high
  }

  // Always off outside of run
  if (Param::GetInt(Param::opmode) != MOD_RUN) {
    SetInt(Param::ShiftLock, 0); // No shift lock out speed too high
  }

  Param::SetInt(Param::dir, selectedDir); // final writing of DIR
}

} // namespace utils
