/*
 * This file is part of the ZombieVerter project.
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

#include "params.h"
#include "shifters/shifter.h"
#include "test_list.h"
#include "utils.h"
#include "vehicles/vehicle.h"

using namespace std;

// Vehicle that always reports the gear selected by the test
class FakeVehicle : public Vehicle {
public:
  gear selectedGear = NEUTRAL;
  void SetRevCounter(int) override {}
  void SetTemperatureGauge(float) override {}
  bool Ready() override { return true; }
  bool GetGear(gear &g) override {
    g = selectedGear;
    return true;
  }
};

static FakeVehicle vehicle;
static Shifter noShifter;

static int Select(Vehicle::gear gear) {
  vehicle.selectedGear = gear;
  utils::SelectDirection(&vehicle, &noShifter);
  return Param::GetInt(Param::dir);
}

static void SetSpeed(int rpm) { Param::SetInt(Param::speed, rpm); }

static void SetBrake(bool pressed) { Param::SetInt(Param::din_brake, pressed); }

static void KeyOff() {
  Param::SetInt(Param::opmode, MOD_OFF);
  Select(Vehicle::NEUTRAL);
  Param::SetInt(Param::opmode, MOD_RUN);
}

// changeLim: 0=None, 1=Speed threshold, 2=Speed threshold + brake
static void TestSetup(int changeLim) {
  Param::SetInt(Param::DirChange, changeLim);
  Param::SetInt(Param::DirChangeRpm, 500);
  Param::SetInt(Param::dir, 0);
  SetSpeed(0);
  SetBrake(false);
  // Starts every test in neutral with no remembered direction
  KeyOff();
}

static void TestNoLimitChangesDirectionAtSpeed() {
  TestSetup(0);
  SetSpeed(3000);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  ASSERT(Select(Vehicle::REVERSE) == -1);
}

static void TestSpeedLimitBlocksForwardToReverseAboveThreshold() {
  TestSetup(1);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  SetSpeed(3000);
  ASSERT(Select(Vehicle::REVERSE) == 1);
}

static void TestSpeedLimitAllowsForwardToReverseBelowThreshold() {
  TestSetup(1);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  SetSpeed(100);
  ASSERT(Select(Vehicle::REVERSE) == -1);
}

static void TestNeutralBackToPreviousDirectionAllowedWhileRolling() {
  TestSetup(1);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  SetSpeed(3000);
  ASSERT(Select(Vehicle::NEUTRAL) == 0);
  ASSERT(Select(Vehicle::DRIVE) == 1);
}

static void TestNeutralToOppositeDirectionBlockedWhileRolling() {
  TestSetup(1);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  SetSpeed(3000);
  ASSERT(Select(Vehicle::NEUTRAL) == 0);
  ASSERT(Select(Vehicle::REVERSE) == 0);
}

static void TestSpeedAndBrakeLimitNeedsBrake() {
  TestSetup(2);
  ASSERT(Select(Vehicle::DRIVE) == 0);
  SetBrake(true);
  ASSERT(Select(Vehicle::DRIVE) == 1);
}

static void TestKeyOffForgetsPreviousDirection() {
  TestSetup(2);
  SetBrake(true);
  ASSERT(Select(Vehicle::DRIVE) == 1);
  SetBrake(false);
  ASSERT(Select(Vehicle::NEUTRAL) == 0);
  KeyOff();
  // New drive cycle: selecting the same gear again needs the brake
  ASSERT(Select(Vehicle::DRIVE) == 0);
  SetBrake(true);
  ASSERT(Select(Vehicle::DRIVE) == 1);
}

void DirectionTest::RunTest() {
  TestNoLimitChangesDirectionAtSpeed();
  TestSpeedLimitBlocksForwardToReverseAboveThreshold();
  TestSpeedLimitAllowsForwardToReverseBelowThreshold();
  TestNeutralBackToPreviousDirectionAllowedWhileRolling();
  TestNeutralToOppositeDirectionBlockedWhileRolling();
  TestSpeedAndBrakeLimitNeedsBrake();
  TestKeyOffForgetsPreviousDirection();
}
