/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "C5DCDC.h"
#include "C5PTECAN.h"
#include "params.h"
#include <algorithm>

namespace {
constexpr int MODE_REQUEST_START_BIT = 9;
}

void C5DCDC::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x105);
  can->RegisterUserMessage(0x314);
  can->RegisterUserMessage(0x315);
}

void C5DCDC::DeInit() {
  counter = 0;
  timer20Ms = 0;
  maxDcdcTemp = 0;
}

void C5DCDC::Task10Ms() {
  timer20Ms++;
  if (timer20Ms >= 2) {
    Send0x11A();
    timer20Ms = 0;
  }
}

void C5DCDC::Send0x11A() {
  uint8_t bytes[8] = {0};
  const int opmode = Param::GetInt(Param::opmode);
  uint8_t modeRequest = 0;

  if (opmode == MOD_RUN || opmode == MOD_CHARGE || opmode == MOD_PREHEAT)
    modeRequest = 1;

  C5PTECAN::PackMotorolaLsb(bytes, MODE_REQUEST_START_BIT, 3, modeRequest);
  C5PTECAN::PackMotorolaLsb(bytes, 29, 11, 0x6FF);
  C5PTECAN::PackMotorolaLsb(bytes, 33, 11, 0x6FF);
  C5PTECAN::PackMotorolaLsb(
      bytes, 40, 9,
      std::min(
          0x1FF,
          std::max(0, (int)(Param::GetFloat(Param::DCSetPnt) * 8.0f + 0.5f))));
  C5PTECAN::FinalizeE2EFrame(bytes, counter++, 12);
  can->Send(0x11A, bytes, 8);
}

void C5DCDC::DecodeCAN(int id, uint8_t *data) {
  switch (id) {
  case 0x105:
    Param::SetFloat(Param::U12V,
                    C5PTECAN::UnpackMotorolaLsb(data, 24, 8) * 0.125f);
    Param::SetFloat(Param::I12V,
                    (float)C5PTECAN::UnpackMotorolaLsb(data, 56, 9));
    break;
  case 0x314: {
    const float m1Temp = C5PTECAN::UnpackMotorolaLsb(data, 16, 8) - 40.0f;
    const float waterTemp = C5PTECAN::UnpackMotorolaLsb(data, 24, 8) - 40.0f;
    const float primaryTemp = C5PTECAN::UnpackMotorolaLsb(data, 32, 8) - 40.0f;
    const float secondaryTemp =
        C5PTECAN::UnpackMotorolaLsb(data, 40, 8) - 40.0f;
    const float ambientTemp = C5PTECAN::UnpackMotorolaLsb(data, 48, 8) - 40.0f;

    maxDcdcTemp =
        std::max(std::max(m1Temp, waterTemp),
                 std::max(std::max(primaryTemp, secondaryTemp), ambientTemp));
    Param::SetFloat(Param::ChgTemp, maxDcdcTemp);
    break;
  }
  case 0x315:
  default:
    break;
  }
}
