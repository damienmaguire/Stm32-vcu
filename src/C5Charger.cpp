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

#include "C5Charger.h"
#include "C5PTECAN.h"
#include "params.h"
#include <algorithm>
#include <string.h>

namespace {
constexpr uint8_t BMS_STS_STANDBY = 0x2;
constexpr uint8_t BMS_STS_HV_ON = 0x4;
constexpr uint8_t BMS_STS_AC_CHARGE = 0x6;
constexpr uint8_t VCU_MODE_INIT = 0x0;
constexpr uint8_t VCU_MODE_LV_ON = 0x1;
constexpr uint8_t VCU_MODE_HV_ON = 0x2;
constexpr uint8_t VCU_MODE_READY = 0x3;
constexpr uint8_t VCU_MODE_SLOW_CHARGE = 0x4;
constexpr uint8_t VCU_MODE_PREHEAT = 0x9;
constexpr int SOC_FULL_THRESHOLD = 99;
constexpr float OBC_OUTPUT_CURRENT_OFFSET = -70.0f;
} // namespace

void C5Charger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x101);
  can->RegisterUserMessage(0x102);
  can->RegisterUserMessage(0x103);
  can->RegisterUserMessage(0x104);
  can->RegisterUserMessage(0x310);
  can->RegisterUserMessage(0x311);
  can->RegisterUserMessage(0x312);
}

void C5Charger::DeInit() {
  chargeCommand = false;
  obcFaultPresent = false;
  obcStatus = 0;
  obcFaultLevel = 0;
  obcWorkMode = 0;
  obcRc = 0;
  bmsCounter = 0;
  vcuNmCounter = 0;
  timer20Ms = 0;
  timer500Ms = 0;
  maxChargeTemp = 0;
}

bool C5Charger::ControlCharge(bool RunCh, bool ACReq) {
  chargeCommand = RunCh && ACReq;
  return ACReq && obcStatus != 0xA && obcFaultLevel < 4 && !obcFaultPresent;
}

float C5Charger::MaxChargePowerWatts() const {
  const float hvVolts =
      std::max(1.0f, (float)std::max(Param::GetInt(Param::udc),
                                     Param::GetInt(Param::Voltspnt)));
  float powerLimit = (float)Param::GetInt(Param::Pwrspnt);
  const int bmsCurrentLimit = Param::GetInt(Param::BMS_ChargeLim);
  const int bmsPowerLimit = Param::GetInt(Param::BMS_MaxCharge);

  if (bmsCurrentLimit > 0)
    powerLimit = std::min(powerLimit, hvVolts * bmsCurrentLimit);

  if (bmsPowerLimit > 0)
    powerLimit = std::min(powerLimit, (float)bmsPowerLimit);

  return std::max(0.0f, powerLimit);
}

float C5Charger::MaxChargeCurrentAmps() const {
  const float hvVolts =
      std::max(1.0f, (float)std::max(Param::GetInt(Param::udc),
                                     Param::GetInt(Param::Voltspnt)));
  float currentLimit = MaxChargePowerWatts() / hvVolts;
  const int bmsCurrentLimit = Param::GetInt(Param::BMS_ChargeLim);

  if (bmsCurrentLimit > 0)
    currentLimit = std::min(currentLimit, (float)bmsCurrentLimit);

  return std::max(0.0f, currentLimit);
}

bool C5Charger::HvConnected() const {
  const int opmode = Param::GetInt(Param::opmode);
  return opmode == MOD_RUN || opmode == MOD_CHARGE || opmode == MOD_PRECHARGE ||
         opmode == MOD_PREHEAT;
}

void C5Charger::Task10Ms() {
  Send0x0AD();
}

void C5Charger::Task100Ms() {
  Send0x09C();
  Send0x304();
  Send0x305();
  Send0x306();
  Send0x308();
  Send0x321();
  Send0x322();

  timer500Ms++;
  if (timer500Ms >= 5) {
    Send0x485();
    timer500Ms = 0;
  }
}

void C5Charger::Send0x0AD() {
  uint8_t bytes[8] = {0};
  const float loadPower = MaxChargePowerWatts();
  const uint32_t chargeLoadRaw =
      std::min(255, std::max(0, (int)(loadPower / 100.0f + 0.5f)));

  C5PTECAN::PackMotorolaLsb(bytes, 16, 8, chargeLoadRaw);
  C5PTECAN::PackMotorolaLsb(bytes, 24, 2, 0);
  C5PTECAN::PackMotorolaLsb(bytes, 32, 8, 0);
  can->Send(0xAD, bytes, 8);
}

void C5Charger::Send0x09C() {
  uint8_t bytes[8] = {0};
  const uint32_t packVoltageRaw =
      std::min(0x1FFF, std::max(0, (int)(Param::GetFloat(Param::udc) * 10.0f)));

  C5PTECAN::PackMotorolaLsb(bytes, 35, 13, packVoltageRaw);
  C5PTECAN::PackMotorolaLsb(bytes, 56, 2, HvConnected() ? 2 : 0);
  can->Send(0x9C, bytes, 8);
}

void C5Charger::Send0x304() {
  uint8_t bytes[8] = {0};
  const float actualChargePower =
      std::max(0.0f, Param::GetFloat(Param::udc) * Param::GetFloat(Param::idc));
  const uint32_t actualChargePowerRaw =
      std::min(0x3FF, std::max(0, (int)(actualChargePower / 100.0f + 0.5f)));
  const uint32_t chargeVoltageRaw = std::min(
      0x1FFF, std::max(0, (int)(Param::GetFloat(Param::Voltspnt) * 10.0f + 0.5f)));
  const uint32_t chargeCurrentReqRaw = std::min(
      0x1FFF,
      std::max(0, (int)((MaxChargeCurrentAmps() + 700.0f) * 10.0f +
                        0.5f))); // protocol offset is -700 A

  C5PTECAN::PackMotorolaLsb(bytes, 18, 10, actualChargePowerRaw);
  C5PTECAN::PackMotorolaLsb(bytes, 37, 13, chargeVoltageRaw);
  C5PTECAN::PackMotorolaLsb(bytes, 40, 13, chargeCurrentReqRaw);
  C5PTECAN::PackMotorolaLsb(bytes, 56, 1, chargeCommand ? 1 : 0);
  C5PTECAN::PackMotorolaLsb(bytes, 57, 1, 0);
  C5PTECAN::PackMotorolaLsb(bytes, 58, 1, chargeCommand ? 1 : 0);
  C5PTECAN::PackMotorolaLsb(bytes, 59, 13, chargeVoltageRaw);
  C5PTECAN::FinalizeE2EFrame(bytes, bmsCounter++, 12);
  can->Send(0x304, bytes, 8);
}

void C5Charger::Send0x305() {
  uint8_t bytes[8] = {0};
  uint8_t status = BMS_STS_STANDBY;

  if (chargeCommand)
    status = BMS_STS_AC_CHARGE;
  else if (HvConnected())
    status = BMS_STS_HV_ON;

  C5PTECAN::PackMotorolaLsb(bytes, 33, 4, status);
  can->Send(0x305, bytes, 8);
}

void C5Charger::Send0x306() {
  uint8_t bytes[8] = {0};
  C5PTECAN::PackMotorolaLsb(bytes, 40, 2, 0);
  can->Send(0x306, bytes, 8);
}

void C5Charger::Send0x308() {
  uint8_t bytes[8] = {0};
  const uint32_t socFull =
      Param::GetInt(Param::SOC) >= SOC_FULL_THRESHOLD ? 1 : 0;

  C5PTECAN::PackMotorolaLsb(bytes, 26, 1, socFull);
  C5PTECAN::PackMotorolaLsb(bytes, 48, 2, 0);
  can->Send(0x308, bytes, 8);
}

void C5Charger::Send0x321() {
  uint8_t bytes[8] = {0};
  uint8_t electricMode = VCU_MODE_INIT;
  const int opmode = Param::GetInt(Param::opmode);

  if (chargeCommand || opmode == MOD_CHARGE)
    electricMode = VCU_MODE_SLOW_CHARGE;
  else if (opmode == MOD_RUN)
    electricMode = VCU_MODE_READY;
  else if (opmode == MOD_PRECHARGE)
    electricMode = VCU_MODE_HV_ON;
  else if (opmode == MOD_PREHEAT)
    electricMode = VCU_MODE_PREHEAT;
  else if (HvConnected())
    electricMode = VCU_MODE_LV_ON;

  C5PTECAN::PackMotorolaLsb(bytes, 60, 4, electricMode);
  can->Send(0x321, bytes, 8);
}

void C5Charger::Send0x322() {
  uint8_t bytes[8] = {0};
  C5PTECAN::PackMotorolaLsb(bytes, 9, 3, 0);
  C5PTECAN::PackMotorolaLsb(bytes, 40, 8, 0);
  C5PTECAN::PackMotorolaLsb(bytes, 57, 7, 0);
  can->Send(0x322, bytes, 8);
}

void C5Charger::Send0x485() {
  uint8_t bytes[8] = {0};
  const bool active = Param::GetInt(Param::opmode) != MOD_OFF;

  C5PTECAN::PackMotorolaLsb(bytes, 0, 8, 0x85);
  C5PTECAN::PackMotorolaLsb(bytes, 8, 1, 0);
  C5PTECAN::PackMotorolaLsb(bytes, 12, 1, active ? 1 : 0);
  C5PTECAN::PackMotorolaLsb(bytes, 22, 1, active ? 1 : 0);
  C5PTECAN::PackMotorolaLsb(bytes, 23, 1, vcuNmCounter++ & 0x1);
  can->Send(0x485, bytes, 8);
}

void C5Charger::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;

  switch (id) {
  case 0x101:
    obcRc = (uint8_t)C5PTECAN::UnpackMotorolaLsb(bytes, 12, 4);
    obcStatus = (uint8_t)C5PTECAN::UnpackMotorolaLsb(bytes, 36, 4);
    obcFaultLevel = (uint8_t)C5PTECAN::UnpackMotorolaLsb(bytes, 33, 3);
    obcWorkMode = (uint8_t)C5PTECAN::UnpackMotorolaLsb(bytes, 26, 3);
    obcFaultPresent = obcStatus == 0xA || obcFaultLevel >= 4;
    break;
  case 0x102: {
    const float acCurrentL1 =
        C5PTECAN::UnpackMotorolaLsb(bytes, 15, 9) * 0.1f;
    const float acCurrentL2 =
        C5PTECAN::UnpackMotorolaLsb(bytes, 22, 9) * 0.1f;
    const float acCurrentL3 =
        C5PTECAN::UnpackMotorolaLsb(bytes, 29, 9) * 0.1f;
    const float acVoltageL1 = C5PTECAN::UnpackMotorolaLsb(bytes, 36, 9);
    const float acVoltageL2 = C5PTECAN::UnpackMotorolaLsb(bytes, 43, 9);
    const float acVoltageL3 = C5PTECAN::UnpackMotorolaLsb(bytes, 56, 9);

    Param::SetFloat(Param::AC_Amps,
                    std::max(acCurrentL1, std::max(acCurrentL2, acCurrentL3)));
    Param::SetFloat(Param::AC_Volts,
                    std::max(acVoltageL1, std::max(acVoltageL2, acVoltageL3)));
    break;
  }
  case 0x103: {
    const float hvVolts = C5PTECAN::UnpackMotorolaLsb(bytes, 24, 14) * 0.1f;
    const float hvCurrent =
        C5PTECAN::UnpackMotorolaLsb(bytes, 50, 11) * 0.1f +
        OBC_OUTPUT_CURRENT_OFFSET;

    if (Param::GetInt(Param::ShuntType) == 0 &&
        Param::GetInt(Param::Inverter) != InvModes::Leaf_Gen1) {
      // Only backfill HV voltage/current from the charger when no dedicated
      // shunt is configured and the Leaf inverter is not already providing it.
      Param::SetFloat(Param::udc, hvVolts);
      Param::SetFloat(Param::idc, std::max(0.0f, hvCurrent));
    }
    break;
  }
  case 0x310: {
    const float wtrInlet = C5PTECAN::UnpackMotorolaLsb(bytes, 48, 8) - 40.0f;
    maxChargeTemp = wtrInlet;
    Param::SetFloat(Param::ChgTemp, maxChargeTemp);
    break;
  }
  case 0x311: {
    const float intAmb = C5PTECAN::UnpackMotorolaLsb(bytes, 0, 8) - 40.0f;
    const float m1Pwr = C5PTECAN::UnpackMotorolaLsb(bytes, 16, 8) - 40.0f;
    const float pfc = C5PTECAN::UnpackMotorolaLsb(bytes, 32, 8) - 40.0f;
    const float llc = C5PTECAN::UnpackMotorolaLsb(bytes, 48, 8) - 40.0f;

    maxChargeTemp =
        std::max(std::max(intAmb, m1Pwr), std::max(std::max(pfc, llc), maxChargeTemp));
    Param::SetFloat(Param::ChgTemp, maxChargeTemp);
    break;
  }
  case 0x312:
    break;
  case 0x104:
  default:
    break;
  }
}
