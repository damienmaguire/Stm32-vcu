/*
 * This file is part of the ZombieVerter project.
 *
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

#include "hyundai_bms.h"

uint32_t slow_can_counter = 0;

void HyundaiBMS::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(ID_BMS_CONTACTOR_STATUS);
  can->RegisterUserMessage(ID_BMS_SOC);
  can->RegisterUserMessage(ID_BMS_VOLTAGE);
  can->RegisterUserMessage(ID_BMS_CURRENT);
  can->RegisterUserMessage(ID_BMS_AVAILABLE_POWER);
}

uint8_t HyundaiBMS::CRC_8(const uint8_t *DataArray, const uint8_t Length) {
  uint8_t CRC = 0;

  for (uint8_t i = 0; i < Length; i++)
    //		CRC = CRC_8_TABLE[CRC ^ DataArray[i]];
    CRC = CRC ^ DataArray[i]; // Crc poly 0x1
  return CRC;
}

bool HyundaiBMS::BMSDataValid() {
  // Return false if primary BMS is not sending data.
  if (timeoutCounter < 1)
    return false;
  return true;
}

// Return whether charging is currently permitted.
bool HyundaiBMS::ChargeAllowed() {
  // Refuse to charge if the BMS is not sending data.
  if (!BMSDataValid())
    return false;

  // Refuse to charge if the current limit is low.
  if (availableChargePower > -1000)
    return false;

  // Otherwise, charging is permitted.
  return true;
}

// Return the maximum charge current allowed by the BMS.
float HyundaiBMS::MaxChargeCurrent() {
  if (!ChargeAllowed() || !BMSDataValid())
    return 0;
  return availableChargePower / voltage;
}

// Process voltage and temperature message from SimpBMS.
void HyundaiBMS::DecodeCAN(int id, uint8_t *data) {

  switch (id) {
  case ID_BMS_CONTACTOR_STATUS:
    contactor_state = data[0];
    /* if (contactor_state == 0x01)
        Param::SetInt(Param::opmode, MOD_RUN);
     else if (contactor_state == 0x02)
        Param::SetInt(Param::opmode, MOD_PRECHARGE);*/

    // Reset timeout counter to the full timeout value
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
    break;

  case ID_BMS_SOC:
    soc = ((data[1] << 8) + data[0]) * 0.05f;
    Param::SetFloat(Param::KWh,
                    (1.0f - (soc * 0.01)) * Param::GetFloat(Param::BattCap));
    Param::SetFloat(Param::SOC, soc);
    break;

  case ID_BMS_VOLTAGE:
    voltage = ((data[1] << 8) + data[0]) * 0.1f;
    Param::SetFloat(Param::udc, voltage);
    break;

  case ID_BMS_CURRENT:
    current = ((int16_t)(data[3] << 8) + data[2]) * 0.05f;
    Param::SetFloat(Param::idc, current);
    break;

  case ID_BMS_AVAILABLE_POWER:
    availableChargePower = (data[0] + (data[1] << 8)) * -10;
    Param::SetFloat(Param::idcmin, availableChargePower / voltage);
    Param::SetFloat(Param::idcmin, availableChargePower / voltage);
    availableDischargePower = ((data[2] + (data[3] << 8)) * 10);
    Param::SetFloat(Param::idcmax, availableDischargePower / voltage);
    Param::SetFloat(Param::idcmax, availableDischargePower / voltage);
    break;
  default:
    break;
  }
}

void HyundaiBMS::ControlContactors(int opmode, CanHardware *can) {
  switch (opmode) {
  case 0: // Off
    hvRequest = false;
    fault = false;
    break;
  case 2: // Precharge
    hvRequest = true;
    break;

  case 1: // Run
    hvRequest = true;
    break;

  case 4: // Charge
    hvRequest = true;
    break;

  case 3: // Precharge fail
    hvRequest = false;
    fault = true;
    break;
  }

  if (!fault) {
    if (hvRequest) {
      txData200[5] |= 0x10;
      txData200[4] |= 0x80;
    } else {
      txData200[5] &= ~0x10;
      txData200[4] &= ~0x80;
    }
  } else {
    txData200[5] = 0x20;
    can->Send(0x200, (uint32_t *)txData200, 8);
  }
}

void HyundaiBMS::Task10Ms() {
  if (Param::GetInt(Param::opmode) != MOD_OFF) {
    if (Param::GetInt(Param::opmode) == MOD_PRECHARGE) {
      slow_can_counter++;
      if (slow_can_counter >= 50)
        slow_can_counter = 0;
      else
        return;
    }
    Inverter_voltage[6] = (Param::GetInt(Param::INVudc) / 2) * 1.025;
    can->Send(ID_BMS_INV_VOLTAGE, (uint32_t *)Inverter_voltage, 8);
    can->Send(0x200, (uint32_t *)txData200, 8);
    can->Send(0x2F0, (uint32_t *)txData2F0, 8);
  }
}

void HyundaiBMS::Task100Ms() {
  // Decrement timeout counter.
  if (timeoutCounter > 0)
    timeoutCounter--;

  uint8_t opmode = Param::GetInt(Param::opmode);
  if (opmode != MOD_OFF) {
    BCM_data[0] = 0x60;
    BCM_data[2] = 0x60;
  } else {
    BCM_data[0] = 0x00;
    BCM_data[2] = 0x00;
  }

  can->Send(0x523, (uint32_t *)BCM_data, 8);
}