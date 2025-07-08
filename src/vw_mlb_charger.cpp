/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024 Mitch Elliott
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



#include <vw_mlb_charger.h>

uCAN_MSG txMsg;
uint8_t CanMsgBuf[8];

struct VehicleStatus{
    bool locked = false;
    bool CANQuiet = false;
} vehicle_status;

struct ChargerStatus {
    uint16_t ACvoltage ;
    uint16_t HVVoltage ;
    int8_t temperature ;
    uint8_t mode;
    uint16_t current;
    uint8_t MaxACAmps;
    uint8_t PPLim;
    uint32_t HVLM_MaxDC_ChargePower;        // maximum DC charging power
    uint16_t HVLM_Max_DC_Voltage_DCLS;      // maximum DC charging voltage
    uint16_t HVLM_Actual_DC_Current_DCLS;   // actual DC charging current
    uint16_t HVLM_Max_DC_Current_DCLS;      // maximum DC charging current
    uint16_t HVLM_Min_DC_Voltage_DCLS;      // minimum DC charging voltage
    uint16_t HVLM_Min_DC_Current_DCLS;      // minimum DC charging current
    uint8_t HVLM_Status_Grid;               // Information as to whether the vehicle is connected to a power grid.
    uint8_t HVLM_EnergyFlowType;            // Display of whether electricity flows into the vehicle and what it is used for.
    uint8_t HVLM_OperationalMode;           // Current Mode: 0=Inactive, 1=Active, 2=Init, 3=Error
    uint8_t HVLM_HV_ActivationRequest;      // HV activation request and reason for the request: 0=No Request, 1=Charging, 2=Battery Balancing, 3=AC/Climate
    uint8_t HVLM_ChargerErrorStatus;        // Current error status of the charger: 0=No Error, 1=DC-NotOK, 2=AC-NotOK, 3=Interlock, 4=Reserved, 5=Reserved, 6=No Component Function, 7=Init
    uint8_t HVLM_Park_Request;              // Request to lock the drive train
    uint8_t HVLM_Park_Request_Maintain;     // Request to keep the drive train locked
    uint8_t HVLM_Plug_Status;               // Plug detection status independent of charging mode (AC or DC): 0=Init, 1=No Plug Inserted, 2=Connector Inserted, Not Locked, 3=Connector Inserted & Locked
    uint8_t HVLM_LoadRequest;               // Charger Status: 0=No Request, 1=AC Charge, 2=DC Charge, 3=Recharge 12V, 4=AC AWC Charge, 5=Reserved, 6=Init, 7=Error
    uint8_t HVLM_MaxBattChargeCurrent;      // Recommended HV battery charging current for a planned charge
    uint8_t LAD_Mode;                       // Operating mode of the charger
    uint16_t LAD_AC_Volt_RMS;               // Actual value AC grid voltage (RMS)
    uint16_t LAD_VoltageOut_HV;             // Output voltage of the charger
    uint16_t LAD_CurrentOut_HV;             // Output current charger
    uint8_t LAD_Status_Voltage; // 
    uint16_t LAD_Temperature;               // Instantaneous value: Charger temperature
    uint16_t LAD_PowerLossVal;              // Instantaneous value: power loss charger
    uint16_t HVLM_HV_StaleTime;             // Period between HV deactivated and HV activated
    uint8_t HVLM_ChargeSystemState;         // Displaying status about the charging system. 0=No issue, 1=System Defective, 2=System Incompatable 3=DC Charge not possible
    uint8_t HVLM_Status_LED;                // Status of the charging LED: 0=Colour1-off, 1=Colour2-White, 2=Colour3-Yellow, 3=Colour4-Green, 4=Colour5-Red, 5=Yellow Pulsing, 6=Green Pulsing, 7=Red Pulsing, 8=Green/Red Pulsing, 9=Green Flashing, 14=Init, 15=Error
    uint8_t HVLM_MaxCurrent_AC;             // Maximum permissible current on the primary side (AC)
    bool HVLM_LG_ChargerTargetMode;         // AC charger target mode: 0=Standby, 1=Mains Charging
    uint8_t HVLM_TankCapReleaseRequest;     // Fuel Cap Release: 0=No Release, 1=Release, 2=Init, 3=Error
    uint8_t HVLM_RequestConnectorLock;      // Request connector lock: 0=Unlock, 1=lock, 2=Init, 3=No Request
    uint8_t HVLM_Start_VoltageMeasure_DCLS; // Message to the BMS that a measurement voltage is being applied to the DC charging station: 0=Inactive, 1=DCLS With Diode, 2=DCLS without Diode, 3=Reserve
    uint8_t HVLM_ChargeReadyStatus;         // Display whether AC or DC charging is not possible: 0=No Error, 1=AC Charge Not Possible, 2=DC Charge Not Possible, 3=AC & DC Charge Not Possible
    uint16_t HVLM_Output_Voltage_HV;        // Output voltage of the charger and DC voltage of the charging station. Measurement between the DC HV lines. 
    bool LAD_Reduction_ChargerTemp;         // Reduction due to internal overtemperature in the charger
    bool LAD_Reduction_Current;             // Regulation due to current or voltage at the input or output
    bool LAD_Reduction_SocketTemp;          // Reduction due to excessive charging socket temperature
    uint16_t LAD_MaxChargerPower_HV;        // Maximum power charger in relation to the maximum power infrastructure (cable, charging station) and taking into account the charger efficiency
    uint8_t LAD_PRX_CableCurrentLimit;      // AC current limit due to PRX cable coding
    bool LAD_ControlPilotStatus;            // Status control pilot monitoring (detection of the control pilot duty cycle)
    bool LAD_LockFeedback;                  // Status of connector lock (feedback contact of the locking actuator)
    uint8_t LAD_ChargerCoolingDemand;       // Cooling demand of the charger
    bool LAD_ChargerWarning;                // Collective warning charger: 0=Normal, 1=Warning
    bool LAD_ChargerFault;                  // Collective error charger: 0=Normal, 1=No Charging Possible
} charger_status;

struct ChargerControl {
    uint16_t HVDCSetpnt;
    uint16_t IDCSetpnt;
    uint16_t HVpwr=0;
    uint16_t HVcur=0;
    uint16_t calcpwr=0;
    uint8_t modeSet;
    bool activate;
} charger_params;

struct BatteryStatus {
    uint16_t SOCx10;         // SOC of battery, With implied decimal place
    uint16_t SOC_Targetx10;   // Target SOC of battery, With implied decimal place
    uint16_t CapkWhx10;      // Usable energy content of the HV battery
    uint16_t BattkWhx10;      // Current energy content of the HV battery
    uint16_t BMSVoltx10;     // BMS Voltage of battery
    uint16_t BMSCurrx10;     // BMS Current of battery, With implied decimal place
    uint16_t BMSMaxVolt;      // BMS Max Voltage of Battery
    uint16_t BMSMinVolt;        // BMS Min Voltage of Battery
    uint16_t BMSMaxChargeCurr;  // BMS Max Charge Curr
    uint16_t BMSBattCellSumx10;
    uint16_t BMSCellAhx10 = 1080;
    uint8_t HV_Status;              // 0=Init (no function), 1=BMS intermediate circuit voltage-free (U_Zwkr < 20V), 2=BMS intermediate circuit not free of voltage (U_Zwkr >/= 25V, hysteresis), 3=Error (e.g. error in the sensors)
    uint8_t BMS_Status;             // 0 "Component_OK" 1 "Limited_CompFct_Isoerror_I" 2 "Limited_CompFct_Isoerror_II" 3 "Limited_CompFct_Interlock" 4 "Limited_CompFct_SD" 5 "Limited_CompFct_Powerred" 6 "No_component_function" 7 "Init" ;
    uint8_t BMS_Mode;               //0 "HV_Off" 1 "Driving HV Active" 2 "Balancing" 3 "External Charger" 4 "AC Charging" 5 "Battery Fault" 6 "DC Charging" 7 "Init" ;
    uint16_t BMS_Battery_Tempx10 = 242;
    uint16_t BMS_Coolant_Tempx10 = 201;
    uint16_t BMS_Cell_H_Tempx10 = 290;
    uint16_t BMS_Cell_L_Tempx10 = 220;
    uint16_t BMS_Cell_H_mV = 3850;
    uint16_t BMS_Cell_L_mV = 3750;
    bool HVIL_Open = false;
    
} battery_status;

uint32_t UnixTime;

//BMS CAN Messages
    uint16_t BMS_Batt_Curr;
    uint16_t BMS_Batt_Volt;
    uint16_t BMS_Batt_Volt_HVterm;
    uint16_t BMS_SOC_HiRes;
    uint16_t BMS_MaxDischarge_Curr;
    uint16_t BMS_Min_Batt_Volt;
    uint16_t BMS_Min_Batt_Volt_Discharge;
    uint16_t BMS_MaxCharge_Curr;
    uint16_t BMS_MaxCharge_Curr_Offset;
    uint16_t BMS_Batt_Max_Volt;
    uint16_t BMS_Min_Batt_Volt_Charge;
    uint16_t BMS_OpenCircuit_Volts;
    bool BMS_Status_ServiceDisconnect;
    uint8_t BMS_HV_Status;
    bool BMS_Faultstatus;
    uint8_t BMS_IstModus;
    uint16_t BMS_Batt_Ah;
    uint16_t BMS_Target_SOC_HiRes;
    uint16_t BMS_Batt_Temp;
    uint16_t BMS_CurrBatt_Temp;
    uint16_t BMS_CoolantTemp_Act;
    uint16_t BMS_Batt_Energy;
    uint16_t BMS_Max_Wh;
    uint16_t BMS_BattEnergy_Wh_HiRes;
    uint16_t BMS_MaxBattEnergy_Wh_HiRes;
    uint16_t BMS_SOC;
    uint16_t BMS_ResidualEnergy_Wh;
    uint16_t BMS_SOC_ChargeLim;
    uint16_t BMS_EnergyCount;
    uint16_t BMS_EnergyReq_Full;
    uint16_t BMS_ChargePowerMax;
    uint16_t BMS_ChargeEnergyCount;
    uint16_t BMS_BattCell_Temp_Max;
    uint16_t BMS_BattCell_Temp_Min;
    uint16_t BMS_BattCell_MV_Max;
    uint16_t BMS_BattCell_MV_Min;
    bool HVEM_Nachladen_Anf;
    uint16_t HVEM_SollStrom_HV;
    uint16_t HVEM_MaxSpannung_HV;
    uint8_t HMS_Systemstatus;
    uint8_t HMS_aktives_System;
    bool HMS_Fehlerstatus;
    uint8_t HVK_HVLM_Sollmodus; // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
    bool HV_Bordnetz_aktiv; // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
    uint8_t HVK_MO_EmSollzustand; // 0 "HvOff" 1 "HvStbyReq" 2 "HvStbyWait" 3 "HvBattOnReq" 4 "HvBattOnWait" 10 "HvOnIdle" 20 "HvOnDrvRdy" 46 "HvAcChPreReq" 47 "HvAcChPreWait" 48 "HvAcChReq" 49 "HvAcChWait" 50 "HvAcCh" 56 "HvDcChPreReq" 57 "HvDcChPreWait" 58 "HvDcChReq" 59 "HvDcChWait" 60 "HvDcCh" 67 "HvChOffReq" 68 "HvChOffWait" 69 "HvOnIdleReq" 70 "HvOnIdleWait" 96 "HvCpntOffReq" 97 "HvCpntOffWait" 98 "HvBattOffReq" 99 "HvBattOffWait" 119 "HvElmOffReq" 120 "HvElmOff"
    uint8_t HVK_BMS_Sollmodus; // 0 "HV_Off" 1 "HV_On" 3 "AC_Charging_ext" 4 "AC_Charging" 6 "DC_Charging" 7 "Init" ;
    uint8_t HVK_DCDC_Sollmodus; //0 "Standby" 1 "HV_On_Precharging" 2 "Step down" 3 "Step up" 4 "Test pulse_12V" 7 "Initialization" ;
    bool ZV_FT_verriegeln;
    bool ZV_FT_entriegeln;
    bool ZV_BT_verriegeln;
    bool ZV_BT_entriegeln;
    bool ZV_entriegeln_Anf;
    bool ZV_verriegelt_intern_ist;
    bool ZV_verriegelt_extern_ist;
    bool ZV_verriegelt_intern_soll;
    bool ZV_verriegelt_extern_soll;
    uint8_t ZV_verriegelt_soll;
    bool BMS_Charger_Active;
    uint16_t BMS_RIso_Ext = 4090;
    uint8_t HVK_Gesamtst_Spgfreiheit;
    uint8_t BMS_Balancing_Active=2;
    uint8_t BMS_Freig_max_Perf =1;
    uint8_t BMS_Battdiag =1; // Battery Display Diagnostics: 1 = Display Battery, 4 = Display Battery OK, 5 = Charging, 6 = Check Battery
    uint8_t DC_IstModus_02 = 2;
    uint8_t BMS_HV_Auszeit_Status = 1; // Status HV timeout.
    uint16_t BMS_HV_Auszeit = 25; // Time since last HV Activity
    uint16_t BMS_Kapazitaet = 1000; //  Total Energy Capacity (aged)
    uint16_t BMS_SOC_Kaltstart = 0; // SOC Cold
    uint8_t BMS_max_Grenz_SOC = 30; // Upper limit of SOC operating strategy (70 offset, so 30 = 100)
    uint8_t BMS_min_Grenz_SOC = 15; // Lower limit of SOC Operating strategy
    uint8_t EM1_Istmodus2; // EM1 Status, 0=standby
    uint8_t EM1_Status_Spgfreiheit; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    bool ZAS_Kl_S; // KeySwitch Inserted
    bool ZAS_Kl_15; // Acc position
    bool ZAS_Kl_X; // Run position
    bool ZAS_Kl_50_Startanforderung; // Start
    uint8_t HVActiveDelayOff;
    uint8_t HVEM_NVNachladen_Energie = 200;
    uint8_t LockState;

void VWMLBClass::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x488);    // HVLM_06
    can->RegisterUserMessage(0x53C);    // HVLM_04
    can->RegisterUserMessage(0x564);    // LAD_01
    can->RegisterUserMessage(0x565);    // HVLM_03
    can->RegisterUserMessage(0x67E);    // LAD_02
}

bool VWMLBClass::ControlCharge(bool RunCh, bool ACReq)
{
    if (charger_status.HVLM_Plug_Status > 1 && RunCh)
   {
        charger_params.activate = 1;
        return true;
   }
    else
    {
        charger_params.activate = 0;
        return false;
    }
}

void VWMLBClass::Simulate()
{
    if(Param::GetInt(Param::opmode) == MOD_PRECHARGE || Param::GetInt(Param::opmode) == MOD_RUN || Param::GetInt(Param::opmode) == MOD_CHARGE)
    {
        Param::SetFloat(Param::udc, 400.0);
    }
    else
    {
        Param::SetFloat(Param::udc, 0.0);
    }
    
    if (charger_status.HVVoltage > 320)
      {
      Param::SetFloat(Param::udc, charger_status.HVVoltage);
      }
}

void VWMLBClass::Task1Ms()
{ static uint8_t counter1ms = 0;
    counter1ms++;  // Increments every 10ms
    if(vehicle_status.CANQuiet == 1){  counter1ms = 0; }      // Quick way of silencing the canbus -- NOT IMPLEMENTED
// - Do 10ms Tasks:
    if (counter1ms % 10 == 0) { // Every 100ms (10ms * 1)
    msg191();   // BMS_01   0x191     CRC
    }
// - Do 20ms, 40ms, 50ms tasks:
    if (counter1ms % 20 == 0) { // Every 20ms (10ms * 2)
    }
    if (counter1ms % 40 == 0) { // Every 40ms (10ms * 4)
    //msg040(); // Airbag_01 - 0x40     CRC
    can->Send(0x2B1, MSG_TME_02, 8);    // MSG_TME_02   0x2B1
    }
    if (counter1ms % 50 == 0) { // Every 50ms (10ms * 5)
    msg2AE(); // DCDC_01    0x2AE     CRC
    counter1ms = 1;
    }
}

void VWMLBClass::Task10Ms()
{
    
}

void VWMLBClass::Task100Ms()
{
Simulate();
static uint8_t counter100ms = 0;
TagParams();
CalcValues100ms();
counter100ms++;  // Increments every 100ms
if(vehicle_status.CANQuiet == 1){ counter100ms = 0; }                      // Quick way of silencing the canbus -- NOT IMPLEMENTED
// - Do 100ms Tasks:
if (counter100ms % 1 == 0) { // Every 100ms (100ms * 2)
    msg3C0(); //  Klemmen_Status_01   CRC
    msg503(); // HVK_01     0x503     CRC
    msg184(); // ZV_01    0x184       CRC
    msg578(); // BMS_DC_01    0x578   CRC
    can->Send(0x1A1, BMS_02, 8);        // BMS_02   0x1A1
    can->Send(0x39D, BMS_03, 8);        // BMS_03   0x39D
    can->Send(0x509, BMS_10, 8);        // BMS_10   0x509
    can->Send(0x552, HVEM_05, 8);       // HVEM_05  0x552 Fun items relating to Max Voltage and Plug Release
    can->Send(0x5AC, HVEM_02, 8);       // HVEM_02  0x5AC
}
// - Do 200ms, 500ms, 1000ms and 2000ms tasks:
if (counter100ms % 2 == 0) { // Every 200ms (100ms * 2)
    msg1A2(); // ESP_15   0x1A2       CRC
    can->Send(0x583, ZV_02, 8);         // ZV_02 - 0x583
}
if (counter100ms % 5 == 0) { // Every 500ms (100ms * 5)
    msg5A2(); // BMS_04   0x5A2       CRC
    msg5CA(); // BMS_07   0x5CA       CRC
    msg5CD(); // DCDC_03    0x5CD     CRC
    can->Send(0x59E, BMS_06, 8);        // BMS_06   0x59E
}
if (counter100ms % 10 == 0) { // Every 1000ms (100ms * 10)
    can->Send(0x485, Authentic_Time_01, 8);     // NavData_02 - 0x485
    can->Send(0x1A555548, ORU_01, 8);   // ORU_01   0x1A555548
    can->Send(0x1A5555AD, Authentic_Time_01, 8);    // Authentic_Time_01   0x1A5555AD
    can->Send(0x96A955EB, BMS_09, 8);   // BMS_09   0x96A955EB
    can->Send(0x9A555539, BMS_16, 8);   // BMS_16   0x9A555539     Just contains IDs for cells with low/high charge or temps
    can->Send(0x9A555552, BMS_27, 8);   // BMS_27   0x9A555552
}
if (counter100ms % 20 == 0) { // Every 2000ms (100ms * 20)
    can->Send(0x96A954A6, BMS_11, 8);   // BMS_11   0x96A954A6
    counter100ms = 1;
}
}

void VWMLBClass::Task200Ms()
{
}

// Msgs with CRC & Counters:

void VWMLBClass::msg040() // Airbag_01 - 0x40
{
    // CanMsgBuf[0] = 0x00 ;
    // CanMsgBuf[1] = (0x00 | vag_cnt040) ;
    // CanMsgBuf[2] = 0x00 ;
    // CanMsgBuf[3] = 0x00 ;
    // CanMsgBuf[4] = 0x00 ;
    // CanMsgBuf[5] = 0x00 ;
    // CanMsgBuf[6] = 0x00 ;
    // CanMsgBuf[7] = 0x00 ;
    // CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x40);
    // can->Send(0x040, CanMsgBuf, 8);
    // vag_cnt040++;
    // if(vag_cnt040>0x0f) vag_cnt040=0x00;
}

void VWMLBClass::msg184() // ZV_01   0x184
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (ZV_01[1]|vag_cnt184) ;
    CanMsgBuf[2] = ZV_01[2] ;
    CanMsgBuf[3] = ZV_01[3] ;
    CanMsgBuf[4] = ZV_01[4] ;
    CanMsgBuf[5] = ZV_01[5] ;
    CanMsgBuf[6] = ZV_01[6] ;
    CanMsgBuf[7] = ZV_01[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x184);
    can->Send(0x184, CanMsgBuf, 8); 
    vag_cnt184++;
        if(vag_cnt184>0x0f) vag_cnt184=0x00;
}

void VWMLBClass::msg191() // BMS_01   0x191
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (BMS_01[1]|vag_cnt191) ;
    CanMsgBuf[2] = BMS_01[2] ;
    CanMsgBuf[3] = BMS_01[3] ;
    CanMsgBuf[4] = BMS_01[4] ;
    CanMsgBuf[5] = BMS_01[5] ;
    CanMsgBuf[6] = BMS_01[6] ;
    CanMsgBuf[7] = BMS_01[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x191);
    can->Send(0x191, CanMsgBuf, 8); 
    vag_cnt191++;
    if(vag_cnt191>0x0f) vag_cnt191=0x00;
}

void VWMLBClass::msg1A2() // ESP_15   0x1A2
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (ESP_15[1]|vag_cnt1A2) ;
    CanMsgBuf[2] = ESP_15[2] ;
    CanMsgBuf[3] = ESP_15[3] ;
    CanMsgBuf[4] = ESP_15[4] ;
    CanMsgBuf[5] = ESP_15[5] ;
    CanMsgBuf[6] = ESP_15[6] ;
    CanMsgBuf[7] = ESP_15[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x1A2);
    can->Send(0x1A2, CanMsgBuf, 8); 
    vag_cnt1A2++;
    if(vag_cnt1A2>0x0f) vag_cnt1A2=0x00;
}

void VWMLBClass::msg2AE() // DCDC_01    0x2AE
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (DCDC_01[1]|vag_cnt2AE) ;
    CanMsgBuf[2] = DCDC_01[2] ;
    CanMsgBuf[3] = DCDC_01[3] ;
    CanMsgBuf[4] = DCDC_01[4] ;
    CanMsgBuf[5] = DCDC_01[5] ;
    CanMsgBuf[6] = DCDC_01[6] ;
    CanMsgBuf[7] = 0xA8 ; // Sets static 13.8V as DC output voltage
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x2AE);
    can->Send(0x2AE, CanMsgBuf, 8);
    vag_cnt2AE++;
    if(vag_cnt2AE>0x0f) vag_cnt2AE=0x00;
}

void VWMLBClass::msg503() // HVK_01     0x503
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (HVK_01[1]|vag_cnt503) ;
    CanMsgBuf[2] = HVK_01[2] ;
    CanMsgBuf[3] = HVK_01[3] ;
    CanMsgBuf[4] = HVK_01[4] ;
    CanMsgBuf[5] = HVK_01[5] ;
    CanMsgBuf[6] = HVK_01[6] ;
    CanMsgBuf[7] = HVK_01[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x503);
    can->Send(0x503, CanMsgBuf, 8); 
    vag_cnt503++;
    if(vag_cnt503>0x0f) vag_cnt503=0x00;
}

void VWMLBClass::msg578() // BMS_DC_01    0x578 
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (BMS_DC_01[1]|vag_cnt578) ;
    CanMsgBuf[2] = BMS_DC_01[2] ;
    CanMsgBuf[3] = BMS_DC_01[3] ;
    CanMsgBuf[4] = BMS_DC_01[4] ;
    CanMsgBuf[5] = BMS_DC_01[5] ;
    CanMsgBuf[6] = BMS_DC_01[6] ;
    CanMsgBuf[7] = BMS_DC_01[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x578);
    can->Send(0x578, CanMsgBuf, 8); 
    vag_cnt578++;
    if(vag_cnt578>0x0f) vag_cnt578=0x00;
}
void VWMLBClass::msg5A2() // BMS_04   0x5A2
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (BMS_04[1]|vag_cnt5A2) ;
    CanMsgBuf[2] = BMS_04[2] ;
    CanMsgBuf[3] = BMS_04[3] ;
    CanMsgBuf[4] = BMS_04[4] ;
    CanMsgBuf[5] = BMS_04[5] ;
    CanMsgBuf[6] = BMS_04[6] ;
    CanMsgBuf[7] = BMS_04[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x5A2);
    can->Send(0x5A2, CanMsgBuf, 8); 
    vag_cnt5A2++;
    if(vag_cnt5A2>0x0f) vag_cnt5A2=0x00;
}
void VWMLBClass::msg5CA() // BMS_07   0x5CA
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (BMS_07[1]|vag_cnt5CA) ;
    CanMsgBuf[2] = BMS_07[2] ;
    CanMsgBuf[3] = BMS_07[3] ;
    CanMsgBuf[4] = BMS_07[4] ;
    CanMsgBuf[5] = BMS_07[5] ;
    CanMsgBuf[6] = BMS_07[6] ;
    CanMsgBuf[7] = BMS_07[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x5CA);
    can->Send(0x5CA, CanMsgBuf, 8); 
    vag_cnt5CA++;
    if(vag_cnt5CA>0x0f) vag_cnt5CA=0x00;
}
void VWMLBClass::msg5CD() // DCDC_03    0x5CD
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (DCDC_03[1]|vag_cnt5CD) ;
    CanMsgBuf[2] = DCDC_03[2] ;
    CanMsgBuf[3] = DCDC_03[3] ;
    CanMsgBuf[4] = DCDC_03[4] ;
    CanMsgBuf[5] = DCDC_03[5] ;
    CanMsgBuf[6] = DCDC_03[6] ;
    CanMsgBuf[7] = DCDC_03[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x5CD);
    can->Send(0x5CD, CanMsgBuf, 8); 
    vag_cnt5CD++;
    if(vag_cnt5CD>0x0f) vag_cnt5CD=0x00;
}

void VWMLBClass::msg3C0() // Klemmen_Status_01
{
    CanMsgBuf[0] = 0x00 ;
    CanMsgBuf[1] = (Klemmen_Status_01[1] |vag_cnt3C0) ;
    CanMsgBuf[2] = Klemmen_Status_01[2] ;
    CanMsgBuf[3] = Klemmen_Status_01[3] ;
    CanMsgBuf[4] = Klemmen_Status_01[4] ;
    CanMsgBuf[5] = Klemmen_Status_01[5] ;
    CanMsgBuf[6] = Klemmen_Status_01[6] ;
    CanMsgBuf[7] = Klemmen_Status_01[7] ;
    CanMsgBuf[0] = vag_utils::vw_crc_calc(CanMsgBuf, 8, 0x3C0);
    can->Send(0x3C0, CanMsgBuf, 8); 
    vag_cnt3C0++;
    if(vag_cnt3C0>0x0f) vag_cnt3C0=0x00;
}

void VWMLBClass::DecodeCAN(int id, uint32_t data[2])
{
uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:) ... Stolen from Zombie, Left comments as they're now famous.
switch (id)
{
    case 0x488: // HVLM_06
        charger_status.HVLM_MaxDC_ChargePower = (((bytes[2] & (0x3FU)) << 4) | ((bytes[1] >> 4) & (0x0FU)))*250;
        charger_status.HVLM_Max_DC_Voltage_DCLS = ((bytes[3] & (0xFFU)) << 2) | ((bytes[2] >> 6) & (0x03U));
        charger_status.HVLM_Actual_DC_Current_DCLS = ((bytes[5] & (0x01U)) << 8) | (bytes[4] & (0xFFU));
        charger_status.HVLM_Max_DC_Current_DCLS = ((bytes[6] & (0x03U)) << 7) | ((bytes[5] >> 1) & (0x7FU));
        charger_status.HVLM_Min_DC_Voltage_DCLS = ((bytes[7] & (0x07U)) << 6) | ((bytes[6] >> 2) & (0x3FU));
        charger_status.HVLM_Min_DC_Current_DCLS = ((bytes[7] >> 3) & (0x1FU));
    break;
    

    case 0x53C: // HVLM_04
        // HVLM_ParkingHeater_Mode = ((HVLM_04[1] >> 4) & (0x07U));
        // HVLM_StationaryClimat_Timer_Stat = ((HVLM_04[1] >> 7) & (0x01U));
        // HVLM_HVEM_MaxPower = ((HVLM_04[3] & (0x01U)) << 8) | (HVLM_04[2] & (0xFFU));
        charger_status.HVLM_Status_Grid = ((bytes[3] >> 1) & (0x01U));
        // HVLM_BEV_LoadingScreen = ((HVLM_04[3] >> 2) & (0x01U));
        charger_status.HVLM_EnergyFlowType = ((bytes[3] >> 3) & (0x03U));
        // HVLM_VK_ParkingHeaterStatus = ((HVLM_04[3] >> 5) & (0x07U));
        // HVLM_VK_ClimateConditioningStat = (HVLM_04[4] & (0x03U));
        charger_status.HVLM_OperationalMode = ((bytes[4] >> 2) & (0x03U));
        charger_status.HVLM_HV_ActivationRequest = ((bytes[4] >> 4) & (0x03U));
        charger_status.HVLM_ChargerErrorStatus = ((bytes[5] & (0x01U)) << 2) | ((bytes[4] >> 6) & (0x03U));
        charger_status.HVLM_Park_Request = ((bytes[5] >> 1) & (0x07U));
        charger_status.HVLM_Park_Request_Maintain = ((bytes[5] >> 4) & (0x03U));
        // HVLM_AWC_Mode = (HVLM_04[6] & (0x07U));
        charger_status.HVLM_Plug_Status = ((bytes[6] >> 3) & (0x03U));
        charger_status.HVLM_LoadRequest = ((bytes[6] >> 5) & (0x07U));
        charger_status.HVLM_MaxBattChargeCurrent = (bytes[7] & (0xFFU));
    break;

    case 0X564: // LAD_01
        charger_status.mode = ((bytes[1] >> 4) & (0x07U));
        charger_status.ACvoltage = ((bytes[2] & (0xFFU)) << 1) | ((bytes[1] >> 7) & (0x01U));
        charger_status.HVVoltage = (((bytes[4] & (0x03U)) << 8) | (bytes[3] & (0xFFU)));
        charger_status.current = ((((bytes[5] & (0x0FU)) << 6) | ((bytes[4] >> 2) & (0x3FU))) * 0.2) - 102;
        charger_status.LAD_Status_Voltage = ((bytes[5] >> 4) & (0x03U));
        charger_status.temperature = bytes[6] - 40;
        charger_status.LAD_PowerLossVal = ((bytes[7] & (0xFFU)))*20;
    break;

    case 0x565: // HVLM_03
        charger_status.HVLM_HV_StaleTime = ((bytes[0] & (0xFFU)))*4;
        charger_status.HVLM_ChargeSystemState = (bytes[1] & (0x03U));
        // HVLM_KESSY_KeySearch = ((HVLM_03[1] >> 2) & (0x03U));
        charger_status.HVLM_Status_LED = ((bytes[1] >> 4) & (0x0FU));
        charger_status.MaxACAmps = ((bytes[3] & (0x7FU)))/2;
        charger_status.HVLM_LG_ChargerTargetMode = ((bytes[3] >> 7) & (0x01U));
        charger_status.HVLM_TankCapReleaseRequest = (bytes[4] & (0x03U));
        charger_status.HVLM_RequestConnectorLock = ((bytes[4] >> 2) & (0x03U));
        charger_status.HVLM_Start_VoltageMeasure_DCLS = ((bytes[4] >> 4) & (0x03U));
        // PnC_Trigger_OBC_cGW = ((HVLM_03[5] & (0x03U)) << 2) | ((HVLM_03[4] >> 6) & (0x03U));
        // HVLM_ReleaseAirConditioning = ((HVLM_03[5] >> 2) & (0x03U));
        charger_status.HVLM_ChargeReadyStatus = ((bytes[6] >> 1) & (0x07U));
        // HVLM_IsolationRequest = ((HVLM_03[6] >> 5) & (0x01U));
        charger_status.HVLM_Output_Voltage_HV = ((bytes[7] & (0xFFU)) << 2) | ((bytes[6] >> 6) & (0x03U));
    break;

    case 0x67E: // LAD_02
        charger_status.LAD_Reduction_ChargerTemp = ((bytes[1] >> 4) & (0x01U));
        charger_status.LAD_Reduction_Current = ((bytes[1] >> 5) & (0x01U));
        charger_status.LAD_Reduction_SocketTemp = ((bytes[1] >> 6) & (0x01U));
        charger_status.LAD_MaxChargerPower_HV = (((bytes[3] & (0x01U)) << 8) | (bytes[2] & (0xFFU)))*100;
        charger_status.PPLim = (bytes[4] & (0x07U));
        charger_status.LAD_ControlPilotStatus = ((bytes[4] >> 3) & (0x01U));
        charger_status.LAD_LockFeedback = ((bytes[4] >> 4) & (0x01U));
        charger_status.LAD_ChargerCoolingDemand = ((bytes[4] >> 6) & (0x03U));
        // LAD_MaxLadLeistung_HV_Offset = ((LAD_02[7] >> 1) & (0x03U));
        charger_status.LAD_ChargerWarning = ((bytes[7] >> 6) & (0x01U));
        charger_status.LAD_ChargerFault = ((bytes[7] >> 7) & (0x01U));
    break;      
}
}


void VWMLBClass::TagParams()  // To make code portable between standalone (more params) vs Zombie (basic params) - This code executed 100ms, uncomment or delete un-needed params
{
    //Common Params (Shared same param names as zombie vs standalone)
    
    battery_status.SOCx10 = int(Param::GetFloat(Param::SOC)*10);
    battery_status.SOC_Targetx10 = 1000;     //Param::GetInt(Param::VCU_SOC_Limit);
    battery_status.CapkWhx10 = int(Param::GetFloat(Param::BattCap)*10);
    battery_status.BMSMinVolt = Param::GetInt(Param::udcmin);
    charger_params.HVpwr=Param::GetInt(Param::Pwrspnt);
    charger_params.HVcur = Param::GetInt(Param::BMS_ChargeLim);  
    charger_params.HVDCSetpnt = Param::GetInt(Param::Voltspnt);
    battery_status.BMSVoltx10 = Param::GetInt(Param::udc);
    battery_status.BMSBattCellSumx10 = Param::GetInt(Param::udc2);
    battery_status.BMSMaxVolt = Param::GetInt(Param::udclim);
    battery_status.BMS_Cell_H_Tempx10 = int(Param::GetFloat(Param::BMS_Tmax)*10);
    battery_status.BMS_Cell_L_Tempx10 = int(Param::GetFloat(Param::BMS_Tmin)*10);
    battery_status.BMS_Cell_H_mV = int(Param::GetFloat(Param::BMS_Vmax)*100);
    battery_status.BMS_Cell_L_mV = int(Param::GetFloat(Param::BMS_Vmin)*100);
    vehicle_status.locked = Param::GetInt(Param::VehLockSt);

    Param::SetInt(Param::CableLim,charger_status.MaxACAmps);
    Param::SetInt(Param::AC_Volts,charger_status.ACvoltage);
    Param::SetInt(Param::ChgTemp,charger_status.temperature);
    switch (charger_status.HVLM_Plug_Status)
    {
        case 0:
            Param::SetInt(Param::PlugDet,0);
        break; 
        case 1:
            Param::SetInt(Param::PlugDet,0);
        break; 
        case 2:
            Param::SetInt(Param::PlugDet,1);
        break; 
        case 3:
            Param::SetInt(Param::PlugDet,1);
        break; 
    }

    switch (charger_status.PPLim)
    {
        case 0:
            Param::SetInt(Param::PilotLim,13);
        break; 
        case 1:
            Param::SetInt(Param::PilotLim,20);
        break; 
        case 2:
            Param::SetInt(Param::PilotLim,32);
        break; 
        case 3:
            Param::SetInt(Param::PilotLim,63);
        break; 
    }
    // Standalone additional params:

    // Param::SetInt(Param::DC_Max_ChargePower,charger_status.HVLM_MaxDC_ChargePower);
    // Param::SetInt(Param::DC_Max_ChargeVoltage,charger_status.HVLM_Max_DC_Voltage_DCLS);
    // Param::SetInt(Param::DC_Actual_Current,charger_status.HVLM_Actual_DC_Current_DCLS);
    // Param::SetInt(Param::DC_Max_ChargeCurrent,charger_status.HVLM_Max_DC_Current_DCLS);
    // Param::SetInt(Param::DC_Min_ChargeVoltage,charger_status.HVLM_Min_DC_Voltage_DCLS);
    // Param::SetInt(Param::DC_Min_ChargeCurrent,charger_status.HVLM_Min_DC_Current_DCLS);
    // Param::SetInt(Param::Status_Grid,charger_status.HVLM_EnergyFlowType);
    // Param::SetInt(Param::ChargeManagerMode,charger_status.HVLM_OperationalMode);
    // Param::SetInt(Param::ChargerRequestingHV,charger_status.HVLM_HV_ActivationRequest);
    // Param::SetInt(Param::ChargerErrorStatus,charger_status.HVLM_ChargerErrorStatus);
    // Param::SetInt(Param::PlugStatus,charger_status.HVLM_Plug_Status);
    // Param::SetInt(Param::LoadRequest,charger_status.HVLM_LoadRequest);
    // Param::SetInt(Param::ChargerState,charger_status.mode);
    // Param::SetInt(Param::Charger_AC_Volt_RMS,charger_status.ACvoltage);
    // Param::SetInt(Param::Charger_VoltageOut_HV,charger_status.HVVoltage);
    // Param::SetInt(Param::Charger_CurrentOut_HV,charger_status.current);
    // Param::SetInt(Param::Charger_Temperature,charger_status.temperature);
    // Param::SetInt(Param::ChargerSystemState,charger_status.HVLM_ChargeSystemState);
    // Param::SetInt(Param::Status_LED,charger_status.HVLM_Status_LED);
    // Param::SetInt(Param::MaxCurrent_AC,charger_status.MaxACAmps);
    // Param::SetInt(Param::LockRequest,charger_status.HVLM_RequestConnectorLock);
    // Param::SetInt(Param::Charger_Ready,charger_status.HVLM_ChargeReadyStatus);
    // Param::SetInt(Param::ChargerTemp_Reduction,charger_status.LAD_Reduction_ChargerTemp);
    // Param::SetInt(Param::ChargerCurrent_Reduction,charger_status.LAD_Reduction_Current);
    // Param::SetInt(Param::SocketTemp_Reduction,charger_status.LAD_Reduction_SocketTemp);
    // Param::SetInt(Param::MaxChargerOutput,charger_status.LAD_MaxChargerPower_HV);
    // Param::SetInt(Param::CableCurrentLimit,charger_status.PPLim);
    // Param::SetInt(Param::ControlPilotStatus,charger_status.LAD_ControlPilotStatus);
    // Param::SetInt(Param::LockState,charger_status.LAD_LockFeedback);
    // Param::SetInt(Param::ChargerWarning,charger_status.LAD_ChargerWarning);
    // Param::SetInt(Param::ChargerFault,charger_status.LAD_ChargerFault);
    // Param::SetInt(Param::OutputVolts,charger_status.HVLM_Output_Voltage_HV);

    // Param::SetFloat(Param::VCU_SOC, (Param::GetInt(Param::SOCx10)));
    // Param::SetFloat(Param::VCU_SOC_Limit, Param::GetInt(Param::SOC_Targetx10));
    // Param::SetInt(Param::VCU_UDCmin, Param::GetInt(Param::BMSMinVolt));
    // Param::SetInt(Param::VCU_Current_SP, Param::GetInt(Param::IDCSetpnt));
    // Param::SetInt(Param::VCU_Volt_SP, Param::GetInt(Param::HVDCSetpnt));
    // Param::SetFloat(Param::BMS_Pack_Voltage, (Param::GetInt(Param::BMSBattCellSumx10)/10));
    // Param::SetInt(Param::VCU_UDCmax, Param::GetInt(Param::BMSMaxVolt));
    // Param::SetFloat(Param::BMS_Highest_Cell_Temp, (Param::GetInt(Param::BMS_Cell_H_Tempx10))/10);
    // Param::SetFloat(Param::BMS_Lowest_Cell_Temp, (Param::GetInt(Param::BMS_Cell_L_Tempx10))/10);
    // Param::SetInt(Param::BMS_Highest_Cell_Volt, Param::GetInt(Param::BMS_Cell_H_mV));
    // Param::SetInt(Param::BMS_Lowest_Cell_Volt, Param::GetInt(Param::BMS_Cell_L_mV));
    // Param::SetInt(Param::VCUChargeRequest, Param::Ge|tInt(Param::Activation_Crg));
    // Param::SetInt(Param::VehicleLockState, Param::GetInt(Param::LockSim));

    // battery_status.SOCx10 = (Param::GetInt(Param::VCU_SOC));
    // battery_status.SOC_Targetx10 = Param::GetInt(Param::VCU_SOC_Limit);
    // battery_status.BMSMinVolt = Param::GetInt(Param::VCU_UDCmin);
    // charger_params.IDCSetpnt = Param::GetInt(Param::VCU_Current_SP);
    // charger_params.HVDCSetpnt = Param::GetInt(Param::VCU_Volt_SP);
    // battery_status.BMSBattCellSumx10 = Param::GetInt(Param::BMS_Pack_Voltage);
    // battery_status.BMSMaxVolt = Param::GetInt(Param::VCU_UDCmax);
    // battery_status.BMS_Cell_H_Tempx10 = Param::GetInt(Param::BMS_Highest_Cell_Temp)*10;
    // battery_status.BMS_Cell_L_Tempx10 = Param::GetInt(Param::BMS_Lowest_Cell_Temp)*10;
    // battery_status.BMS_Cell_H_mV = Param::GetInt(Param::BMS_Highest_Cell_Volt);
    // battery_status.BMS_Cell_L_mV = Param::GetInt(Param::BMS_Lowest_Cell_Volt);
    // ZV_verriegelt_extern_ist = Param::GetInt(Param::VehicleLockState);
    
}

void VWMLBClass::CalcValues100ms() // Run to calculate values every 100 ms
{
// Static Set Values - TODO: Roll these up into the CAN msg and eliminate useless variables    
    BMS_MaxDischarge_Curr = 1500;
    BMS_MaxCharge_Curr_Offset = 0;
    BMS_Status_ServiceDisconnect = 0;
    BMS_Battdiag = 0;
    BMS_BattEnergy_Wh_HiRes = 0;
    BMS_MaxBattEnergy_Wh_HiRes = 0;
    BMS_ResidualEnergy_Wh = 0;
    BMS_ChargePowerMax = 625;
    BMS_ChargeEnergyCount = 0;
    BMS_EnergyCount = 0;

//BMS charge current limit but needs to be power for most AC charger types.
    if(charger_params.HVcur > 1000)
    {
    charger_params.calcpwr = 12000;
    }
    else
    {
    charger_params.calcpwr = charger_params.HVcur*(battery_status.BMSVoltx10/10);
    }

    charger_params.HVpwr=MIN(charger_params.HVpwr,charger_params.calcpwr);

    charger_params.IDCSetpnt = charger_params.HVpwr / (battery_status.BMSVoltx10/10);
  
// BMS SOC:
    BMS_Batt_Curr = charger_status.current + 2047;
    BMS_SOC = battery_status.SOCx10 * .05;
    BMS_SOC_HiRes = battery_status.SOCx10 * 2;
    BMS_SOC_Kaltstart = battery_status.SOCx10 * 2;
    BMS_Batt_Energy = battery_status.CapkWhx10 * 2;


// BMS Current:

// BMS Voltages:
    BMS_Batt_Volt = battery_status.BMSVoltx10 * 4;
    BMS_Batt_Volt_HVterm = battery_status.BMSVoltx10 * 2;
    BMS_BattCell_MV_Max = battery_status.BMS_Cell_H_mV - 1000;
    BMS_BattCell_MV_Min = battery_status.BMS_Cell_L_mV - 1000;

// BMS Temps:
    BMS_Batt_Temp = (battery_status.BMS_Battery_Tempx10 + 400) / 5;
    BMS_CurrBatt_Temp = (battery_status.BMS_Battery_Tempx10 + 400) / 5;
    BMS_CoolantTemp_Act = (battery_status.BMS_Coolant_Tempx10 + 400) / 5;
    BMS_BattCell_Temp_Max = (battery_status.BMS_Cell_H_Tempx10 + 400) / 5;
    BMS_BattCell_Temp_Min = (battery_status.BMS_Cell_L_Tempx10 + 400) / 5;

// BMS SOC Limits:
    BMS_Max_Wh = battery_status.CapkWhx10*2;
    BMS_SOC_ChargeLim = battery_status.SOC_Targetx10/10;
    BMS_max_Grenz_SOC = (battery_status.SOC_Targetx10 - 700)/10;
    BMS_EnergyReq_Full =((battery_status.SOC_Targetx10 - battery_status.SOCx10)*battery_status.CapkWhx10)/2500;

// BMS Limits Discharge:
    BMS_Min_Batt_Volt = battery_status.BMSMinVolt;
    BMS_Min_Batt_Volt_Discharge = battery_status.BMSMinVolt;

// BMS Limits Charge:
    BMS_MaxCharge_Curr = 1500;
    HVEM_SollStrom_HV = (charger_params.IDCSetpnt+205)*5;
    BMS_Batt_Max_Volt = charger_params.HVDCSetpnt;
    BMS_Min_Batt_Volt_Charge = battery_status.BMSMinVolt;;
    BMS_OpenCircuit_Volts = battery_status.BMSBattCellSumx10;
    HVEM_MaxSpannung_HV = battery_status.BMSMaxVolt;
    BMS_Faultstatus = battery_status.BMS_Status;
    BMS_Batt_Ah = (battery_status.CapkWhx10*100)/350;
    BMS_Target_SOC_HiRes = battery_status.SOC_Targetx10*2;

  // ESP15
    HMS_Systemstatus = 3 ; //0 "No_function_active" 1 "Hold_active" 2 "Parking_requested" 3 "Parking_active" 4 "Keep parking_active" 5 "Start_active" 6 "Release_request_active" 7 "Release_request_by_driver" 8 "Slipping_detected" 9 "Hold_standby_active" 10 "Start_standby_active" 14 "Init" 15 "Error " ; 
    HMS_aktives_System = 6; //0 "No_System__Init_Error" 1 "Driver request_active" 2 "HMS_internal_active" 3 "ACC_active" 4 "Autohold_active" 5 "HHC_active" 6 "HVLM_active" 7 "Getriebe_aktiv" 8 "EBKV_aktiv" 9 "ParkAssist_aktiv" 10 "ARA_aktiv" 12 "Autonomous_Hold_aktiv" 13 "STA_aktiv " 14 "Motor_aktiv" 15 "EA_aktiv" 16 "VLK_aktiv" ;
    
  // Lock Status:
  if (vehicle_status.locked == 0)
  {
    ZV_verriegelt_soll = 1;
  }
  if (vehicle_status.locked  == 1)
  {
    ZV_verriegelt_soll = 2;
  }


  // Charger Activation State Logic:
  if( charger_status.HVLM_HV_ActivationRequest == 1)
  {
    HV_Bordnetz_aktiv = true;       // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
    //HVK_BMS_Sollmodus = 4;
    BMS_IstModus = 4;               // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
    BMS_HV_Status = 2;              // HV System Voltage Detected  // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_MO_EmSollzustand = 50;
    BMS_Charger_Active = 1;
    HVActiveDelayOff = 20;
  }
    
  if( charger_status.HVLM_HV_ActivationRequest == 0)
  {
    BMS_Charger_Active = 0;
    if (HVActiveDelayOff >= 1)
    {
        BMS_HV_Status = 2;            // HV No Voltage // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
        BMS_IstModus = 1;             // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
        HVK_BMS_Sollmodus = 1;
        HVK_MO_EmSollzustand = 67;
        HVActiveDelayOff = HVActiveDelayOff - 1;
    }
        
    if (HVActiveDelayOff == 0)
    {
    //   HV_Bordnetz_aktiv = false; // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
    //   BMS_HV_Status = 1; // HV No Voltage // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    //   BMS_IstModus = 0; // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
    //   HVK_BMS_Sollmodus = 0;
    //   HVK_MO_EmSollzustand = 0;
    //   BMS_Batt_Volt = charger_status.HVVoltage*4; // Modify after testing to actual values from BMS/VCU
    //   BMS_Batt_Volt_HVterm = charger_status.HVVoltage*2; // Modify after testing to actual values from BMS/VCU

        BMS_HV_Status = 2;            // Voltage Applied // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
        BMS_IstModus = 1;             // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
        HVK_BMS_Sollmodus = 1;
        HVK_MO_EmSollzustand = 67;
    } 
  }

  if(BMS_HV_Status == 2)
  {
    HVK_DCDC_Sollmodus = 2;         // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    EM1_Status_Spgfreiheit = 2;     // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_Gesamtst_Spgfreiheit = 2;   // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
  }
  if(BMS_HV_Status == 1)
  {
    HVK_DCDC_Sollmodus = 1;         // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    EM1_Status_Spgfreiheit = 1;     // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_Gesamtst_Spgfreiheit = 1;   // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
  }

  switch (charger_params.activate)
  {
    case 0:     // Charger Standby
      HVK_HVLM_Sollmodus = false;   // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
      HVEM_Nachladen_Anf = false;   // Request for HV charging with plugged in connector and deactivated charging request
      BMS_Charger_Active = 0;
    break;
            
    case 1:     // HV Active - Charger Active
      HVEM_Nachladen_Anf = true;    // Request for HV charging with plugged in connector and deactivated charging request
      HVK_HVLM_Sollmodus = true;    // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
      BMS_Charger_Active = 1;
      HVK_BMS_Sollmodus = 4;
            
     break;
  }

  //  BMS_01
    BMS_01[0] = 0x00;
    BMS_01[1] = (0x00 & (0x0FU)) | ((BMS_Batt_Curr & (0x0FU)) << 4);
    BMS_01[2] = ((BMS_Batt_Curr >> 4) & (0xFFU));
    BMS_01[3] = (BMS_Batt_Volt& (0xFFU));
    BMS_01[4] = ((BMS_Batt_Volt >> 8) & (0x0FU)) | ((BMS_Batt_Volt_HVterm & (0x0FU)) << 4);
    BMS_01[5] = ((BMS_Batt_Volt_HVterm >> 4) & (0x7FU)) | ((BMS_SOC_HiRes & (0x01U)) << 7);
    BMS_01[6] = ((BMS_SOC_HiRes >> 1) & (0xFFU));
    BMS_01[7] = ((BMS_SOC_HiRes >> 9) & (0x03U)) | ((0x00 & (0x01U)) << 2) | ((0x00 & (0x0FU)) << 4);

  //  BMS_02
    BMS_02[0] = 0x00;
    BMS_02[1] = (BMS_MaxCharge_Curr_Offset & (0x0FU)) | ((BMS_MaxDischarge_Curr & (0x0FU)) << 4);
    BMS_02[2] = ((BMS_MaxDischarge_Curr >> 4) & (0x7FU)) | ((BMS_MaxCharge_Curr & (0x01U)) << 7);
    BMS_02[3] = ((BMS_MaxCharge_Curr >> 1) & (0xFFU));
    BMS_02[4] = ((BMS_MaxCharge_Curr >> 9) & (0x03U)) | ((BMS_Min_Batt_Volt & (0x3FU)) << 2);
    BMS_02[5] = ((BMS_Min_Batt_Volt >> 6) & (0x0FU)) | ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
    BMS_02[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) | ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
    BMS_02[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  BMS_03
    BMS_03[0] = (BMS_OpenCircuit_Volts & (0xFFU));
    BMS_03[1] = ((BMS_OpenCircuit_Volts >> 8) & (0x03U)) | ((BMS_Batt_Max_Volt & (0x0FU)) << 4);
    BMS_03[2] = ((BMS_Batt_Max_Volt >> 4) & (0x3FU)) | ((BMS_MaxDischarge_Curr & (0x03U)) << 6);
    BMS_03[3] = ((BMS_MaxDischarge_Curr >> 2) & (0xFFU));
    BMS_03[4] = ((BMS_MaxDischarge_Curr >> 10) & (0x01U)) | ((BMS_MaxCharge_Curr & (0x7FU)) << 1);
    BMS_03[5] = ((BMS_MaxCharge_Curr >> 7) & (0x0FU)) | ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
    BMS_03[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) | ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
    BMS_03[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  BMS_04
    BMS_04[0] = 0x00;
    BMS_04[1] = (0x00 & (0x0FU)) | ((BMS_Status_ServiceDisconnect & (0x01U)) << 5) | ((BMS_HV_Status & (0x03U)) << 6);
    BMS_04[2] = 0x00 | ((BMS_IstModus & (0x07U)) << 1) | ((BMS_Faultstatus & (0x07U)) << 4) | ((BMS_Batt_Ah & (0x01U)) << 7);
    BMS_04[3] = ((BMS_Batt_Ah >> 1) & (0xFFU));
    BMS_04[4] = ((BMS_Batt_Ah >> 9) & (0x03U));
    BMS_04[6] = ((BMS_Target_SOC_HiRes & (0x07U)) << 5);
    BMS_04[7] = ((BMS_Target_SOC_HiRes >> 3) & (0xFFU));

  //  BMS_06
    BMS_06[2] = (BMS_Batt_Temp & (0xFFU));
    BMS_06[3] = (BMS_CurrBatt_Temp & (0xFFU));
    BMS_06[7] = (BMS_CoolantTemp_Act & (0xFFU));
  //  BMS_07
    BMS_07[0] = 0x00;
    BMS_07[1] = (0x00 & (0x0FU)) | ((BMS_Batt_Energy & (0x0FU)) << 4);
    BMS_07[2] = ((BMS_Batt_Energy >> 4) & (0x7FU)) | ((BMS_Charger_Active & (0x01U)) << 7); //BMS_07[2] = ((BMS_Batt_Energy >> 4) & (0x7FU)) | ((BMS_Charger_Active & (0x01U)) << 7);
    BMS_07[3] = (BMS_Battdiag & (0x07U)) | ((BMS_Freig_max_Perf & (0x03U)) << 3) | ((BMS_Balancing_Active & (0x03U)) << 6);
    BMS_07[4] = (BMS_Max_Wh & (0xFFU));
    BMS_07[5] = ((BMS_Max_Wh >> 8) & (0x07U)) | ((0x0 & (0x01U)) << 3) | ((0x00 & (0x03U)) << 4) | ((0x00& (0x03U)) << 6); //BMS_07[5] = ((BMS_Max_Wh >> 8) & (0x07U)) | ((0x0 & (0x01U)) << 3) | ((BMS_Gesamtst_Spgfreiheit & (0x03U)) << 4) | ((BMS_RIso_Ext & (0x03U)) << 6);
    BMS_07[6] = ((BMS_RIso_Ext >> 2) & (0xFFU));
    BMS_07[7] = ((BMS_RIso_Ext >> 10) & (0x03U)) | ((0x00 & (0x03U)) << 2) | ((0x00 & (0x03U)) << 4);
  //  BMS_09
    BMS_09[2] = ((BMS_HV_Auszeit_Status & (0x03U)) << 5) | ((BMS_HV_Auszeit & (0x01U)) << 7);
    BMS_09[3] = ((BMS_HV_Auszeit >> 1) & (0xFFU));
    BMS_09[4] = (BMS_Kapazitaet & (0xFFU));
    BMS_09[5] = ((BMS_Kapazitaet >> 8) & (0x07U)) | ((BMS_SOC_Kaltstart & (0x1FU)) << 3);
    BMS_09[6] = ((BMS_SOC_Kaltstart >> 5) & (0x3FU)) | ((BMS_max_Grenz_SOC & (0x03U)) << 6);
    BMS_09[7] = ((BMS_max_Grenz_SOC >> 2) & (0x07U)) | ((BMS_min_Grenz_SOC & (0x1FU)) << 3);
  //  BMS_10
    BMS_10[0] = (BMS_BattEnergy_Wh_HiRes & (0xFFU));
    BMS_10[1] = ((BMS_BattEnergy_Wh_HiRes >> 8) & (0x7FU)) | ((BMS_MaxBattEnergy_Wh_HiRes & (0x01U)) << 7);
    BMS_10[2] = ((BMS_MaxBattEnergy_Wh_HiRes >> 1) & (0xFFU));
    BMS_10[3] = ((BMS_MaxBattEnergy_Wh_HiRes >> 9) & (0x3FU)) | ((BMS_SOC & (0x03U)) << 6);
    BMS_10[4] = ((BMS_SOC >> 2) & (0x3FU)) | ((BMS_ResidualEnergy_Wh & (0x03U)) << 6);
    BMS_10[5] = ((BMS_ResidualEnergy_Wh >> 2) & (0xFFU));
    BMS_10[6] = ((BMS_ResidualEnergy_Wh >> 10) & (0x03U)) | ((0x64 & (0x3FU)) << 2);
    BMS_10[7] = ((0x64 >> 6) & (0x01U)) | ((0x64 & (0x7FU)) << 1);
  //  BMS_11
    BMS_11[0] = 0x00;
    BMS_11[1] = 0x00;
    BMS_11[2] = ((0x02 & (0x0FU)) << 1) | ((0x01 & (0x07U)) << 5);
    BMS_11[3] = (BMS_BattCell_Temp_Max & (0xFFU));
    BMS_11[4] = (BMS_BattCell_Temp_Min & (0xFFU));
    BMS_11[5] = (BMS_BattCell_MV_Max & (0xFFU));
    BMS_11[6] = ((BMS_BattCell_MV_Max >> 8) & (0x0FU)) | ((BMS_BattCell_MV_Min & (0x0FU)) << 4);
    BMS_11[7] = ((BMS_BattCell_MV_Min >> 4) & (0xFFU));
  //  BMS_27
    BMS_27[0] = 0x00;
    BMS_27[1] = 0x00;
    BMS_27[2] = 0x00;
    BMS_27[3] = ((BMS_SOC_ChargeLim & (0x3FU)) << 2);
    BMS_27[4] = ((BMS_SOC_ChargeLim >> 6) & (0x01U)) | ((BMS_EnergyCount & (0x0FU)) << 1) | ((BMS_EnergyReq_Full & (0x07U)) << 5);
    BMS_27[5] = ((BMS_EnergyReq_Full >> 3) & (0xFFU));
    BMS_27[6] = (BMS_ChargePowerMax & (0xFFU));
    BMS_27[7] = ((BMS_ChargePowerMax >> 8) & (0x0FU)) | ((BMS_ChargeEnergyCount & (0x0FU)) << 4);
  //  BMS_DC_01

      //  BMS_Status_DCLS = Status of the voltage monitoring at the DC charging interface | 0=inactive, 1= i.O, 2= n.i.O, 3= Active
      //  BMS_DCLS_Spannung = DC voltage of the charging station. Measurement between the DC HV lines.
      //  BMS_DCLS_MaxLadeStrom = maximum permissible DC charging current

  // BMS_DC_01[0] = (BMS_DC_01_CRC & (0xFFU));
  // BMS_DC_01[1] = (BMS_DC_01_BZ & (0x0FU)) | ((BMS_Status_DCLS & (0x03U)) << 4) | ((BMS_DCLS_Spannung & (0x03U)) << 6);
  // BMS_DC_01[2] = ((BMS_DCLS_Spannung >> 2) & (0xFFU));
  // BMS_DC_01[3] = (BMS_DCLS_MaxLadeStrom & (0xFFU));
  // BMS_DC_01[4] = ((BMS_DCLS_MaxLadeStrom >> 8) & (0x01U));
  // BMS_DC_01[5] = 0x00;
  // BMS_DC_01[6] = 0x00;
  // BMS_DC_01[7] = 0x00;

  //  DCDC_01 - For DC/DC 12V Converter
      //  Intend to mirror HV voltage from main bus (unless found elsewhere)
      //  Charger only seems interested in the 12V output Current & Voltage from module?
  // DCDC_01[0] = 0x00;
  // DCDC_01[1] = (0x00 & (0x0FU)) | ((DC_IstSpannung_HV & (0x0FU)) << 4);
  // DCDC_01[2] = ((DC_IstSpannung_HV >> 4) & (0xFFU));
  // DCDC_01[3] = (DC_IstStrom_HV_02 & (0xFFU));
  // DCDC_01[4] = ((DC_IstStrom_HV_02 >> 8) & (0x03U)) | ((DC_IstStrom_NV & (0x3FU)) << 2);
  // DCDC_01[5] = ((DC_IstStrom_NV >> 6) & (0x0FU));
  // DCDC_01[7] = (DC_IstSpannung_NV & (0xFFU));

  //  DCDC_03 - For DC/DC 12V Converter
    DCDC_03[2] = (0x00 & (0x07U)) | ((0x00 & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((DC_IstModus_02 & (0x07U)) << 5);


  //  Dimmung_01
  // Dimmung_01[0] = (DI_KL_58xd & (0xFFU));
  // Dimmung_01[1] = (DI_KL_58xs & (0x7FU)) | ((DI_Display_Nachtdesign & (0x01U)) << 7);
  // Dimmung_01[2] = (DI_KL_58xt & (0x7FU));
  // Dimmung_01[3] = (DI_Fotosensor & (0xFFU));
  // Dimmung_01[4] = ((DI_Fotosensor >> 8) & (0xFFU));
  // Dimmung_01[5] = (BCM1_Stellgroesse_Kl_58s & (0x7FU));
  // Dimmung_01[6] = 0x00;
  // Dimmung_01[7] = 0x00;

  //  HVEM_05
    HVEM_05[0] = 0x00;
    HVEM_05[1] = ((HVEM_NVNachladen_Energie & (0x0FU)) << 4);
    HVEM_05[2] = (HVEM_NVNachladen_Energie >> 4) & (0x0FU);
    HVEM_05[3] = 0x00;
    HVEM_05[4] = (HVEM_Nachladen_Anf & (0x01U)) | ((HVEM_SollStrom_HV & (0x7FU)) << 1);
    HVEM_05[5] = ((HVEM_SollStrom_HV >> 7) & (0x0FU)) | ((HVEM_MaxSpannung_HV & (0x0FU)) << 4);
    HVEM_05[6] = ((HVEM_MaxSpannung_HV >> 4) & (0x3FU)) | ((0x00& (0x03U)) << 6);
    HVEM_05[7] = 0x00;

  // Authentic_Time_01 & NavData_02
    Authentic_Time_01[4] = (UnixTime & (0xFFU));
    Authentic_Time_01[5] = ((UnixTime >> 8) & (0xFFU));
    Authentic_Time_01[6] = ((UnixTime >> 16) & (0xFFU));
    Authentic_Time_01[7] = ((UnixTime >> 24) & (0xFFU));

    ESP_15[4] = (0x00 & (0x01U)) | ((0x00 & (0x07U)) << 1) | ((HMS_Systemstatus & (0x0FU)) << 4);
    ESP_15[5] = (0x00 & (0x07U)) | ((HMS_aktives_System & (0x1FU)) << 3);
    ESP_15[6] = (0x00 & (0x01U)) | ((0x00 & (0x01U)) << 1) | ((HMS_Fehlerstatus & (0x07U)) << 2) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x03U)) << 6);

    Klemmen_Status_01[2] = (ZAS_Kl_S & (0x01U)) | ((ZAS_Kl_15 & (0x01U)) << 1) | ((ZAS_Kl_X & (0x01U)) << 2) | ((ZAS_Kl_50_Startanforderung & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
    
    HVK_01[1] = (0x00 & (0x0FU)) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x03U)) << 5);
    HVK_01[2] = (HVK_MO_EmSollzustand & (0xFFU));
    HVK_01[3] = (HVK_BMS_Sollmodus & (0x07U)) | ((HVK_DCDC_Sollmodus & (0x07U)) << 3) | ((0x00 & (0x03U)) << 6);
    HVK_01[4] = ((0x00 >> 2) & (0x01U)) | ((0x00 & (0x07U)) << 1) | ((HVK_HVLM_Sollmodus & (0x07U)) << 4) | ((0x00 & (0x01U)) << 7);
    HVK_01[5] = ((0x00 >> 1) & (0x01U)) | ((HV_Bordnetz_aktiv & (0x01U)) << 1) | ((0x00 & (0x01U)) << 2) | ((HVK_Gesamtst_Spgfreiheit & (0x03U)) << 3) | ((0x00 & (0x01U)) << 5);

    
    ZV_01[1] = (0x00 & (0x0FU)) | ((ZV_FT_verriegeln & (0x01U)) << 4) | ((ZV_FT_entriegeln & (0x01U)) << 5) | ((ZV_BT_verriegeln & (0x01U)) << 6) | ((ZV_BT_entriegeln & (0x01U)) << 7);
    ZV_01[7] = ((0x00 >> 5) & (0x3FU)) | ((ZV_entriegeln_Anf & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
    
    ZV_02[2] = (ZV_verriegelt_intern_ist & (0x01U)) | ((ZV_verriegelt_extern_ist & (0x01U)) << 1) | ((ZV_verriegelt_intern_soll & (0x01U)) << 2) | ((ZV_verriegelt_extern_soll & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
    ZV_02[7] = (0x00 & (0x01U)) | ((0x00 & (0x01U)) << 1) | ((0x00 & (0x01U)) << 2) | ((0x00 & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) | ((ZV_verriegelt_soll & (0x03U)) << 6);



    EM_HYB_11[1] = (0x00 & (0x0FU)) | ((EM1_Istmodus2 & (0x0FU)) << 4);
    EM_HYB_11[2] = (0x00 & (0x07U)) | ((EM1_Status_Spgfreiheit & (0x03U)) << 3) | ((0x00 & (0x01U)) << 5);

}
