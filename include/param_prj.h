/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#define VER 1.00.A


/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 102
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_SETUP,Inverter,     INVMODES    ,  0,      5,      0,      5  ) \
    PARAM_ENTRY(CAT_SETUP,Vehicle,      VEHMODES    ,  0,      6,      0,      6  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmin,      "dig",     0,      4095,   0,      17  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmax,      "dig",     0,      4095,   4095,   18  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2min,     "dig",     0,      4095,   4095,   63  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2max,     "dig",     0,      4095,   4095,   64  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmode,     POTMODES,  0,      2,      0,      82  ) \
    PARAM_ENTRY(CAT_THROTTLE,dirmode,     DIRMODES,  0,      4,      1,      95  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramp,   "%/10ms",  0.1,    100,    100,    81  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramprpm,"rpm",     0,      20000,  20000,  85  )  \
    PARAM_ENTRY(CAT_THROTTLE,revlim,        "rpm",      0,     20000,   6000,  19   ) \
    PARAM_ENTRY(CAT_THROTTLE,brkout,      "%",       -100,   -1,     -50,    67  ) \
    PARAM_ENTRY(CAT_THROTTLE,  bmslimhigh,  "%",       0,      100,    50,     55  ) \
    PARAM_ENTRY(CAT_THROTTLE,  bmslimlow,   "%",       -100,   0,      -1,     56  ) \
    PARAM_ENTRY(CAT_THROTTLE,  udcmin,      "V",       0,      1000,   450,    42  ) \
    PARAM_ENTRY(CAT_THROTTLE,  udclim,      "V",       0,      1000,   520,    43  ) \
    PARAM_ENTRY(CAT_THROTTLE,  idcmax,      "A",       0,      5000,   5000,   96  ) \
    PARAM_ENTRY(CAT_THROTTLE,  idcmin,      "A",       -5000,  0,     -5000,   98  ) \
    PARAM_ENTRY(CAT_THROTTLE,  tmphsmax,    "°C",      50,     150,   85,      125 ) \
    PARAM_ENTRY(CAT_THROTTLE,  tmpmmax,     "°C",      70,     300,   300,     127 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtmax,    "%",       0,      100,   100,     97  ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtmin,    "%",       -100,   0,     -100,    119 ) \
    PARAM_ENTRY(CAT_LEXUS,      GEAR    ,LOWHIGH,  0,      2,      0,      7  ) \
    PARAM_ENTRY(CAT_LEXUS,      OilPump,    "%",  0,      100,      50,      8  ) \
    PARAM_ENTRY(CAT_CRUISE,   cruisestep,  "rpm",     1,      1000,   200,    3   ) \
    PARAM_ENTRY(CAT_CRUISE,   cruiseramp,  "rpm/100ms",1,     1000,   20,     9   ) \
    PARAM_ENTRY(CAT_CRUISE,   regenlevel,  "",        0,      3,      2,      101  ) \
    PARAM_ENTRY(CAT_CONTACT,    udcsw,       "V",       0,      1000,   330,    20  ) \
    PARAM_ENTRY(CAT_CONTACT,  cruiselight, ONOFF,     0,      1,      0,      0   ) \
    PARAM_ENTRY(CAT_CONTACT,  errlights,   ERRLIGHTS, 0,      255,    0,      25  ) \
    PARAM_ENTRY(CAT_COMM,     canspeed,    CANSPEEDS, 0,      3,      1,      83  ) \
    PARAM_ENTRY(CAT_COMM,     canperiod,   CANPERIODS,0,      1,      1,      88  ) \
    PARAM_ENTRY(CAT_CHARGER, chargemodes,  CHGMODS,0,      4,      0,      74  ) \
    PARAM_ENTRY(CAT_CHARGER, BattCap,       "Wh",       0,      65534,   21000,    82  ) \
    PARAM_ENTRY(CAT_CHARGER, interface,  CHGint,0,      2,      0,      78  ) \
    PARAM_ENTRY(CAT_CHARGER, Voltspnt,       "V",       0,      1000,   395,    75  ) \
    PARAM_ENTRY(CAT_CHARGER, Pwrspnt,       "W",       0,      12000,   1500,    76  ) \
    PARAM_ENTRY(CAT_CHARGER, CCS_ICmd,       "A",       0,      100,   0,    120  ) \
    PARAM_ENTRY(CAT_CHARGER, CCS_VLim,       "V",       0,      1000,   400,    79  ) \
    PARAM_ENTRY(CAT_CHARGER, CCS_ILim,       "A",       0,      350,   100,    80  ) \
    PARAM_ENTRY(CAT_CHARGER, CCS_SOCLim,       "%",       0,      100,   80,    81  ) \
    PARAM_ENTRY(CAT_CHARGER, Chgctrl,  CHGCTRL,0,      1,      0,      77  ) \
    VALUE_ENTRY(version,      VERSTR,  2039 ) \
    VALUE_ENTRY(hwver,        HWREVS,  2036 ) \
    VALUE_ENTRY(opmode,       OPMODES, 2000 ) \
    VALUE_ENTRY(chgtyp,       CHGTYPS, 2113 ) \
    VALUE_ENTRY(lasterr,      errorListString,  2038 ) \
    VALUE_ENTRY(status,      STATUS,  2044 ) \
    VALUE_ENTRY(udc,         "V",     2001 ) \
    VALUE_ENTRY(udc2,         "V",     2002 ) \
    VALUE_ENTRY(udc3,         "V",     2003 ) \
    VALUE_ENTRY(deltaV,         "V",     2005 ) \
    VALUE_ENTRY(INVudc,      "V",     2004 ) \
    VALUE_ENTRY(power,        "kW",    2051 ) \
    VALUE_ENTRY(idc,          "A",     2047 ) \
    VALUE_ENTRY(KWh,          "kwh",   2048 ) \
    VALUE_ENTRY(AMPh,          "Ah",   2049 ) \
    VALUE_ENTRY(SOC,          "%",   2110 ) \
    VALUE_ENTRY(speed,        "rpm",   2012 ) \
    VALUE_ENTRY(torque,       "dig",   2013 ) \
    VALUE_ENTRY(pot,          "dig",   2015 ) \
    VALUE_ENTRY(pot2,         "dig",   2016 ) \
    VALUE_ENTRY(potbrake,     "dig",   2075 ) \
    VALUE_ENTRY(brakepressure,"dig",   2074 ) \
    VALUE_ENTRY(potnom,       "%",     2017 ) \
    VALUE_ENTRY(dir,         DIRS,    2018 ) \
    VALUE_ENTRY(inv,         INVMODES,    2099 ) \
    VALUE_ENTRY(veh,         VEHMODES,    2098 ) \
    VALUE_ENTRY(Charger,     CHGMODS,    2097 ) \
    VALUE_ENTRY(tmphs,        "°C",    2019 ) \
    VALUE_ENTRY(tmpm,         "°C",    2020 ) \
    VALUE_ENTRY(tmpaux,       "°C",    2072 ) \
    VALUE_ENTRY(uaux,         "V",     2021 ) \
    VALUE_ENTRY(canio,        CANIOS,  2022 ) \
    VALUE_ENTRY(cruisespeed,  "rpm",   2059 ) \
    VALUE_ENTRY(cruisestt,CRUISESTATES,2055 ) \
    VALUE_ENTRY(din_cruise,   ONOFF,   2023 ) \
    VALUE_ENTRY(din_start,    ONOFF,   2024 ) \
    VALUE_ENTRY(din_brake,    ONOFF,   2025 ) \
    VALUE_ENTRY(din_forward,  ONOFF,   2027 ) \
    VALUE_ENTRY(din_reverse,  ONOFF,   2028 ) \
    VALUE_ENTRY(din_bms,      ONOFF,   2032 ) \
    VALUE_ENTRY(handbrk,      ONOFF,   2071 ) \
    VALUE_ENTRY(Gear1,      ONOFF,   2091 ) \
    VALUE_ENTRY(Gear2,      ONOFF,   2092 ) \
    VALUE_ENTRY(Gear3,      ONOFF,   2093 ) \
    VALUE_ENTRY(T15Stat,      ONOFF,   2094 ) \
    VALUE_ENTRY(InvStat,      ONOFF,   2095 ) \
    VALUE_ENTRY(GearFB,        LOWHIGH,   2073 ) \
    VALUE_ENTRY(CableLim,        "A",   2100 ) \
    VALUE_ENTRY(PilotLim,        "A",   2101 ) \
    VALUE_ENTRY(PlugDet,        ONOFF,   2102 ) \
    VALUE_ENTRY(PilotTyp,       PLTMODES,   2103 ) \
    VALUE_ENTRY(CCS_I_Avail,     "A",   2104 ) \
    VALUE_ENTRY(CCS_V_Avail,     "V",   2105 ) \
    VALUE_ENTRY(CCS_I,     "A",   2106 ) \
    VALUE_ENTRY(CCS_V,     "V",   2107 ) \
    VALUE_ENTRY(CCS_V_Min, "V",   2108 ) \
    VALUE_ENTRY(CCS_V_Con, "V",   2114 ) \
    VALUE_ENTRY(CCS_ConStat,   ONOFF,   2109 ) \
    VALUE_ENTRY(hvChg,   ONOFF,   2111 ) \
    VALUE_ENTRY(CCS_State,   "S",   2211 ) \
    VALUE_ENTRY(cpuload,      "%",     2035 ) \

//Next value Id: 2080

#define VERSTR STRINGIFY(4=VER)
#define POTMODES     "0=SingleRegen, 1=DualChannel, 2=CAN"
#define BTNSWITCH    "0=Button, 1=Switch, 2=CAN"
#define DIRMODES     "0=Button, 1=Switch, 2=ButtonReversed, 3=SwitchReversed, 4=DefaultForward"
#define INVMODES     "0=Leaf_Gen1, 1=GS450H, 2=UserCAN, 3=Zombie, 4=Prius_Gen3"
#define PLTMODES     "0=Absent, 1=ACStd, 2=ACchg, 3=Error, 4=CCS Not Rdy, 5=CCS Rdy, 6=Static"
#define VEHMODES     "0=BMW_E46, 1=BMW_E65, 2=User, 3=None , 5=BMW_E39 , 6=VAG"
//#define OPMODES      "0=Off, 1=Run, 2=ManualRun, 3=Boost, 4=Buck, 5=Sine, 6=AcHeat, 7=ChargeStart, 8=ConnectorLock, 9=Charge, 10=ChargeStop"
#define OPMODES      "0=Off, 1=Run, 2=Precharge, 3=PchFail, 4=Charge"
#define CHGTYPS      "0=Off, 1=AC, 2=DCFC"
#define STATUS       "0=None, 1=UdcLow, 2=UdcHigh, 4=UdcBelowUdcSw, 8=UdcLim, 16=EmcyStop, 32=MProt, 64=PotPressed, 128=TmpHs, 256=WaitStart"
#define DIRS         "-1=Reverse, 0=Neutral, 1=Forward"
#define ONOFF        "0=Off, 1=On, 2=na"
#define LOWHIGH        "0=LOW, 1=HIGH, 2=AUTO"
#define OKERR        "0=Error, 1=Ok, 2=na"
#define CANSPEEDS    "0=250k, 1=500k, 2=800k, 3=1M"
#define CANIOS       "1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms"
#define CANPERIODS   "0=100ms, 1=10ms"
#define HWREVS       "0=Rev1, 1=Rev2, 2=Rev3, 3=Tesla"
#define ERRLIGHTS    "0=Off, 4=EPC, 8=engine"
#define CRUISESTATES "0=None, 1=On, 2=Disable, 4=SetN, 8=SetP"
#define CDMSTAT      "1=Charging, 2=Malfunction, 4=ConnLock, 8=BatIncomp, 16=SystemMalfunction, 32=Stop"
#define CAT_THROTTLE "Throttle"
#define CAT_POWER    "Power Limit"
#define CAT_CONTACT  "Contactor Control"
#define CAT_TEST     "Testing"
#define CAT_COMM     "Communication"
#define CAT_SETUP      "Vehicle Module"
#define CAT_CRUISE   "Cruise Control"
#define CAT_LEXUS   "Gearbox Control"
#define CAT_CHARGER  "Charger Control"
#define CHGMODS   "0=Off, 1=HV_ON, 2=EXT_CAN 3=EXT_DIGI, 4=Volt_Ampera"
#define CHGCTRL   "0=Enable, 1=Disable"
#define CHGint   "0=Unused, 1=i3LIM"
#define CAN_PERIOD_100MS    0
#define CAN_PERIOD_10MS     1

enum modes
{
    MOD_OFF = 0,
    MOD_RUN,
    MOD_PRECHARGE,
    MOD_PCHFAIL,
    MOD_CHARGE,
    MOD_LAST
};

enum ctyps
{
    OFF = 0,
    AC,
    DCFC
};

enum cruisestate
{
    CRUISE_ON = 1,
    CRUISE_DISABLE = 2,
    CRUISE_SETN = 4,
    CRUISE_SETP = 8
};

enum _tripmodes
{
    TRIP_ALLOFF = 0,
    TRIP_DCSWON,
    TRIP_PRECHARGEON
};

enum _dirmodes
{
    DIR_BUTTON = 0,
    DIR_SWITCH = 1,
    DIR_REVERSED = 2,
    DIR_DEFAULTFORWARD = 4
};

enum _invmodes
{
    Leaf_Gen1 = 0,
    GS450H = 1,
    UserCAN = 2,
    Zombie = 3,
    Prius_Gen3 = 4
};

enum _chgmodes
{
    Off = 0,
    HV_ON=1,
    EXT_CAN=2,
    EXT_DIGI = 3,
    Volt_Ampera = 4
};

enum _interface
{
    Unused = 0,
    i3LIM =1

};

enum _chgctrl
{
    Enable = 0,
    Disable =1

};

enum _gear
{
    LOW = 0,
    HIGH = 1,
    AUTO = 2
};

enum _vehmodes
{
    BMW_E46 = 0,
    BMW_E65 = 1,
    User = 2, //used as a flag
    None = 4,
    BMW_E39 = 5,
    VAG = 6
};

enum _potmodes
{
    POTMODE_REGENADJ = 0,
    POTMODE_DUALCHANNEL,
    POTMODE_CAN
};

enum _canio
{
    CAN_IO_CRUISE = 1,
    CAN_IO_START = 2,
    CAN_IO_BRAKE = 4,
    CAN_IO_FWD = 8,
    CAN_IO_REV = 16,
    CAN_IO_BMS = 32
};

enum status
{
    STAT_NONE = 0,
    STAT_UDCLOW = 1,
    STAT_UDCHIGH = 2,
    STAT_UDCBELOWUDCSW = 4,
    STAT_UDCLIM = 8,
    STAT_EMCYSTOP = 16,
    STAT_MPROT = 32,
    STAT_POTPRESSED = 64,
    STAT_TMPHS = 128,
    STAT_WAITSTART = 256
};

extern const char* errorListString;

