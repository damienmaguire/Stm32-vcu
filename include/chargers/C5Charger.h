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

#ifndef C5CHARGER_H
#define C5CHARGER_H

#include "chargers/chargerhw.h"

class C5Charger : public Chargerhw {
public:
  void Task10Ms() override;
  void Task100Ms() override;
  void DecodeCAN(int id, uint32_t data[2]) override;
  bool ControlCharge(bool RunCh, bool ACReq) override;
  void DeInit() override;
  void SetCanInterface(CanHardware *c) override;

private:
  void Send0x0AD();
  void Send0x09C();
  void Send0x304();
  void Send0x305();
  void Send0x306();
  void Send0x308();
  void Send0x321();
  void Send0x322();
  void Send0x485();
  float MaxChargePowerWatts() const;
  float MaxChargeCurrentAmps() const;
  bool HvConnected() const;

  bool chargeCommand = false;
  bool obcFaultPresent = false;
  uint8_t obcStatus = 0;
  uint8_t obcFaultLevel = 0;
  uint8_t obcWorkMode = 0;
  uint8_t obcRc = 0;
  uint8_t bmsCounter = 0;
  uint8_t vcuNmCounter = 0;
  uint8_t timer20Ms = 0;
  uint8_t timer500Ms = 0;
  float maxChargeTemp = 0;
};

#endif // C5CHARGER_H
