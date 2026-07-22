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

#ifndef C5DCDC_H
#define C5DCDC_H

#include "dcdc.h"

class C5DCDC : public DCDC {
public:
  void DecodeCAN(int id, uint8_t *data) override;
  void DeInit() override;
  void Task10Ms() override;
  void SetCanInterface(CanHardware *c) override;

private:
  void Send0x11A();

  uint8_t counter = 0;
  uint8_t timer20Ms = 0;
  float maxDcdcTemp = 0;
};

#endif // C5DCDC_H
