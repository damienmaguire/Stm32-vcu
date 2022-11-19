/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include <iostream>
#include "test.h"
#define EXPORT_TESTLIST
#include "test_list.h"

using namespace std;

int _failedAssertions = 0;

int main()
{
   int dummy;
   IUnitTest** currentTest = testList;

   cout << "Starting unit Tests" << endl;

   while (*currentTest)
   {
      (*currentTest)->RunTest();
      currentTest++;
   }

   if (_failedAssertions > 0)
   {
      cout << _failedAssertions << " assertions failed" << endl;
      return -1;
   }

   cout << "All tests passed" << endl;

   return 0;
}
