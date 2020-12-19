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
#ifndef ERRORMESSAGE_H
#define ERRORMESSAGE_H

#include "errormessage_prj.h"
#include <stdint.h>

#define ERROR_MESSAGE_ENTRY(id, type) ERR_##id,
typedef enum
{
   ERROR_NONE,
   ERROR_MESSAGE_LIST
   ERROR_MESSAGE_LAST
} ERROR_MESSAGE_NUM;
#undef ERROR_MESSAGE_ENTRY

typedef enum
{
   ERROR_STOP,
   ERROR_DERATE,
   ERROR_DISPLAY,
   ERROR_LAST
} ERROR_TYPE;

class ErrorMessage
{
   public:
      static void SetTime(uint32_t time);
      static void Post(ERROR_MESSAGE_NUM err);
      static void UnpostAll();
      static void PrintAllErrors();
      static void PrintNewErrors();
      static ERROR_MESSAGE_NUM GetLastError();
   protected:
   private:
      static void PrintError(uint32_t time, ERROR_MESSAGE_NUM err);

      static uint32_t timeTick;
      static uint32_t currentBufIdx;
      static uint32_t lastPrintIdx;
      static bool posted[ERROR_MESSAGE_LAST];
      static ERROR_MESSAGE_NUM lastError;
};

#endif // ERRORMESSAGE_H
