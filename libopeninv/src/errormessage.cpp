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
#include "errormessage.h"
#include "printf.h"
#include "my_string.h"

struct ErrorDescriptor
{
   const char* msg;
   ERROR_TYPE type;
};

struct BufferEntry
{
   ERROR_MESSAGE_NUM msg;
   uint32_t time;
};

#define ERROR_MESSAGE_ENTRY(id, type) { #id, type },
static const struct ErrorDescriptor errorDescriptors[] =
{
   { "", ERROR_LAST },
   ERROR_MESSAGE_LIST
};
#undef ERROR_MESSAGE_ENTRY

#define EXPANDED_LIST ERROR_MESSAGE_ENTRY(NONE, ERROR_DISPLAY) ERROR_MESSAGE_LIST
#define ERROR_MESSAGE_ENTRY(id, type) __COUNTER__=id,
const char* errorListString = STRINGIFY(EXPANDED_LIST);
#undef ERROR_MESSAGE_ENTRY

static const char* types[ERROR_LAST] =
{
   "STOP",
   "DERATE",
   "WARN"
};

struct BufferEntry errorBuffer[ERROR_BUF_SIZE] = { { ERROR_MESSAGE_LAST, 0 } };

uint32_t ErrorMessage::timeTick = 0;
uint32_t ErrorMessage::currentBufIdx = 0;
uint32_t ErrorMessage::lastPrintIdx = 0;
ERROR_MESSAGE_NUM ErrorMessage::lastError = ERROR_NONE;
bool ErrorMessage::posted[ERROR_MESSAGE_LAST] = { false };

/** Set timestamp for error message
* @param time Current timestamp, will be displayed as is in message */
void ErrorMessage::SetTime(uint32_t time)
{
   timeTick = time;
}

/** Post an error message.
 Every message can only be posted once, then UnpostAll() must be called to post it again
 @post Message is displayed and written to error memory
 @param msg message number */
void ErrorMessage::Post(ERROR_MESSAGE_NUM msg)
{
   if (!posted[msg] && timeTick > 0)
   {
      lastError = msg;
      errorBuffer[currentBufIdx].msg = msg;
      errorBuffer[currentBufIdx].time = timeTick;
      posted[msg] = true;
      currentBufIdx = (currentBufIdx + 1) % ERROR_BUF_SIZE;
   }
}

/** Unpost all error message, i.e. make them postable again.
 Does not reset the error buffer */
void ErrorMessage::UnpostAll()
{
   for (uint32_t i = 0; i < ERROR_MESSAGE_LAST; i++)
      posted[i] = false;
}

/** Print errors that have been posted since last print */
void ErrorMessage::PrintNewErrors()
{
   while (lastPrintIdx != currentBufIdx)
   {
      PrintError(errorBuffer[lastPrintIdx].time, errorBuffer[lastPrintIdx].msg);
      lastPrintIdx = (lastPrintIdx + 1) % ERROR_BUF_SIZE;
   }
}

ERROR_MESSAGE_NUM ErrorMessage::GetLastError()
{
   return lastError;
}

/** Print all errors currently in error memory */
void ErrorMessage::PrintAllErrors()
{
   if (errorBuffer[0].time == 0)
   {
      printf("No Errors\r\n");
      return;
   }

   for (uint32_t i = 0; i < ERROR_BUF_SIZE && errorBuffer[i].time > 0; i++)
      PrintError(errorBuffer[i].time, errorBuffer[i].msg);
}

void ErrorMessage::PrintError(uint32_t time, ERROR_MESSAGE_NUM msg)
{
   printf("[%u]: %s - %s\r\n", time, types[errorDescriptors[msg].type], errorDescriptors[msg].msg);
}
