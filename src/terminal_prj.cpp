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

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include "hwdefs.h"
#include "terminal.h"
#include "params.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "param_save.h"
#include "errormessage.h"
#include "stm32_can.h"
#include "terminalcommands.h"

static void LoadDefaults(Terminal* t, char *arg);
static void GetAll(Terminal* t, char *arg);
static void PrintList(Terminal* t, char *arg);
static void PrintAtr(Terminal* t, char *arg);
static void PrintSerial(Terminal* t, char *arg);
static void PrintErrors(Terminal* t, char *arg);

extern const TERM_CMD TermCmds[] =
{
   { "set", TerminalCommands::ParamSet },
   { "get", TerminalCommands::ParamGet },
   { "flag", TerminalCommands::ParamFlag },
   { "stream", TerminalCommands::ParamStream },
   { "defaults", LoadDefaults },
   { "all", GetAll },
   { "list", PrintList },
   { "atr",  PrintAtr },
   { "save", TerminalCommands::SaveParameters },
   { "load", TerminalCommands::LoadParameters },
   { "json", TerminalCommands::PrintParamsJson },
   { "can", TerminalCommands::MapCan },
   { "serial", PrintSerial },
   { "errors", PrintErrors },
   { "reset", TerminalCommands::Reset },
   { NULL, NULL }
};

static void PrintList(Terminal* t, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;

   fprintf(t, "Available parameters and values\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);

      if ((Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
         printf("%s [%s]\r\n", pAtr->name, pAtr->unit);
   }
}

static void PrintAtr(Terminal* term, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;
   term = term;

   printf("Parameter attributes\r\n");
   printf("Name\t\tmin - max [default]\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      /* Only display for params */
      if ((Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM ||
           Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_TESTPARAM) &&
          (Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
      {
         printf("%s\t\t%f - %f [%f]\r\n", pAtr->name,pAtr->min,pAtr->max,pAtr->def);
      }
   }
}

static void LoadDefaults(Terminal* t, char *arg)
{
   arg = arg;
   Param::LoadDefaults();
   fprintf(t, "Defaults loaded\r\n");
}

static void GetAll(Terminal* t, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;

   for (uint32_t  idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      fprintf(t, "%s\t\t%f\r\n", pAtr->name, Param::Get((Param::PARAM_NUM)idx));
   }
}

static void PrintErrors(Terminal* t, char *arg)
{
   t = t;
   arg = arg;
   ErrorMessage::PrintAllErrors();
}

static void PrintSerial(Terminal* t, char *arg)
{
   arg = arg;
   fprintf(t, "%X%X%X\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);
}
