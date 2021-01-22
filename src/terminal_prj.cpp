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

static void LoadDefaults(char *arg);
static void GetAll(char *arg);
static void PrintList(char *arg);
static void PrintAtr(char *arg);
static void Help(char *arg);
static void PrintSerial(char *arg);
static void PrintErrors(char *arg);

extern "C" const TERM_CMD TermCmds[] =
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
    { "help", Help },
    { "json", TerminalCommands::PrintParamsJson },
    { "can", TerminalCommands::MapCan },
    { "serial", PrintSerial },
    { "errors", PrintErrors },
    { "reset", TerminalCommands::Reset },
    //{ "fastuart", FastUart },
    { NULL, NULL }
};

static void PrintList(char *arg)
{
    const Param::Attributes *pAtr;

    arg = arg;

    printf("Available parameters and values\r\n");

    for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
    {
        pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);

        if ((Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
            printf("%s [%s]\r\n", pAtr->name, pAtr->unit);
    }
}

static void PrintAtr(char *arg)
{
    const Param::Attributes *pAtr;

    arg = arg;

    printf("Parameter attributes\r\n");
    printf("Name\t\tmin - max [default]\r\n");

    for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
    {
        pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
        /* Only display for params */
        if (Param::IsParam((Param::PARAM_NUM)idx) && (Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
        {
            printf("%s\t\t%f - %f [%f]\r\n", pAtr->name,pAtr->min,pAtr->max,pAtr->def);
        }
    }
}

static void LoadDefaults(char *arg)
{
    arg = arg;
    Param::LoadDefaults();
    printf("Defaults loaded\r\n");
}

static void GetAll(char *arg)
{
    const Param::Attributes *pAtr;

    arg = arg;

    for (uint32_t  idx = 0; idx < Param::PARAM_LAST; idx++)
    {
        pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
        printf("%s\t\t%f\r\n", pAtr->name, Param::Get((Param::PARAM_NUM)idx));
    }
}

static void PrintErrors(char *arg)
{
    arg = arg;
    ErrorMessage::PrintAllErrors();
}

static void PrintSerial(char *arg)
{
    arg = arg;
    printf("%X%X%X\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);
}

static void Help(char *arg)
{
    arg = arg;
}
