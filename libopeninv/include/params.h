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
#ifndef PARAM_H_INCLUDED
#define PARAM_H_INCLUDED

#include "param_prj.h"
#include "my_fp.h"

namespace Param
{
   #define PARAM_ENTRY(category, name, unit, min, max, def, id) name,
   #define VALUE_ENTRY(name, unit, id) name,
   typedef enum
   {
       PARAM_LIST
       PARAM_LAST,
       PARAM_INVALID
   } PARAM_NUM;
   #undef PARAM_ENTRY
   #undef VALUE_ENTRY

   typedef enum
   {
       TYPE_PARAM,
       TYPE_VALUE,
       TYPE_LAST
   } PARAM_TYPE;

   typedef enum
   {
      FLAG_NONE = 0,
      FLAG_HIDDEN = 1
   } PARAM_FLAG;

   typedef struct
   {
      char const *category;
      char const *name;
      char const *unit;
      s32fp min;
      s32fp max;
      s32fp def;
      uint32_t id;
   } Attributes;

   int Set(PARAM_NUM ParamNum, s32fp ParamVal);
   s32fp  Get(PARAM_NUM ParamNum);
   int    GetInt(PARAM_NUM ParamNum);
   s32fp  GetScl(PARAM_NUM ParamNum);
   bool   GetBool(PARAM_NUM ParamNum);
   void SetInt(PARAM_NUM ParamNum, int ParamVal);
   void SetFlt(PARAM_NUM ParamNum, s32fp ParamVal);
   PARAM_NUM NumFromString(const char *name);
   PARAM_NUM NumFromId(uint32_t id);
   const Attributes *GetAttrib(PARAM_NUM ParamNum);
   int IsParam(PARAM_NUM ParamNum);
   void LoadDefaults();
   void SetFlagsRaw(PARAM_NUM param, uint8_t rawFlags);
   void SetFlag(PARAM_NUM param, PARAM_FLAG flag);
   void ClearFlag(PARAM_NUM param, PARAM_FLAG flag);
   PARAM_FLAG GetFlag(PARAM_NUM param);
}

//User defined callback
extern void parm_Change(Param::PARAM_NUM ParamNum);

#endif //PARAM_H_INCLUDED
