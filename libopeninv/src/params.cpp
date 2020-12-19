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

#include "params.h"
#include "my_string.h"

namespace Param
{

#define PARAM_ENTRY(category, name, unit, min, max, def, id) { category, #name, unit, FP_FROMFLT(min), FP_FROMFLT(max), FP_FROMFLT(def), id },
#define VALUE_ENTRY(name, unit, id) { 0, #name, unit, 0, 0, 0, id },
static const Attributes attribs[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

#define PARAM_ENTRY(category, name, unit, min, max, def, id) FP_FROMFLT(def),
#define VALUE_ENTRY(name, unit, id) 0,
static s32fp values[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

#define PARAM_ENTRY(category, name, unit, min, max, def, id) 0,
#define VALUE_ENTRY(name, unit, id) 0,
static uint8_t flags[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef VALUE_ENTRY

/**
* Set a parameter
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
* @return 0 if set ok, -1 if ParamVal outside of allowed range
*/
int Set(PARAM_NUM ParamNum, s32fp ParamVal)
{
    char res = -1;

    if (ParamVal >= attribs[ParamNum].min && ParamVal <= attribs[ParamNum].max)
    {
        values[ParamNum] = ParamVal;
        parm_Change(ParamNum);
        res = 0;
    }
    return res;
}

/**
* Get a parameters fixed point value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
s32fp Get(PARAM_NUM ParamNum)
{
    return values[ParamNum];
}

/**
* Get a parameters unscaled digit value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
int GetInt(PARAM_NUM ParamNum)
{
    return FP_TOINT(values[ParamNum]);
}

/**
* Get a parameters boolean value, 1.00=True
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
bool GetBool(PARAM_NUM ParamNum)
{
    return FP_TOINT(values[ParamNum]) == 1;
}

/**
* Set a parameters digit value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void SetInt(PARAM_NUM ParamNum, int ParamVal)
{
   values[ParamNum] = FP_FROMINT(ParamVal);
}

/**
* Set a parameters fixed point value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void SetFlt(PARAM_NUM ParamNum, s32fp ParamVal)
{
   values[ParamNum] = ParamVal;
}

/**
* Get the paramater index from a parameter name
*
* @param[in] name Parameters name
* @return Parameter index if found, PARAM_INVALID otherwise
*/
PARAM_NUM NumFromString(const char *name)
{
    PARAM_NUM paramNum = PARAM_INVALID;
    const Attributes *pCurAtr = attribs;

    for (int i = 0; i < PARAM_LAST; i++, pCurAtr++)
    {
         if (0 == my_strcmp(pCurAtr->name, name))
         {
             paramNum = (PARAM_NUM)i;
             break;
         }
    }
    return paramNum;
}

/**
* Get the paramater index from a parameters unique id
*
* @param[in] id Parameters unique id
* @return Parameter index if found, PARAM_INVALID otherwise
*/
PARAM_NUM NumFromId(uint32_t id)
{
    PARAM_NUM paramNum = PARAM_INVALID;
    const Attributes *pCurAtr = attribs;

    for (int i = 0; i < PARAM_LAST; i++, pCurAtr++)
    {
         if (pCurAtr->id == id)
         {
             paramNum = (PARAM_NUM)i;
             break;
         }
    }
    return paramNum;
}

/**
* Get the parameter attributes
*
* @param[in] ParamNum Parameter index
* @return Parameter attributes
*/
const Attributes *GetAttrib(PARAM_NUM ParamNum)
{
    return &attribs[ParamNum];
}

/** Find out if ParamNum is a parameter or display value
 * @retval 1 it is a parameter
 * @retval 0 otherwise
 */
int IsParam(PARAM_NUM ParamNum)
{
   return attribs[ParamNum].min != attribs[ParamNum].max;
}

/** Load default values for all parameters */
void LoadDefaults()
{
   const Attributes *curAtr = attribs;

   for (int idx = 0; idx < PARAM_LAST; idx++, curAtr++)
   {
      if (curAtr->id > 0)
         SetFlt((PARAM_NUM)idx, curAtr->def);
   }
}

void SetFlagsRaw(PARAM_NUM param, uint8_t rawFlags)
{
   flags[param] = rawFlags;
}

void SetFlag(PARAM_NUM param, PARAM_FLAG flag)
{
   flags[param] |= (uint8_t)flag;
}

void ClearFlag(PARAM_NUM param, PARAM_FLAG flag)
{
   flags[param] &= (uint8_t)~flag;
}

PARAM_FLAG GetFlag(PARAM_NUM param)
{
   return (PARAM_FLAG)flags[param];
}

}
