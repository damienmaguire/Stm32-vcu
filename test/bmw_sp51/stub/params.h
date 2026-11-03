// Host stub of libopeninv params.h for the BMW_SP51 host check
// (test/bmw_sp51). Parameters are plain floats.
#pragma once
#include "param_prj.h"
#include <stdint.h>
namespace Param {
#define PARAM_ENTRY(category, name, unit, min, max, def, id) name,
#define TESTP_ENTRY(category, name, unit, min, max, def, id) name,
#define VALUE_ENTRY(name, unit, id) name,
typedef enum { PARAM_LIST PARAM_LAST } PARAM_NUM;
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY
extern float g[PARAM_LAST];
inline int GetInt(PARAM_NUM p) { return (int)g[p]; }
inline bool GetBool(PARAM_NUM p) { return g[p] != 0; }
inline float GetFloat(PARAM_NUM p) { return g[p]; }
inline void SetInt(PARAM_NUM p, int v) { g[p] = (float)v; }
inline void SetFloat(PARAM_NUM p, float v) { g[p] = v; }
} // namespace Param
