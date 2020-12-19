#ifndef MY_FP_H_INCLUDED
#define MY_FP_H_INCLUDED

#include <stdint.h>

#define FRAC_DIGITS 5

#ifndef CST_DIGITS
#define CST_DIGITS FRAC_DIGITS
#endif

#define FRAC_FAC (1 << CST_DIGITS)

#define CST_CONVERT(a) ((a) << (CST_DIGITS - FRAC_DIGITS))
#define CST_ICONVERT(a) ((a) >> (CST_DIGITS - FRAC_DIGITS))

#define UTOA_FRACDEC 100
#define FP_DECIMALS 2

#define FP_FROMINT(a) ((s32fp)((a) << CST_DIGITS))
#define FP_TOINT(a)   ((s32fp)((a) >> CST_DIGITS))
#define FP_FROMFLT(a) ((s32fp)((a) * FRAC_FAC))

#define FP_MUL(a, b) (((a) * (b)) >> CST_DIGITS)
#define FP_DIV(a, b) (((a) << CST_DIGITS) / (b))

typedef uint32_t u32fp;
typedef int32_t s32fp;
typedef int16_t s16fp;
typedef uint16_t u16fp;

#ifdef __cplusplus
extern "C"
{
#endif

char* fp_itoa(char * buf, s32fp a);
s32fp fp_atoi(const char *str);
u32fp fp_sqrt(u32fp rad);
s32fp fp_ln(unsigned int x);

#ifdef __cplusplus
}
#endif

#endif
