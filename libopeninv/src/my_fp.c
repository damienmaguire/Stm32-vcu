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
#include "my_string.h"
#include "my_fp.h"

#define FRAC_MASK ((1 << FRAC_DIGITS) - 1)

static s32fp log2_approx(s32fp x, int loopLimit);

char* fp_itoa(char * buf, s32fp a)
{
   int sign = a < 0?-1:1;
   int32_t nat = (sign * a) >> FRAC_DIGITS;
   uint32_t frac = ((UTOA_FRACDEC * ((sign * a) & FRAC_MASK))) >> FRAC_DIGITS;
   char *p = buf;
   if (sign < 0)
   {
      *p = '-';
      p++;
   }
   p += my_ltoa(p, nat, 10);
   *p = '.';
   p++;
   for (uint32_t dec = UTOA_FRACDEC / 10; dec > 1; dec /= 10)
   {
      if ((frac / dec) == 0)
      {
         *p = '0';
         p++;
      }
   }
   my_ltoa(p, frac, 10);
   return buf;
}

s32fp fp_atoi(const char *str)
{
   int nat = 0;
   int frac = 0;
   int div = 10;
   int sign = 1;
   if ('-' == *str)
   {
      sign = -1;
      str++;
   }
   for (; *str >= '0' && *str <= '9'; str++)
   {
      nat *= 10;
      nat += *str - '0';
   }
   if (*str != 0)
   {
      for (str++; *str >= '0' && *str <= '9'; str++)
      {
         frac += FP_FROMINT(*str - '0') / div;
         div *= 10;
      }
   }

   return sign * (FP_FROMINT(nat) + frac);
}

u32fp fp_sqrt(u32fp rad)
{
   u32fp sqrt = rad >> (rad<1000?4:8); //Starting value for newton iteration
   u32fp sqrtl;
   sqrt = sqrt>FP_FROMINT(1)?sqrt:FP_FROMINT(1); //Must be > 0

   do {
      sqrtl = sqrt;
      sqrt = (sqrt + FP_DIV(rad, sqrt)) >> 1;
   } while ((sqrtl - sqrt) > 1);

   return sqrt;
}

s32fp fp_ln(unsigned int x)
{
   int n = 0;
   const s32fp ln2 = FP_FROMFLT(0.6931471806);

   if (x == 0)
   {
      return -1;
   }
   else
   { //count leading zeros
      uint32_t mask = 0xFFFFFFFF;
      for (int i = 16; i > 0; i /= 2)
      {
         mask <<= i;
         if ((x & mask) == 0)
         {
            n += i;
            x <<= i;
         }
      }
   }

   s32fp ln = FP_FROMINT(31 - n);
   x >>= 32 - FRAC_DIGITS - 1; //will result in fixed point number in [1,2)
   ln += log2_approx(x, 5);
   ln = FP_MUL(ln2, ln);
   return ln;
}

static s32fp log2_approx(s32fp x, int loopLimit)
{
   int m = 0;
   s32fp result = 0;

   if (loopLimit == 0) return FP_FROMINT(1);
   if (x == FP_FROMINT(1)) return 0;

   while (x < FP_FROMINT(2))
   {
      x = FP_MUL(x, x);
      m++;
   }
   s32fp p = FRAC_FAC >> m;
   result = FP_MUL(p, FP_FROMINT(1) + log2_approx(x / 2, loopLimit - 1));

   return result;
}
