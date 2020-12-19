/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2012 Johannes Huebner <contact@johanneshuebner.com>
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
 /** @addtogroup G_sine Sine wave generation
  * @{
 */
#include "sine_core.h"

#define SINTAB_ARGDIGITS 11
#define SINTAB_ENTRIES  (1 << SINTAB_ARGDIGITS)
/* Value range of sine lookup table */
#define SINTAB_MAX      (1 << BITS)
#define BRAD_PI         (1 << (BITS - 1))

#define PHASE_SHIFT90   ((uint32_t)(     SINLU_ONEREV / 4))
#define PHASE_SHIFT120  ((uint32_t)(     SINLU_ONEREV / 3))
#define PHASE_SHIFT240  ((uint32_t)(2 * (SINLU_ONEREV / 3)))

uint32_t SineCore::minPulse = 0;
uint32_t SineCore::ampl = 0;
const int16_t SineCore::SinTab[] = { SINTAB };/* sine LUT */
const uint16_t SineCore::ZERO_OFFSET = SINTAB_MAX / 2;
const int SineCore::BITS = 16;
const uint16_t SineCore::MAXAMP = 37813;
uint32_t SineCore::DutyCycles[3];

/** Calculate the next dutycyles.
  * This function is meant to be called by your timer interrupt handler
  */
void SineCore::Calc(uint16_t angle)
{
    int32_t Ofs;
    uint32_t Idx;

    int32_t sine[3];

    /* 1. Calculate sine */
    sine[0] = SineLookup(angle);
    sine[1] = SineLookup((angle + PHASE_SHIFT120) & 0xFFFF);
    sine[2] = SineLookup((angle + PHASE_SHIFT240) & 0xFFFF);

    for (Idx = 0; Idx < 3; Idx++)
    {
       /* 2. Set desired amplitude  */
       sine[Idx] = MultiplyAmplitude(ampl, sine[Idx]);
    }

    /* 3. Calculate the offset of SVPWM */
    Ofs = CalcSVPWMOffset(sine[0], sine[1], sine[2]);

    for (Idx = 0; Idx < 3; Idx++)
    {
       /* 4. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
       sine[Idx] -= Ofs;
       /* Shift above 0 */
       DutyCycles[Idx] = sine[Idx] + ZERO_OFFSET;
       /* Short pulse supression */
       if (DutyCycles[Idx] < minPulse)
       {
          DutyCycles[Idx] = 0U;
       }
       else if (DutyCycles[Idx] > (SINTAB_MAX - minPulse))
       {
          DutyCycles[Idx] = SINTAB_MAX;
       }
    }
}

s32fp SineCore::Sine(uint16_t angle)
{
   return SineLookup(angle);
}

s32fp SineCore::Cosine(uint16_t angle)
{
   return SineLookup((PHASE_SHIFT90 + angle) & 0xFFFF);
}

//Found here: http://www.coranac.com/documents/arctangent/
uint16_t SineCore::Atan2(int32_t x, int32_t y)
{
   if(y==0)
      return (x>=0 ? 0 : BRAD_PI);

   static const int fixShift = 15;
   int  phi = 0, t, t2, dphi;

   if (y < 0)
   {
      x = -x;
      y = -y;
      phi += 4;
   }
   if (x <= 0)
   {
      int temp = x;
      x = y;
      y = -temp;
      phi += 2;
   }
   if (x <= y)
   {
      int temp = y - x;
      x = x + y;
      y = temp;
      phi += 1;
   }

   phi *= BRAD_PI/4;

   t= (y << fixShift) / x;
   t2= -t*t>>fixShift;

   dphi= 0x0470;
   dphi= 0x1029 + (t2*dphi>>fixShift);
   dphi= 0x1F0B + (t2*dphi>>fixShift);
   dphi= 0x364C + (t2*dphi>>fixShift);
   dphi= 0xA2FC + (t2*dphi>>fixShift);
   dphi= dphi*t>>fixShift;

   return phi + ((dphi+2)>>2);
}

/** Set amplitude of the synthesized sine wave */
void SineCore::SetAmp(uint32_t amp /**< amplitude in digit. Largest value is 37813 */)
{
   ampl = amp;
}

uint32_t SineCore::GetAmp()
{
   return ampl;
}

/** Sets the minimum pulse width in normalized digits.
  * @post duty cylcles shorter than minWidth are supressed, both on the negative and the positive pulse
  */
void SineCore::SetMinPulseWidth(uint32_t minWidth)
{
   minPulse = minWidth;
}

/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
int32_t SineCore::SineLookup(uint16_t Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINLU_ARGDIGITS - SINTAB_ARGDIGITS;
    return (int32_t)SinTab[Arg];
}

/* 0 = 0, 1 = 32767 */
int32_t SineCore::MultiplyAmplitude(uint16_t Amplitude, int32_t Baseval)
{
    int32_t temp = (int32_t)((uint32_t)Amplitude * Baseval);
    /* Divide by 32768 */
    /* -> Allow overmodulation, for SVPWM or FTPWM */
    temp >>= (BITS - 1);
    return temp;
}


int32_t SineCore::CalcSVPWMOffset(int32_t a, int32_t b, int32_t c)
{
    int32_t Minimum = min(min(a, b), c);
    int32_t Maximum = max(max(a, b), c);
    int32_t Offset = Minimum + Maximum;

    return (Offset >> 1);
}

int32_t SineCore::min(int32_t a, int32_t b)
{
   return (a <= b)?a:b;
}

int32_t SineCore::max(int32_t a, int32_t b)
{
   return (a >= b)?a:b;
}

/** @} */
