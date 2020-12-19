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
#define CST_DIGITS 15
#include "my_fp.h"
#include "my_math.h"
#include "foc.h"
#include "sine_core.h"

#define SQRT3 FP_FROMFLT(1.732050807568877293527446315059)
#define R1 FP_FROMFLT(0.03)
#define S1 FP_FROMFLT(0.15)
#define R2 FP_FROMFLT(0.5)
#define S2 FP_FROMFLT(0.5)
#define S3 FP_FROMFLT(1)
#define RADSTART(x) x < R1 ? S1 : (x < R2 ? S2 : S3)

static const s32fp fluxLinkage = FP_FROMFLT(0.09);
static const s32fp fluxLinkage2 = FP_MUL(fluxLinkage, fluxLinkage);
static const s32fp lqminusldSquaredBs10 = FP_FROMFLT(0.01722); //additional 10-bit left shift because otherwise it can't be represented
static const s32fp lqminusld = FP_FROMFLT(0.0058);
static const u32fp sqrt3 = SQRT3;
static const s32fp sqrt3inv1 = FP_FROMFLT(0.57735026919); //1/sqrt(3)
static const s32fp sqrt3inv2 = 2*sqrt3inv1; //2/sqrt(2)
static const s32fp zeroOffset = FP_FROMINT(1);
static const int32_t modMax = FP_DIV(FP_FROMINT(2U), sqrt3);
static const int32_t modMaxPow2 = modMax * modMax;
static int32_t minPulse = 1000;
static int32_t maxPulse = FP_FROMINT(2) - 1000;

s32fp FOC::id;
s32fp FOC::iq;
s32fp FOC::DutyCycles[3];

/** @brief Transform current to rotor system using Clarke and Park transformation
  * @post flux producing (id) and torque producing (iq) current are written
  *       to FOC::id and FOC::iq
  */
void FOC::ParkClarke(s32fp il1, s32fp il2, uint16_t angle)
{
   s32fp sin = SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);
   //Clarke transformation
   s32fp ia = il1;
   s32fp ib = FP_MUL(sqrt3inv1, il1) + FP_MUL(sqrt3inv2, il2);
   //Park transformation
   id = FP_MUL(cos, ia) + FP_MUL(sin, ib);
   iq = FP_MUL(cos, ib) - FP_MUL(sin, ia);
}

/** \brief distribute motor current in magnetic torque and reluctance torque with the least total current
 *
 * \param is int32_t total motor current
 * \param[out] idref int32_t& resulting direct current reference
 * \param[out] iqref int32_t& resulting quadrature current reference
 * \return void
 *
 */
void FOC::Mtpa(int32_t is, int32_t& idref, int32_t& iqref)
{
   int32_t isSquared = is * is;
   int32_t sign = is < 0 ? -1 : 1;
   s32fp term1 = fpsqrt(fluxLinkage2 + ((lqminusldSquaredBs10 * isSquared) >> 10));
   idref = FP_TOINT(FP_DIV(fluxLinkage - term1, lqminusld));
   iqref = sign * (int32_t)sqrt(isSquared - idref * idref);
}

int32_t FOC::GetQLimit(int32_t ud)
{
   return sqrt(modMaxPow2 - ud * ud);
}

/** \brief Returns the resulting modulation index from uq and ud
 *
 * \param ud d voltage modulation index
 * \param uq q voltage modulation index
 * \return sqrt(ud²+uq²)
 *
 */
int32_t FOC::GetTotalVoltage(int32_t ud, int32_t uq)
{
   return sqrt((uint32_t)(ud * ud) + (uint32_t)(uq * uq));
}

/** \brief Calculate duty cycles for generating ud and uq at given angle
 *
 * \param ud int32_t direct voltage
 * \param uq int32_t quadrature voltage
 * \param angle uint16_t rotor angle
 * \return void
 *
 */
void FOC::InvParkClarke(int32_t ud, int32_t uq, uint16_t angle)
{
   s32fp sin = SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);

   //Inverse Park transformation
   s32fp ua = (cos * ud - sin * uq) >> CST_DIGITS;
   s32fp ub = (cos * uq + sin * ud) >> CST_DIGITS;
   //Inverse Clarke transformation
   DutyCycles[0] = ua;
   DutyCycles[1] = (-ua + FP_MUL(SQRT3, ub)) / 2;
   DutyCycles[2] = (-ua - FP_MUL(SQRT3, ub)) / 2;

   int32_t offset = SineCore::CalcSVPWMOffset(DutyCycles[0], DutyCycles[1], DutyCycles[2]);

   for (int i = 0; i < 3; i++)
   {
      /* subtract it from all 3 phases -> no difference in phase-to-phase voltage */
      DutyCycles[i] -= offset;
      /* Shift above 0 */
      DutyCycles[i] += zeroOffset;
      /* Short pulse suppression */
      if (DutyCycles[i] < minPulse)
      {
         DutyCycles[i] = 0U;
      }
      else if (DutyCycles[i] > maxPulse)
      {
         DutyCycles[i] = FP_FROMINT(2);
      }
   }
}

int32_t FOC::GetMaximumModulationIndex()
{
   return modMax;
}

uint32_t FOC::sqrt(uint32_t rad)
{
   uint32_t radshift = (rad < 10000 ? 5 : (rad < 10000000 ? 9 : (rad < 1000000000 ? 13 : 15)));
   uint32_t sqrt = (rad >> radshift) + 1; //Starting value for newton iteration
   uint32_t sqrtl;

   do {
      sqrtl = sqrt;
      sqrt = (sqrt + rad / sqrt) / 2;
   } while ((sqrtl - sqrt) > 1);

   return sqrt;
}

u32fp FOC::fpsqrt(u32fp rad)
{
   u32fp sqrt = RADSTART(rad);
   u32fp sqrtl;

   do {
      sqrtl = sqrt;
      sqrt = (sqrt + FP_DIV(rad, sqrt)) >> 1;
   } while ((sqrtl - sqrt) > 1);

   return sqrt;
}

