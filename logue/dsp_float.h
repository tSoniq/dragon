//  Copyright 2021 Mark Moore.  https://tsoniq.com
//
//  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following
//  conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
//  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
//  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
//  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/** DSP 32bit floating point support functions.
 *
 *  Copyright (c) 2021 tSoniq. All Rights Reserved.
 */
#pragma once

#include "dsp_platform.h"

namespace dsp {

    /** @fn             template <unsigned fbits> static int32_t floatToS32(float n)
     *  @brief          Cast a float to an int32 with specified number of fractional bits, rounding towards zero.
     *  @tparam fbits   The number of fractional integer bits in the output.
     *  @param  n       The float value.
     *  @return         The corresponding int32 value. Undefined if f exceeds the output range.
     *
     *  Examples:
     *
     *      int32_t a = floatToS32<0>(-10.50);      // a = -10
     *      int32_t b = floatToS32<0>(10.50);       // b = 10
     *      int32_t c = floatToS32<1>(12.50);       // c = 25
     *      int32_t d = floatToS32<31>(-2.0);       // d = undefined
     *      int32_t e = floatToS32<31>(2.0);        // e = undefined
     *      int32_t y = floatToS32<31>(x);          // x:[-1, +1) => y:[0x80000000, 0x7fffffff], undefined elsewhere.
     */


    /** @fn             template <unsigned fbits> static uint32_t floatToU32<>(float n)
     *  @brief          Cast a float to an uint32 with specified number of fractional bits, rounding towards zero.
     *  @tparam fbits   The number of fractional integer bits in the output.
     *  @param  n       The float value.
     *  @return         The corresponding uint32 value. Undefined if f exceeds the output range.
     *
     *  Examples:
     *
     *      uint32_t a = floatToU32<0>(-10.50);     // a = undefined
     *      uint32_t b = floatToU32<0>(10.50);      // a = 10
     *      uint32_t c = floatToU32<1>(12.50);      // a = 25
     *      uint32_t y = floatToU32<32>(x);         // x:[0, +1) => y:[0, 0xffffffff], undefined elsewhere.
     */


    /** @fn             template <unsigned fbits> static int32_t floatToS32_saturating(float n)
     *  @brief          Cast a float to an int32 with specified number of fractional bits, rounding towards zero, with saturation.
     *  @tparam fbits   The number of fractional integer bits in the output.
     *  @param  n       The float value.
     *  @return         The corresponding saturated int32 value.
     *
     *  Examples:
     *
     *      int32_t a = floatToS32_saturating<0>(-10.50);       // a = -10
     *      int32_t b = floatToS32_saturating<0>(10.50);        // b = 10
     *      int32_t c = floatToS32_saturating<1>(12.50);        // c = 25
     *      int32_t d = floatToS32_saturating<31>(-2.0);        // d = INT32_MIN
     *      int32_t e = floatToS32_saturating<31>(2.0);         // e = INT32_MAX
     *      int32_t y = floatToS32_saturating<31>(x);           // y:[0x80000000, 0x7fffffff]
     */


    /** @fn             template <unsigned fbits> static uint32_t saturatingFloatToU32(float n)
     *  @brief          Cast a float to an int32 with specified number of fractional bits, rounding towards zero, with saturation.
     *  @tparam fbits   The number of fractional integer bits in the output.
     *  @param  n       The float value.
     *  @return         The corresponding saturated uint32 value.
     *
     *  Examples:
     *
     *      int32_t a = floatToU32_saturating<0>(-10.50);       // a = -10
     *      int32_t b = floatToU32_saturating<0>(10.50);        // b = 10
     *      int32_t c = floatToU32_saturating<1>(12.50);        // c = 25
     *      int32_t d = floatToU32_saturating<31>(-2.0);        // d = 0
     *      int32_t e = floatToU32_saturating<31>(2.0);         // e = UINT32_MAX
     *      int32_t y = floatToU32_saturating<31>(x);           // y:[0, 0xffffffff]
     */


#if DSP_CORTEX_M4
    // Template specialisations for Cortex M4.
    // Cortex M4's standard float conversion primitives are saturating. Unfortunately, gcc will not
    // generate these opcodes, so inline assembler is the only way to benefit.
    // NB: the _0 macros are because the assembler is too dumb to accept #0 for the fractional bit count.
#define DSP_CM4_FLOAT_TO_INTEGER(output, input, fbits, type32)  asm (                   \
            "vcvt." type32 ".f32 %[s_input], %[s_input], %[k_fbits]\n"                  \
            "vmov                %[r_output], %[s_input]\n"                             \
            : [r_output] "=r" (output), [s_input] "=t" (input) : "1" (input), [k_fbits] "n" (fbits))

#define DSP_CM4_INTEGER_TO_FLOAT(output, input, fbits, type32)  asm (                   \
            "vmov                %[s_output], %[r_input]\n"                             \
            "vcvt.f32." type32 " %[s_output], %[s_output], %[k_fbits]\n"                \
            : [s_output] "=t" (output) : [r_input] "r" (input), [k_fbits] "n" (fbits))

#define DSP_CM4_FLOAT_TO_INTEGER_0(output, input, type32)  asm (                        \
            "vcvt." type32 ".f32 %[s_input], %[s_input]\n"                              \
            "vmov                %[r_output], %[s_input]\n"                             \
            : [r_output] "=r" (output), [s_input] "=t" (input) : "1" (input))

#define DSP_CM4_INTEGER_TO_FLOAT_0(output, input, type32)  asm (                        \
            "vmov                %[s_output], %[r_input]\n"                             \
            "vcvt.f32." type32 " %[s_output], %[s_output]\n"                            \
            : [s_output] "=t" (output) : [r_input] "r" (input))

    template <unsigned fbits> DSP_INLINE static float    s32ToFloat(int32_t n) { float result; DSP_CM4_INTEGER_TO_FLOAT(result, n, fbits, "s32"); return result; }
    template <unsigned fbits> DSP_INLINE static float    u32ToFloat(uint32_t n) { float result; DSP_CM4_INTEGER_TO_FLOAT(result, n, fbits, "u32"); return result; }
    template <unsigned fbits> DSP_INLINE static int32_t  floatToS32(float n) { int32_t result; DSP_CM4_FLOAT_TO_INTEGER(result, n, fbits, "s32"); return result; }
    template <unsigned fbits> DSP_INLINE static uint32_t floatToU32(float n) { uint32_t result; DSP_CM4_FLOAT_TO_INTEGER(result, n, fbits, "u32"); return result; }
    template <unsigned fbits> DSP_INLINE static int32_t  floatToS32_saturating(float n) { return floatToS32<fbits>(n); }
    template <unsigned fbits> DSP_INLINE static uint32_t floatToU32_saturating(float n) { return floatToU32<fbits>(n); }

    template <> float    DSP_INLINE s32ToFloat<0>(int32_t n) { float result; DSP_CM4_INTEGER_TO_FLOAT_0(result, n, "s32"); return result; }
    template <> float    DSP_INLINE u32ToFloat<0>(uint32_t n) { float result; DSP_CM4_INTEGER_TO_FLOAT_0(result, n, "u32"); return result; }
    template <> int32_t  DSP_INLINE floatToS32<0>(float n) { int32_t result; DSP_CM4_FLOAT_TO_INTEGER_0(result, n, "s32"); return result; }
    template <> uint32_t DSP_INLINE floatToU32<0>(float n) { uint32_t result; DSP_CM4_FLOAT_TO_INTEGER_0(result, n, "u32"); return result; }

#else   // Platform independent float<->int conversions.

    template <unsigned fbits> DSP_INLINE static float    s32ToFloat(int32_t n) { return float(n) * (1.0f / float(1ull << fbits)); }
    template <unsigned fbits> DSP_INLINE static float    u32ToFloat(uint32_t n) { return float(n) * (1.0f / float(1ull << fbits)); }
    template <unsigned fbits> DSP_INLINE static int32_t  floatToS32(float n) { return int32_t(n * float(1ull << fbits)); }
    template <unsigned fbits> DSP_INLINE static uint32_t floatToU32(float n) { return uint32_t(n * float(1ull << fbits)); }
    template <unsigned fbits> DSP_INLINE static int32_t  floatToS32_saturating(float n)
    {
        if (n >= float(1ull << (31-fbits))) return INT32_MAX;
        else if (n <= -float(1ull << (31-fbits))) return INT32_MIN;
        else return floatToS32<fbits>(n);
    }
    template <unsigned fbits> DSP_INLINE static uint32_t  floatToU32_saturating(float n)
    {
        if (n >= float(1ull << (32-fbits))) return UINT32_MAX;
        else if (n <= 0.0f) return 0;
        else return floatToU32<fbits>(n);
    }

#endif



    /** Fast multiplication of a float by a integral power of two.
     *
     *  @param x    Initial value.
     *  @param y    The power of two to scale by.
     *  @return     x * 2^y
     *  @warning    If overflow or underflow occurs, the result will be junk. In particular, underflow will
     *              result in a garbage result rather than something more helpful, like zero. Use this method
     *              only where the numeric ranges are well known.
     */
    DSP_INLINE static float floatScaleByPow2(float x, int y)
    {
        union { float f; uint32_t i; } xu;
        xu.f = x;
        xu.i += uint32_t(y << 23);      // Assumes IEEE float-32 format.
        return xu.f;
    }




}   // namespace



