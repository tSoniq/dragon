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

/** DSP platform specific definitions.
 *
 *  This file contains a baseline collection of macros and functions to support DSP classes and functions
 *  in a fairly platform agnostic manner.
 *
 *  Copyright (c) 2021 tSoniq. All Rights Reserved.
 */
#pragma once


// Import C99 standard integer types (int8_t etc).
#include <cstdint>
#include <cfloat>
#include <climits>



// Conditional compilation control macros:
//
//      DSP_CORTEX_M4           If true, the build target is an ARM Cortex M4
//      DSP_ARM_THUMB2          If true, the build target is using the Thumb2 instruction set.
//      DSP_ARM_AARCH32         If true, the build target is using the 32 bit (v7) ARM instruction set.
//      DSP_ARM_AARCH64         If true, the build target is using the 64 bit (v8/9( ARM instruction set.
//      DSP_INTEL_X64           If true, the build target is X86_64.
//
#if !defined(DSP_CORTEX_M4)
#if (ARM_MATH_CM4)
#define DSP_CORTEX_M4           (1)
#else
#define DSP_CORTEX_M4           (0)
#endif
#endif

#if !defined(DSP_ARM_THUMB2)
#if (ARM_MATH_CM4)
#define DSP_ARM_THUMB2          (1)
#else
#define DSP_ARM_THUMB2          (0)
#endif
#endif

#if !defined(DSP_ARM_AARCH32)
#define DSP_ARM_AARCH32         (0)
#endif

#if !defined(DSP_ARM_AARCH64)
#define DSP_ARM_AARCH64         (0)
#endif

#if !defined(DSP_INTEL_X64)
#define DSP_INTEL_X64           (0)
#endif



// Numeric format information.
//
//      DSP_IEEE_FLOAT32            true if the 32 bit float format is IEEE compliant.
//      DSP_IEEE_FLOAT64            true if the 64 bit float format is IEEE compliant.
//      DSP_LITTLE_ENDIAN           true if compiling for little endian integer system.
//      DSP_BIG_ENDIAN              true if compiling for a big-endian integer system.
//
#if !defined(DSP_IEEE_FLOAT32)
#define DSP_IEEE_FLOAT32        (1)
#endif

#if !defined(DSP_IEEE_FLOAT64)
#define DSP_IEEE_FLOAT64        (1)
#endif

#if !defined(DSP_LITTLE_ENDIAN)
#define DSP_LITTLE_ENDIAN       (1)
#endif

#if !defined(DSP_BIG_ENDIAN)
#define DSP_BIG_ENDIAN          (!(DSP_LITTLE_ENDIAN))
#endif



// Macros to support common compiler and/or build-target specific optimisations.
//
//      DSP_INLINE              Use in place of the inline keyword to indicate that inlining should be a priority.
//      DSP_NOALIAS             Prefix a function definition to indicate that no memory changes (other than arguments) are local only.
//      __restrict              Indicates that a pointer is not aliased.
//
#if defined(__clang__)
#define DSP_INLINE      inline __attribute__((always_inline))
#define DSP_NOALIAS     __attribute__ ((noalias))
#elif defined(__GNUC__)
#define DSP_INLINE      inline __attribute__((optimize("Ofast"),always_inline))
#define DSP_NOALIAS     __attribute__ ((pure))
#elif _MSC_VER
#define DSP_INLINE      inline
#define NOALIAS __declspec (noalias)
#else
#define DSP_INLINE      inline
#define DSP_NOALIAS
#endif



// Macros for support assert.
//
//      DSP_ASSERT              Maps to C assert() if DSP_USE_CASSERT() is true.
//
#if !defined(DSP_USE_CASSERT)
#define DSP_USE_CASSERT (0)
#endif

#if DSP_USE_CASSERT
#include <cassert>
#define DSP_ASSERT(_x)  do { assert(_x); } while (false)
#else
#define DSP_ASSERT(_x)  do { (void)(_x); } while (false)
#endif



// Macros to support branch hints.
//
//  if (DSP_EXPECT(condition))
//  {
//      // normal case code here
//  }
//  else
//  {
//      // exceptional case(s)
//  }
//
#if defined(__GNUC__) || defined(__clang__)
#define DSP_EXPECT(x)   __builtin_expect(!!(x), 1)
#else
#define DSP_EXPECT(x)   (!!(x))
#endif

