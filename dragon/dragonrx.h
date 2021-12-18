/** Dragon communication receiver.
 *
 *  This header is *not* suitable for embedded use. It is typically used by debug
 *  tools designed to receive output from a target.
 */
#pragma once

#if USE_ACCELERATE_FRAMEWORK
#include <Accelerate/Accelerate.h>
#endif

#include <cstdint>
#include "dsp_platform.h"
#include "dsp_float.h"
#include <cstdlib>
#include <vector>
#include "dsp_dragon.h"


namespace dsp
{

    /** Hamming 7,4 decoding lookup table.
     */
    static constexpr uint8_t kDragonDecodeLookup[128] =
    {
        0x00, 0x00, 0x00, 0x0c, 0x00, 0x0a, 0x07, 0x0e, 0x00, 0x09, 0x04, 0x0e, 0x02, 0x0e, 0x0e, 0x0e,
        0x00, 0x09, 0x07, 0x0d, 0x07, 0x0b, 0x07, 0x07, 0x09, 0x09, 0x05, 0x09, 0x03, 0x09, 0x07, 0x0e,
        0x00, 0x0a, 0x04, 0x0d, 0x0a, 0x0a, 0x06, 0x0a, 0x04, 0x08, 0x04, 0x04, 0x03, 0x0a, 0x04, 0x0e,
        0x01, 0x0d, 0x0d, 0x0d, 0x03, 0x0a, 0x07, 0x0d, 0x03, 0x09, 0x04, 0x0d, 0x03, 0x03, 0x03, 0x0f,
        0x00, 0x0c, 0x0c, 0x0c, 0x02, 0x0b, 0x06, 0x0c, 0x02, 0x08, 0x05, 0x0c, 0x02, 0x02, 0x02, 0x0e,
        0x01, 0x0b, 0x05, 0x0c, 0x0b, 0x0b, 0x07, 0x0b, 0x05, 0x09, 0x05, 0x05, 0x02, 0x0b, 0x05, 0x0f,
        0x01, 0x08, 0x06, 0x0c, 0x06, 0x0a, 0x06, 0x06, 0x08, 0x08, 0x04, 0x08, 0x02, 0x08, 0x06, 0x0f,
        0x01, 0x01, 0x01, 0x0d, 0x01, 0x0b, 0x06, 0x0f, 0x01, 0x08, 0x05, 0x0f, 0x03, 0x0f, 0x0f, 0x0f
    };



    /** Decode a block of data, 8 bits per symbol.
     *
     *  @param  data        Returns the data.
     *  @param  syms        The input symbols.
     */
    static DSP_INLINE void DragonDecode8(uint8_t data[4], const uint8_t syms[7])
    {
        const auto h0 = dsp::DragonEB(syms[0], 7, 0) | dsp::DragonEB(syms[1], 7, 1) | dsp::DragonEB(syms[2], 7, 2) | dsp::DragonEB(syms[3], 7, 3) | dsp::DragonEB(syms[4], 7, 4) | dsp::DragonEB(syms[5], 7, 5) | dsp::DragonEB(syms[6], 7, 6);
        const auto h1 = dsp::DragonEB(syms[0], 6, 0) | dsp::DragonEB(syms[1], 6, 1) | dsp::DragonEB(syms[2], 6, 2) | dsp::DragonEB(syms[3], 6, 3) | dsp::DragonEB(syms[4], 6, 4) | dsp::DragonEB(syms[5], 6, 5) | dsp::DragonEB(syms[6], 6, 6);
        const auto h2 = dsp::DragonEB(syms[0], 5, 0) | dsp::DragonEB(syms[1], 5, 1) | dsp::DragonEB(syms[2], 5, 2) | dsp::DragonEB(syms[3], 5, 3) | dsp::DragonEB(syms[4], 5, 4) | dsp::DragonEB(syms[5], 5, 5) | dsp::DragonEB(syms[6], 5, 6);
        const auto h3 = dsp::DragonEB(syms[0], 4, 0) | dsp::DragonEB(syms[1], 4, 1) | dsp::DragonEB(syms[2], 4, 2) | dsp::DragonEB(syms[3], 4, 3) | dsp::DragonEB(syms[4], 4, 4) | dsp::DragonEB(syms[5], 4, 5) | dsp::DragonEB(syms[6], 4, 6);
        const auto h4 = dsp::DragonEB(syms[0], 3, 0) | dsp::DragonEB(syms[1], 3, 1) | dsp::DragonEB(syms[2], 3, 2) | dsp::DragonEB(syms[3], 3, 3) | dsp::DragonEB(syms[4], 3, 4) | dsp::DragonEB(syms[5], 3, 5) | dsp::DragonEB(syms[6], 3, 6);
        const auto h5 = dsp::DragonEB(syms[0], 2, 0) | dsp::DragonEB(syms[1], 2, 1) | dsp::DragonEB(syms[2], 2, 2) | dsp::DragonEB(syms[3], 2, 3) | dsp::DragonEB(syms[4], 2, 4) | dsp::DragonEB(syms[5], 2, 5) | dsp::DragonEB(syms[6], 2, 6);
        const auto h6 = dsp::DragonEB(syms[0], 1, 0) | dsp::DragonEB(syms[1], 1, 1) | dsp::DragonEB(syms[2], 1, 2) | dsp::DragonEB(syms[3], 1, 3) | dsp::DragonEB(syms[4], 1, 4) | dsp::DragonEB(syms[5], 1, 5) | dsp::DragonEB(syms[6], 1, 6);
        const auto h7 = dsp::DragonEB(syms[0], 0, 0) | dsp::DragonEB(syms[1], 0, 1) | dsp::DragonEB(syms[2], 0, 2) | dsp::DragonEB(syms[3], 0, 3) | dsp::DragonEB(syms[4], 0, 4) | dsp::DragonEB(syms[5], 0, 5) | dsp::DragonEB(syms[6], 0, 6);

        const auto n0 = kDragonDecodeLookup[h0];
        const auto n1 = kDragonDecodeLookup[h1];
        const auto n2 = kDragonDecodeLookup[h2];
        const auto n3 = kDragonDecodeLookup[h3];
        const auto n4 = kDragonDecodeLookup[h4];
        const auto n5 = kDragonDecodeLookup[h5];
        const auto n6 = kDragonDecodeLookup[h6];
        const auto n7 = kDragonDecodeLookup[h7];

        data[0] = (n1 << 4) | n0;
        data[1] = (n3 << 4) | n2;
        data[2] = (n5 << 4) | n4;
        data[3] = (n7 << 4) | n6;
    }


    /** Decode a block of data, 4 bits per symbol.
     *
     *  @param  data        Returns the data.
     *  @param  syms        The input symbols.
     */
    static DSP_INLINE void DragonDecode4(uint8_t data[4], const uint8_t syms[14])
    {
        const auto h0 = dsp::DragonEB(syms[0], 3, 0) | dsp::DragonEB(syms[2], 3, 1) | dsp::DragonEB(syms[4], 3, 2) | dsp::DragonEB(syms[6], 3, 3) | dsp::DragonEB(syms[8], 3, 4) | dsp::DragonEB(syms[10], 3, 5) | dsp::DragonEB(syms[12], 3, 6);
        const auto h1 = dsp::DragonEB(syms[0], 2, 0) | dsp::DragonEB(syms[2], 2, 1) | dsp::DragonEB(syms[4], 2, 2) | dsp::DragonEB(syms[6], 2, 3) | dsp::DragonEB(syms[8], 2, 4) | dsp::DragonEB(syms[10], 2, 5) | dsp::DragonEB(syms[12], 2, 6);
        const auto h2 = dsp::DragonEB(syms[0], 1, 0) | dsp::DragonEB(syms[2], 1, 1) | dsp::DragonEB(syms[4], 1, 2) | dsp::DragonEB(syms[6], 1, 3) | dsp::DragonEB(syms[8], 1, 4) | dsp::DragonEB(syms[10], 1, 5) | dsp::DragonEB(syms[12], 1, 6);
        const auto h3 = dsp::DragonEB(syms[0], 0, 0) | dsp::DragonEB(syms[2], 0, 1) | dsp::DragonEB(syms[4], 0, 2) | dsp::DragonEB(syms[6], 0, 3) | dsp::DragonEB(syms[8], 0, 4) | dsp::DragonEB(syms[10], 0, 5) | dsp::DragonEB(syms[12], 0, 6);
        const auto h4 = dsp::DragonEB(syms[1], 3, 0) | dsp::DragonEB(syms[3], 3, 1) | dsp::DragonEB(syms[5], 3, 2) | dsp::DragonEB(syms[7], 3, 3) | dsp::DragonEB(syms[9], 3, 4) | dsp::DragonEB(syms[11], 3, 5) | dsp::DragonEB(syms[13], 3, 6);
        const auto h5 = dsp::DragonEB(syms[1], 2, 0) | dsp::DragonEB(syms[3], 2, 1) | dsp::DragonEB(syms[5], 2, 2) | dsp::DragonEB(syms[7], 2, 3) | dsp::DragonEB(syms[9], 2, 4) | dsp::DragonEB(syms[11], 2, 5) | dsp::DragonEB(syms[13], 2, 6);
        const auto h6 = dsp::DragonEB(syms[1], 1, 0) | dsp::DragonEB(syms[3], 1, 1) | dsp::DragonEB(syms[5], 1, 2) | dsp::DragonEB(syms[7], 1, 3) | dsp::DragonEB(syms[9], 1, 4) | dsp::DragonEB(syms[11], 1, 5) | dsp::DragonEB(syms[13], 1, 6);
        const auto h7 = dsp::DragonEB(syms[1], 0, 0) | dsp::DragonEB(syms[3], 0, 1) | dsp::DragonEB(syms[5], 0, 2) | dsp::DragonEB(syms[7], 0, 3) | dsp::DragonEB(syms[9], 0, 4) | dsp::DragonEB(syms[11], 0, 5) | dsp::DragonEB(syms[13], 0, 6);

        const auto n0 = kDragonDecodeLookup[h0];
        const auto n1 = kDragonDecodeLookup[h1];
        const auto n2 = kDragonDecodeLookup[h2];
        const auto n3 = kDragonDecodeLookup[h3];
        const auto n4 = kDragonDecodeLookup[h4];
        const auto n5 = kDragonDecodeLookup[h5];
        const auto n6 = kDragonDecodeLookup[h6];
        const auto n7 = kDragonDecodeLookup[h7];

        data[0] = (n1 << 4) | n0;
        data[1] = (n3 << 4) | n2;
        data[2] = (n5 << 4) | n4;
        data[3] = (n7 << 4) | n6;
    }


    /** Decode a block of data, 2 bits per symbol.
     *
     *  @param  data        Returns the data.
     *  @param  syms        The input symbols.
     */
    static DSP_INLINE void DragonDecode2(uint8_t data[4], const uint8_t syms[28])
    {
        const auto h0 = dsp::DragonEB(syms[0], 1, 0) | dsp::DragonEB(syms[4], 1, 1) | dsp::DragonEB(syms[8], 1, 2) | dsp::DragonEB(syms[12], 1, 3) | dsp::DragonEB(syms[16], 1, 4) | dsp::DragonEB(syms[20], 1, 5) | dsp::DragonEB(syms[24], 1, 6);
        const auto h1 = dsp::DragonEB(syms[0], 0, 0) | dsp::DragonEB(syms[4], 0, 1) | dsp::DragonEB(syms[8], 0, 2) | dsp::DragonEB(syms[12], 0, 3) | dsp::DragonEB(syms[16], 0, 4) | dsp::DragonEB(syms[20], 0, 5) | dsp::DragonEB(syms[24], 0, 6);
        const auto h2 = dsp::DragonEB(syms[1], 1, 0) | dsp::DragonEB(syms[5], 1, 1) | dsp::DragonEB(syms[9], 1, 2) | dsp::DragonEB(syms[13], 1, 3) | dsp::DragonEB(syms[17], 1, 4) | dsp::DragonEB(syms[21], 1, 5) | dsp::DragonEB(syms[25], 1, 6);
        const auto h3 = dsp::DragonEB(syms[1], 0, 0) | dsp::DragonEB(syms[5], 0, 1) | dsp::DragonEB(syms[9], 0, 2) | dsp::DragonEB(syms[13], 0, 3) | dsp::DragonEB(syms[17], 0, 4) | dsp::DragonEB(syms[21], 0, 5) | dsp::DragonEB(syms[25], 0, 6);
        const auto h4 = dsp::DragonEB(syms[2], 1, 0) | dsp::DragonEB(syms[6], 1, 1) | dsp::DragonEB(syms[10], 1, 2) | dsp::DragonEB(syms[14], 1, 3) | dsp::DragonEB(syms[18], 1, 4) | dsp::DragonEB(syms[22], 1, 5) | dsp::DragonEB(syms[26], 1, 6);
        const auto h5 = dsp::DragonEB(syms[2], 0, 0) | dsp::DragonEB(syms[6], 0, 1) | dsp::DragonEB(syms[10], 0, 2) | dsp::DragonEB(syms[14], 0, 3) | dsp::DragonEB(syms[18], 0, 4) | dsp::DragonEB(syms[22], 0, 5) | dsp::DragonEB(syms[26], 0, 6);
        const auto h6 = dsp::DragonEB(syms[3], 1, 0) | dsp::DragonEB(syms[7], 1, 1) | dsp::DragonEB(syms[11], 1, 2) | dsp::DragonEB(syms[15], 1, 3) | dsp::DragonEB(syms[19], 1, 4) | dsp::DragonEB(syms[23], 1, 5) | dsp::DragonEB(syms[27], 1, 6);
        const auto h7 = dsp::DragonEB(syms[3], 0, 0) | dsp::DragonEB(syms[7], 0, 1) | dsp::DragonEB(syms[11], 0, 2) | dsp::DragonEB(syms[15], 0, 3) | dsp::DragonEB(syms[19], 0, 4) | dsp::DragonEB(syms[23], 0, 5) | dsp::DragonEB(syms[27], 0, 6);

        const auto n0 = kDragonDecodeLookup[h0];
        const auto n1 = kDragonDecodeLookup[h1];
        const auto n2 = kDragonDecodeLookup[h2];
        const auto n3 = kDragonDecodeLookup[h3];
        const auto n4 = kDragonDecodeLookup[h4];
        const auto n5 = kDragonDecodeLookup[h5];
        const auto n6 = kDragonDecodeLookup[h6];
        const auto n7 = kDragonDecodeLookup[h7];

        data[0] = (n1 << 4) | n0;
        data[1] = (n3 << 4) | n2;
        data[2] = (n5 << 4) | n4;
        data[3] = (n7 << 4) | n6;
    }


    /** Decode a block of data, 1 bit per symbol.
     *
     *  @param  data        Returns the data.
     *  @param  syms        The input symbols.
     */
    static DSP_INLINE void DragonDecode1(uint8_t data[4], const uint8_t syms[56])
    {
        const auto h0 = dsp::DragonEB(syms[0], 0, 0) | dsp::DragonEB(syms[8], 0, 1) | dsp::DragonEB(syms[16], 0, 2) | dsp::DragonEB(syms[24], 0, 3) | dsp::DragonEB(syms[32], 0, 4) | dsp::DragonEB(syms[40], 0, 5) | dsp::DragonEB(syms[48], 0, 6);
        const auto h1 = dsp::DragonEB(syms[1], 0, 0) | dsp::DragonEB(syms[9], 0, 1) | dsp::DragonEB(syms[17], 0, 2) | dsp::DragonEB(syms[25], 0, 3) | dsp::DragonEB(syms[33], 0, 4) | dsp::DragonEB(syms[41], 0, 5) | dsp::DragonEB(syms[49], 0, 6);
        const auto h2 = dsp::DragonEB(syms[2], 0, 0) | dsp::DragonEB(syms[10], 0, 1) | dsp::DragonEB(syms[18], 0, 2) | dsp::DragonEB(syms[26], 0, 3) | dsp::DragonEB(syms[34], 0, 4) | dsp::DragonEB(syms[42], 0, 5) | dsp::DragonEB(syms[50], 0, 6);
        const auto h3 = dsp::DragonEB(syms[3], 0, 0) | dsp::DragonEB(syms[11], 0, 1) | dsp::DragonEB(syms[19], 0, 2) | dsp::DragonEB(syms[27], 0, 3) | dsp::DragonEB(syms[35], 0, 4) | dsp::DragonEB(syms[43], 0, 5) | dsp::DragonEB(syms[51], 0, 6);
        const auto h4 = dsp::DragonEB(syms[4], 0, 0) | dsp::DragonEB(syms[12], 0, 1) | dsp::DragonEB(syms[20], 0, 2) | dsp::DragonEB(syms[28], 0, 3) | dsp::DragonEB(syms[36], 0, 4) | dsp::DragonEB(syms[44], 0, 5) | dsp::DragonEB(syms[52], 0, 6);
        const auto h5 = dsp::DragonEB(syms[5], 0, 0) | dsp::DragonEB(syms[13], 0, 1) | dsp::DragonEB(syms[21], 0, 2) | dsp::DragonEB(syms[29], 0, 3) | dsp::DragonEB(syms[37], 0, 4) | dsp::DragonEB(syms[45], 0, 5) | dsp::DragonEB(syms[53], 0, 6);
        const auto h6 = dsp::DragonEB(syms[6], 0, 0) | dsp::DragonEB(syms[14], 0, 1) | dsp::DragonEB(syms[22], 0, 2) | dsp::DragonEB(syms[30], 0, 3) | dsp::DragonEB(syms[38], 0, 4) | dsp::DragonEB(syms[46], 0, 5) | dsp::DragonEB(syms[54], 0, 6);
        const auto h7 = dsp::DragonEB(syms[7], 0, 0) | dsp::DragonEB(syms[15], 0, 1) | dsp::DragonEB(syms[23], 0, 2) | dsp::DragonEB(syms[31], 0, 3) | dsp::DragonEB(syms[39], 0, 4) | dsp::DragonEB(syms[47], 0, 5) | dsp::DragonEB(syms[55], 0, 6);

        const auto n0 = kDragonDecodeLookup[h0];
        const auto n1 = kDragonDecodeLookup[h1];
        const auto n2 = kDragonDecodeLookup[h2];
        const auto n3 = kDragonDecodeLookup[h3];
        const auto n4 = kDragonDecodeLookup[h4];
        const auto n5 = kDragonDecodeLookup[h5];
        const auto n6 = kDragonDecodeLookup[h6];
        const auto n7 = kDragonDecodeLookup[h7];

        data[0] = (n1 << 4) | n0;
        data[1] = (n3 << 4) | n2;
        data[2] = (n5 << 4) | n4;
        data[3] = (n7 << 4) | n6;
    }


    static void DSP_INLINE DragonDecode8(uint32_t& data, const uint8_t syms[7]) { return DragonDecode8((uint8_t*)&data, syms); }
    static void DSP_INLINE DragonDecode4(uint32_t& data, const uint8_t syms[14]) { return DragonDecode4((uint8_t*)&data, syms); }
    static void DSP_INLINE DragonDecode2(uint32_t& data, const uint8_t syms[28]) { return DragonDecode2((uint8_t*)&data, syms); }
    static void DSP_INLINE DragonDecode1(uint32_t& data, const uint8_t syms[56]) { return DragonDecode1((uint8_t*)&data, syms); }


    /** Calculate the mean for a set of samples.
     *
     *  @param  x       The sample data (kDragonSamplesPerChip in length).
     *  @return         The mean value.
     */
    double DragonRxChipMean(const double* x)
    {
    #if USE_ACCELERATE_FRAMEWORK
        double result;
        vDSP_meanvD(x, 1, &result, dsp::kDragonSamplesPerChip);
        return result;
    #else
        // Calculate the means.
        auto px = x;
        auto pxe = px + dsp::kDragonSamplesPerChip;
        double sum = 0.0;
        do {
            sum += (*px++);
        } while (px != pxe);
        return sum / dsp::kDragonSamplesPerChip;
    #endif
    }


    /** Calculate the deviation for a set of samples.
     *
     *  @param  x       The sample data.
     *  @param  m       The mean of the samples.
     *  @return         The mean value.
     */
    double DragonRxChipDev(const double* x, double m)
    {
        // Calculate the means.
        auto px = x;
        auto pxe = px + dsp::kDragonSamplesPerChip;
        double sum = 0.0;
        do {
            double d = (*px++) - m;
            sum += (d * d);
        } while (px != pxe);
        return sum;
    }


    /** Calculate a pattern detection.
     *
     *  @param  a       The input sample data. kDragonSamplesPerChip in length.
     *  @param  b       The reference pattern sample data. kDragonSamplesPerChip in length.
     *  @param  mb      The mean value of b. See DragonRxMean().
     *  @param  sdb     The std deviation measure for b. See DragonRxDev().
     *  @return         The detection score. 1.0 => perfect detection.
     */
    static inline double DragonRxChipDetect(const double* a, const double* b, double mb, double sdb)
    {
    #if USE_ACCELERATE_FRAMEWORK
        double worka[dsp::kDragonSamplesPerChip];
        double workb[dsp::kDragonSamplesPerChip];

        double ma = DragonRxChipMean(a);
        double sub = -ma;
        vDSP_vsaddD(a, 1, &sub, worka, 1, dsp::kDragonSamplesPerChip);      // subtract mean
        sub = -mb;
        vDSP_vsaddD(b, 1, &sub, workb, 1, dsp::kDragonSamplesPerChip);      // subtract mean

        double sda = 0.0;
        vDSP_dotprD(worka, 1, worka, 1, &sda, dsp::kDragonSamplesPerChip);  // a.a
        double cor = 0.0;
        vDSP_dotprD(worka, 1, workb, 1, &cor, dsp::kDragonSamplesPerChip);  // a.b

        double s = sqrt(sda * sdb);
        double result = 0.0;
        if (s > 0.000001) result = cor / s;
        return result;
    #else
        double ma = DragonRxChipMean(a);
        double sda = 0.0;
        double cor = 0.0;

        // Calculate the correlation and corrected std deviations.
        auto pa = a;
        auto pb = b;
        auto pae = pa + dsp::kDragonSamplesPerChip;
        while (pa != pae)
        {
            auto p = (*pa++) - ma;
            auto q = (*pb++) - mb;
            sda += (p * p);
            cor += (p * q);
        }

        double s = sqrt(sda * sdb);
        double result = 0.0;
        if (s > 0.000001) result = cor / s;
        return result;
    #endif
    }


    /** Search an array of samples for a specific sample sequence and returns the best fit.
     *
     *  @param  score           Returns the detection score.
     *  @param  location        Returns the detected location.
     *  @param  pattern         The sample pattern to detect. kDragonSamplesPerChip in length.
     *  @param  patternMean     Precomputed mean for the pattern.
     *  @param  patternDev      Precomputed deviation for the pattern.
     *  @param  sampleData      The sample data to search.
     *  @param  sampleCount     The number of samples provided. kDragonSamplesPerChip or more.
     *
     *  Note that the returned score and location are just the best found in the search window. If there is no
     *  actual pattern present, the result will be essentially random. The returned score is normalised according
     *  to the input sample data, with a value of +1 => perfect correlation.
     */
    static inline void DragonRxFindPattern(double& score, unsigned& location, const double* pattern, double patternMean, double patternDev, const double* sampleData, unsigned sampleCount)
    {
        double bestScore = 0.0;
        unsigned bestLocation = 0;
        if (sampleCount >= dsp::kDragonSamplesPerChip)
        {
            const unsigned searchLength = sampleCount - dsp::kDragonSamplesPerChip + 1;
            for (unsigned i = 0; i != searchLength; ++i)
            {
                double thisScore = DragonRxChipDetect(sampleData + i, pattern, patternMean, patternDev);
                thisScore = fabs(thisScore);    // allow anti-correlations.
                if (thisScore > bestScore)
                {
                    bestLocation = i;
                    bestScore = thisScore;
                }
            }
        }
        score = bestScore;
        location = bestLocation;
    }




    /** Dragon receive class. Feed it sample data, and detects and returns packets.
     *
     *  This class is *not* suited for embedded use. It relies on the C++ STL and buffers
     *  huge numbers of samples. None of the signal processing is remotely optimised.
     *
     *  To use, create an instance of this class, then:
     *
     *      call addSampleData() to add incoming samples to the internal sample buffer.
     *      call getNextPacket() to retrieve the next packet.
     *
     *  Internally, sample data is buffered until getNextPacket() is called. Clients should
     *  call getNextPacket() frequently to prevent the buffer memory growing indefinitely.
     */
    class DragonRx
    {
    public:

        /** Constructor.
         */
        DragonRx()
        {
            uint8_t sequence[1] = { 0 };
            int32_t chip[dsp::kDragonSamplesPerChip];
            dsp::DragonChipper chipper;

            // Pre-render the preamble chips and convert to unnormalised double.
            for (unsigned symbol = 0; symbol != dsp::kDragonPreambleSequenceLength; ++symbol)
            {
                sequence[0] = uint8_t(symbol);
                chipper.start(dsp::kDragonSeedPreamble, sequence, 1);
                chipper.render(chip, dsp::kDragonSamplesPerChip);
                for (unsigned i = 0; i != dsp::kDragonSamplesPerChip; ++i) m_preamble[symbol][i] = double(chip[i]);
                m_preambleMeans[symbol] = DragonRxChipMean(&(m_preamble[symbol][0]));
                m_preambleDevs[symbol] = DragonRxChipDev(&(m_preamble[symbol][0]), m_preambleMeans[symbol]);
            }

            // Pre-render the data chips.
            for (unsigned symbol = 0; symbol != dsp::kDragonSymbolCount; ++symbol)
            {
                sequence[0] = uint8_t(symbol);
                chipper.start(dsp::kDragonSeedData, sequence, 1);
                chipper.render(chip, dsp::kDragonSamplesPerChip);
                for (unsigned i = 0; i != dsp::kDragonSamplesPerChip; ++i) m_chips[symbol][i] = double(chip[i]);
                m_chipsMeans[symbol] = DragonRxChipMean(&(m_chips[symbol][0]));
                m_chipsDevs[symbol] = DragonRxChipDev(&(m_chips[symbol][0]), m_chipsMeans[symbol]);
            }
        }


        /** Reset all state.
         */
        void reset()
        {
            m_samples.clear();
        }


        size_t sampleBufferSize() const
        {
            return m_samples.size();
        }


        /** Add received samples to the internal buffer. Samples are implicitly removed from the buffer
         *  on calls to getNextPacket().
         *
         *  @param  samples     A pointer to the sample buffer.
         *  @param  count       The number of samples.
         */
        void addSampleData(const int32_t* samples, unsigned count)
        {
            if (0 == count) return;

            auto oldSize = m_samples.size();
            auto newSize = oldSize + count;
            m_samples.resize(newSize);
            const int32_t* src = samples;
            double* dst = &m_samples[oldSize];
            double* end = &m_samples[newSize];
            do {
                *dst ++ = double(*src ++);
            } while (dst != end);
        }


        /** Add received samples to the internal buffer. Samples are implicitly removed from the buffer
         *  on calls to getNextPacket().
         *
         *  @param  samples     A pointer to the sample buffer.
         *  @param  count       The number of samples.
         */
        void addSampleData(const float* samples, unsigned count)
        {
            if (0 == count) return;

            auto oldSize = m_samples.size();
            auto newSize = oldSize + count;
            m_samples.resize(newSize);
            auto src = samples;
            auto end = src + count;
            auto dst = &m_samples[oldSize];
            while (src != end)
            {
                *dst ++ = double((*src ++) * double(1u << 31));
            }
        }


        /** Get the next packet, if any.
         *
         *  @param  packet      Returns the packet data.
         *  @param  meta        Returns the packet meta data field.
         *  @param  sequence    Returns the packet sequence number.
         *  @return             true for success, false if no packet was found.
         */
        int getNextPacket(std::vector<uint8_t>& packet, unsigned& meta, unsigned& sequence)
        {
            static constexpr unsigned kMaxChipsPerPacket = (dsp::kDragonPreambleSequenceLength + ((2 + dsp::kDragonMaxPayloadBlocks) * dsp::kDragonChipsPerBlock));
            static constexpr unsigned kMaxSamplesPerPacket = dsp::kDragonSamplesPerChip * kMaxChipsPerPacket;
            static constexpr unsigned kMinSampleCount = kMaxSamplesPerPacket + (kMaxSamplesPerPacket / 8);  // one packet plus allowance for some clock drift
            static constexpr unsigned kSamplesPerPass = (dsp::kDragonSamplesPerChip * 7) / 8;               // number of samples to window in searches.

            packet.clear();
            meta = 0;
            sequence = 0;

            bool didFind = false;
            while (true)
            {
                if (m_samples.size() < kMinSampleCount) return false;       // Not enough samples to justify searching yet.

                // Search for a preamble.
                unsigned next = 0;
                unsigned samplesToDiscard = kSamplesPerPass;
                if (findPreamble(next, kSamplesPerPass))
                {
                    memset(m_blocks, 0, sizeof m_blocks);
                    next = next - 1;    // offset for the seach window in loadBlocks()
                    samplesToDiscard = next;
                    loadBlock(m_blocks[0], next, next);
                    unsigned length = 0;
                    if (!dsp::DragonParseHeader(sequence, meta, length, m_blocks[0]))
                    {
                        printf("invalid header %08x / %08x: %u, %u, %u\n", unsigned(m_blocks[0]), unsigned(m_blocks[0]^dsp::kDragonHeaderScramble), sequence, meta, length);
                    }
                    else
                    {
                        //printf("valid header %08x / %08x: %u, %u, %u\n", unsigned(m_blocks[0]), unsigned(m_blocks[0]^kDragonHeaderScramble), sequence, meta, length);
                        unsigned blockCount = ((length + dsp::kDragonBlockSize - 1) / dsp::kDragonBlockSize) + 2;
                        uint32_t seed = m_blocks[0];
                        if (0 == seed) seed = 1;
                        dsp::DragonScrambler scrambler(seed);
                        for (unsigned blockIndex = 1; blockIndex != blockCount; ++blockIndex)
                        {
                            uint32_t block;
                            loadBlock(block, next, next);
                            m_blocks[blockIndex] = block ^ scrambler.next();
                        }
                        dsp::DragonChecksum csum;
                        for (unsigned blockIndex = 0; blockIndex != blockCount - 1; ++blockIndex)
                        {
                            csum.process32(m_blocks[blockIndex]);
                        }
                        uint32_t expectedChecksum = csum.checksum32();
                        bool checksumOk = (expectedChecksum == m_blocks[blockCount - 1]);
                        if (!checksumOk)
                        {
                            printf("checksum error %08x %08x: ", unsigned(expectedChecksum), unsigned(m_blocks[blockCount - 1]));
                            auto ptr = (const char*)&m_blocks[1];
                            auto end = ptr + length;
                            while (ptr != end)
                            {
                                char ch = *ptr ++;
                                if (!isprint(ch)) ch = '.';
                                printf("%c", ch);
                            }
                            printf("\n");
                        }
                        packet.resize(length);
                        memcpy(&packet[0], &m_blocks[1], length);

                        if (checksumOk) samplesToDiscard = next - (dsp::kDragonSamplesPerChip / 2); // Successful packet receive, so drop the whole packet
                        else samplesToDiscard = dsp::kDragonSamplesPerChip / 2;                     // Unsuccessful, so drop just the preamble
                        didFind = checksumOk;
                    }
                }
                discardSamples(samplesToDiscard);
                if (didFind) break;
            }

            return didFind;
        }

    private:

        /** Drop samples from the start of the buffer.
         */
        void discardSamples(size_t n)
        {
            if (m_samples.size() < n) m_samples.clear();
            else m_samples.erase(m_samples.begin(), m_samples.begin() + n);
        }


        /** Search for a preamble.
         *
         *  @param  next        Returns the location of the next set of samples following the preamble.
         *  @param  limit       The maximum search distance.
         *  @return             true for success, false otherwise.
         */
        bool findPreamble(unsigned& next, unsigned limit)
        {
            // We look for individual chips, allowing up to 1 sample jitter between each.
            bool result = false;
            double bestScore = 0.0;
            unsigned bestLocation = 0;

            if (m_samples.size() >= limit + ((2 + dsp::kDragonSamplesPerChip) * dsp::kDragonPreambleSequenceLength))
            {
                // Loop for each sample location in the search limit.
                for (unsigned offset = 0; offset != limit; ++offset)
                {
                    // Calculate a score for this location.
                    double scores[dsp::kDragonPreambleSequenceLength];
                    unsigned locations[dsp::kDragonPreambleSequenceLength];
                    unsigned x = offset;
                    for (unsigned index = 0; index != dsp::kDragonPreambleSequenceLength; ++index)
                    {
                        const double* pattern = &(m_preamble[index][0]);
                        const double* samples = &m_samples[x];
                        double patternMean = m_preambleDevs[index];
                        double patternDev = m_preambleDevs[index];
                        DragonRxFindPattern(scores[index], locations[index], pattern, patternMean, patternDev, samples, dsp::kDragonSamplesPerChip + 2);
                        x += dsp::kDragonSamplesPerChip;
                    }
                    double meanScore = 0.0;
                    unsigned meanLocation = 0;
                    for (unsigned index = 0; index != dsp::kDragonPreambleSequenceLength; ++index)
                    {
                        meanScore += scores[index];
                        meanLocation += locations[index];
                    }
                    meanScore *= (1.0 / dsp::kDragonPreambleSequenceLength);
                    meanLocation = (meanLocation + (dsp::kDragonPreambleSequenceLength / 2)) / dsp::kDragonPreambleSequenceLength;
                    if (meanScore > bestScore)
                    {
                        bestScore = meanScore;
                        bestLocation = x + meanLocation;
                        if (meanScore >= 0.6)
                        {
                            //printf("find preamble: match %.5lf  %.5lf  %.5lf  %.5lf -->   %.5lf / %u\n", scores[0], scores[1], scores[2], scores[3], meanScore, meanLocation);
                            result = true;
                        }
                        // Do not break here. If there is low frequency filtering, we may match a better peak nearby.
                    }
                }

                //printf("find preamble: best score %.5lf\n", bestScore);
            }

            next = bestLocation;
            return result;
        }


        /** Load a block of data.
         *
         *  @param  block       Returns the 32 bit data value.
         *  @param  next        Returns the start location for the next block.
         *  @param  start       The start location for this block.
         *  @return             true for success, false if there were insufficient samples.
         */
        bool loadBlock(uint32_t& block, unsigned& next, unsigned start)
        {
            if (m_samples.size() < start + ((2 + dsp::kDragonSamplesPerChip) * dsp::kDragonChipsPerBlock))
            {
                // Insufficient sample data
                block = 0;
                return false;
            }

            // We look for individual chips, allowing for some sample jitter between each.
            unsigned location = start;
            uint8_t symbols[dsp::kDragonChipsPerBlock];
            for (unsigned chipIndex = 0; chipIndex != dsp::kDragonChipsPerBlock; ++chipIndex)
            {
                // Search for the best fitting symbol.
                unsigned bestSymbol = 0;
                double bestScore = 0.0;
                for (unsigned symbol = 0; symbol != dsp::kDragonSymbolCount; ++symbol)
                {
                    const double* pattern = &(m_chips[symbol][0]);
                    double patternMean = m_chipsDevs[symbol];
                    double patternDev = m_chipsDevs[symbol];
                    double thisScore = 0.0;
                    unsigned thisLocation = 0;
                    DragonRxFindPattern(thisScore, thisLocation, pattern, patternMean, patternDev, &m_samples[location], dsp::kDragonSamplesPerChip + 4);
                    if (thisScore > bestScore)
                    {
                        bestScore = thisScore;
                        bestSymbol = symbol;
    //                        printf("match %.5lf  %02x  %u\n", bestScore, bestSymbol, thisLocation);
                    }
                }
    //                printf("match %.5lf  %02x  %u\n", bestScore, bestSymbol, bestLocation);
                symbols[chipIndex] = bestSymbol;
                location += dsp::kDragonSamplesPerChip;
            }
            next = location;

            // Decode to generate the block.
            if (dsp::kDragonBitsPerSymbol == 1)      DragonDecode1(block, symbols);
            else if (dsp::kDragonBitsPerSymbol == 2) DragonDecode2(block, symbols);
            else if (dsp::kDragonBitsPerSymbol == 4) DragonDecode4(block, symbols);
            else if (dsp::kDragonBitsPerSymbol == 8) DragonDecode8(block, symbols);

            return true;
        }

    private:

        std::vector<double> m_samples;
        uint32_t m_blocks[2 + dsp::kDragonMaxPayloadBlocks];
        double m_preamble[dsp::kDragonPreambleSequenceLength][dsp::kDragonSamplesPerChip];
        double m_chips[dsp::kDragonSymbolCount][dsp::kDragonSamplesPerChip];
        double m_preambleMeans[dsp::kDragonPreambleSequenceLength];
        double m_preambleDevs[dsp::kDragonPreambleSequenceLength];
        double m_chipsMeans[dsp::kDragonSymbolCount];
        double m_chipsDevs[dsp::kDragonSymbolCount];
    };

}   // namespace
