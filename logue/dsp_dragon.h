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

/** Dragon communication protocol shared definitions and transmit engine.
 *
 *  This file holds provides both the parameters of the Dragon protocol, plus the implementation
 *  of the transmitter. The transmitter is designed for a minimal memory and CPU overhead, suitable
 *  for embedded use.
 *
 *  For the receiver implementation, see dragonrx.h.
 *
 *  Concepts
 *  --------
 *
 *      chip            :=      A short fixed length set of samples, coding for a symbol value.
 *      sync-chip       :=      A unique chip that marks the start of a block
 *      symbol          :=      A numeric value corresponding to a specific chip.
 *      block           :=      A sequence of chips that after decoding generate a single 32 bit value.
 *      packet          :=      A packet structure formed from 3 or more blocks
 *
 *  Physical Layer Description
 *  --------------------------
 *
 *  The inaccurately named 'chip' corresponds to a fixed length block of samples. Each chip corresponds
 *  to a numeric symbol value. Chip samples are generated pseudo-randomly, with the seed value selecting
 *  the corresponding symbol value.
 *
 *  Data is encoded in blocks of 32 bits, using Hamming 7,4 with interleaving. This results in 56 bits
 *  of symbol data, which is then mapped to chips at 1, 2, 4 or 8 bits/symbol.
 *
 *  A packet transmission comprises the following:
 *
 *      packet          :=      <preamble><header><payload><checksum>
 *      preamble        :=      Predefined chip sequence, used for detection/channel estimation.
 *      header          :=      32 bit header word, describing the packet.
 *      payload         :=      A sequence of 1 or more 32 bit words.
 *      checksum        :=      A 32 bit checksum, covering the header and payload.
 *
 *  Scrambling is performed on the header (fixed hash) and payload (seeded from the header value). For
 *  implementation efficiency, the scrambling is performed before Hamming coding and interleaving.
 *
 *  The header contains the following fields:
 *
 *      Bits 24-31      Header checksum (8 bits).
 *      Bits 16-23      Packet sequence number (8 bits).
 *      Bits 12-15      Application meta-data (4 bits).
 *      Bits 10-11      Reserved. Set zero.
 *      Bits 0-9        Number of bytes in the payload - 1. Allows lengths from 1 to 1024 bytes.
 *
 *  The application meta-data field is typically used to signal the format of data in the packet. It
 *  is not interpreted by the logical layer.
 *
 *  The checksum uses a 32 bit FNV1a algorithm. This offers performance close to a standard CRC32, but
 *  without the computational cost or requirement for a large lookup table.
 *
 *  Note that while the header implies a maximum packet size of 1k bytes, the implementation may be
 *  constrained to shorter packets in order to conserve memory (see kDragonMaxPayloadBlocks).
 */
#pragma once

#if DSP_KORG_LOGUE
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wimplicit"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#include "arm_math.h"
#include "osc_api.h"
#include "userosc.h"
#pragma GCC diagnostic pop
#endif


#include "dsp_platform.h"
#include "dsp_float.h"

namespace dsp {

    // Configurable parameters.
    //
    //  kDragonSamplesPerChip       Increase to improve SNR performance, but with reduced clock-rate discrepancy tolerance.
    //  kDragonBitsPerSymbol        Increase for faster data rates, but at expense of a higher error rate.
    //  kDragonMaxPayloadBlocks     Increase for larger packets, at the expense of increased memory usage.
    //  kDragonSeedSync             Selects the sync chip symbol set. Critical with smaller values of kDragonSamplesPerChip.
    //  kDragonSeedData             Selects the data chip symbol set. Critical with smaller values of kDragonSamplesPerChip.
    //  kDragonSeedHeader           Not critical. Set zero to disable scrambling of the header.
    //
    static constexpr unsigned kDragonSamplesPerChip     =   48;                     ///< The number of samples in a single LFSR burst.
    static constexpr unsigned kDragonBitsPerSymbol      =   4;                      ///< The number of bits per symbol (1, 2, 4 or 8).
    static constexpr unsigned kDragonMaxPayloadBlocks   =   32;                     ///< The maximum length of the payload, as number of 32 bit words.
    static constexpr uint32_t kDragonSeedPreamble       =   0xd9295542u;            ///< LFSR seed (preamble chipsequence).
    static constexpr uint32_t kDragonSeedData           =   0x7b882ebdu;            ///< LFSR seed (data chip sequence).
    static constexpr uint32_t kDragonHeaderScramble     =   0xed12cbe1u;            ///< Fixed scrambler code for the header.
    static constexpr double kDragonDetectorSyncThreshold =   4.0;                   ///< The threshold for sync chip detection (multiple of std deviation).
    static constexpr double kDragonDetectorDataThreshold =   2.0;                   ///< The threshold for data chip detection (multiple of std deviation).

    // Constants that can not be modified without corresponding code changes. You really do not want to change these.
    static constexpr unsigned kDragonBlockSize          =   4;                      ///< Number of bytes per block. Do not change this.
    static constexpr unsigned kDragonCodedBlockSize     =   7;                      ///< Number of bytes per coded payload block. Do not change this.

    // Derived parameters.
    static constexpr unsigned kDragonMaxPayloadBytes    =   kDragonMaxPayloadBlocks * 4;        ///< The maximum payload size, in bytes.
    static constexpr unsigned kDragonSymbolCount        =   (1u << kDragonBitsPerSymbol);       ///< The total number of symbols.
    static constexpr unsigned kDragonChipsPerBlock      =   (56 / kDragonBitsPerSymbol);        ///< The number of chips required to send one encoded 32 bit block, excluding sync.

    // Checks.
    static_assert(kDragonMaxPayloadBytes <= 1024, "Absolute limit implied by the header structure.");
    static_assert(kDragonBitsPerSymbol == 1 || kDragonBitsPerSymbol == 2 || kDragonBitsPerSymbol == 4 || kDragonBitsPerSymbol == 8, "Coding only supports 1, 2, 4 or 8");

    // The preamble sequence.
    static constexpr uint8_t kDragonPreambleSequence[] = { 0, 1, 2, 3 };
    static constexpr unsigned kDragonPreambleSequenceLength = sizeof kDragonPreambleSequence / sizeof kDragonPreambleSequence[0];



    /** The LFSR algorithm. This is used for all random sources (scrambling, preamble and payload).
     *
     *  @param  state       The current state (random value). On return this will have been updated.
     *  @return             The random value to use.
     *
     *  Note that setting the initial state value to zero will result in all random values being zero.
     *  This is exploited by DragonScrambler, so if changing this algorithm, also update the scrambler.
     *
     *  Do *not* modify this algorthim withouit regenerating the (very carefully chosen) chip seed values.
     *
     *  To use:
     *
     *      uint32_t state = <seed value>;
     *      uint32_t random0 = DragonRandom(&state);        // The first random value to use
     *      uint32_t random1 = DragonRanomd(&state);        // The second random value to use
     *      ...
     *
     *  For sample value generation, cast the returned value to int32_t.
     *
     *  The current implementation is based o a trivial 32 bit XOR based LFSR. This is not very random (one
     *  reason why the seeds have to be carefully selected), but it is very fast and requires only minimal
     *  state. See https://en.wikipedia.org/wiki/Xorshift for more information.
     */
    DSP_INLINE uint32_t DragonRandom(uint32_t* state)
    {
        auto a = *state;
        a ^= a << 13;
        a ^= a >> 17;
        a ^= a << 5;
        *state = a;
        return a;
    }




    /** The data scrambler. See dsp_com_dragoncoding.h to see how this is applied to the
     *  coded data stream. The intent is to avoid systematic failures.
     */
    class DragonScrambler
    {
    public:
        DragonScrambler() = delete;
        DragonScrambler(uint32_t seed) : m_state(seed) { }              ///< Constructor.
        void start(uint32_t seed) { m_state = seed; }                   ///< Reset the sequence. A value of zero disables scrambling.
        uint32_t next() { DragonRandom(&m_state); return m_state; }     ///< Generate the next 32 bit scramble code.
    private:
        uint32_t m_state;
    };


    /** The checksum algorithm. This is used to check integrity of data in a packet.
     *
     *  This is a 32bit FNV1a algorithm.
     */
    class DragonChecksum
    {
    public:
        DragonChecksum() : state(basis) { };                                        ///< Constructor.
        void start()                { state = basis; }                              ///< Start a new sequence.
        void process8(uint8_t n)    { state ^= n; state *= prime; }                 ///< Add a byte.
        void process32(uint32_t n)                                                  ///< Add a 32 bit word.
        {
            process8(uint8_t((n >> 0) & 0xffu));
            process8(uint8_t((n >> 8) & 0xffu));
            process8(uint8_t((n >> 16) & 0xffu));
            process8(uint8_t((n >> 24) & 0xffu));
        }
        uint32_t checksum32() const { return state; }                               ///< Return the current 32bit checksum.
        uint32_t checksum24() const { return (state ^ (state >> 8)) & 0xffffffu; }  ///< Return the current 24bit checksum.
        uint32_t checksum20() const { return (state ^ (state >> 12)) & 0xfffffu; }  ///< Return the current 20bit checksum.
        uint32_t checksum8() const  { return ((state ^ (state >> 8)) & 0xffu); }    ///< Return the current 8bit checksum.
    private:
        static constexpr uint32_t basis = 0x811C9DC5u;
        static constexpr uint32_t prime = 0x01000193u;
        uint32_t state;
    };


    /** Construct a header word.
     *
     *  @param  sequence        The packet sequence.
     *  @param  meta            The meta data.
     *  @param  length          The packet length, in bytes.
     *  @return                 The packet header.
     */
    static inline uint32_t DragonMakeHeader(unsigned sequence, unsigned meta, unsigned length)
    {
        uint32_t header = (((length - 1) & 0x3ffu) << 0) | ((meta & 0xfu) << 12) | ((sequence & 0xffu) << 16);
        DragonChecksum csum;
        csum.process8(uint8_t((header >> 0) & 0xffu));
        csum.process8(uint8_t((header >> 8) & 0xffu));
        csum.process8(uint8_t((header >> 16) & 0xffu));
        header = uint32_t(header | (csum.checksum8() << 24));
        header ^= kDragonHeaderScramble;
        return header;
    }


    /** Parse a header word.
     *
     *  @param  sequence        Returns the packet sequence.
     *  @param  meta            Returns the meta data.
     *  @param  length          Returns the packet length, in bytes.
     *  @param  header          The header to parse.
     *  @return                 true if the header checksum is valid, false if not.
     */
    static inline bool DragonParseHeader(unsigned& sequence, unsigned& meta, unsigned& length, uint32_t header)
    {
        header ^= kDragonHeaderScramble;
        sequence = (header >> 16) & 0xffu;
        meta = (header >> 12) & 0xfu;
        length = 1 + (header & 0x3ffu);
        DragonChecksum csum;
        csum.process8(uint8_t((header >> 0) & 0xffu));
        csum.process8(uint8_t((header >> 8) & 0xffu));
        csum.process8(uint8_t((header >> 16) & 0xffu));
        bool result = (csum.checksum8() == ((header >> 24) & 0xffu)) && (length <= kDragonMaxPayloadBytes);
        return result;
    }


    /** Hamming 7,4 encoding lookup table.
     */
    static constexpr uint8_t kDragonEncodeLookup[16] =
    {
        0x00, 0x70, 0x4c, 0x3c, 0x2a, 0x5a, 0x66, 0x16, 0x69, 0x19, 0x25, 0x55, 0x43, 0x33, 0x0f, 0x7f
    };



    /** Helper used to extract a bit from a 32 bit word to a new bit position.
     */
    static DSP_INLINE uint32_t DragonEB(uint32_t data, const unsigned srcBit, const unsigned dstBit)
    {
        if (srcBit == dstBit)       return uint32_t(data & (1u << srcBit));
        else if (srcBit > dstBit)   return uint32_t((data >> (srcBit - dstBit)) & (1u << dstBit));
        else                        return uint32_t((data << (dstBit - srcBit)) & (1u << dstBit));
    }


    /** Encode data, 8 bits per symbol.
     *
     *  @param  syms        Returns the symbols, one byte per symbol.
     *  @param  data        The data to encode. Fixed length, 4 bytes.
     */
    static DSP_INLINE void DragonEncode8(uint8_t syms[7], const uint8_t data[4])
    {
        const auto h0 = kDragonEncodeLookup[(data[0] >> 0) & 15];
        const auto h1 = kDragonEncodeLookup[(data[0] >> 4) & 15];
        const auto h2 = kDragonEncodeLookup[(data[1] >> 0) & 15];
        const auto h3 = kDragonEncodeLookup[(data[1] >> 4) & 15];
        const auto h4 = kDragonEncodeLookup[(data[2] >> 0) & 15];
        const auto h5 = kDragonEncodeLookup[(data[2] >> 4) & 15];
        const auto h6 = kDragonEncodeLookup[(data[3] >> 0) & 15];
        const auto h7 = kDragonEncodeLookup[(data[3] >> 4) & 15];
        syms[0]  = uint8_t(DragonEB(h0, 0, 7) | DragonEB(h1, 0, 6) | DragonEB(h2, 0, 5) | DragonEB(h3, 0, 4) | DragonEB(h4, 0, 3) | DragonEB(h5, 0, 2) | DragonEB(h6, 0, 1) | DragonEB(h7, 0, 0));
        syms[1]  = uint8_t(DragonEB(h0, 1, 7) | DragonEB(h1, 1, 6) | DragonEB(h2, 1, 5) | DragonEB(h3, 1, 4) | DragonEB(h4, 1, 3) | DragonEB(h5, 1, 2) | DragonEB(h6, 1, 1) | DragonEB(h7, 1, 0));
        syms[2]  = uint8_t(DragonEB(h0, 2, 7) | DragonEB(h1, 2, 6) | DragonEB(h2, 2, 5) | DragonEB(h3, 2, 4) | DragonEB(h4, 2, 3) | DragonEB(h5, 2, 2) | DragonEB(h6, 2, 1) | DragonEB(h7, 2, 0));
        syms[3]  = uint8_t(DragonEB(h0, 3, 7) | DragonEB(h1, 3, 6) | DragonEB(h2, 3, 5) | DragonEB(h3, 3, 4) | DragonEB(h4, 3, 3) | DragonEB(h5, 3, 2) | DragonEB(h6, 3, 1) | DragonEB(h7, 3, 0));
        syms[4]  = uint8_t(DragonEB(h0, 4, 7) | DragonEB(h1, 4, 6) | DragonEB(h2, 4, 5) | DragonEB(h3, 4, 4) | DragonEB(h4, 4, 3) | DragonEB(h5, 4, 2) | DragonEB(h6, 4, 1) | DragonEB(h7, 4, 0));
        syms[5]  = uint8_t(DragonEB(h0, 5, 7) | DragonEB(h1, 5, 6) | DragonEB(h2, 5, 5) | DragonEB(h3, 5, 4) | DragonEB(h4, 5, 3) | DragonEB(h5, 5, 2) | DragonEB(h6, 5, 1) | DragonEB(h7, 5, 0));
        syms[6]  = uint8_t(DragonEB(h0, 6, 7) | DragonEB(h1, 6, 6) | DragonEB(h2, 6, 5) | DragonEB(h3, 6, 4) | DragonEB(h4, 6, 3) | DragonEB(h5, 6, 2) | DragonEB(h6, 6, 1) | DragonEB(h7, 6, 0));
    }



    /** Encode data, 4 bits per symbol.
     *
     *  @param  syms        Returns the symbols, one byte per symbol.
     *  @param  data        The data to encode. Fixed length, 4 bytes.
     */
    static DSP_INLINE void DragonEncode4(uint8_t syms[14], const uint8_t data[4])
    {
        const auto h0 = kDragonEncodeLookup[(data[0] >> 0) & 15];
        const auto h1 = kDragonEncodeLookup[(data[0] >> 4) & 15];
        const auto h2 = kDragonEncodeLookup[(data[1] >> 0) & 15];
        const auto h3 = kDragonEncodeLookup[(data[1] >> 4) & 15];
        syms[0]  = uint8_t(DragonEB(h0, 0, 3) | DragonEB(h1, 0, 2) | DragonEB(h2, 0, 1) | DragonEB(h3, 0, 0));
        syms[2]  = uint8_t(DragonEB(h0, 1, 3) | DragonEB(h1, 1, 2) | DragonEB(h2, 1, 1) | DragonEB(h3, 1, 0));
        syms[4]  = uint8_t(DragonEB(h0, 2, 3) | DragonEB(h1, 2, 2) | DragonEB(h2, 2, 1) | DragonEB(h3, 2, 0));
        syms[6]  = uint8_t(DragonEB(h0, 3, 3) | DragonEB(h1, 3, 2) | DragonEB(h2, 3, 1) | DragonEB(h3, 3, 0));
        syms[8]  = uint8_t(DragonEB(h0, 4, 3) | DragonEB(h1, 4, 2) | DragonEB(h2, 4, 1) | DragonEB(h3, 4, 0));
        syms[10] = uint8_t(DragonEB(h0, 5, 3) | DragonEB(h1, 5, 2) | DragonEB(h2, 5, 1) | DragonEB(h3, 5, 0));
        syms[12] = uint8_t(DragonEB(h0, 6, 3) | DragonEB(h1, 6, 2) | DragonEB(h2, 6, 1) | DragonEB(h3, 6, 0));
        const auto h4 = kDragonEncodeLookup[(data[2] >> 0) & 15];
        const auto h5 = kDragonEncodeLookup[(data[2] >> 4) & 15];
        const auto h6 = kDragonEncodeLookup[(data[3] >> 0) & 15];
        const auto h7 = kDragonEncodeLookup[(data[3] >> 4) & 15];
        syms[1]  = uint8_t(DragonEB(h4, 0, 3) | DragonEB(h5, 0, 2) | DragonEB(h6, 0, 1) | DragonEB(h7, 0, 0));
        syms[3]  = uint8_t(DragonEB(h4, 1, 3) | DragonEB(h5, 1, 2) | DragonEB(h6, 1, 1) | DragonEB(h7, 1, 0));
        syms[5]  = uint8_t(DragonEB(h4, 2, 3) | DragonEB(h5, 2, 2) | DragonEB(h6, 2, 1) | DragonEB(h7, 2, 0));
        syms[7]  = uint8_t(DragonEB(h4, 3, 3) | DragonEB(h5, 3, 2) | DragonEB(h6, 3, 1) | DragonEB(h7, 3, 0));
        syms[9]  = uint8_t(DragonEB(h4, 4, 3) | DragonEB(h5, 4, 2) | DragonEB(h6, 4, 1) | DragonEB(h7, 4, 0));
        syms[11] = uint8_t(DragonEB(h4, 5, 3) | DragonEB(h5, 5, 2) | DragonEB(h6, 5, 1) | DragonEB(h7, 5, 0));
        syms[13] = uint8_t(DragonEB(h4, 6, 3) | DragonEB(h5, 6, 2) | DragonEB(h6, 6, 1) | DragonEB(h7, 6, 0));
    }


    /** Encode data, 2 bits per symbol.
     *
     *  @param  syms        Returns the symbols, one byte per symbol.
     *  @param  data        The data to encode. Fixed length, 4 bytes.
     */
    static DSP_INLINE void DragonEncode2(uint8_t syms[28], const uint8_t data[4])
    {
        const auto h0 = kDragonEncodeLookup[(data[0] >> 0) & 15];
        const auto h1 = kDragonEncodeLookup[(data[0] >> 4) & 15];
        syms[0]  = uint8_t(DragonEB(h0, 0, 1) | DragonEB(h1, 0, 0));
        syms[4]  = uint8_t(DragonEB(h0, 1, 1) | DragonEB(h1, 1, 0));
        syms[8]  = uint8_t(DragonEB(h0, 2, 1) | DragonEB(h1, 2, 0));
        syms[12] = uint8_t(DragonEB(h0, 3, 1) | DragonEB(h1, 3, 0));
        syms[16] = uint8_t(DragonEB(h0, 4, 1) | DragonEB(h1, 4, 0));
        syms[20] = uint8_t(DragonEB(h0, 5, 1) | DragonEB(h1, 5, 0));
        syms[24] = uint8_t(DragonEB(h0, 6, 1) | DragonEB(h1, 6, 0));
        const auto h2 = kDragonEncodeLookup[(data[1] >> 0) & 15];
        const auto h3 = kDragonEncodeLookup[(data[1] >> 4) & 15];
        syms[1]  = uint8_t(DragonEB(h2, 0, 1) | DragonEB(h3, 0, 0));
        syms[5]  = uint8_t(DragonEB(h2, 1, 1) | DragonEB(h3, 1, 0));
        syms[9]  = uint8_t(DragonEB(h2, 2, 1) | DragonEB(h3, 2, 0));
        syms[13] = uint8_t(DragonEB(h2, 3, 1) | DragonEB(h3, 3, 0));
        syms[17] = uint8_t(DragonEB(h2, 4, 1) | DragonEB(h3, 4, 0));
        syms[21] = uint8_t(DragonEB(h2, 5, 1) | DragonEB(h3, 5, 0));
        syms[25] = uint8_t(DragonEB(h2, 6, 1) | DragonEB(h3, 6, 0));
        const auto h4 = kDragonEncodeLookup[(data[2] >> 0) & 15];
        const auto h5 = kDragonEncodeLookup[(data[2] >> 4) & 15];
        syms[2]  = uint8_t(DragonEB(h4, 0, 1) | DragonEB(h5, 0, 0));
        syms[6]  = uint8_t(DragonEB(h4, 1, 1) | DragonEB(h5, 1, 0));
        syms[10] = uint8_t(DragonEB(h4, 2, 1) | DragonEB(h5, 2, 0));
        syms[14] = uint8_t(DragonEB(h4, 3, 1) | DragonEB(h5, 3, 0));
        syms[18] = uint8_t(DragonEB(h4, 4, 1) | DragonEB(h5, 4, 0));
        syms[22] = uint8_t(DragonEB(h4, 5, 1) | DragonEB(h5, 5, 0));
        syms[26] = uint8_t(DragonEB(h4, 6, 1) | DragonEB(h5, 6, 0));
        const auto h6 = kDragonEncodeLookup[(data[3] >> 0) & 15];
        const auto h7 = kDragonEncodeLookup[(data[3] >> 4) & 15];
        syms[3]  = uint8_t(DragonEB(h6, 0, 1) | DragonEB(h7, 0, 0));
        syms[7]  = uint8_t(DragonEB(h6, 1, 1) | DragonEB(h7, 1, 0));
        syms[11] = uint8_t(DragonEB(h6, 2, 1) | DragonEB(h7, 2, 0));
        syms[15] = uint8_t(DragonEB(h6, 3, 1) | DragonEB(h7, 3, 0));
        syms[19] = uint8_t(DragonEB(h6, 4, 1) | DragonEB(h7, 4, 0));
        syms[23] = uint8_t(DragonEB(h6, 5, 1) | DragonEB(h7, 5, 0));
        syms[27] = uint8_t(DragonEB(h6, 6, 1) | DragonEB(h7, 6, 0));
    }


    /** Encode data, 1 bit per symbol.
     *
     *  @param  syms        Returns the symbols, one byte per symbol.
     *  @param  data        The data to encode. Fixed length, 4 bytes.
     */
    static DSP_INLINE void DragonEncode1(uint8_t syms[56], const uint8_t data[4])
    {
        const auto h0 = kDragonEncodeLookup[(data[0] >> 0) & 15];
        syms[0]  = (h0 >> 0) & 1;
        syms[8]  = (h0 >> 1) & 1;
        syms[16] = (h0 >> 2) & 1;
        syms[24] = (h0 >> 3) & 1;
        syms[32] = (h0 >> 4) & 1;
        syms[40] = (h0 >> 5) & 1;
        syms[48] = (h0 >> 6) & 1;
        const auto h1 = kDragonEncodeLookup[(data[0] >> 4) & 15];
        syms[1]  = (h1 >> 0) & 1;
        syms[9]  = (h1 >> 1) & 1;
        syms[17] = (h1 >> 2) & 1;
        syms[25] = (h1 >> 3) & 1;
        syms[33] = (h1 >> 4) & 1;
        syms[41] = (h1 >> 5) & 1;
        syms[49] = (h1 >> 6) & 1;
        const auto h2 = kDragonEncodeLookup[(data[1] >> 0) & 15];
        syms[2]  = (h2 >> 0) & 1;
        syms[10] = (h2 >> 1) & 1;
        syms[18] = (h2 >> 2) & 1;
        syms[26] = (h2 >> 3) & 1;
        syms[34] = (h2 >> 4) & 1;
        syms[42] = (h2 >> 5) & 1;
        syms[50] = (h2 >> 6) & 1;
        const auto h3 = kDragonEncodeLookup[(data[1] >> 4) & 15];
        syms[3]  = (h3 >> 0) & 1;
        syms[11] = (h3 >> 1) & 1;
        syms[19] = (h3 >> 2) & 1;
        syms[27] = (h3 >> 3) & 1;
        syms[35] = (h3 >> 4) & 1;
        syms[43] = (h3 >> 5) & 1;
        syms[51] = (h3 >> 6) & 1;
        const auto h4 = kDragonEncodeLookup[(data[2] >> 0) & 15];
        syms[4]  = (h4 >> 0) & 1;
        syms[12] = (h4 >> 1) & 1;
        syms[20] = (h4 >> 2) & 1;
        syms[28] = (h4 >> 3) & 1;
        syms[36] = (h4 >> 4) & 1;
        syms[44] = (h4 >> 5) & 1;
        syms[52] = (h4 >> 6) & 1;
        const auto h5 = kDragonEncodeLookup[(data[2] >> 4) & 15];
        syms[5]  = (h5 >> 0) & 1;
        syms[13] = (h5 >> 1) & 1;
        syms[21] = (h5 >> 2) & 1;
        syms[29] = (h5 >> 3) & 1;
        syms[37] = (h5 >> 4) & 1;
        syms[45] = (h5 >> 5) & 1;
        syms[53] = (h5 >> 6) & 1;
        const auto h6 = kDragonEncodeLookup[(data[3] >> 0) & 15];
        syms[6]  = (h6 >> 0) & 1;
        syms[14] = (h6 >> 1) & 1;
        syms[22] = (h6 >> 2) & 1;
        syms[30] = (h6 >> 3) & 1;
        syms[38] = (h6 >> 4) & 1;
        syms[46] = (h6 >> 5) & 1;
        syms[54] = (h6 >> 6) & 1;
        const auto h7 = kDragonEncodeLookup[(data[3] >> 4) & 15];
        syms[7]  = (h7 >> 0) & 1;
        syms[15] = (h7 >> 1) & 1;
        syms[23] = (h7 >> 2) & 1;
        syms[31] = (h7 >> 3) & 1;
        syms[39] = (h7 >> 4) & 1;
        syms[47] = (h7 >> 5) & 1;
        syms[55] = (h7 >> 6) & 1;
    }


    static DSP_INLINE void DragonEncode8(uint8_t syms[7], const uint32_t data) { return DragonEncode8(syms, (const uint8_t*)&data); }
    static DSP_INLINE void DragonEncode4(uint8_t syms[14], const uint32_t data) { return DragonEncode4(syms, (const uint8_t*)&data); }
    static DSP_INLINE void DragonEncode2(uint8_t syms[28], const uint32_t data) { return DragonEncode2(syms, (const uint8_t*)&data); }
    static DSP_INLINE void DragonEncode1(uint8_t syms[56], const uint32_t data) { return DragonEncode1(syms, (const uint8_t*)&data); }



    /** Class used to sequence chips, outputting these as samples.
     *
     *  To use, call start(), specifying the sequence of chips to send.
     *  Calls to render() will then render progressive samples from the sequence
     *  to a sample buffer.
     */
    class DragonChipper
    {
    public:

        /** Constructor.
         */
        DragonChipper() : m_seed(0), m_seq(nullptr), m_end(nullptr), m_chipGen()
        {
            // Nothing.
        }


        /** Test if the chipper is active (has more samples still to render).
         */
        bool isActive() const
        {
            return m_chipGen.isActive() || (m_seq != m_end);
        }


        /** Initialise to generate a supplied sequence of payload symbols.
         *
         *  @param  seed        The LFSR seed.
         *  @param  seq         A pointer to a sequence of symbols. This must be non-null.
         *  @param  len         The length of the sequence. This must be at least 1.
         *  @warning            The caller must preserve the storage of @e sequence for as long as is needed.
         *  @warning            To avoid garbage output, the caller should ensure @e !isActive() before calling start().
         */
        void start(uint32_t seed, const uint8_t* seq, unsigned len)
        {
            m_seed = seed;
            m_seq = seq;
            m_end = seq + len;
        }


        /** Render output samples.
         *
         *  @param  yn          The sample buffer.
         *  @param  count       The length of the sample buffer.
         *  @return             The number of samples written.
         *
         *  If no sequence is currently available, this method will return zero.
         *  Otherwise, the number of samples written to @e yn are returned. If this
         *  is equal to @e count, there are more samples still to be generated. If
         *  it is less than @e count then the sequence has just completed.
         */
        DSP_INLINE unsigned render(int32_t* const yn, const unsigned count)
        {
            int32_t* ptr = yn;
            unsigned rendered = 0;
            while (rendered != count)
            {
                if (m_chipGen.remaining() == 0)
                {
                    if (m_seq == m_end) break;              // finished the complete sequence
                    uint32_t seed = m_seed + (*m_seq ++);   // else start next chip
                    m_chipGen.start(seed);                  //
                }
                unsigned remaining = m_chipGen.remaining();
                unsigned samplesThisPass = (count < remaining) ? count : remaining;
                int32_t* end = ptr + samplesThisPass;
                do {
                    *ptr ++ = m_chipGen.next();
                } while (ptr != end);
                rendered += samplesThisPass;
            }
            return rendered;
        }

    public:

        /** Helper class used to generate samples for a single chip.
         *
         *  Note: this is public to allow the chip-designer to use the same method as the chipper
         *  when searching for the ideal codes. Application code should not use this class directly.
         */
        class ChipGen
        {
        public:
            DSP_INLINE void start(uint32_t seed)    { m_state = seed; m_remaining = kDragonSamplesPerChip; }
            DSP_INLINE bool isActive() const        { return m_remaining != 0; }
            DSP_INLINE unsigned remaining() const   { return m_remaining; }
            DSP_INLINE int32_t next()               { --m_remaining; return int32_t(DragonRandom(&m_state)); }
        private:
            uint32_t m_state { 0 };
            unsigned m_remaining { 0 };
        };


    private:
        uint32_t m_seed;
        const uint8_t* m_seq;
        const uint8_t* m_end;
        ChipGen m_chipGen;
    };


    /** Class used to convert a payload 32 bit block in to a chip sequence.
     */
    class DragonSequencer
    {
    public:

        DragonSequencer() : m_sequence() { }                                    ///< Constructor.
        const uint8_t* sequence() const { return &m_sequence[0]; }              ///< Return the sequence address.
        unsigned length() const         { return kDragonChipsPerBlock; }        ///< Return the length of the sequence.


        /** Encode a block. This overwrites any existing sequence.
         *
         *  @param  block       The data to encode.
         */
        void encode(uint32_t block)
        {
            if (kDragonBitsPerSymbol == 1)      DragonEncode1(m_sequence, block);
            else if (kDragonBitsPerSymbol == 2) DragonEncode2(m_sequence, block);
            else if (kDragonBitsPerSymbol == 4) DragonEncode4(m_sequence, block);
            else if (kDragonBitsPerSymbol == 8) DragonEncode8(m_sequence, block);
        }

    private:
        uint8_t m_sequence[kDragonChipsPerBlock];
    };



    /** Dragon packet transmitter.
     */
    class DragonTx
    {
    public:

        /** Constructor.
         */
        DragonTx()
            :
            m_state(kStateIdle),
            m_nextPacketSequence(0),
            m_nextBlockIndex(0),
            m_validBlockCount(0),
            m_blocks(),
            m_chipper(),
            m_sequencer(),
            m_scrambler(0)
        {
            // Nothing.
        }


        /** Test if the transmitter is active (processing a previous send() call).
         */
        bool isActive() const
        {
            return m_state != kStateIdle;
        }


        /** Start a new packet transmission.
         *
         *  @param  data        A pointer to the payload data.
         *  @param  length      The length, in bytes, of the application payload. The maximum length is @e kDragonPayloadSize.
         *  @param  meta        Optional 4 bit meta data, available for client use.
         *  @return             true for success, false if a parameter is invalid or if a send is already in progress.
         *
         *  A copy of the data is taken - the caller does not need to preserve the contents of the payload buffer
         *  while transmission progresses. If @e length is less than @e kDragonPayloadSize, the message will be padded with
         *  zeros.
         *
         *  If @e length is greater than @e kDragonPayloadSize, an error will be returned.
         *
         *  Calling @e send() simply sets up the data transmission. The client must make subsequent calls to
         *  @e process() to generate the output samples. A new transmission can be queued only when @e process()
         *  returns zero.
         *
         *  Ideally we would just code the complete packet before sending. However, this results in very uneven CPU
         *  loading, so blocks of 4 bytes are coded on the fly in the @e render() method.
         */
        bool send(const void* data, unsigned length, unsigned meta=0)
        {
            // Check preconditions.
            if (length > kDragonMaxPayloadBytes) return false;  // bad argument
            if (!data || 0 == length) return false;             // bad argument

#if DSP_CORTEX_M4
            // Atomic check and update for the state. This avoids race conditions when starting sends.
            // Only the initial state change from idle needs to be locked in this way.
            bool didChangeState = false;
            __DMB();
            uint8_t oldState = __LDREXB(&m_state);
            if (oldState == kStateIdle) didChangeState = (0 == __STREXB(kStateInit, &m_state));
            else __CLREX();
            __DMB();
            if (!didChangeState) return false;          // busy
#else
            if (m_state != kStateIdle) return false;    // busy
            m_state = kStateInit;
#endif

            // Set the block count for this packet.
            // In addition to the application payload, we add 1 header word and one trailer.
            m_validBlockCount = 2 + ((length + 3) >> 2);

            // Set the header.
            m_blocks[0] = DragonMakeHeader(m_nextPacketSequence++, meta, length);

            // Copy the packet data.
            m_blocks[m_validBlockCount - 2] = 0;        // clear unwritten pad bytes at the end
            auto dataDst = (uint8_t*)&m_blocks[1];
            auto dataSrc = (const uint8_t*)data;
            auto dataEnd = dataSrc + (length & 0xfffcu);
            while (dataSrc != dataEnd) {
                *dataDst ++ = *dataSrc ++;
                *dataDst ++ = *dataSrc ++;
                *dataDst ++ = *dataSrc ++;
                *dataDst ++ = *dataSrc ++;
            }
            dataEnd = dataSrc + length;
            while (dataSrc != dataEnd) {
                *dataDst ++ = *dataSrc ++;
            }

            // Set the checksum.
            auto csumSrc = &m_blocks[0];
            auto csumEnd = csumSrc + m_validBlockCount - 1;
            DragonChecksum csum;
            do {
                csum.process32(*csumSrc ++);
            } while (csumSrc != csumEnd);
            m_blocks[m_validBlockCount - 1] = csum.checksum32();

            // Setup the state ready for the next call to process().
            m_chipper.start(kDragonSeedPreamble, kDragonPreambleSequence, kDragonPreambleSequenceLength);
            m_scrambler.start(0);
            m_nextBlockIndex = 0;
            m_state = kStateActive;     // go-go-gadget-render
            return true;
        }


        /** Get the next sample data to send.
         *
         *  @param  yn          The output sample buffer.
         *  @param  count       The number of samples to generate.
         *  @return             The number of samples that were written.
         *
         *  If this method returns zero, there was no packet to send, and a new
         *  packet can be queued by calling send().
         */
        DSP_INLINE unsigned render(int32_t* yn, const uint32_t count)
        {
            unsigned renderedCount = 0;
            if (kStateActive == m_state)
            {
                while (true)
                {
                    // Return the next block from the chipper.
                    unsigned samplesToChip = count - renderedCount;
                    unsigned actualSamplesChipped = m_chipper.render(&yn[renderedCount], samplesToChip);
                    renderedCount += actualSamplesChipped;
                    if (renderedCount == count) break;

                    // If we rendered fewer samples than requested we are at the end of the chipper sequence
                    // and need to load the next block.
                    if (samplesToChip != actualSamplesChipped)
                    {
                        if (m_nextBlockIndex >= m_validBlockCount) break;       // No more blocks

                        uint32_t scram = m_scrambler.next();
                        uint32_t block = m_blocks[m_nextBlockIndex];

                        if (m_nextBlockIndex == 0) m_scrambler.start(block);
                        else block ^= scram;

                        m_nextBlockIndex += 1;

                        m_sequencer.encode(block);
                        m_chipper.start(kDragonSeedData, m_sequencer.sequence(), m_sequencer.length());
                    }
                }

                // Update the state on completion.
                if (!m_chipper.isActive() && m_nextBlockIndex >= m_validBlockCount)
                {
                    // Finished the packet.
                    m_state = kStateIdle;
                }
            }
            return renderedCount;
        }


    private:

        static constexpr uint8_t kStateIdle         =   0;      ///< Nothing to send.
        static constexpr uint8_t kStateInit         =   1;      ///< Preparing a new send packet.
        static constexpr uint8_t kStateActive       =   2;      ///< Transmitting.

    private:

        volatile uint8_t m_state;                               ///< Current transmission state.
        unsigned m_nextPacketSequence;                          ///< Packet sequencing counter.
        unsigned m_nextBlockIndex;                              ///< Index of the next payload block to send.
        unsigned m_validBlockCount;                             ///< The number of valid blocks to send.
        uint32_t m_blocks[2 + kDragonMaxPayloadBlocks];         ///< The current packet data.
        DragonChipper m_chipper;                                ///< The chipper.
        DragonSequencer m_sequencer;                            ///< The sequencer.
        DragonScrambler m_scrambler;                            ///< The scrambler.
    };

}   // namespace
