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


/** Debug-over-audio support.
 *
 *  Debug support is enabled via macros, allowing all debug code to be stripped from a build if needed.
 *
 *  To enable/disable debug, define DDEBUG_ENABLE as non-zero or zero.
 *
 *  In the main file that includes OSC_CYCLE(), add the following:
 *
 *      DDEBUG_INSTANTIATE;     // Create the debug object. 'count' is the number of messages to buffer.
 *
 *      void OSC_CYCLE(int32_t* yn, uint32_t count)
 *      {
 *          if (DDEBUG_RENDER(yn, count))
 *          {
 *              // Output is debug rendering. Do not write to yn here.
 *              return;
 *          }
 *          else
 *          {
 *              // No debug output. Process normally.
 *              ...
 *          }
 *      }
 *
 *  This will ensure that by default, normal audio is sent. It will be interrupted whenever a
 *  debug message has to be transmitted.
 *
 *  To write debug output, use:
 *
 *      DDEBUG(value0);
 *      DDEBUG(value0, value1);
 *      DDEBUG(value0, value1, value2);
 *      DDEBUG(value0, value1, value2, value3);
 *      DDEBUG(value0, value1, value2, value3, value4);
 *      DDEBUG_DUMP(address, count);                        // dump 32bit words
 *
 *  The supplied values may be any mix of 8, 16 or 32 bit signed or unsigned integers, float, or
 *  null-terminated ASCII text. Using constant integer values as arguments may require an explicit
 *  cast so that the compiler can work out the correct type (but why would you log constant integer
 *  values anyway?).
 *
 *  Be aware that the debug class currently is unbuffered and output will be lost if trying to send
 *  debug content while the last message is rendering.
 *
 *  Implementation of extended buffering is left as an exercise for the future. The easiest way to
 *  provide this would be in the DragonTx class, so that the render() call can pull the next buffer
 *  when the current one has completed. However, be aware that any implementation must be fully
 *  re-entrant without the need for any blocking lock mechanism.
 */

#pragma once



// Global enable.
#if !defined(DDEBUG_ENABLE)
#define DDEBUG_ENABLE   (1)                     ///< Global enable/disable for DDBUG macros and DDebugXxx entities.
#endif




#if !DDEBUG_ENABLE



#pragma mark    -   Debug Disabled


#include <cstdint>

#define DDEBUG_INSTANTIATE      extern uint8_t g_DDebugInstance
static inline bool DDEBUG_RENDER(int32_t* yn, uint32_t count) { (void)yn; (void)count; return false; }
static inline bool DDEBUG_IS_READY() { return false; }
template <typename T0> static inline void DDEBUG(T0 a0) { (void)a0; }
template <typename T0, typename T1> static inline void DDEBUG(T0 a0, T1 a1) { (void)a0; (void)a1; }

template <typename T0, typename T1, typename T2> static inline void DDEBUG(T0 a0, T1 a1, T2 a2) { (void)a0; (void)a1; (void)a2; }
template <typename T0, typename T1, typename T2, typename T3> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3) { (void)a0; (void)a1; (void)a2; (void)a3; }
template <typename T0, typename T1, typename T2, typename T3, typename T4> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4) { (void)a0; (void)a1; (void)a2; (void)a3; (void)a4; }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5) { (void)a0; (void)a1; (void)a2; (void)a3; (void)a4; (void)a5; }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6) { (void)a0; (void)a1; (void)a2; (void)a3; (void)a4; (void)a5; (void)a6; }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6, T7 a7) { (void)a0; (void)a1; (void)a2; (void)a3; (void)a4; (void)a5; (void)a6; (void)a7; }
static inline void DDEBUG_DUMP(const void* address, unsigned count) { (void)address; (void)count; }



#else



#pragma mark    -   Debug Enabled



#include "dsp_platform.h"
#include "dsp_float.h"
#include "dsp_dragon.h"

namespace dsp {

    /** Class used to construct and parse debug messages.
     *
     *  Messages consist of a sequence of typed fields. A single
     *  field byte prefixes a block of subsequent data.
     *
     *  Message append functions are "lossy" in that they will silently drop
     *  data if the message buffer would overflow.
     */
    class DDebugMessage
    {
    private:

        uint8_t m_payload[kDragonMaxPayloadBytes];
        unsigned m_index;

    public:

        static const uint8_t kFieldTypeInvalid  =   0;      ///< Unused.
        static const uint8_t kFieldTypeString   =   1;      ///< Following data is a zero terminated string.
        static const uint8_t kFieldTypeU8       =   2;      ///< Following data is a little-endian int32_t.
        static const uint8_t kFieldTypeU16      =   3;      ///< Following data is a little-endian uint32_t.
        static const uint8_t kFieldTypeU32      =   4;      ///< Following data is a little-endian float.
        static const uint8_t kFieldTypeS8       =   5;      ///< Following data is a little-endian int32_t.
        static const uint8_t kFieldTypeS16      =   6;      ///< Following data is a little-endian uint32_t.
        static const uint8_t kFieldTypeS32      =   7;      ///< Following data is a little-endian float.
        static const uint8_t kFieldTypeF32      =   8;      ///< Following data is a little-endian float.
        static const uint8_t kFieldTypeF64      =   9;      ///< Following data is a little-endian float.

        DDebugMessage() : m_payload(), m_index(0) { }                                              ///< Constructor.
        void clear() { m_index = 0; }                                                                   ///< Erase the current message.
        bool checkAvailableSpace(unsigned n) const { return (m_index + n) <= kDragonMaxPayloadBytes; }  ///< Overrun check.
        const uint8_t* payload() const { return &m_payload[0]; }                                        ///< Return the payload.
        unsigned size() const { return m_index; }                                                       ///< Return the payload size.

        /** Generic append of a binary value.
         *
         *  @param  fieldType   The field type.
         *  @param  value       Pointer to the data to append.
         *  @param  valueSize   The size of the data, in bytes.
         */
        inline void append(const uint8_t fieldType, const void* value, const unsigned valueSize)
        {
            if (checkAvailableSpace(1 + valueSize))
            {
                auto bytes = (const uint8_t*)value;
                m_payload[m_index++] = fieldType;
                if (valueSize > 0) m_payload[m_index++] = bytes[0];
                if (valueSize > 1) m_payload[m_index++] = bytes[1];
                if (valueSize > 2) m_payload[m_index++] = bytes[2];
                if (valueSize > 3) m_payload[m_index++] = bytes[3];
            }
        }

        inline void append(int8_t value) { append(kFieldTypeS8, &value, sizeof value); }           ///< Append an integer value.
        inline void append(int16_t value) { append(kFieldTypeS16, &value, sizeof value); }         ///< Append an integer value.
        inline void append(int32_t value) { append(kFieldTypeS32, &value, sizeof value); }         ///< Append an integer value.

        inline void append(uint8_t value) { append(kFieldTypeU8, &value, sizeof value); }          ///< Append an integer value.
        inline void append(uint16_t value) { append(kFieldTypeU16, &value, sizeof value); }        ///< Append an integer value.
        inline void append(uint32_t value) { append(kFieldTypeU32, &value, sizeof value); }        ///< Append an integer value.

        inline void append(float value) { append(kFieldTypeF32, &value, sizeof value); }           ///< Append a float32 value.
        inline void append(double value) { append(kFieldTypeF64, &value, sizeof value); }          ///< Append a float64 value.

        inline void append(bool value) { int8_t n = value ? 1 : 0; append(n); }                    ///< Append a bool value.

        // Workaround for ambiguity when promoting integer values.
        template <typename T> inline void append(T value);

        inline void append(const void* value) { auto n = uint32_t(uint64_t(value)); append(kFieldTypeU32, &n, sizeof n); }  ///< Append a 32 bit pointer.
        template <typename T> inline void append(T* value) { append((const void*)value); }          ///< Append a 32 bit pointer.


        /** Append a string.
         *
         *  @param  value       A null terminated string.
         */
        void append(const char* value)
        {
            if (checkAvailableSpace(3))
            {
                m_payload[m_index++] = kFieldTypeString;
                while (true) {
                    auto ch = *value ++;
                    if (m_index == kDragonMaxPayloadBytes - 1) ch = 0;      // force null terminator.
                    m_payload[m_index++] = uint8_t(ch);
                    if (0 == ch) break;
                }
            }
        }
    };

    // Template specialisations to work around implicit type promotion ambiguities in append() calls.
    template <> inline void DDebugMessage::append<int>(int value) { append(int32_t(value)); }
    template <> inline void DDebugMessage::append<unsigned>(unsigned value) { append(uint32_t(value)); }




    /** The debug transmitter class.
     *
     *  This generates formatted payloads for debug use. Formats are signalled via the DragonTx meta field (2 bits only).
     *  Two formats are defined: message and dump.
     *
     *  The message format is:
     *
     *      uint32_t argument0;         // Numeric argument
     *      uint32_t argument1;         // Numeric argument
     *      uint32_t argument2;         // Numeric argument
     *      uint32_t argument3;         // Numeric argument
     *      uint8_t argumentFlags;      // Bit mask to indicate the presence and type of numeric arguments
     *      uint8_t sequence;           // Sequence number for calls (not necessarilty sends() to message().
     *      uint8_t text[];             // Variable length message text
     *
     * The dump format is:
     *
     *      uint32_t address;           // The address
     *      uint32_t data[];            // Variangle length data
     *
     */
    class DDebug
    {
    public:

        static const unsigned kMetaMessage      =   0;                              ///< Payload contains a formatted message.
        static const unsigned kMetaDump         =   1;                              ///< Payload contains a memory dump.


    public:


        /** Constructor.
         */
        DDebug() : m_transmitter()
        {
            // Nothing.
        }


        /** Test if we can accept another debug send.
         */
        bool ready() const
        {
            return !m_transmitter.isActive();
        }


        /** Send a message with one argument.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0>
        bool message(T0 a0)
        {
            DDebugMessage msg;
            msg.append(a0);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1>
        bool message(T0 a0, T1 a1)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2>
        bool message(T0 a0, T1 a1, T2 a2)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2, typename T3>
        bool message(T0 a0, T1 a1, T2 a2, T3 a3)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            msg.append(a3);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2, typename T3, typename T4>
        bool message(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            msg.append(a3);
            msg.append(a4);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5>
        bool message(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            msg.append(a3);
            msg.append(a4);
            msg.append(a5);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
        bool message(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            msg.append(a3);
            msg.append(a4);
            msg.append(a5);
            msg.append(a6);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a message with multiple arguments.
         *
         *  @return             true if the send could be issued, false if we are not ready.
         */
        template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
        bool message(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6, T7 a7)
        {
            DDebugMessage msg;
            msg.append(a0);
            msg.append(a1);
            msg.append(a2);
            msg.append(a3);
            msg.append(a4);
            msg.append(a5);
            msg.append(a6);
            msg.append(a7);
            return m_transmitter.send(msg.payload(), msg.size(), kMetaMessage);
        }


        /** Send a memory dump.
         *
         *  @param  address     The data address.
         *  @param  count       The number of 32bit words to send.
         *  @return             true if the send could be issued, false if we are not ready.
         *
         *  Dump messages are just an array of u32 values. The first gives the address,
         *  then subsequent values are the data from that address.
         */
        bool dump(const uint32_t* address, unsigned count)
        {
            static const uint32_t kMaxWords = kDragonMaxPayloadBytes / sizeof (uint32_t);
            static_assert(kMaxWords >= 2, "code assumes packet size is usable");
            uint32_t payload[kMaxWords];
            count += 1;
            if (count > kMaxWords) count = kMaxWords;
            auto ptr = &payload[0];
            auto end = &payload[count];
            *ptr++ = uint32_t(uint64_t(address));
            do {
                *ptr++ = *address++;
            } while (ptr != end);
            return m_transmitter.send(&payload[0], count * sizeof (uint32_t), kMetaDump);
        }


        /** Render to the sample stream.
         *
         *  @param  yn          Pointer to the stream.
         *  @param  count       The number of samples to render.
         *  @return             true if we rendered, false if there was nothing to be rendered (*yn unchanged).
         */
        bool render(int32_t* yn, unsigned count)
        {
            unsigned actual = m_transmitter.render(yn, count);
            if (0 == actual) return false;
            if (actual < count)
            {
                // If there are any residual samples unwritten, just pad with zero.
                int32_t* p = &yn[actual];
                int32_t* e = &yn[count];
                do { *p ++ = 0; } while (p != e);
            }
            return true;
        }

    private:

        DragonTx m_transmitter;
    };
}

extern dsp::DDebug g_DDebugInstance;
#define DDEBUG_INSTANTIATE  dsp::DDebug g_DDebugInstance
static inline bool DDEBUG_RENDER(int32_t* yn, uint32_t count) { return g_DDebugInstance.render(yn, count); }
static inline bool DDEBUG_IS_READY() { return g_DDebugInstance.ready(); }
template <typename T0> static inline void DDEBUG(T0 a0) { g_DDebugInstance.message(a0); }
template <typename T0, typename T1> static inline void DDEBUG(T0 a0, T1 a1) { g_DDebugInstance.message(a0, a1); }
template <typename T0, typename T1, typename T2> static inline void DDEBUG(T0 a0, T1 a1, T2 a2) { g_DDebugInstance.message(a0, a1, a2); }
template <typename T0, typename T1, typename T2, typename T3> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3) { g_DDebugInstance.message(a0, a1, a2, a3); }
template <typename T0, typename T1, typename T2, typename T3, typename T4> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4) { g_DDebugInstance.message(a0, a1, a2, a3, a4); }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5) { g_DDebugInstance.message(a0, a1, a2, a3, a4, a5); }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6) { g_DDebugInstance.message(a0, a1, a2, a3, a4, a5, a6); }
template <typename T0, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7> static inline void DDEBUG(T0 a0, T1 a1, T2 a2, T3 a3, T4 a4, T5 a5, T6 a6, T7 a7) { g_DDebugInstance.message(a0, a1, a2, a3, a4, a5, a6, a7); }
static inline void DDEBUG_DUMP(const void* address, unsigned count) { g_DDebugInstance.dump((const uint32_t*)address, count); }

#endif  // DDEBUG_ENABLE
