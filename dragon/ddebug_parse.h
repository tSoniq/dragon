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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dsp_ddebug.h"


/** Parser for debug messages.
 */
class DDebugMessageParser
{
public:

    DDebugMessageParser() { stop(); }                                  ///< Constructor.
    DDebugMessageParser(const std::vector<uint8_t>& d) { start(d); }   ///< Constructor. Passed vector with the packet data.
    DDebugMessageParser(const uint8_t* d, unsigned s) { start(d, s); } ///< Constructor. Passed the packet data.

    /** Stop parsing.
     */
    void stop()
    {
        parserStop();
    }

    /** Begin parsing.
     */
    void start(const uint8_t* data, unsigned size)
    {
        parserStart(data, size);
    }

    /** Begin parsing.
     */
    void start(const std::vector<uint8_t>& data)
    {
        if (data.size() != 0) parserStart(&data[0], unsigned(data.size()));
        else parserStop();
    }

    /** Get the next field.
     *
     *  @param  str     Returns the field data.
     *  @return         true for success, false if no more fields exist.
     */
    bool getNext(std::string& str)
    {
        uint8_t fieldType = dsp::DDebugMessage::kFieldTypeInvalid;
        bool result = parserGetByte(fieldType);
        if (result)
        {
            switch (fieldType)
            {
            case dsp::DDebugMessage::kFieldTypeInvalid:     getInvalid(str);                break;
            case dsp::DDebugMessage::kFieldTypeString:      getString(str);                 break;
            case dsp::DDebugMessage::kFieldTypeU8:          getInteger(str, 1);             break;
            case dsp::DDebugMessage::kFieldTypeS8:          getInteger(str, 1);             break;
            case dsp::DDebugMessage::kFieldTypeU16:         getInteger(str, 2);             break;
            case dsp::DDebugMessage::kFieldTypeS16:         getInteger(str, 2);             break;
            case dsp::DDebugMessage::kFieldTypeU32:         getInteger(str, 4);             break;
            case dsp::DDebugMessage::kFieldTypeS32:         getInteger(str, 4);             break;
            case dsp::DDebugMessage::kFieldTypeF32:         getFloat32(str);                break;
            case dsp::DDebugMessage::kFieldTypeF64:         getFloat64(str);                break;
            default:                                        getUnknown(str, fieldType);     break;
            }
        }
        if (!result) str.clear();
        return result;
    }

private:

    bool getInvalid(std::string& str) {
        str = "<invalid>";
        return true;
    }

    bool getUnknown(std::string& str, unsigned fieldType) {
        char buf[64];
        snprintf(buf, sizeof buf, "<unknown field type 0x%02x>", unsigned(fieldType));
        str = buf;
        return true;
    }

    bool getString(std::string& str)
    {
        return parserGetString(str);
    }

    bool getInteger(std::string& str, unsigned size) {
        uint64_t n = 0;
        if (parserGetInteger(n, size))
        {
            char buf[128];
            if (size <= 1)      snprintf(buf, sizeof buf, "0x%02llx", (unsigned long long)n);
            else if (size <= 2) snprintf(buf, sizeof buf, "0x%04llx", (unsigned long long)n);
            else if (size <= 4) snprintf(buf, sizeof buf, "0x%08llx", (unsigned long long)n);
            else                snprintf(buf, sizeof buf, "0x%16llx", (unsigned long long)n);
            str = buf;
            return true;
        }
        else return false;
    }

    bool getFloat32(std::string& str) {
        float n = 0.0;
        if (parserGetF32(n)) {
            char buf[128];
            snprintf(buf, sizeof buf, "%.8f", n);
            str = buf;
            return true;
        }
        else return false;
    }

    bool getFloat64(std::string& str) {
        double n = 0.0;
        if (parserGetF64(n)) {
            char buf[128];
            snprintf(buf, sizeof buf, "%.16lf", n);
            str = buf;
            return true;
        }
        else return false;
    }

private:

    void parserStart(const uint8_t* data, unsigned size)
    {
        m_parserData.resize(size);
        if (size != 0) memcpy(&m_parserData[0], data, size);
        m_parserIndex = 0;
    }

    void parserStop()
    {
        m_parserData.clear();
        m_parserIndex = 0;
    }

    unsigned parserRemaining() const
    {
        return unsigned(m_parserData.size() - m_parserIndex);
    }

    bool parserGetByte(uint8_t& value)
    {
        if (parserRemaining() >= 1)     { value = m_parserData[m_parserIndex++]; return true; }
        else                            { value = 0; return false; }
    }

    template <typename T> bool parserGetInteger(T& value, unsigned size)
    {
        bool result = false;
        T accumulator = 0;
        uint8_t byte = 0;
        for (unsigned i = 0; i != size; ++i)
        {
            result = parserGetByte(byte);
            if (!result) break;
            accumulator |= (T(byte) << (8 * i));
        }
        value = accumulator;
        return result;
    }

    bool parserGetF32(float& value)
    {
        union { float f; uint32_t i; } fix = { 0.0f };
        if (parserGetInteger(fix.i, 4)) { value = fix.f; return true; }
        else {value = 0.0f; return false; }
    }

    bool parserGetF64(double& value)
    {
        union { double d; uint64_t i; } fix = { 0.0 };
        if (parserGetInteger(fix.i, 8)) { value = fix.d; return true; }
        else {value = 0.0; return false; }
    }

    bool parserGetString(std::string& str)
    {
        str = "\"";
        uint8_t ch;
        while (parserGetByte(ch)) {
            if (0 == ch) break;
            str.append(1, (char)ch);
        }
        str += "\"";
        return true;
    }

private:
    std::vector<uint8_t> m_parserData;
    unsigned m_parserIndex;
};


/** Convert any dragon debug message to text.
 *
 *  @param  payload     The payload data.
 *  @param  meta        The meta field.
 *  @return             A string representation.
 */
static inline std::string DDebugParse(const std::vector<uint8_t>& payload, unsigned meta, unsigned sequence)
{
    std::string result;
    if (dsp::DDebug::kMetaMessage == meta)
    {
        // "ddebug: [<value>[, <value>, [<value>...]]]"
        result = "ddebug: ";
        char buf[8];
        snprintf(buf, sizeof buf, "%02x: ", unsigned(sequence & 0xffu));
        result += buf;
        DDebugMessageParser parser(payload);
        std::string item;
        bool first = true;
        while (parser.getNext(item))
        {
            if (!first) result += ", ";
            first = false;
            result += item;
        }
    }
    else if (dsp::DDebug::kMetaDump == meta)
    {
        // "ddebug-dump: <address> = [<value>[, <value>, [<value>...]]]"
        result = "ddebug-dump: ";
        unsigned n = unsigned(payload.size() / 4);
        for (unsigned i = 0; i != n*4; i += 4)
        {
            char buf[64];
            snprintf(buf, sizeof buf, "0x%02x%02x%02x%02x",
                     unsigned(payload[i+3]),
                     unsigned(payload[i+2]),
                     unsigned(payload[i+1]),
                     unsigned(payload[i+0]));
            if (i > 4)
            {
                result += ", ";
                if (((i - 4) % 16) == 0) result += " ";
            }
            result += buf;
            if (i == 0) result += " = ";
        }
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof buf, "ddebug-error: meta %u", meta);
        result = buf;
    }
    return result;
}
