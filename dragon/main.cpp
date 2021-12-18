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

//
//  main.cpp
//  dragon
//
//  This is a real-time receiver for the DragonDebug protocol.
//  macOS only.
//



#define DDEBUG_ENABLE (1)                   // Enable DDBUG builds.
#define DDEBUG_ENABLE_MESSAGEPARSER (1)     // Enable message parsing.

#define USE_ACCELERATE_FRAMEWORK    (1)     // Use Apple vector library for performance.

#include <AudioToolbox/AudioToolbox.h>
#include "dragonrx.h"
#include "ddebug_parse.h"



static constexpr double kDragonSampleRate = 48000.0;    ///< The nominal sample rate in the Logue device.



/** Helper for audio errors.
 */
static void trapError(OSStatus status, const char* file, int line)
{
    if (status == noErr) return;

    char a = (char)((status >> 24) & 0xffu);
    char b = (char)((status >> 16) & 0xffu);
    char c = (char)((status >> 8) & 0xffu);
    char d = (char)((status >> 0) & 0xffu);
    if (!isprint(a)) a = '.';
    if (!isprint(b)) a = '.';
    if (!isprint(c)) a = '.';
    if (!isprint(d)) a = '.';
    fprintf(stderr, "CoreAudio error %08x  %d  '%c%c%c%c': %s: %d\n",
            unsigned(status),
            int(status),
            a, b, c, d,
            file,
            line);
    exit(-1);
}

#define TRAP_ERROR(_sts)    trapError((_sts), __FILE__, __LINE__)



/** Context for render callbacks.
 */
typedef struct Context
{
    static const unsigned kBufferLength = 8*1024;
    static const unsigned kBufferCount = 64;

    AudioUnit unit;
    AudioStreamBasicDescription streamFormat;
    AudioStreamBasicDescription deviceFormat;
    AudioBufferList* audioBuffer;
    dsp::DragonRx dragonRx;
    unsigned bufferWriteIndex;
    unsigned bufferPosition;
    float buffer[kBufferCount][kBufferLength];


    Context() : unit(0), streamFormat(), deviceFormat(), audioBuffer(nullptr), dragonRx(), bufferWriteIndex(0), bufferPosition(0), buffer()
    {
        memset(&streamFormat, 0, sizeof streamFormat);
        memset(&deviceFormat, 0, sizeof deviceFormat);
        memset(&buffer, 0, sizeof buffer);
    }

} Context;



static OSStatus CoreAudioInputProc(
                                   void* inRefCon,
                                   AudioUnitRenderActionFlags* ioActionFlags,
                                   const AudioTimeStamp* inTimeStamp,
                                   UInt32 inBusNumber,
                                   UInt32 inNumberFrames,
                                   AudioBufferList* ioData)
{
    Context& context = *(Context*)inRefCon;

    OSStatus status = AudioUnitRender(context.unit, ioActionFlags, inTimeStamp, inBusNumber, inNumberFrames, context.audioBuffer);
    if (status == kAUGraphErr_CannotDoInCurrentContext)
    {
        printf("cannot do\n");
        return status;
    }
    TRAP_ERROR(status);

    // If there is more than one channel of data, use only the first (whatever that might be).
    const float* src = (const float*)(context.audioBuffer->mBuffers[0].mData);
    auto remaining = inNumberFrames;
    while (0 != remaining)
    {
        auto space = Context::kBufferLength - context.bufferPosition;
        auto thisPass = (remaining < space) ? remaining : space;
        auto thisBuffer = &(context.buffer[context.bufferWriteIndex][0]);
        memcpy(thisBuffer + context.bufferPosition, src, thisPass * sizeof (float));
        remaining -= thisPass;
        context.bufferPosition += thisPass;
        if (context.bufferPosition == Context::kBufferLength)
        {
            context.bufferPosition = 0;
            context.bufferWriteIndex = (context.bufferWriteIndex + 1) % Context::kBufferCount;
            dispatch_async(dispatch_get_main_queue(), ^{
                context.dragonRx.addSampleData(thisBuffer, Context::kBufferLength);
                unsigned meta = 0;
                unsigned seq = 0;
                std::vector<uint8_t> packet;
                while (context.dragonRx.getNextPacket(packet, meta, seq))
                {
                    auto str = DDebugParse(packet, meta, seq);
                    printf("%s\n", str.c_str());
                }
            });
        }
    }
    return noErr;
}


void Connect(Context& context)
{
    AudioComponentDescription cd = {0};
    cd.componentType = kAudioUnitType_Output;                   // This is dumb, but not an error.
    cd.componentSubType = kAudioUnitSubType_HALOutput;          // This is dumb, but not an error.
    cd.componentManufacturer = kAudioUnitManufacturer_Apple;
    AudioComponent component = AudioComponentFindNext(NULL, &cd);
    if (nullptr == component)
    {
        fprintf(stderr, "Failed to find audio input device\n");
        exit(-1);
    }

    AudioDeviceID deviceId = kAudioObjectUnknown;
    UInt32 propertySize = sizeof deviceId;
    AudioObjectPropertyAddress deviceIdProperty;
    deviceIdProperty.mSelector = kAudioHardwarePropertyDefaultInputDevice;
    deviceIdProperty.mScope = kAudioObjectPropertyScopeGlobal;
    deviceIdProperty.mElement = kAudioObjectPropertyElementMaster;
    TRAP_ERROR(AudioObjectGetPropertyData(kAudioObjectSystemObject, &deviceIdProperty, 0, nullptr, &propertySize, &deviceId));


    UInt32 zero = 0;
    UInt32 one = 1;
    TRAP_ERROR(AudioComponentInstanceNew(component, &context.unit));
    TRAP_ERROR(AudioUnitSetProperty(context.unit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &zero, sizeof zero));
    TRAP_ERROR(AudioUnitSetProperty(context.unit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, 1, &one, sizeof one));
    TRAP_ERROR(AudioUnitSetProperty(context.unit, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &deviceId, sizeof deviceId));
    propertySize = sizeof context.deviceFormat;
    TRAP_ERROR(AudioUnitGetProperty(context.unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 1, &context.deviceFormat, &propertySize));
    if (fabs(context.deviceFormat.mSampleRate - kDragonSampleRate) > 0.1)
    {
        fprintf(stderr, "Device sample rate is %.1lf - ideally change this to %.1lf in Audio MIDI Setup.app\n", context.deviceFormat.mSampleRate, kDragonSampleRate);
    }
    propertySize = sizeof context.streamFormat;
    TRAP_ERROR(AudioUnitGetProperty(context.unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 1, &context.streamFormat, &propertySize));
    if (fabs(context.streamFormat.mSampleRate - kDragonSampleRate) > 0.1)
    {
        fprintf(stderr, "Stream sample rate is %.1lf - changing  to %.1lf\n", context.streamFormat.mSampleRate, kDragonSampleRate);
        context.streamFormat.mSampleRate = kDragonSampleRate;
        TRAP_ERROR(AudioUnitSetProperty(context.unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 1, &context.streamFormat, sizeof context.streamFormat));
        TRAP_ERROR(AudioUnitGetProperty(context.unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 1, &context.streamFormat, &propertySize));
        if (fabs(context.streamFormat.mSampleRate - kDragonSampleRate) > 0.1)
        {
            fprintf(stderr, "Unable to configure necessary stream format\n");
            exit(-1);
        }
    }

    UInt32 frames = 0;
    propertySize = sizeof frames;
    TRAP_ERROR(AudioUnitGetProperty(context.unit, kAudioDevicePropertyBufferFrameSize, kAudioUnitScope_Global, 0, &frames, &propertySize));
    UInt32 frameSizeBytes = frames * sizeof (float);

    auto size = offsetof(AudioBufferList, mBuffers[0]) + (sizeof(AudioBuffer) * context.streamFormat.mChannelsPerFrame);
    context.audioBuffer = (AudioBufferList*)malloc(size);
    context.audioBuffer->mNumberBuffers = context.streamFormat.mChannelsPerFrame;
    for (unsigned i = 0; i != context.audioBuffer->mNumberBuffers ; ++i)
    {
        context.audioBuffer->mBuffers[i].mNumberChannels = 1;
        context.audioBuffer->mBuffers[i].mDataByteSize = frameSizeBytes;
        context.audioBuffer->mBuffers[i].mData = malloc(frameSizeBytes);
    }

    AURenderCallbackStruct callbackStruct;
    callbackStruct.inputProc = CoreAudioInputProc;
    callbackStruct.inputProcRefCon = &context;
    TRAP_ERROR(AudioUnitSetProperty(context.unit, kAudioOutputUnitProperty_SetInputCallback, kAudioUnitScope_Global, 0, &callbackStruct, sizeof callbackStruct));
}


void Start(Context& context)
{
    TRAP_ERROR(AudioUnitInitialize(context.unit));
    TRAP_ERROR(AudioOutputUnitStart(context.unit));
}



int main(int argc, const char* argv[])
{
    Context context;
    Connect(context);
    Start(context);
    while (true) dispatch_main();
    return 0;
}
