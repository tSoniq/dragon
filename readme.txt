Dragon: Debug Output over Audio
================================

Dragon is a protocol to allow simple uni-directional communication over an audio channel.
Its primary application is to assist debug on audio hardware that lacks a dedicated debug
port, by transmitting debug packets over an audio channel.

Dragon uses a relatively lightweight transmission protocol and supports Korg Minilogue XD
and Prologue devices as well as other embedded systems.

Dragon's main goals are:

    - point to point, unidirectional, uncoordinated data transfer
    - reasonable robustness, to accomodate a basic ADC-cable-DAC signal path
    - low memory and CPU footprint on transmit (running on a constrained embedded processor)
    - unconstrained CPU and complexity on receive (running on a mainstream laptop)
    - simple, portable implementation

Dragon is supplied as a set of header files that implement the protocol, and a macOS project
(Xcode) that implements a receiver terminal.




How to use
==========

Setup:

    - connect the Logue synthesiser's output to an audio input on a Mac
    - ensure that the Mac's default audio input is set to the correct source
    - for reliable operation, on the Logue bypass the VCF and disable the LF Comp.
    - analogue signal levels are not critical, bit try to avoid overload distortion
    - make sure that the 'dragon' program on the Mac has permission to access the microphone

Modifying a Logue project for debug:

    - add the header files from the 'logue' folder to your project
    - include "dsp_ddebug.h"
    - in one non-header file, instantiate the debugger via DDEBUG_INSTANTIATE (see below)
    - enable DDEBUG by defining DDEBUG_ENABLE as a non-zero value
    - use DDEBUG() and DDEBUG_DUMP() calls to send output

Viewing debug output:

    - on the Mac run 'dragon'
    - on the Logue, load the user oscillator





Sample Korg Logue Oscillator Code
---------------------------------

Make the header files in the 'logue' folder available to the oscillator project.
All code is contained in the header files alone.

The following illustrates how to use DDEBUG macros in a Logue OSC project:


    #define DDEBUG_ENABLE   (1)         // Enable DDEBUG macros. Set zero to disable and remove all code
    #include "dsp_ddebug.h"             // Include the main header file.


    // Instantiate the debug object. Put this in one non-header source file - it creates
    // a static shared instance of an object that is used by the DDEBUG() and DDEBUG_DUMP()
    // macros.
    DDEBUG_INSTANTIATE;


    // Sample code showing how to use DDEBUG(). Up to 6 arguments of C-String, integer, float or
    // address values may be passed to the macro. See the header documentation in dsp_ddebug.h
    // for more information.
    void OSC_NOTEON(const user_osc_param_t* const params)
    {
        DDEBUG("OSC_NOTEON", params->pitch, params->shape_lfo);
    }


    // Sample code showing how to use DDEBUG_DUMP(). A block of 32bit words can be sent
    // with one call. Be warned that dragon does not buffer debug output, so if you make
    // several back-to-back calls to DDEBUG_DUMP() it is likley that only the first will be
    // sent. To send large blocks of data, sequence calls to DDEBUG_DUMP() from the OSC_CYCLE()
    // method, sending data when the debug engine is not actively rendering.
    //
    // The maximum number of words that can be sent is limited by the size of the Dragon TX
    // buffer, as specified by kDragonMaxPayloadBlocks. To conserve memory and reduce the risk
    // of data loss it is recommened that no more than 16 to 24 words are sent per DDEBUG_DUMP()
    // call.
    void OSC_INIT(uint32_t platform, uint32_t api)
    {
        DDEBUG_DUMP((void)address, number_of_32bit_words);
    }


    // Sample code showing how to render the debug output. This will briefly interrupt normal
    // audio when a packet is ready to be sent. The transmitted packets will sound like a
    // short burst of white noise.
    void OSC_CYCLE(const user_osc_param_t* const params, int32_t* yn, const uint32_t frames)
    {
        if (DDEBUG_RENDER(yn, frames))
        {
            // Debug data is being output. A complete sample frame has been written to
            // the output sample buffer and no further audio processing should be performed.
            return;
        }
        else
        {
            // No debug output is being sent. Process as normal for the plugin.
            ...
        }
    }


All DDEBUG code can be removed by defining DDEBUG_ENABLE as zero.




Running The 'dragon' Debug Console
----------------------------------

The "dragon" debug console is built using Xcode. It supports macOS only at this time.

To use the debug console:

1. Connect an audio-out from the hardware to an audio-in on the debug computer. For example,
connect the headphone jack on the synthesiser to the audio-in on the computer, leaving the
line-out synthesiser connections free to use with monitor speakers.

2. Disable heavy post-processing of the audio signal on the synthesiser. In particular,
bypass the analogue low-pass filter (or set the filter cutoff to maximum), and disable
heavy effects processing.

3. Make sure that the default audio input (configured in macOS System Preferences/Sound) is
set appropriately.

4. run "dragon" from the Xcode project and look for debug messages on the console output.


The debug console code is very much a quick-and-dirty implementation, and is neither an
optimal implementation nor functionally friendly. For example, if you change the default
audio input, you will need to quite and re-launch the debug console to pick up the changed
audio interface.





Solving Common Problems
-----------------------

If no debug output is seen, check:

1. Can you hear bursts of white noise corresponding to DDEBUG() calls on the synthesiser? If
not, check that DDEBUG_ENABLE is non-zero and the code is issuing DDEBUG() calls.

2. Is the default audio-input set correctly in macOS system preferences? If not, correct it and
restart the dragon terminal program.



If no debug output is seen or if CRC-error messages are frequently displayed:

1. Is the audio volume level too low or too high? Try adjusting the volume until
transmission is reliable.

2. Take care to only play one voice at a time. If multiple voices sound at the same time the
data transfer reliability will be reduced.

3. Keep in mind that audio output on Logue oscillators is gated by the amplitude envelope. If
this cuts out the audio, packets may be corrupted or lost. Increase the envelope sustain
setting if this is an issue (for example, if logging from OSC_NOTEOFF()).

4. Strong filtering, effects processing, or overlapping debug transmissions (ie from separate
voice cards) will all increase the chance of errors. Turn off effects (including the LF Comp.)
and avoid filtering for best reliability. If you are using Dragon you are debugging, not
playing!





The Dragon Transmission Protocol
--------------------------------

Dragon uses a form of direct-sequence coding. Unusually, symbols are generated as blocks
of white noise using an XOR based LFSR, making the transmitter simple and efficient.

All transmissions consist of a sequence of fixed-length noise bursts sent back-to-back.
Each burst is (misleadingly) termed a 'chip'. Chips are concatenated to build a packet
structure similar to that seen in wireless protocols:

    <preamble><header><payload><crc>

The <preamble> consists of a fixed sequence of chips and is used to detect the start of a new
packets. It may also be used to perform channel estimation and correction at the receiver,
although no implementation for this is provided at present.

All data after the preamble is sent using coding and scrambling to provide forward error
correction. A simple Hamming 7,4 code is used, with interleaving to increase tolerance
to burst errors. All data is encoded in blocks of 32 bits of application data (56 bits
of symbol data). Symbols are mapped to a chip LFSR seed by adding the symbol value to
a base LFSR seed.

For implementation efficiency, scrambling is performed on the payload data prior to coding.

The header is a 32 bit value that encodes the payload length, a sequence number and an
application meta-data fields (typically used to denote the following payload format).
The header is scrambled with a fixed pattern and Hamming 7,4 encoded. A maximum payload
length of 1024 bytes is supported, although DDEBUG imposes a more constrained maximum
packet length to conserve memory

The application data is padded to a multiple of 32 bits and then transmitted using the
same coding as the header. Scrambling is performed using a pseudo random generator
seeded with the header word.

Lastly, a 32 bit checksum is appended. The FNV1a checksum algorithm is used to avoid
the need for large lookup tables when calculating a CRC. (Note: the Cortex M4 CPUs used
in some Logue devices contain CRC32 accelerator hardware, but it seems prudent to not
rely on access to this).

The main transmission parameters are defined in dsp_dragontx.h:

    kDragonSamplesPerChip     =   48;
    kDragonBitsPerSymbol      =   8;
    kDragonMaxPayloadBlocks   =   32;
    kDragonSeedPreamble       =   0xd9295542u;
    kDragonSeedData           =   0x7b882ebdu;

With these parameters, the on-air symbol rate is 1KHz, or 8kbits/second. After coding and other overheads,
this results in a typical application throughput of around 400 bytes/second.

If robustness is an issue, the number of samples-per-chips can be increased, or the number of bits-per-symbol
decreased. The parameters must agree in both the compiled Logue plugin and the dragon console application.

It is not recommended that the LFSR seeds be changed, as these have been selected to give good properties
such as orthogonality, power and DC bias.



Open Issues and Limitations
---------------------------

1. The transmitter is single-buffered. Attempts to call DDEBUG() while an existing packet is being
rendered will result in the new debug output being dropped. This could be resolved by implementing
a packet queue in the transmitter, although this would considerably increase the memory footprint
and presents challenges due to the need to operate in a lock-less multi-threaded environment.

2. The receiver code is very crude and extremely inefficient. It should be regarded as a proof of
concept rather than production software. Future enhancements should include channel estimation and
correction (to improve reliability on impaired channels) and a more efficient sample-buffer
mechanism. In practise, such enhancements have proved unnecessary in real-world use, where the
audio channel is fairly well behaved.



About tSoniq
------------

tSoniq is a Barcelona based consultancy specialising in wireless multimedia, hardware and semiconductor
design, supported application and driver development.

For more information on what we offer, please visit https://tsoniq.com.
