/*
MIT License

Copyright (c) 2020 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef CR95HF_H
#define CR95HF_H

#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#ifndef TRACE_GROUP
    #define TRACE_GROUP "NFC "
#endif

#if !DEVICE_SERIAL
    #error [NOT_SUPPORTED] serial communication not supported for this target
#endif

// NXP AN10927 - Fig 1
#define MIFARE_UID_SINGLE_SIZE 4
#define MIFARE_UID_DOUBLE_SIZE 7
#define MIFARE_UID_TRIPLE_SIZE 10

// NXP AN10927 - Fig 2
#define MIFARE_CL_1 0x93
#define MIFARE_CL_2 0x95
#define MIFARE_CL_3 0x97

// NXP AN10833 - Table 6
#define MIFARE_SAK_UID_NOT_COMPLETE 0x04

// datasheet - Figure 6
#define CR95HF_HEADER_LEN 2

// datasheet - Table 21
#define CR95HF_ANALOG_CONFIGURATION_ADDR 0x68

// datasheet - Table 25
#define CR95HF_TIMERW_VALUE        0x3A
#define CR95HF_TIMERW_CONFIRMATION 0x04

// datasheet - table 6
enum cr95hf_error {
    CR95HF_ERROR_OK             = 0x00,
    CR95HF_ERROR_EMD_SOF23      = 0x63,
    CR95HF_ERROR_EMD_SOF10      = 0x65,
    CR95HF_ERROR_EMD_EGT        = 0x66,
    CR95HF_ERROR_TOO_BIG        = 0x67,
    CR95HF_ERROR_TOO_SMALL      = 0x68,
    CR95HF_ERROR_INTERNAL       = 0x71,
    CR95HF_ERROR_FRAME_RECV_OK  = 0x80,
    CR95HF_ERROR_COMM           = 0x86,
    CR95HF_ERROR_TIMEOUT        = 0x87,
    CR95HF_ERROR_INVALID_SOF    = 0x88,
    CR95HF_ERROR_OVERFLOW       = 0x89,
    CR95HF_ERROR_FRAMING        = 0x8A,
    CR95HF_ERROR_EGT            = 0x8B,
    CR95HF_ERROR_INVALID_LENGTH = 0x8C,
    CR95HF_ERROR_CRC            = 0x8D,
    CR95HF_ERROR_LOST           = 0x8E,
    CR95HF_ERROR_NO_FIELD       = 0x8E,
    CR95HF_ERROR_UNINT_BYTE     = 0x90,
    CR95HF_ERROR_UNSUPPORTED    = 0xF0, // added, for simplicity
};

typedef uint8_t cr95hf_error_t;

// library event flags
#define CR95HF_FLAGS_TIMEOUT  0b1
#define CR95HF_FLAGS_RX_DONE  0b10

class CR95HF {
  public:
    // datasheet - table 11
    typedef enum {
        FIELD_OFF = 0x00,
        ISO15693,
        ISO14443A,
        ISO14443B,
        ISO18092
    } protocol_t;

    CR95HF(PinName tx, PinName rx, int baud = 57600);
    ~CR95HF();

    /**
     * @brief Initialize communication
     *
     * @return cr95hf_error_t error
     */
    cr95hf_error_t init();

    /**
     * @brief Poll to see if a tag is in range
     *
     * @return true/false
     */
    bool isTagInRange();

    /**
     * @brief Set the Protocol
     *
     * @param protocol
     * @return cr95hf_error_t error
     */
    cr95hf_error_t setProtocol(protocol_t protocol);

    /**
     * @brief Get the Tag UID, isTagInRange() has to be called first!
     *
     * @param uid pointer to where to copy received UID
     * @param tag_type optional variable to get a TAG type
     * @return 0 in case failure or length of UID
     */
    uint8_t getTagUID(uint8_t *uid, uint8_t *tag_type = nullptr);

    /**
     * @brief Performs a field calibration
     *
     * @return cr95hf_error_t error
     */
    cr95hf_error_t calibration();

  private:
    // datasheet - table 9
    typedef enum {
        IDN = 0x01,
        PROTOCOL_SELECT,
        POLL_FIELD,
        SEND_RECV,
        LISTEN,
        SEND,
        IDLE,
        READ_REGISTER,
        WRITE_REGISTER,
        BAUD_RATE,
        SUB_FREQ_RES,
        AC_FILTER = 0x0D,
        TEST_MOD,
        SLEEP_MODE,
        ECHO = 0x55
    } command_t;

    /**
     * @brief Sends data located in _tx_buf; buffer length is taken from _tx_buf[1]
     *
     * @return cr95hf_error_t response code
     */
    cr95hf_error_t send();

    /**
     * @brief Callback for incoming data (ISR)
     *
     */
    void rxCb();

    /**
     * @brief Performs ISO/IEC 14443 ANTICOLLISION command
     *
     * @param level CL1, CL2 or CL3
     * @return success/failure
     */
    bool anticol(uint8_t level);

    /**
     * @brief Performs ISO/IEC 14443 SELECT command
     *
     * @param level CL1, CL2 or CL3
     * @return SAK (select acknowledge) or 0xFF in case of failure
     */
    uint8_t select(uint8_t level);

    UnbufferedSerial _serial;
    EventFlags _flags;
    protocol_t _protocol = FIELD_OFF;

    uint8_t _tx_buf[24];
    uint8_t _rx_buf[127];
    uint8_t _rx_buf_size = 0; // for serial parser
    uint8_t _data_len = 0; // for serial parser

    uint8_t _uid[10];
    uint8_t _uid_offset = 0;
    uint8_t _uid_len = 0;
    const uint8_t _cl_level[3] = {
        MIFARE_CL_1,
        MIFARE_CL_2,
        MIFARE_CL_3
    };
};

#endif  // CR95HF_H
