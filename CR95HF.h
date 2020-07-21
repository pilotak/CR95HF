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

#define CR95HF_HEADER_LEN 2
#define CR95HF_IC_REV_QJE 0x34
#define CR95HF_ROM_CODE_REVISION_OFFSET 13

// table 21
#define CR95HF_ANALOG_CONFIGURATION_ADDR 0x68

//table 25
#define CR95HF_TIMERW_VALUE        0x3A
#define CR95HF_TIMERW_CONFIRMATION 0x04

#define CR95HF_SEL_CASCADE_LVL_1 0x93
#define CR95HF_SEL_CASCADE_LVL_2 0x95
#define CR95HF_SEL_CASCADE_LVL_3 0x97

#define CR95RF_SAK_FLAG_UID_NOT_COMPLETE 0x04


#define CR95HF_ERROR_OK       0x00
#define CR95HF_ERROR_RESPONSE 0x80
#define CR95HF_ERROR_COMM     0x86
#define CR95HF_ERROR_TIMEOUT  0x87
#define CR95HF_ERROR_SOF      0x88
#define CR95HF_ERROR_OVERFLOW 0x89
#define CR95HF_ERROR_FRAMING  0x8A
#define CR95HF_ERROR_EGT      0x8B
#define CR95HF_ERROR_LENGTH   0x8C
#define CR95HF_ERROR_CRC      0x8D
#define CR95HF_ERROR_LOST     0x8E

#define TIMEOUT 500
#define CR95HF_FLAGS_TIMEOUT  0b1
#define CR95HF_FLAGS_RX_DONE  0b10

class CR95HF {
  public:
    typedef enum {
        FIELD_OFF = 0x00,
        ISO15693,
        ISO14443A,
        ISO14443B,
        ISO18092
    } Protocol;

    CR95HF(PinName tx, PinName rx);
    ~CR95HF();
    bool init();
    bool setProtocol(Protocol protocol);
    bool poll();
    bool calibration();

    uint8_t UID[10];

  private:

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
    } Command;

    bool    send(bool wait = true);
    uint8_t checksum(const void *data, size_t len);
    void    rxCb();
    void    process();
    bool    anticol(uint8_t level, uint8_t tag_copy_len);
    bool    select(uint8_t level, uint8_t tag_copy_len);

    UnbufferedSerial _serial;
    EventFlags _flags;
    Protocol _protocol = FIELD_OFF;

    uint8_t _tx_buf[24];
    uint8_t _rx_buf[127];
    uint8_t _rx_buf_size = 0;
    uint8_t _data_len = 0;
    uint8_t _ic_rev = CR95HF_IC_REV_QJE;
    uint8_t _sak = 0;
    uint8_t _uid_offset = 0;
};

#endif  // CR95HF_H
