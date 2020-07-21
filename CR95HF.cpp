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
#include <CR95HF.h>

CR95HF::CR95HF(PinName tx, PinName rx) :
    _serial(tx, rx, 57600) {
    _serial.attach(
        callback(this, &CR95HF::rxCb),
        SerialBase::RxIrq
    );
}

CR95HF::~CR95HF() {};

bool CR95HF::init() {
    tr_info("CR95HF init");
    _tx_buf[0] = IDN;
    _tx_buf[1] = 0;

    if (!send()) {
        return false;
    }

    if (_rx_buf[0] != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return false;
    }

    if (_rx_buf[CR95HF_HEADER_LEN] != 'N' ||
            _rx_buf[CR95HF_HEADER_LEN + 1] != 'F' ||
            _rx_buf[CR95HF_HEADER_LEN + 2] != 'C') {
        tr_error("Invalid packet");
        return false;
    }

    _ic_rev = _rx_buf[CR95HF_ROM_CODE_REVISION_OFFSET];

    tr_debug("IC rev: %02X", _ic_rev);

    return true;
}

bool CR95HF::setProtocol(Protocol protocol) {
    bool state = false;
    _protocol = protocol;

    _tx_buf[0] = PROTOCOL_SELECT;
    _tx_buf[1] = 1; // payload_len
    _tx_buf[2] = protocol;

    switch (protocol) {
        case FIELD_OFF:
            tr_info("Turning off");
            _tx_buf[3] = 0x00;
            _tx_buf[1] += 1; // payload_len

            send();

            state = true;
            break;

        case ISO14443A:
            tr_info("Protocol set to ISO14443A");
            // select protocol, datasheet - page 22
            _tx_buf[3] = 0x00;
            _tx_buf[4] = 0x00;
            _tx_buf[5] = 0x00;
            _tx_buf[6] = 0x00;
            _tx_buf[1] += 4; // payload_len

            if (_ic_rev >= CR95HF_IC_REV_QJE) {
                _tx_buf[7] = 0x02;
                _tx_buf[8] = 0x02;

                _tx_buf[1] += 2; // payload_len
            }

            if (!send() || _rx_buf[0] != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return false;
            }

            // set synchronization level
            _tx_buf[0] = WRITE_REGISTER;
            _tx_buf[1] = 4; // payload_len
            _tx_buf[2] = CR95HF_TIMERW_VALUE;
            _tx_buf[3] = 0x00;
            _tx_buf[4] = 0x58; // TimerW - value between 0x50-0x60
            _tx_buf[5] = CR95HF_TIMERW_CONFIRMATION;

            if (!send() || _rx_buf[0] != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return false;
            }

            // modulation and gain
            _tx_buf[0] = WRITE_REGISTER;
            _tx_buf[1] = 4; // payload_len
            _tx_buf[2] = CR95HF_ANALOG_CONFIGURATION_ADDR;
            _tx_buf[3] = 0x01;
            _tx_buf[4] = 0x01;
            _tx_buf[5] = 0xD7; // modulation & gain - value 0xD0, 0xD1, 0xD3, 0xD7 or 0xDF

            if (!send() || _rx_buf[0] != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return false;
            }

            state = true;
            break;

        default:
            break;
    }

    return state;
}

// TODO
bool CR95HF::calibration() {
    _tx_buf[0]  = IDLE; // cmd
    _tx_buf[1]  = 0x0E; // payload_len
    _tx_buf[2]  = 0x03; // WU control
    _tx_buf[3]  = 0xA1; // Enter control - tag detector calibration
    _tx_buf[4]  = 0x00;
    _tx_buf[5]  = 0xF8; // WU control - tag detector calibration
    _tx_buf[6]  = 0x01;
    _tx_buf[7]  = 0x18; // Leave control
    _tx_buf[8]  = 0x00;
    _tx_buf[9]  = 0x20; // WU period
    _tx_buf[10] = 0x60; // Osc start
    _tx_buf[11] = 0x60; // DAC start
    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x00;
    _tx_buf[14] = 0x3F; // Swing count
    _tx_buf[15] = 0x01; // Max sleep

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0xFC;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x7C;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x3C;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x5C;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x6C;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x74;

    send();

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x70;

    send();

    return true;
}

bool CR95HF::anticol(uint8_t level, uint8_t tag_copy_len) {
    tr_debug("ANTICOL");
    _tx_buf[0] = SEND_RECV;
    _tx_buf[1] = 3; // payload_len
    _tx_buf[2] = level;
    _tx_buf[3] = 0x20;
    _tx_buf[4] = 0x08;

    if (!send() || _rx_buf[0] != CR95HF_ERROR_RESPONSE) {
        return false;
    }

    uint8_t collision = (_rx_buf[_rx_buf[1] - 1] & 0x80);
    uint8_t len_origin = _rx_buf[1];
    uint8_t byte_collision_index = _rx_buf[_rx_buf[1]];
    uint8_t bit_collision_index = _rx_buf[_rx_buf[1] + 1];
    uint8_t new_byte_collision_index = 0;
    uint8_t new_bit_collision_index = 0;
    uint8_t remaining_bit = 0;
    uint8_t tag[4] = {0};

    tr_debug("Collision: %u", collision);

    if (bit_collision_index == 0x08) {
        tr_error("Previous collision missed");
        return false;
    }

    while (collision == 0x80) {
        collision = 0x00;
        _tx_buf[2 + 1] = 0x20 + ((byte_collision_index) << 4) + (bit_collision_index + 1);

        if (byte_collision_index == 0) {
            _tx_buf[2 + 2] = _rx_buf[2] &
                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[2 + 3] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[2 + 2];

        } else if (byte_collision_index == 1) {
            _tx_buf[2 + 2] = _rx_buf[2];
            _tx_buf[2 + 3] = _rx_buf[3] &
                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[2 + 4] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[2 + 2];
            tag[1] = _tx_buf[2 + 3];

        } else if (byte_collision_index == 2) {
            _tx_buf[2 + 2] = _rx_buf[2];
            _tx_buf[2 + 3] = _rx_buf[3];
            _tx_buf[2 + 4] = _rx_buf[4] &
                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[2 + 5] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[2 + 2];
            tag[1] = _tx_buf[2 + 3];
            tag[2] = _tx_buf[2 + 4];

        } else if (byte_collision_index == 3) {
            _tx_buf[2 + 2] = _rx_buf[2];
            _tx_buf[2 + 3] = _rx_buf[3];
            _tx_buf[2 + 4] = _rx_buf[4];
            _tx_buf[2 + 5] = _rx_buf[5] &
                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[2 + 6] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[2 + 2];
            tag[1] = _tx_buf[2 + 3];
            tag[2] = _tx_buf[2 + 4];
            tag[3] = _tx_buf[2 + 5];

        } else {
            tr_error("Invalid byte collision");
            return false;
        }

        _tx_buf[1] = 3 + byte_collision_index + 1; // payload_len

        if (!send() || _rx_buf[0] != CR95HF_ERROR_RESPONSE) {
            tr_error("Invalid response");
            return false;
        }

        // check if there is another collision to take into account
        collision = (_rx_buf[_rx_buf[1] - 1]) & 0x80;

        if (collision == 0x80) {
            new_byte_collision_index = _rx_buf[_rx_buf[1]];
            new_bit_collision_index = _rx_buf[_rx_buf[1] + 1];
        }

        /* we can check that non-alignement is the one expected */
        remaining_bit = 8 - (0x0F & (_rx_buf[2 + (_rx_buf[1] - 2) - 1]));

        if (remaining_bit == bit_collision_index + 1) {
            /* recreate the good UID*/
            if (byte_collision_index == 0) {
                tag[0] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[2 + 2]) | _rx_buf[2] ;
                tag[1] = _rx_buf[3];
                tag[2] = _rx_buf[4];
                tag[3] = _rx_buf[5];

            } else if (byte_collision_index == 1) {
                tag[1] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[2 + 3]) | _rx_buf[2] ;
                tag[2] = _rx_buf[3];
                tag[3] = _rx_buf[4];

            } else if (byte_collision_index == 2) {
                tag[2] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[2 + 4]) | _rx_buf[2] ;
                tag[3] = _rx_buf[3];

            } else if (byte_collision_index == 3) {
                tag[3] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[2 + 5]) | _rx_buf[2] ;

            } else {
                tr_error("Invalid byte collision");
                return false;
            }

        } else {
            tr_error("Invalid remaining bit");
            return false;
        }

        /* prepare the buffer expected by the caller */
        _rx_buf[0] = 0x80;
        _rx_buf[1] = len_origin;
        _rx_buf[2] = tag[0];
        _rx_buf[3] = tag[1];
        _rx_buf[4] = tag[2];
        _rx_buf[5] = tag[3];
        _rx_buf[6] = tag[0] ^ tag[1] ^ tag[2] ^ tag[3];

        // if collision was detected restart anticol
        if (collision == 0x80) {
            if (byte_collision_index != new_byte_collision_index) {
                byte_collision_index += new_byte_collision_index;
                bit_collision_index = new_bit_collision_index;

            } else {
                byte_collision_index += new_byte_collision_index;
                bit_collision_index += (new_bit_collision_index + 1);
            }
        }
    }

    if (tag_copy_len == 4) {
        memcpy(UID, _rx_buf + CR95HF_HEADER_LEN, 4);

    } else {
        memcpy(UID, _rx_buf + CR95HF_HEADER_LEN + 1, 3);
    }

    tr_debug("UID part[%u]: %s", tag_copy_len, tr_array(UID, tag_copy_len));

    return true;
}

bool CR95HF::select(uint8_t level, uint8_t tag_copy_len) {
    tr_debug("SELECT");
    uint8_t bcc_byte = _rx_buf[2 + 4];

    _tx_buf[0] = SEND_RECV;
    _tx_buf[2] = level;
    _tx_buf[3] = 0x70;

    if (tag_copy_len == 4) {
        memcpy(_tx_buf + 4, UID, 4);

    } else {
        _tx_buf[4] = 0x88;
        memcpy(_tx_buf + 5, UID, 3);
    }

    _tx_buf[8] = bcc_byte;
    _tx_buf[9] = 0x20 | 0x8; // append CRC + 8 bits in first byte

    _tx_buf[1] = 8; // payload_len

    if (!send() || _rx_buf[0] != CR95HF_ERROR_RESPONSE) {
        return false;
    }

    tr_debug("SAK: %u", _rx_buf[2]);

    return true;
}

bool CR95HF::poll() {
    switch (_protocol) {
        case ISO14443A: {
            // REQA
            _tx_buf[0] = SEND_RECV;
            _tx_buf[1] = 2; // payload_len
            _tx_buf[2] = 0x26;
            _tx_buf[3] = 0x07;

            // catch timeout or other errors
            if (!send() || _rx_buf[0] != CR95HF_ERROR_RESPONSE) {
                return false;
            }

            uint8_t tag_copy_len = _rx_buf[CR95HF_HEADER_LEN];

            tr_info("TAG in range");

            if (!anticol(CR95HF_SEL_CASCADE_LVL_1, tag_copy_len)) {
                return false;
            }

            if (!select(CR95HF_SEL_CASCADE_LVL_1, tag_copy_len)) {
                return false;
            }
        }
        break;

        default:
            break;
    }

    return true;
}

bool CR95HF::send(bool wait) {
    uint8_t len = _tx_buf[1] + CR95HF_HEADER_LEN;
    tr_debug("Sending[%u]: %s", len, tr_array(static_cast<const uint8_t *>(_tx_buf), len));

    // zero RX buffer
    memset(_rx_buf, 0, sizeof(_rx_buf));
    _rx_buf_size = 0;

    _serial.write(_tx_buf, len);

    if (wait) {
        uint32_t wait = _flags.wait_all(CR95HF_FLAGS_RX_DONE, TIMEOUT);

        if (wait != CR95HF_FLAGS_RX_DONE) {
            tr_error("RX timeout");
            return false;
        }

        tr_debug("Response[%u]: %s", _rx_buf_size, tr_array(_rx_buf, _rx_buf_size));
    }

    return true;
}

void CR95HF::rxCb() { // ISR
    uint8_t character;

    while (_serial.readable()) {
        ssize_t len = _serial.read(&character, 1);

        if (len == 1) {
            _rx_buf[_rx_buf_size] = character;
            _rx_buf_size++;

            process();

            // prevent overflow
            if (_rx_buf_size >= sizeof(_rx_buf)) {
                _rx_buf_size = 0;
            }
        }
    }
}

void CR95HF::process() { // ISR
    if (_rx_buf_size == 2) { // got length
        if (_rx_buf[1] == 0) {
            _flags.set(CR95HF_FLAGS_RX_DONE);

        } else {
            _data_len = _rx_buf[1] + CR95HF_HEADER_LEN;
        }

    } else if (_rx_buf_size > 2 && _rx_buf_size == _data_len) {
        _flags.set(CR95HF_FLAGS_RX_DONE);
    }
}

uint8_t CR95HF::checksum(const void *data, size_t len) {
    const auto *buffer = static_cast<const char *>(data);
    uint8_t result = 0;

    for (size_t i = 0 ; i < len ; i++) {
        result ^= buffer[i];
    }

    return result;
}
