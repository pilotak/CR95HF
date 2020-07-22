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

CR95HF::CR95HF(PinName tx, PinName rx, int baud):
    _serial(tx, rx, baud) {
    _serial.attach(
        callback(this, &CR95HF::rxCb),
        SerialBase::RxIrq
    );
}

CR95HF::~CR95HF() {};

cr95hf_error_t CR95HF::init() {
    tr_info("Init");
    cr95hf_error_t res = CR95HF_ERROR_OK;
    _tx_buf[0] = IDN;
    _tx_buf[1] = 0;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    // IDN device type check
    if (_rx_buf[CR95HF_HEADER_LEN] != 'N' ||
            _rx_buf[CR95HF_HEADER_LEN + 1] != 'F' ||
            _rx_buf[CR95HF_HEADER_LEN + 2] != 'C') {
        tr_error("Invalid device type");
        return CR95HF_ERROR_UNSUPPORTED;
    }

    return res;
}

cr95hf_error_t CR95HF::setProtocol(protocol_t protocol) {
    cr95hf_error_t res = CR95HF_ERROR_OK;
    _protocol = protocol;

    _tx_buf[0] = PROTOCOL_SELECT;
    _tx_buf[1] = 1; // payload_len
    _tx_buf[2] = protocol;

    switch (protocol) {
        case FIELD_OFF:
            tr_info("Turning off");
            _tx_buf[3] = 0x00;
            _tx_buf[1] += 1; // payload_len

            res = send();

            if (res != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return res;
            }

            break;

        case ISO14443A:
            tr_info("Setting protocol to ISO14443A");
            // select protocol, datasheet table 12
            _tx_buf[3] = 0x00;
            _tx_buf[1] += 1; // payload_len

            res = send();

            if (res != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return res;
            }

            // set synchronization level
            _tx_buf[0] = WRITE_REGISTER;
            _tx_buf[1] = 4; // payload_len
            _tx_buf[2] = CR95HF_TIMERW_VALUE;
            _tx_buf[3] = 0x00;
            _tx_buf[4] = 0x58; // TimerW - value between 0x50-0x60
            _tx_buf[5] = CR95HF_TIMERW_CONFIRMATION;

            res = send();

            if (res != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return res;
            }

            // modulation and gain
            _tx_buf[0] = WRITE_REGISTER;
            _tx_buf[1] = 4; // payload_len
            _tx_buf[2] = CR95HF_ANALOG_CONFIGURATION_ADDR;
            _tx_buf[3] = 0x01;
            _tx_buf[4] = 0x01;
            _tx_buf[5] = 0xD7; // modulation & gain - value 0xD0, 0xD1, 0xD3, 0xD7 or 0xDF

            res = send();

            if (res != CR95HF_ERROR_OK) {
                tr_error("Invalid response");
                return res;
            }

            break;

        default:
            tr_info("Unsupported protocol");
            res = CR95HF_ERROR_UNSUPPORTED;
            break;
    }

    return res;
}

// TODO
cr95hf_error_t CR95HF::calibration() {
    tr_info("Performing calibration");
    cr95hf_error_t res = CR95HF_ERROR_OK;

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

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0xFC;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x7C;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x3C;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x5C;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x6C;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x74;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    _tx_buf[12] = 0x00; // DAC data
    _tx_buf[13] = 0x70;

    res = send();

    if (res != CR95HF_ERROR_OK) {
        tr_error("Invalid response");
        return res;
    }

    return res;
}

bool CR95HF::isTagInRange() {
    tr_debug("Searching to tag");

    switch (_protocol) {
        case ISO14443A: {

            // REQA
            _tx_buf[0] = SEND_RECV;
            _tx_buf[1] = 2; // payload_len
            _tx_buf[2] = 0x26;
            _tx_buf[3] = 0x07;

            if (send() != CR95HF_ERROR_FRAME_RECV_OK) {
                return false;
            }

            // _rx_buf holds ATQA
            if (_rx_buf[CR95HF_HEADER_LEN] & 0b00100000) { // RFU
                tr_error("RFU must be zero");
                return false;
            }

            _uid_offset = 0; // reset offset
            _uid_len = _rx_buf[CR95HF_HEADER_LEN] >> 6;

            if (_uid_len == 0b00) {
                _uid_len = MIFARE_UID_SINGLE_SIZE;

            } else if (_uid_len == 0b01) {
                _uid_len = MIFARE_UID_DOUBLE_SIZE;

            } else if (_uid_len == 0b10) {
                _uid_len = MIFARE_UID_TRIPLE_SIZE;

            } else {
                tr_error("Invalid UID len");
                _uid_len = 0;
                return false;
            }

            tr_info("TAG in range, len: %u", _uid_len);
            return true;
        }

        default:
            break;

    }

    return false;
}

uint8_t CR95HF::getTagUID(uint8_t *uid, uint8_t *tag_type) {
    tr_info("Getting tag UID");

    switch (_protocol) {
        case ISO14443A: {
            uint8_t sak = MIFARE_SAK_UID_NOT_COMPLETE;

            // Anticollision loop
            for (auto i = 0; i < 3; i++) {
                if (sak == MIFARE_SAK_UID_NOT_COMPLETE) {
                    if (!anticol(_cl_level[i])) {
                        return 0;
                    }

                    sak = select(_cl_level[i]);

                    if (sak == 0xFF) {
                        return 0;
                    }
                }
            }

            if (uid) {
                memcpy(uid, _uid, _uid_offset);
            }

            if (tag_type) {
                *tag_type = sak;
            }

            tr_info("Tag UID: %s", tr_array(_uid, _uid_offset));
            return _uid_offset;
        }
        break;

        default:
            break;
    }

    return 0;
}

bool CR95HF::anticol(uint8_t level) {
    tr_debug("Anticol request");
    _tx_buf[0] = SEND_RECV;
    _tx_buf[1] = 3; // payload_len
    _tx_buf[2] = level;
    _tx_buf[3] = 0x20;
    _tx_buf[4] = 0x08;

    if (send() != CR95HF_ERROR_FRAME_RECV_OK) {
        tr_error("Invalid response");
        return false;
    }

    uint8_t collision = (_rx_buf[_rx_buf[1] - 1] & 0x80);
    uint8_t len_origin = _rx_buf[1];
    uint8_t byte_collision_index = _rx_buf[_rx_buf[1]];
    uint8_t bit_collision_index = _rx_buf[_rx_buf[1] + 1];
    uint8_t new_byte_collision_index = 0;
    uint8_t new_bit_collision_index = 0;
    uint8_t tag[4] = {0};

    tr_debug("Collision: %u", collision);

    if (bit_collision_index == 0x08) {
        tr_error("Previous collision missed");
        return false;
    }

    while (collision == 0x80) {
        collision = 0x00;
        _tx_buf[CR95HF_HEADER_LEN + 1] = 0x20 + ((byte_collision_index) << 4) + (bit_collision_index + 1);

        if (byte_collision_index == 0) {
            _tx_buf[CR95HF_HEADER_LEN + 2] = _rx_buf[2] &
                                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[CR95HF_HEADER_LEN + 3] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[CR95HF_HEADER_LEN + 2];

        } else if (byte_collision_index == 1) {
            _tx_buf[CR95HF_HEADER_LEN + 2] = _rx_buf[2];
            _tx_buf[CR95HF_HEADER_LEN + 3] = _rx_buf[3] &
                                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[CR95HF_HEADER_LEN + 4] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[CR95HF_HEADER_LEN + 2];
            tag[1] = _tx_buf[CR95HF_HEADER_LEN + 3];

        } else if (byte_collision_index == 2) {
            _tx_buf[CR95HF_HEADER_LEN + 2] = _rx_buf[2];
            _tx_buf[CR95HF_HEADER_LEN + 3] = _rx_buf[3];
            _tx_buf[CR95HF_HEADER_LEN + 4] = _rx_buf[4] &
                                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[CR95HF_HEADER_LEN + 5] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[CR95HF_HEADER_LEN + 2];
            tag[1] = _tx_buf[CR95HF_HEADER_LEN + 3];
            tag[2] = _tx_buf[CR95HF_HEADER_LEN + 4];

        } else if (byte_collision_index == 3) {
            _tx_buf[CR95HF_HEADER_LEN + 2] = _rx_buf[2];
            _tx_buf[CR95HF_HEADER_LEN + 3] = _rx_buf[3];
            _tx_buf[CR95HF_HEADER_LEN + 4] = _rx_buf[4];
            _tx_buf[CR95HF_HEADER_LEN + 5] = _rx_buf[5] &
                                             ((uint8_t)(~(0xFF << (bit_collision_index + 1)))); // ISO said it's better to put collision bit to value 1
            _tx_buf[CR95HF_HEADER_LEN + 6] = (bit_collision_index + 1) | 0x40; // add split frame bit
            tag[0] = _tx_buf[CR95HF_HEADER_LEN + 2];
            tag[1] = _tx_buf[CR95HF_HEADER_LEN + 3];
            tag[2] = _tx_buf[CR95HF_HEADER_LEN + 4];
            tag[3] = _tx_buf[CR95HF_HEADER_LEN + 5];

        } else {
            tr_error("Invalid byte collision");
            return false;
        }

        _tx_buf[1] = 3 + byte_collision_index + 1; // payload_len

        if (send() != CR95HF_ERROR_FRAME_RECV_OK) {
            tr_error("Invalid response");
            return false;
        }

        // check if there is another collision to take into account
        collision = (_rx_buf[_rx_buf[1] - 1]) & 0x80;

        if (collision == 0x80) {
            new_byte_collision_index = _rx_buf[_rx_buf[1]];
            new_bit_collision_index = _rx_buf[_rx_buf[1] + 1];
        }

        // we can check that non-alignement is the one expected
        uint8_t remaining_bit = 8 - (0x0F & (_rx_buf[CR95HF_HEADER_LEN + (_rx_buf[1] - 2) - 1]));

        if (remaining_bit == bit_collision_index + 1) {
            // recreate the good UID
            if (byte_collision_index == 0) {
                tag[0] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[CR95HF_HEADER_LEN + 2]) | _rx_buf[2] ;
                tag[1] = _rx_buf[3];
                tag[2] = _rx_buf[4];
                tag[3] = _rx_buf[5];

            } else if (byte_collision_index == 1) {
                tag[1] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[CR95HF_HEADER_LEN + 3]) | _rx_buf[2] ;
                tag[2] = _rx_buf[3];
                tag[3] = _rx_buf[4];

            } else if (byte_collision_index == 2) {
                tag[2] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[CR95HF_HEADER_LEN + 4]) | _rx_buf[2] ;
                tag[3] = _rx_buf[3];

            } else if (byte_collision_index == 3) {
                tag[3] = ((~(0xFF << (bit_collision_index + 1))) & _tx_buf[CR95HF_HEADER_LEN + 5]) | _rx_buf[2] ;

            } else {
                tr_error("Invalid byte collision");
                return false;
            }

        } else {
            tr_error("Invalid remaining bit");
            return false;
        }

        // prepare the buffer expected by the caller
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

    if ((_uid_len == MIFARE_UID_SINGLE_SIZE && level == MIFARE_CL_1) ||
            (_uid_len == MIFARE_UID_DOUBLE_SIZE && level == MIFARE_CL_2) ||
            (_uid_len == MIFARE_UID_TRIPLE_SIZE && level == MIFARE_CL_3)) {
        memcpy(_uid + _uid_offset, _rx_buf + CR95HF_HEADER_LEN, 4);
        _uid_offset += 4;

    } else { // UID_PART
        memcpy(_uid + _uid_offset, _rx_buf + CR95HF_HEADER_LEN + 1, 3);
        _uid_offset += 3;
    }

    tr_debug("UID[%u] so far: %s", _uid_offset, tr_array(_uid, _uid_offset));

    return true;
}

uint8_t CR95HF::select(uint8_t level) {
    tr_debug("Select request");
    _tx_buf[0] = SEND_RECV;
    _tx_buf[1] = 8; // payload_len
    _tx_buf[2] = level;
    _tx_buf[3] = 0x70;

    memcpy(_tx_buf + 4, _rx_buf + 2, 4); // copy UID or CT+UID

    _tx_buf[8] = _rx_buf[_rx_buf[1] - 2]; // copy BCC
    _tx_buf[9] = 0x20 | 0x8; // append CRC + 8 bits in first byte

    if (send() != CR95HF_ERROR_FRAME_RECV_OK) {
        tr_error("Invalid response");
        return 0xFF;
    }

    tr_debug("sak: %u", _rx_buf[2]);

    return _rx_buf[2];
}

cr95hf_error_t CR95HF::send() {
    uint8_t len = _tx_buf[1] + CR95HF_HEADER_LEN;
    tr_debug("Sending[%u]: %s", len, tr_array(static_cast<const uint8_t *>(_tx_buf), len));

    // zero RX buffer
    memset(_rx_buf, 0, sizeof(_rx_buf));
    _rx_buf_size = 0;

    _serial.write(_tx_buf, len);

    uint32_t wait = _flags.wait_all(CR95HF_FLAGS_RX_DONE, MBED_CONF_CR95HF_TIMEOUT);

    if (wait != CR95HF_FLAGS_RX_DONE) {
        tr_error("RX timeout");
        return CR95HF_ERROR_TIMEOUT;
    }

    tr_debug("Response[%u]: %s", _rx_buf_size, tr_array(_rx_buf, _rx_buf_size));

    return _rx_buf[0];
}

void CR95HF::rxCb() {
    uint8_t character;

    while (_serial.readable()) {
        ssize_t len = _serial.read(&character, 1);

        if (len == 1) {
            _rx_buf[_rx_buf_size] = character;
            _rx_buf_size++;

            if (_rx_buf_size == 2) { // got length byte
                if (_rx_buf[1] == 0) { // zero len
                    _flags.set(CR95HF_FLAGS_RX_DONE);

                } else {
                    _data_len = _rx_buf[1] + CR95HF_HEADER_LEN;
                }

            } else if (_rx_buf_size > CR95HF_HEADER_LEN && _rx_buf_size == _data_len) {
                _flags.set(CR95HF_FLAGS_RX_DONE);
            }

            // prevent overflow
            if (_rx_buf_size >= sizeof(_rx_buf)) {
                _rx_buf_size = 0;
            }
        }
    }
}
