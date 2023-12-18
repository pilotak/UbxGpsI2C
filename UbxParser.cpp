/*
MIT License

Copyright (c) 2024 Pavel Slama

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

#include "UbxParser.h"

UbxParser::UbxParser() {}

UbxParser::~UbxParser(void) {
    while (_oobs) {
        struct oob_t *oob = _oobs;
        _oobs = oob->next;
        delete oob;
    }
}

uint16_t UbxParser::buildMessage(char *buffer, UbxClassId class_id, char id, const void *payload,
                                 uint16_t payload_len) {
    uint8_t checksum[2] = {0};

    buffer[0] = UBX_SYNC_CHAR1;
    buffer[1] = UBX_SYNC_CHAR2;
    buffer[2] = class_id;
    buffer[3] = id;
    buffer[4] = payload_len & UCHAR_MAX;
    buffer[5] = payload_len >> 8;

    if (payload_len > 0 && payload != nullptr) {
        memcpy(buffer + UBX_HEADER_LEN, payload, payload_len);
    }

    for (uint16_t i = 2; i < (UBX_HEADER_LEN + payload_len); i++) {  // exclude SYNC
        checksum[0] += buffer[i];
        checksum[1] += checksum[0];
    }

    memcpy(buffer + UBX_HEADER_LEN + payload_len, checksum, sizeof(checksum));

    return UBX_HEADER_LEN + payload_len + UBX_CHECKSUM_LEN;
}

void UbxParser::oob(UbxClassId class_id, char id, Callback<void()> cb) {
    // In case we have it already in, remove it
    remove_oob(class_id, id);

    ubx_debug("Registering oob CLASS: %02hhX, ID: %02hhX", class_id, id);

    struct oob_t *oob = new struct oob_t;
    oob->class_id = class_id;
    oob->id = id;
    oob->cb = cb;
    oob->next = _oobs;
    _oobs = oob;
}

void UbxParser::remove_oob(UbxClassId class_id, char id) {
    struct oob_t *prev = nullptr;
    struct oob_t *oob = _oobs;

    while (oob) {
        if (oob->class_id == class_id && oob->id == id) {
            if (prev) {
                prev->next = oob->next;

            } else {
                _oobs = oob->next;
            }

            ubx_debug("Removing oob CLASS: %02hhX, ID: %02hhX", class_id, id);
            delete oob;
            return;
        }

        prev = oob;
        oob = oob->next;
    }
}

void UbxParser::parse(const char *buffer, uint16_t length) {
    static StateMachine state = StateMachine::SYNC_SEARCH_1;
    static uint16_t data_index = 0;

    // ubx_debug("Read: %s", tr_array(reinterpret_cast<const uint8_t *>(buffer), length));

    for (uint16_t i = 0; i < length; i++) {
        switch (state) {
            case StateMachine::SYNC_SEARCH_1:
                if (buffer[i] == UBX_SYNC_CHAR1) {
                    state = StateMachine::SYNC_SEARCH_2;
                }
                break;

            case StateMachine::SYNC_SEARCH_2:
                if (buffer[i] == UBX_SYNC_CHAR2) {
                    state = StateMachine::CLASS_READ;
                } else {
                    state = StateMachine::SYNC_SEARCH_1;
                }
                break;

            case StateMachine::CLASS_READ:
                msg.classId = buffer[i];
                state = StateMachine::ID_READ;
                add_checksum(buffer[i]);

                break;

            case StateMachine::ID_READ: {
                bool process = false;
                msg.id = buffer[i];

                for (struct oob_t *oob = _oobs; oob; oob = oob->next) {
                    if (oob->class_id == msg.classId && (oob->class_id == UBX_ACK || oob->id == msg.id)) {
                        ubx_debug("This is a packet we are looking for");
                        process = true;
                        break;
                    }
                }

                if (process) {
                    state = StateMachine::LENGTH_READ_1;
                    add_checksum(buffer[i]);
                } else {
                    ubx_warning("Skipping class: %02X, id: %02X", msg.classId, msg.id);
                    state = StateMachine::SYNC_SEARCH_1;
                    memset(_checksum_calc, 0, sizeof(_checksum_calc));  // reset checksum
                }
                break;
            }

            case StateMachine::LENGTH_READ_1:
                msg.length = buffer[i];
                state = StateMachine::LENGTH_READ_2;
                add_checksum(buffer[i]);
                break;

            case StateMachine::LENGTH_READ_2:
                msg.length |= ((uint16_t)buffer[i] << 8);
                msg.data = std::make_unique<uint8_t[]>(msg.length);
                data_index = 0;
                state = StateMachine::DATA_READ;
                add_checksum(buffer[i]);

                ubx_debug("Message length: %u", msg.length);
                break;

            case StateMachine::DATA_READ:
                msg.data[data_index++] = buffer[i];
                add_checksum(buffer[i]);

                if (data_index >= msg.length) {
                    state = StateMachine::CHECKSUM_READ_1;
                }
                break;

            case StateMachine::CHECKSUM_READ_1:
                msg.checksum = ((uint16_t)buffer[i] << 8);
                state = StateMachine::CHECKSUM_READ_2;
                break;

            case StateMachine::CHECKSUM_READ_2:
                msg.checksum |= buffer[i];
                uint16_t calculatedChecksum = (_checksum_calc[0] << 8) | _checksum_calc[1];

                if (msg.checksum == calculatedChecksum) {
                    ubx_debug("Checksum OK");

                    for (struct oob_t *oob = _oobs; oob; oob = oob->next) {
                        if (oob->class_id == msg.classId && (oob->class_id == UBX_ACK || oob->id == msg.id)) {
                            oob->cb.call();
                            break;
                        }
                    }

                } else {
                    ubx_error("Wrong checksum");
                }

                msg = message_t();                    // resets automatically due to unique_ptr
                state = StateMachine::SYNC_SEARCH_1;  // reset to initial state
                data_index = 0;
                memset(_checksum_calc, 0, sizeof(_checksum_calc));  // reset checksum

                break;
        }
    }
}

void UbxParser::add_checksum(char b) {
    _checksum_calc[0] += +b;
    _checksum_calc[1] += _checksum_calc[0];
}