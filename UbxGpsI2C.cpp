/*
MIT License

Copyright (c) 2019 Pavel Slama

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

#include "mbed.h"
#include "UbxGpsI2C.h"

UbxGpsI2C::UbxGpsI2C(int8_t address):
    _cb(NULL),
    _i2c_addr(address) {
}

UbxGpsI2C::UbxGpsI2C(PinName sda, PinName scl, int8_t address, uint32_t frequency):
    _cb(NULL),
    _i2c_addr(address) {
    ThisThread::sleep_for(100);
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
}

UbxGpsI2C::~UbxGpsI2C(void) {
    _cb = NULL;

    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

uint16_t UbxGpsI2C::checksum(const char * data, uint16_t payload_len) {
    uint8_t ca = 0;
    uint8_t cb = 0;
    uint16_t res = 0;

    for (uint16_t i = 2; i < (UBX_HEADER_LEN + payload_len); i++) {
        ca += data[i];
        cb += ca;
    }

    res =  (cb << 8) | ca;

    return res;
}

bool UbxGpsI2C::correctIndex() {
    uint16_t offset = USHRT_MAX;

    for (uint16_t i = 0; i < MBED_CONF_UBXGPSI2C_RX_SIZE; i++) {
        if (buffer[i] == UBX_SYNC_CHAR1 && buffer[i + 1] == UBX_SYNC_CHAR2) {  // find index of UBX_SYNC_CHAR1
            offset = i;
            break;
        }
    }

    if (offset != USHRT_MAX) {  // SYNC header found
        if (offset > 0) {
            memmove(buffer, buffer + offset, MBED_CONF_UBXGPSI2C_RX_SIZE - offset);  // correct
        }

        return true;
    }

    return false;
}

bool UbxGpsI2C::setOutputRate(uint16_t ms, uint16_t cycles) {
    cfg_rate_t cfg_rate;
    uint16_t payload_len = sendUbxSync(UBX_CFG, UBX_CFG_RATE);  // read current setting

    if (payload_len == sizeof(cfg_rate_t)) {
        memcpy(&cfg_rate, buffer + UBX_HEADER_LEN, sizeof(cfg_rate_t));
        cfg_rate.measRate = ms;
        cfg_rate.navRate = cycles;

        memcpy(_tx_buf, &cfg_rate, sizeof(cfg_rate_t));

        if (sendUbxSyncAck(UBX_CFG, UBX_CFG_RATE, _tx_buf, sizeof(cfg_rate_t))) {
            return true;
        }
    }

    return false;
}

bool UbxGpsI2C::init(event_callback_t cb, I2C * i2c_obj) {
    cfg_prt_t cfg_prt;
    _cb = cb;

    if (i2c_obj != NULL) {
        _i2c = i2c_obj;
    }

    char data = 0;  // DDC port
    uint16_t payload_len = sendUbxSync(UBX_CFG, UBX_CFG_PRT, &data, 1);  // read current setting

    if (payload_len == sizeof(cfg_prt_t)) {
        memcpy(&cfg_prt, buffer + UBX_HEADER_LEN, sizeof(cfg_prt_t));
        cfg_prt.outProtoMask &= ~0b110;  // output only UBX

        memcpy(_tx_buf, &cfg_prt, sizeof(cfg_prt_t));

        if (sendUbxSyncAck(UBX_CFG, UBX_CFG_PRT, _tx_buf, sizeof(cfg_prt_t))) {
            return true;
        }
    }

    return false;
}

uint16_t UbxGpsI2C::packetBuilder(UbxClassId class_id, uint8_t id, const char * data, uint16_t data_len) {
    if ((data_len + UBX_HEADER_LEN + UBX_CHECKSUM_LEN) <= MBED_CONF_UBXGPSI2C_TX_SIZE) {
        if (data != NULL && data_len > 0) {
            if (data == _tx_buf) {
                memmove(_tx_buf + UBX_HEADER_LEN, _tx_buf, data_len);

            } else {
                memcpy(_tx_buf + UBX_HEADER_LEN, data, data_len);
            }
        }

        _tx_buf[0] = UBX_SYNC_CHAR1;
        _tx_buf[1] = UBX_SYNC_CHAR2;
        _tx_buf[2] = class_id;
        _tx_buf[3] = id;
        _tx_buf[4] = data_len & UCHAR_MAX;
        _tx_buf[5] = (data_len >> 8) & UCHAR_MAX;

        uint16_t ck = checksum(_tx_buf, data_len);

        _tx_buf[(UBX_HEADER_LEN + data_len)] = (ck & 0xFF);
        _tx_buf[(UBX_HEADER_LEN + 1 + data_len)] = (ck >> 8);

        return (UBX_HEADER_LEN + UBX_CHECKSUM_LEN + data_len); // header + checksum + data_len
    }

    return USHRT_MAX;
}

bool UbxGpsI2C::sendUbx(UbxClassId class_id, uint8_t id, const char * data, uint16_t data_len) {
    uint16_t len = packetBuilder(class_id, id, data, data_len);

    if (len != USHRT_MAX) {
        if (_i2c != NULL && _i2c->transfer(
                    _i2c_addr,
                    _tx_buf,
                    len,
                    buffer,
                    MBED_CONF_UBXGPSI2C_RX_SIZE,
                    _cb,
                    I2C_EVENT_ALL) == 0) {
            return true;
        }
    }

    return false;
}

uint16_t UbxGpsI2C::sendUbxSync(UbxClassId class_id, uint8_t id, const char * data, uint16_t data_len) {
    uint16_t len = packetBuilder(class_id, id, data, data_len);
    int32_t ack = -1;

    if (len != USHRT_MAX) {
        if (_i2c != NULL) {
            // printf("send data: ");

            // for (int i = 0; i < len; ++i) {
            //     printf("%02X ", _tx_buf[i]);
            // }

            // printf("\n");
            _i2c->lock();
            ack = _i2c->write(_i2c_addr, _tx_buf, len);
            _i2c->unlock();

            if (ack == 0) {
                memset(buffer, UCHAR_MAX, MBED_CONF_UBXGPSI2C_RX_SIZE);

                _i2c->lock();
                ack = _i2c->read(_i2c_addr, buffer, MBED_CONF_UBXGPSI2C_RX_SIZE);
                _i2c->unlock();

                // printf("read data: ");

                // for (int i = 0; i < MBED_CONF_UBXGPSI2C_RX_SIZE; ++i) {
                //     printf("%02X ", buffer[i]);
                // }

                // printf("\n");

                if (ack == 0) {
                    if (correctIndex()) {
                        uint16_t payload_len = ((buffer[5] << 8) | buffer[4]);

                        if ((UBX_HEADER_LEN + payload_len) <= MBED_CONF_UBXGPSI2C_RX_SIZE) {
                            uint16_t ck = checksum(buffer, payload_len);

                            if (buffer[UBX_HEADER_LEN + payload_len] == (ck & UCHAR_MAX) &&
                                    buffer[UBX_HEADER_LEN + payload_len + 1] == ((ck >> 8))) {
                                return payload_len;
                            }
                        }
                    }
                }
            }
        }
    }

    return USHRT_MAX;
}

bool UbxGpsI2C::sendUbxSyncAck(UbxClassId class_id, uint8_t id, const char * data, uint16_t data_len) {
    uint16_t payload_len = sendUbxSync(class_id, id, data, data_len);

    if (payload_len > 0 && payload_len != USHRT_MAX) {
        if (buffer[2] == UBX_ACK && buffer[3] == UBX_ACK_ACK && buffer[6] == class_id && buffer[7] == id) {
            return true;
        }
    }

    return false;
}

bool UbxGpsI2C::setOdometer(bool enable, UbxOdoProfile profile, uint8_t velocity_filter) {
    uint16_t payload_len = sendUbxSync(UBX_CFG, UBX_CFG_ODO);  // read current setting

    if (payload_len == sizeof(odometer_t)) {
        odometer_t odo;
        memcpy(&odo, buffer + UBX_HEADER_LEN, sizeof(odometer_t));

        odo.odoCfg &= ~0b111;
        odo.odoCfg |= (uint8_t)profile;

        if (velocity_filter > 0) {
            odo.flags |= 0b100;

        } else {
            odo.flags &= ~0b100;
        }

        odo.velLpGain = velocity_filter;

        if (enable) {
            odo.flags |= 0b1;

        } else {
            odo.flags &= ~0b1;
        }

        memcpy(_tx_buf, &odo, sizeof(odometer_t));

        if (sendUbxSyncAck(UBX_CFG, UBX_CFG_ODO, _tx_buf, sizeof(odometer_t))) {
            return true;
        }
    }

    return false;
}

bool UbxGpsI2C::resetOdometer() {
    if (sendUbxSyncAck(UBX_NAV, UBX_NAV_RESETODO)) {
        return true;
    }

    return false;
}

bool UbxGpsI2C::autoSend(UbxClassId class_id, uint8_t id, uint8_t rate) {
    _tx_buf[0] = class_id;
    _tx_buf[1] = id;
    _tx_buf[2] = rate;

    return sendUbxSyncAck(UBX_CFG, UBX_CFG_MSG, _tx_buf, 3);
}

bool UbxGpsI2C::poll() {
    if (_i2c != NULL && _i2c->transfer(
                _i2c_addr,
                NULL,
                0,
                buffer,
                MBED_CONF_UBXGPSI2C_RX_SIZE,
                _cb,
                I2C_EVENT_ALL) == 0) {
        return true;
    }

    return false;
}
