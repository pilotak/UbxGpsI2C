/*
MIT License

Copyright (c) 2018 Pavel Slama

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

UbxGpsI2C::UbxGpsI2C(I2C * i2c_obj, char * buffer, const uint16_t buf_size, int8_t address):
    _address(address),
    _buf_size(buf_size) {
    _i2c = i2c_obj;
    _buf = buffer;
}

UbxGpsI2C::UbxGpsI2C(PinName sda, PinName scl, char * buffer, const uint16_t buf_size, int8_t address, uint32_t frequency):
    _address(address),
    _buf_size(buf_size) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
    _buf = buffer;
}

UbxGpsI2C::~UbxGpsI2C(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool UbxGpsI2C::init() {
    char tx_buf[sizeof(cfg_prt_t)];

    if (_buf) {
        if (sendUbx(UBX_CFG, CFG_PRT, _buf_size - UBX_MIN_BUFFER_LEN) == sizeof(cfg_prt_t)) {  // get cfg
            cfg_prt_t cfg_prt;
            memcpy(&cfg_prt, _buf, sizeof(cfg_prt_t));

            cfg_prt.outProtoMask = 1;  // output only UBX

            memcpy(tx_buf, &cfg_prt, sizeof(cfg_prt_t));

            if (sendUbxAck(UBX_CFG, CFG_PRT, tx_buf, sizeof(cfg_prt_t))) {
                return true;
            }
        }
    }

    return false;
}

bool UbxGpsI2C::sendUbxAck(UbxClassId class_id, uint8_t id, const char * tx_data, uint16_t tx_len) {
    if (sendUbx(class_id, id, 0, tx_data, tx_len, true) > -1) {
        if (_buf[2] == UBX_ACK && _buf[3] == ACK_ACK && _buf[6] == class_id && _buf[7] == id) {
            return true;
        }
    }

    return false;
}

int16_t UbxGpsI2C::sendUbx(UbxClassId class_id, uint8_t id, uint16_t req_len,
                           const char * tx_data, uint16_t tx_len, bool include_header_checksum) {
    int32_t ack;
    uint16_t rx_len = req_len;
    int16_t ret = -1;

    if (max(req_len, tx_len) > _buf_size - UBX_MIN_BUFFER_LEN) {  // prevent buffer overflow
        return ret;
    }

    memset(_buf, 0, _buf_size);

    _buf[0] = SYNC_CHAR1;
    _buf[1] = SYNC_CHAR2;
    _buf[2] = class_id;
    _buf[3] = id;
    _buf[4] = static_cast<char>(tx_len);
    _buf[5] = static_cast<char>(tx_len >> 8);

    if (tx_len > 0) {
        memcpy(_buf + 6, tx_data, tx_len);
    }

    uint16_t ck = checksum(_buf, (6 + tx_len));

    _buf[(6 + tx_len)] = (ck & 0xFF);
    _buf[(7 + tx_len)] = (ck >> 8);

    tx_len += UBX_MIN_BUFFER_LEN;  // include header + checksum

    if (req_len > 0 || include_header_checksum) {
        rx_len += (UBX_MIN_BUFFER_LEN);  // header, checksum
    }

    if ((rx_len + 10) < _buf_size) {  // extra space
        rx_len += 10;
    }

    _i2c->lock();
    ack = _i2c->write(_address, _buf, tx_len);
    _i2c->unlock();

    if (ack == 0) {
        _i2c->lock();
        ack = _i2c->read(_address, _buf, rx_len);
        _i2c->unlock();

        if (ack == 0) {
            int16_t index = -1;

            for (uint16_t i = 0; i < rx_len; i++) {
                if (_buf[i] == SYNC_CHAR1 && _buf[i + 1] == SYNC_CHAR2) {  // find index of SYNC_CHAR1
                    index = i;
                    break;
                }
            }

            if (index > -1) {
                uint16_t size = ((_buf[index + 5] << 8) | _buf[index + 4]);
                uint16_t ck = checksum(_buf, size + 6, index);  // exclude header and checksum

                if (_buf[index + size + 6] == (ck & 0xFF) && _buf[index + size + 7] == (ck >> 8)) {
                    size = min(size, req_len);
                    ret = (include_header_checksum ? size + UBX_MIN_BUFFER_LEN : size);

                    for (uint16_t i = 0; i < ret; i++) {
                        _buf[i] = _buf[i + index + (include_header_checksum ? 0 : (UBX_MIN_BUFFER_LEN - 2))];  // correct index
                    }
                }
            }
        }
    }

    return ret;
}

uint16_t UbxGpsI2C::checksum(const char * data, uint16_t len, uint16_t offset) {
    uint32_t ca = 0;
    uint32_t cb = 0;
    uint16_t res = 0;

    if (data && len >= 6) {  // min msg length is 8 bytes including checksum
        for (uint16_t i = (2 + offset); i < (len + offset); i++) {  // calculate checksum, exclude sync chars
            ca += data[i];
            cb += ca;
        }

        res =  ((cb & 0xFF) << 8) | (ca & 0xFF);
    }

    return res;
}
