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

UbxGpsI2C::UbxGpsI2C(char * buf, uint16_t buf_size, int8_t address):
    _buf_size(buf_size),
    _i2c_addr(address),
    _req_len(0),
    _got_ubx_data(false),
    _include_header_checksum(false) {
    _rx_buf = buf;
}

bool UbxGpsI2C::init(I2C * i2c_obj) {
    _i2c = i2c_obj;

    if (_i2c && _rx_buf) {
        if (sendUbx(UBX_CFG, CFG_PRT, NULL, 0, 20)) {
            _semaphore.wait(DEFAULT_TIMEOUT);

            if (_got_ubx_data) {
                cfg_prt_t cfg_prt;
                memcpy(&cfg_prt, _rx_buf + 6, sizeof(cfg_prt_t));
                cfg_prt.outProtoMask &= ~0b110;  // output only UBX

                char tx[sizeof(cfg_prt_t)];
                memcpy(tx, &cfg_prt, sizeof(cfg_prt_t));

                if (sendUbxAck(UBX_CFG, CFG_PRT, tx, sizeof(cfg_prt_t))) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool UbxGpsI2C::send(uint8_t tx_len, uint16_t rx_len) {
    if (_i2c->transfer(
                _i2c_addr,
                reinterpret_cast<char*>(_tx_buf),
                tx_len,
                _rx_buf,
                rx_len,
                event_callback_t(this, &UbxGpsI2C::internalCb),
                I2C_EVENT_ALL) == 0) {
        return true;
    }

    return false;
}

void UbxGpsI2C::internalCb(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        if (_done_cb) {
            _done_cb.call(-1);
        }

    } else if (event & I2C_EVENT_ERROR) {
        if (_done_cb) {
            _done_cb.call(-2);
        }

    } else {
        if (_req_len > 0) {
            int16_t index = -1;

            for (uint16_t i = 0; i < _buf_size; i++) {
                if (_rx_buf[i] == SYNC_CHAR1 && _rx_buf[i + 1] == SYNC_CHAR2) {  // find index of SYNC_CHAR1
                    index = i;
                    break;
                }
            }

            if (index > -1) {
                uint16_t size = ((_rx_buf[index + 5] << 8) | _rx_buf[index + 4]);

                if (_req_len <= size) {
                    uint16_t ck = checksum(_rx_buf, size + 6, index);  // exclude header and checksum

                    if (_rx_buf[index + size + 6] == (ck & 0xFF) && _rx_buf[index + size + 7] == (ck >> 8)) {
                        uint16_t offset = (_include_header_checksum ? 0 : 6);

                        for (uint16_t i = 0; i < size + (_include_header_checksum ? 8 : 0) ; ++i) {  // data len + header + checksum
                            _rx_buf[i] = _rx_buf[i + index + offset];  // correct index
                        }

                        _got_ubx_data = true;
                        wait_ms(5);

                        _semaphore.release();

                        if (_done_cb) {
                            _done_cb.call(size + (_include_header_checksum ? 6 : 0));  // data len + header + checksum
                        }
                    }
                }
            }

        } else if (_done_cb) {
            _done_cb.call(0);
        }
    }
}

bool UbxGpsI2C::sendUbxAck(UbxClassId class_id, uint8_t id, const char * data, uint16_t tx_len) {
    if (sendUbx(class_id, id, data, tx_len, 2)) {
        _semaphore.wait(DEFAULT_TIMEOUT);

        if (_got_ubx_data) {
            if (_rx_buf[2] == UBX_ACK && _rx_buf[3] == ACK_ACK && _rx_buf[6] == class_id && _rx_buf[7] == id) {
                return true;
            }
        }
    }

    return false;
}

bool UbxGpsI2C::sendUbx(UbxClassId class_id, uint8_t id, const char * data, uint16_t tx_len, uint16_t rx_len, event_callback_t function,
                        bool include_header_checksum) {
    _done_cb = function;
    _req_len = rx_len;
    _include_header_checksum = include_header_checksum;

    if (tx_len > (TX_BUFFER_SIZE - 8)) {  // prevent buffer overflow
        return false;
    }

    _tx_buf[0] = SYNC_CHAR1;
    _tx_buf[1] = SYNC_CHAR2;
    _tx_buf[2] = class_id;
    _tx_buf[3] = id;
    _tx_buf[4] = static_cast<char>(tx_len);
    _tx_buf[5] = static_cast<char>(tx_len >> 8);

    if (tx_len > 0) {
        memcpy(_tx_buf + 6, data, tx_len);
    }

    uint16_t ck = checksum(_tx_buf, (6 + tx_len));

    _tx_buf[(6 + tx_len)] = (ck & 0xFF);
    _tx_buf[(7 + tx_len)] = (ck >> 8);

    if (rx_len > 0) {
        rx_len += (8 + 10);  // header, checksum + extra space
        _got_ubx_data = false;
    }

    if (send((8 + tx_len), rx_len)) {
        return true;
    }

    return false;
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
