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
    _initialized(false),
    _step(0),
    _rx_len(0),
    _repeat_timeout(DEFAULT_REPEAT_TIMEOUT) {
    _rx_buf = buf;
}

bool UbxGpsI2C::init(I2C * i2c_obj, EventQueue * queue) {
    _i2c = i2c_obj;
    _queue = queue;

    if (_i2c && _rx_buf) {
        _tx_buf[0] = 0xFF;

        _initialized = !_i2c->write(_i2c_addr, _tx_buf, 1);

        printf("res: %i\n", _initialized);
        return _initialized;
    }

    _initialized = false;
    return false;
}

void UbxGpsI2C::get(event_callback_t function, uint8_t repeat_timeout) {
    _repeat_timeout = repeat_timeout;
    _done_cb = function;
    getLen();
}

bool UbxGpsI2C::sender(uint8_t tx_size, uint16_t rx_size) {
    if (_initialized) {
        if (_i2c->transfer(
                    _i2c_addr,
                    reinterpret_cast<char*>(_tx_buf),
                    tx_size,
                    _rx_buf,
                    rx_size,
                    event_callback_t(this, &UbxGpsI2C::internalCb),
                    I2C_EVENT_ALL) == 0) {
            return true;
        }
    }

    return false;
}

void UbxGpsI2C::getLen() {
    _step = 1;
    _tx_buf[0] = 0xFD;
    sender(1, 2);
}

void UbxGpsI2C::internalCb(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        printf("not responding\n");

    } else if (event & I2C_EVENT_ERROR) {
        printf("error\n");

    } else {
        if (_step == 1) {
            _rx_len = (_rx_buf[0] << 8) | _rx_buf[1];

            if (_rx_len == 0) {
                if (_queue) {
                    _queue->call_in(_repeat_timeout, callback(this, &UbxGpsI2C::getLen));
                }

            } else {
                if (_rx_len > _buf_size) {  // prevent buffer overflow
                    _rx_len = _buf_size;
                }

                _tx_buf[0] = 0xFF;

                if (sender(1, _rx_len)) {
                    _step = 2;
                }
            }

        } else if (_step == 2) {
            _step = 0;

            if (_done_cb) {
                _done_cb.call(_rx_len - 2);
            }
        }
    }
}


bool UbxGpsI2C::sendUbx(uint8_t class_id, uint8_t id, const char * data, uint16_t len) {
    uint32_t ca = 0;
    uint32_t cb = 0;
    char buf[16] = {0};

    if (len > (TX_BUFFER_SIZE - 8)) {  // prevent buffer overflow
        len = (TX_BUFFER_SIZE - 8);
    }

    buf[0] = 0xB5;  // sync char 1
    buf[1] = 0x62;  // sync char 2
    buf[2] = class_id;
    buf[3] = id;
    buf[4] = static_cast<char>(len);
    buf[5] = static_cast<char>(len >> 8);

    memcpy(buf + 6, data, len);

    for (uint16_t i = 2; i < (6 + len); i++) {
        ca += buf[i];
        cb += ca;
    }

    buf[(6 + len)] = (ca & 0xFF);
    buf[(7 + len)] = (cb & 0xFF);

    if (_i2c->write(_i2c_addr, buf, 8 + len) == 0) {
        _i2c->read(_i2c_addr, _rx_buf, 10);

        // printf("ACK: %02X %02X\n",_rx_buf[0], _rx_buf[1]);

        return true;
    }

    return false;
}
