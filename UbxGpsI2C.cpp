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
    _repeat_timeout(DEFAULT_REPEAT_TIMEOUT),
    _send_status(false) {
    _rx_buf = buf;
}

bool UbxGpsI2C::init(I2C * i2c_obj, EventQueue * queue) {
    _i2c = i2c_obj;
    _queue = queue;

    if (_i2c && _rx_buf) {
        _tx_buf[0] = 0xFF;
        _initialized = true;

        if (send(1, 0)) {
            _send_status = false;
            _event.wait_all(0x1);
            _initialized = _send_status;
            return _initialized;
        }
    }

    _initialized = false;
    return false;
}

void UbxGpsI2C::get(event_callback_t function, uint8_t repeat_timeout) {
    _repeat_timeout = repeat_timeout;
    _done_cb = function;
    getLen();
}

bool UbxGpsI2C::send(uint8_t tx_size, uint16_t rx_size) {
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
    send(1, 2);
}

void UbxGpsI2C::internalCb(int event) {
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        _send_status = false;

    } else if (event & I2C_EVENT_ERROR) {
        _send_status = false;

    } else {
        _send_status = true;

        if (_step == 1) {  // len
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

                if (send(1, _rx_len)) {
                    _step = 2;
                }
            }

        } else if (_step == 2) {  // data
            _step = 0;

            if (_done_cb) {
                _done_cb.call(_rx_len - 2);  // remove CR&LF
            }

        } else if (_step == 20) {  // sending done
        }
    }


    _event.set(0x1);
}

bool UbxGpsI2C::sendUbxAck(uint8_t class_id, uint8_t id, const char * data, uint16_t len) {
    if (sendUbx(class_id, id, data, len)) {
        char buf[2] = {class_id, id};

        if (sendUbx(UBX_ACK, UBX_ACK_ACK, buf, 2, 10)) {  // get ACK
            if (_rx_buf[2] == UBX_ACK && _rx_buf[3] == UBX_ACK_ACK) {
                /*printf("ACK: ");

                for (int i = 0; i < 10; ++i) {
                    printf("%02X ", _rx_buf[i]);
                }

                printf("\n");*/

                return true;
            }
        }
    }

    return false;
}

bool UbxGpsI2C::sendUbx(uint8_t class_id, uint8_t id, const char * data, uint16_t len, uint16_t rx_len) {
    uint32_t ca = 0;
    uint32_t cb = 0;

    if (len > (TX_BUFFER_SIZE - 8)) {  // prevent buffer overflow
        len = (TX_BUFFER_SIZE - 8);
    }

    _tx_buf[0] = 0xB5;  // sync char 1
    _tx_buf[1] = 0x62;  // sync char 2
    _tx_buf[2] = class_id;
    _tx_buf[3] = id;
    _tx_buf[4] = static_cast<char>(len);
    _tx_buf[5] = static_cast<char>(len >> 8);

    memcpy(_tx_buf + 6, data, len);

    for (uint16_t i = 2; i < (6 + len); i++) {
        ca += _tx_buf[i];
        cb += ca;
    }

    _tx_buf[(6 + len)] = (ca & 0xFF);
    _tx_buf[(7 + len)] = (cb & 0xFF);

    /*printf("data: ");

    for (int i = 0; i < (8 + len); i++) {
        printf("%02X ", _tx_buf[i]);
    }

    printf("\n");*/
    _step = 20;

    if (send((8 + len), rx_len)) {
        _send_status = false;
        _event.wait_all(0x1);

        if (_send_status) {
            return true;
        }
    }

    _step = 0;

    return false;
}

