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
#include "UbxGpsI2C.h"

UbxGpsI2C::UbxGpsI2C(int8_t address) : _i2c(nullptr), _i2c_addr(address) {
    MBED_STATIC_ASSERT((UBX_HEADER_LEN + sizeof(cfg_prt_t) + UBX_CHECKSUM_LEN) < sizeof(_tx_buffer),
                       "Buffer size is too small");
}

UbxGpsI2C::UbxGpsI2C(PinName sda, PinName scl, int8_t address, uint32_t frequency) : _i2c_addr(address) {
    MBED_STATIC_ASSERT((UBX_HEADER_LEN + sizeof(cfg_prt_t) + UBX_CHECKSUM_LEN) < sizeof(_tx_buffer),
                       "Buffer size is too small");

    _i2c = new (_i2c_obj) I2C(sda, scl);
    _i2c->frequency(frequency);
}

UbxGpsI2C::~UbxGpsI2C() {}

bool UbxGpsI2C::init(Callback<void()> cb) {
    ubx_info("Setting up ublox GPS");

    MBED_ASSERT(_i2c);

    _cb = cb;

    cfg_prt_t cfg_prt = {0};

    cfg_prt.mode = UBX_DEFAULT_ADDRESS;
    cfg_prt.inProtoMask = 1;   // input only UBX
    cfg_prt.outProtoMask = 1;  // output only UBX
    cfg_prt.flags = 0b10;      // enable extendedTxTimeout

    return send_ack(UBX_CFG, UBX_CFG_PRT, &cfg_prt, sizeof(cfg_prt_t));
}

bool UbxGpsI2C::poll() {
    int available = bytes_available();

    if (available < 0) {
        ubx_error("Invalid");
        return false;
    }

    _rx_buffer_len = available;

    if (available > 0) {
        return _i2c->transfer(_i2c_addr, nullptr, 0, _rx_buffer, available, callback(this, &UbxGpsI2C::rx_cb),
                              I2C_EVENT_ALL) == 0;
    }

    return true;
};

bool UbxGpsI2C::auto_send(UbxClassId class_id, char id, uint8_t rate, Callback<void()> cb) {
    ubx_debug("Autosend request");
    cfg_msg_t cfg_msg = {.classId = class_id, .id = id, .rate = rate};

    this->oob(class_id, id, cb);

    return send_ack(UBX_CFG, UBX_CFG_MSG, &cfg_msg, sizeof(cfg_msg_t));
}

void UbxGpsI2C::read() {
    this->parse(_rx_buffer, _rx_buffer_len);
}

int UbxGpsI2C::bytes_available() {
    _tx_buffer[0] = 0xFD;

    int32_t ack = _i2c->write(UBX_DEFAULT_ADDRESS, _tx_buffer, 1, true);

    if (ack != 0) {
        return -1;
    }

    ack = _i2c->read(UBX_DEFAULT_ADDRESS, _tx_buffer, 2);

    if (ack != 0) {
        return -1;
    }

    uint16_t bytes_available = (uint16_t)_tx_buffer[0] << 8 | _tx_buffer[1];

    ubx_debug("Bytes available: %u", bytes_available);
    return min(bytes_available, (uint16_t)sizeof(_rx_buffer));
}

bool UbxGpsI2C::read_sync(int length) {
    int ack = _i2c->read(UBX_DEFAULT_ADDRESS, _rx_buffer, length);
    // ubx_debug("Read: %s", tr_array(reinterpret_cast<uint8_t*>(rx_buffer), length));

    if (ack == 0) {
        this->parse(_rx_buffer, length);
        return true;
    }

    return false;
}

bool UbxGpsI2C::send(UbxClassId class_id, char id, const void *payload, uint16_t payload_len) {
    uint16_t len = this->buildMessage(_tx_buffer, class_id, id, payload, payload_len);

    ubx_debug("Sending[%u]: %s", len, tr_array((uint8_t *)_tx_buffer, len));

    return _i2c->write(_i2c_addr, _tx_buffer, len) == 0;
}

bool UbxGpsI2C::send_ack(UbxClassId class_id, char id, const void *payload, uint16_t payload_len) {
    bool ok = false;
    _ack[0] = class_id;
    _ack[1] = id;

    if (!send(class_id, id, payload, payload_len)) {
        return false;
    }

    uint8_t try_counter = 0;
    this->oob(UBX_ACK, UBX_ACK_ACK, callback(this, &UbxGpsI2C::ack_cb));

    while (1) {
        int len = bytes_available();

        if (len == 0) {
            try_counter++;
            if (try_counter >= MBED_CONF_UBXGPS_REPEAT_COUNT) {
                ubx_error("This was last try");
                break;
            } else {
                ubx_debug("Data not yet ready, will wait");
                ThisThread::sleep_for(milliseconds{MBED_CONF_UBXGPS_REPEAT_DELAY});
            }

        } else if (len == -1) {
            ubx_error("Invalid");
            break;
        } else {
            bool status = read_sync(len);
            try_counter = 0;

            if (!status) {
                ubx_error("Read error");
                break;
            }

            // flags will be set in ack_cb() which will be called in read_sync();
            uint32_t ack = _flags.get();

            if (ack & UBX_FLAGS_ACK_DONE) {
                if (!(ack & UBX_FLAGS_NAK)) {
                    ok = true;
                }

                _flags.clear(UBX_FLAGS_ACK_DONE & UBX_FLAGS_NAK);

                break;
            }
        }
    }

    this->remove_oob(UBX_ACK, UBX_ACK_ACK);

    return ok;
}

void UbxGpsI2C::rx_cb(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        _cb();
    }
}

void UbxGpsI2C::ack_cb() {
    if (_ack[0] == msg.data[0] && _ack[1] == msg.data[1]) {  // compare ACK class & id
        ubx_debug("Got ACK: %u for class: %02X, id: %02X", msg.id, msg.data[0], msg.data[1]);

        if (msg.id == UBX_ACK_NAK) {
            _flags.set(UBX_FLAGS_NAK);
            ubx_error("ACK failed");
        }

        _flags.set(UBX_FLAGS_ACK_DONE);

    } else {
        ubx_debug("Different UBX ACK class");
    }
}