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
                       "TX buffer size is too small");
}

UbxGpsI2C::UbxGpsI2C(PinName sda, PinName scl, int8_t address, uint32_t frequency) : _i2c_addr(address) {
    MBED_STATIC_ASSERT((UBX_HEADER_LEN + sizeof(cfg_prt_t) + UBX_CHECKSUM_LEN) < sizeof(_tx_buffer),
                       "TX buffer size is too small");

    _i2c = new (_i2c_obj) I2C(sda, scl);
    _i2c->frequency(frequency);
}

UbxGpsI2C::~UbxGpsI2C() {}

bool UbxGpsI2C::init(Callback<void()> cb, I2C *i2c_obj) {
    ubx_info("Setting up ublox GPS");

    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    _cb = cb;

    // get protocol version
    char ver[6] = {0};

    if (!protocol_version(ver)) {
        return false;
    }

    float version = strtof(ver, nullptr);

    if (version > 23.01) {
        _new_cfg = true;
    }

    if (!_new_cfg) {
        cfg_prt_t cfg_prt = {0};

        cfg_prt.mode = UBX_DEFAULT_ADDRESS;
        cfg_prt.inProtoMask = 1;   // input only UBX
        cfg_prt.outProtoMask = 1;  // output only UBX
        cfg_prt.flags = 0b10;      // enable extendedTxTimeout

        return send_ack(UBX_CFG, UBX_CFG_PRT, &cfg_prt, sizeof(cfg_prt_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::poll() {
    int available = bytes_available();

    if (available < 0 || !_mutex.trylock()) {
        ubx_error("Invalid");
        return false;
    }

    _rx_buffer_len = available;
    _mutex.unlock();  // in case we succeeded

    if (available > 0) {
        return _i2c->transfer(_i2c_addr, nullptr, 0, _rx_buffer, available, callback(this, &UbxGpsI2C::rx_cb),
                              I2C_EVENT_ALL) == 0;
    }

    return true;
};

bool UbxGpsI2C::auto_send(UbxClassId class_id, char id, uint8_t rate, Callback<void()> cb) {
    ubx_info("Autosend request");

    if (!_new_cfg) {
        cfg_msg_t cfg_msg = {.classId = class_id, .id = id, .rate = rate};

        return send_ack(UBX_CFG, UBX_CFG_MSG, &cfg_msg, sizeof(cfg_msg_t));
    }

    // TODO new cfg
    return false;
}

void UbxGpsI2C::process() {
    _mutex.lock();
    this->parse(_rx_buffer, _rx_buffer_len);
    _mutex.unlock();
}

bool UbxGpsI2C::set_output_rate(milliseconds ms, uint16_t cycles) {
    ubx_info("Setting output rate");

    if (ms < 50ms || cycles > 127 || ms > 65535ms) {
        ubx_error("Wrong parameter");
        return false;
    }

    if (!_new_cfg) {
        cfg_rate_t cfg_rate = {
            .measRate = (uint16_t)duration_cast<milliseconds>(ms).count(),
            .navRate = cycles,
            .timeRef = 0  // UTC
        };

        return send_ack(UBX_CFG, UBX_CFG_RATE, &cfg_rate, sizeof(cfg_rate));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::set_odometer(bool enable, OdoCfgProfile profile, uint8_t velocity_filter) {
    ubx_info("Setting odometer");

    if (!_new_cfg) {
        if (!send(UBX_CFG, UBX_CFG_ODO)) {
            return false;
        }

        if (!get(UBX_CFG, UBX_CFG_ODO)) {
            return false;
        }

        cfg_odo_t cfg_odo;
        memcpy(&cfg_odo, _tx_buffer + 2, sizeof(cfg_odo_t));

        cfg_odo.odoCfg &= ~0b111;
        cfg_odo.odoCfg |= (uint8_t)profile;

        if (velocity_filter > 0) {
            cfg_odo.flags |= 0b100;

        } else {
            cfg_odo.flags &= ~0b100;
        }

        cfg_odo.velLpGain = velocity_filter;

        if (enable) {
            cfg_odo.flags |= 0b1;

        } else {
            cfg_odo.flags &= ~0b1;
        }

        return send_ack(UBX_CFG, UBX_CFG_ODO, &cfg_odo, sizeof(cfg_odo_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::set_power_mode(PowerSetupValue mode, uint16_t period, uint16_t on_time) {
    ubx_info("Setting power mode");

    if (!_new_cfg) {
        if (!send(UBX_CFG, UBX_CFG_PMS)) {
            return false;
        }

        if (!get(UBX_CFG, UBX_CFG_PMS)) {
            return false;
        }

        cfg_pms_t cfg_pms;
        memcpy(&cfg_pms, _tx_buffer + 2, sizeof(cfg_pms_t));

        cfg_pms.powerSetupValue = (uint8_t)mode;
        cfg_pms.period = (mode == PSV_INTERVAL) ? period : 0;
        cfg_pms.onTime = (mode == PSV_INTERVAL) ? on_time : 0;

        return send_ack(UBX_CFG, UBX_CFG_PMS, &cfg_pms, sizeof(cfg_pms_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::set_psm(bool low_power) {
    ubx_info("Setting low power");

    if (!_new_cfg) {
        if (!send(UBX_CFG, UBX_CFG_RXM)) {
            return false;
        }

        if (!get(UBX_CFG, UBX_CFG_RXM)) {
            return false;
        }

        cfg_rxm_t cfg_rxm;
        memcpy(&cfg_rxm, _tx_buffer + 2, sizeof(cfg_rxm_t));

        cfg_rxm.lpMode = low_power;

        return send_ack(UBX_CFG, UBX_CFG_RXM, &cfg_rxm, sizeof(cfg_rxm_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::reset(ResetMode mode, uint16_t bbr_mask) {
    ubx_info("Resetting");

    if (!_new_cfg) {
        cfg_rst_t cfg_rst = {.navBbrMask = bbr_mask, .resetMode = mode, .reserved1 = 0};

        return send(UBX_CFG, UBX_CFG_RST, &cfg_rst, sizeof(cfg_rst_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::reset_odometer() {
    ubx_info("Resetting odometer");

    return send_ack(UBX_NAV, UBX_NAV_RESETODO);
}

bool UbxGpsI2C::permanent_configuration(PermanentConfig type, uint32_t mask, uint8_t device_mask) {
    ubx_info("Setting permanent configuration");

    if (!_new_cfg) {
        cfg_cfg_t cfg_cfg = {.clearMask = 0, .saveMask = 0, .loadMask = 0, .deviceMask = device_mask};

        switch (type) {
            case Clear:
                cfg_cfg.clearMask = mask;
                break;

            case Save:
                cfg_cfg.saveMask = mask;
                break;

            case Load:
                cfg_cfg.loadMask = mask;
                break;
        }

        return send_ack(UBX_CFG, UBX_CFG_CFG, &cfg_cfg, sizeof(cfg_cfg_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::set_dynamic_model(DynamicModel model) {
    ubx_info("Setting low power");

    if (!_new_cfg) {
        if (!send(UBX_CFG, UBX_CFG_NAV5)) {
            return false;
        }

        if (!get(UBX_CFG, UBX_CFG_NAV5)) {
            return false;
        }

        cfg_nav5_t cfg_nav5;
        memcpy(&cfg_nav5, _tx_buffer + 2, sizeof(cfg_nav5_t));

        cfg_nav5.dynModel = model;

        return send_ack(UBX_CFG, UBX_CFG_NAV5, &cfg_nav5, sizeof(cfg_nav5_t));
    }

    // TODO new cfg
    return false;
}

bool UbxGpsI2C::wakeup() {
    _tx_buffer[0] = 0xFF;

    int32_t ack = _i2c->write(_i2c_addr, _tx_buffer, 1);

    return (ack == 0);
}

bool UbxGpsI2C::protocol_version(char *version) {
    ubx_info("Getting procotol version");

    if (!send(UBX_MON, UBX_MON_VER)) {
        return false;
    }

    if (!get(UBX_MON, UBX_MON_VER)) {
        return false;
    }

    ubx_info("Protocol version: %.5s", _tx_buffer + 2);

    if (version != nullptr) {
        memcpy(version, _tx_buffer + 2, 5);
    }

    return true;
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
    bool ok = false;

    _mutex.lock();
    int ack = _i2c->read(UBX_DEFAULT_ADDRESS, _rx_buffer, length);
    ubx_debug("Read: %s", tr_array((uint8_t *)_rx_buffer, length));

    if (ack == 0) {
        ok = true;
        this->parse(_rx_buffer, length);
    }

    _mutex.unlock();
    return ok;
}

bool UbxGpsI2C::send(UbxClassId class_id, char id, const void *payload, uint16_t payload_len) {
    if ((uint16_t)(UBX_HEADER_LEN + payload_len + UBX_CHECKSUM_LEN) > sizeof(_tx_buffer)) {
        ubx_error("TX buffer overflow");
        return false;
    }

    uint16_t len = this->buildMessage(_tx_buffer, class_id, id, payload, payload_len);

    ubx_debug("Sending[%u]: %s", len, tr_array((uint8_t *)_tx_buffer, len));

    return _i2c->write(_i2c_addr, _tx_buffer, len) == 0;
}

bool UbxGpsI2C::send_ack(UbxClassId class_id, char id, const void *payload, uint16_t payload_len) {
    if (!send(class_id, id, payload, payload_len)) {
        return false;
    }

    if (!get(UBX_ACK, UBX_ACK_ACK)) {
        return false;
    }

    tr_debug("ACK OK: %u", _tx_buffer[1] == UBX_ACK_ACK);

    return _tx_buffer[0] == UBX_ACK && _tx_buffer[1] == UBX_ACK_ACK;
}

void UbxGpsI2C::rx_cb(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        _cb();
    }
}

void UbxGpsI2C::done_cb() {
    _tx_buffer[0] = msg.classId;
    _tx_buffer[1] = msg.id;

    if (msg.classId == UBX_MON && msg.id == UBX_MON_VER) {
        const char protver[] = "PROTVER=";
        const uint16_t protver_len = strlen(protver);

        for (uint16_t i = 0; i < msg.length - protver_len + 1; ++i) {
            if (memcmp(msg.data.get() + i, protver, protver_len) == 0) {
                memcpy(_tx_buffer + 2, msg.data.get() + i + protver_len, 5);
                _done_cb_called = true;
            }
        }

    } else {
        memcpy(_tx_buffer + 2, msg.data.get(), min(msg.length, (uint16_t)sizeof(_tx_buffer)));
        _done_cb_called = true;
    }
}

bool UbxGpsI2C::get(UbxClassId class_id, char id) {
    bool ok = false;

    uint8_t try_counter = 0;
    this->oob(class_id, id, callback(this, &UbxGpsI2C::done_cb));

    while (1) {
        int len = bytes_available();

        if (len == 0) {
            try_counter++;
            if (try_counter >= MBED_CONF_UBXGPS_REPEAT_COUNT) {
                ubx_warning("This was last try");
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

            if (!status) {
                ubx_error("Read error");
                break;
            }

            if (_done_cb_called) {
                _done_cb_called = false;
                ok = true;
                ubx_info("Internal message");
                break;
            }
        }
    }

    this->remove_oob(class_id, id);

    return ok;
}