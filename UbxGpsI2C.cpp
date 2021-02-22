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

#include "UbxGpsI2C.h"

UbxGpsI2C::UbxGpsI2C(EventQueue *queue, int8_t address):
    _oobs(nullptr),
    _i2c(nullptr),
    _queue(queue),
    _i2c_addr(address) {
    MBED_ASSERT(queue);
    MBED_STATIC_ASSERT(
        (UBX_HEADER_LEN + sizeof(cfg_prt_t) + UBX_CHECKSUM_LEN) < sizeof(_buf),
        "Buffer size is too small");
}

UbxGpsI2C::UbxGpsI2C(PinName sda, PinName scl, EventQueue *queue, int8_t address, uint32_t frequency):
    _oobs(nullptr),
    _queue(queue),
    _i2c_addr(address) {
    MBED_ASSERT(queue);
    MBED_STATIC_ASSERT(
        (UBX_HEADER_LEN + sizeof(cfg_prt_t) + UBX_CHECKSUM_LEN) < sizeof(_buf),
        "Buffer size is too small");

    ThisThread::sleep_for(100ms);

    _i2c = new (_i2c_obj) I2C(sda, scl);
    _i2c->frequency(frequency);
}

UbxGpsI2C::~UbxGpsI2C(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_obj)) {
        _i2c->~I2C();
    }

    while (_oobs) {
        struct oob_t *oob = _oobs;
        _oobs = oob->next;
        delete oob;
    }
}

bool UbxGpsI2C::send(UbxClassId class_id, char id, const char *payload, uint16_t payload_len) {
    uint16_t len = packet_builder(class_id, id, payload, payload_len);
    tr_debug("Sending[%u]: %s", len, tr_array((uint8_t *)_buf, len));

    if (len != USHRT_MAX) {
        if (_i2c != nullptr && _i2c->transfer(
                    _i2c_addr,
                    _buf,
                    len,
                    nullptr,
                    0,
                    callback(this, &UbxGpsI2C::tx_cb),
                    I2C_EVENT_ALL) == 0) {
            return true;
        }

        tr_error("I2C TX transfer error");
    }

    return false;
}

bool UbxGpsI2C::send_ack(UbxClassId class_id, char id, const char *payload, uint16_t payload_len) {
    bool ok = false;

    if (send(class_id, id, payload, payload_len)) {
        oob(UBX_ACK, UBX_ACK_ACK, callback(this, &UbxGpsI2C::ack_cb));
        _ack[0] = class_id;
        _ack[1] = id;

        if (poll()) {
            tr_debug("Waiting for UBX ACK");
            uint32_t ack = _flags.wait_all(UBX_FLAGS_ACK_DONE | UBX_FLAGS_SEARCH_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT);

            if (!(ack & UBX_FLAGS_ERROR)) {
                ok = ack & UBX_FLAGS_NAK ? false : true;

            } else {
                tr_error("UBX ACK timeout");
            }
        }

        remove_oob(UBX_ACK, UBX_ACK_ACK);
    }

    return ok;
}

bool UbxGpsI2C::read() {
    return poll(false);
}

bool UbxGpsI2C::poll(bool await) {
    if (_i2c != nullptr) {
        if (await) {
            tr_debug("Waiting for TX transfer to be done");
            uint32_t sent = _flags.wait_any(UBX_FLAGS_TRANSFER_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT);

            if (!(sent & UBX_FLAGS_TRANSFER_DONE)) {
                tr_error("TX transfer timeout");
                return false;
            }

            tr_debug("TX transfer done");
        }

        for (uint8_t i = 0; i < MBED_CONF_UBXGPSI2C_REPEAT_COUNT; i++) {
            _bytes_available = bytes_available();

            if (_bytes_available == 0 || _bytes_available == USHRT_MAX) {
                tr_debug("Data not yet ready, will wait");
                ThisThread::sleep_for(milliseconds{MBED_CONF_UBXGPSI2C_REPEAT_DELAY});

            } else {
                return get_data();
            }
        }
    }

    return false;
};

uint16_t UbxGpsI2C::packet_builder(UbxClassId class_id, char id, const char *payload, uint16_t payload_len) {
    if ((payload_len + UBX_HEADER_LEN + UBX_CHECKSUM_LEN) <= MBED_CONF_UBXGPSI2C_BUFFER_SIZE) {
        if (payload != nullptr && payload_len > 0) {
            if (payload == _buf) {  // we are reusing the same buffer so we will shift the payload
                memmove(_buf + UBX_HEADER_LEN, _buf, payload_len);

            } else {
                memcpy(_buf + UBX_HEADER_LEN, payload, payload_len);
            }
        }

        _buf[0] = UBX_SYNC_CHAR1;
        _buf[1] = UBX_SYNC_CHAR2;
        _buf[2] = class_id;
        _buf[3] = id;
        _buf[4] = payload_len & UCHAR_MAX;
        _buf[5] = payload_len >> 8;

        uint16_t ck = checksum(_buf, UBX_HEADER_LEN + payload_len);

        _buf[(UBX_HEADER_LEN + payload_len)] = (ck & UCHAR_MAX);
        _buf[(UBX_HEADER_LEN + 1 + payload_len)] = ck >> 8;

        return (UBX_HEADER_LEN + UBX_CHECKSUM_LEN + payload_len);
    }

    return USHRT_MAX;
}

uint16_t UbxGpsI2C::bytes_available() {
    int32_t ack = -1;
    // check if there are any bytes available to read
    _buf[0] = 0xFD;

    _i2c->lock();
    ack = _i2c->write(_i2c_addr, _buf, 1, true);
    _i2c->unlock();

    if (ack == 0) {
        _i2c->lock();
        ack = _i2c->read(_i2c_addr, _buf, 2);
        _i2c->unlock();

        if (ack == 0) {
            if (_buf[1] != UCHAR_MAX) { // lsb should never be 0xFF
                uint16_t bytes_available = (uint16_t)_buf[0] << 8 | _buf[1];

                // Check for undocumented bit error. For more info search for SparkFun_Ublox_Arduino_Library
                if (bytes_available & 0b1000000000000000) {
                    bytes_available &= ~(0b1000000000000000);
                }

                tr_debug("Bytes available: %u", bytes_available);
                return bytes_available;

            } else {
                tr_error("Invalid size");
            }
        }
    }

    tr_error("No I2C ACK from GPS");

    return USHRT_MAX;
}

uint16_t UbxGpsI2C::checksum(const char *packet, uint16_t len) {
    uint8_t ca = 0;
    uint8_t cb = 0;
    uint16_t res = 0;

    for (uint16_t i = 2; i < len; i++) {  // exclude SYNC
        ca += packet[i];
        cb += ca;
    }

    res = (cb << 8) | ca;

    return res;
}

bool UbxGpsI2C::get_data() {
    uint16_t len = (_bytes_available - _data_len) > MBED_CONF_UBXGPSI2C_DATA_SIZE ?
                   MBED_CONF_UBXGPSI2C_DATA_SIZE :
                   (_bytes_available - _data_len);

    if (_i2c->transfer(
                _i2c_addr,
                nullptr,
                0,
                data + _data_len,
                len,
                callback(this, &UbxGpsI2C::rx_cb),
                I2C_EVENT_ALL) == 0) {
        _data_len += len;
        return true;
    }

    tr_error("I2C RX transfer error");
    return false;
}

uint16_t UbxGpsI2C::get_sync_index(const char *buf, uint16_t buf_size, char c, uint16_t offset) {
    uint16_t ret = USHRT_MAX;

    for (uint16_t i = offset; i < buf_size; i++) {
        if (buf[i] == c) {
            ret = i;
            break;
        }
    }

    return ret;
}

void UbxGpsI2C::search_data() {
    uint16_t max = _data_len;
    bool packet_increment = true;

    tr_debug("Got data[%u]: %s", _data_len, tr_array((uint8_t *)data, _data_len));

    if (_data_len != 0 && _data_len <= _bytes_available) {
        for (uint16_t i = 0; i < max; i++) {
            if (_packet_index == 0) {
                uint16_t index = get_sync_index(data, _data_len, UBX_SYNC_CHAR1, i);

                if (index != USHRT_MAX) {
                    _packet_checksum_index = 0;
                    tr_debug("UBX_SYNC_CHAR1 found index: %u", index);

                    if (index > 0) {
                        tr_debug("Correcting data buffer");
                        _data_len -= index;
                        _bytes_available -= index;
                        _packet_index = 1;
                        _packet_len = 0;
                        memmove(data, data + index, _data_len);

                        if (_data_len > 1) {
                            search_data();  // start again with corrected index
                            return;
                        }

                        goto END;
                    }

                } else {
                    tr_debug("UBX_SYNC_CHAR1 not found");
                    _bytes_available -= _data_len;
                    _data_len = 0;
                    _packet_index = 0;
                    _packet_len = 0;
                    break;
                }

            } else if (_packet_index == 1) {
                if (data[1] != UBX_SYNC_CHAR2) {
                    tr_debug("UBX_SYNC_CHAR2 not found");
                    _packet_index = 0;
                    _packet_len = 0;
                    packet_increment = false;

                } else {
                    tr_debug("Found UBX_SYNC_CHAR2");
                }

            } else if (_packet_index == 3) {
                bool process = false;

                for (struct oob_t *oob = _oobs; oob; oob = oob->next) {
                    if (oob->class_id == UBX_ACK || (oob->class_id == data[2] && oob->id == data[3])) {
                        tr_debug("This is a packet we are looking for");
                        process = true;
                        break;
                    }
                }

                if (!process) {
                    tr_debug("Different package");
                    _packet_index = 0;
                    _packet_len = 0;
                    packet_increment = false;
                }

            } else if (_packet_index == 5) {
                _packet_len = UBX_HEADER_LEN + ((data[5] << 8) | data[4]) + UBX_CHECKSUM_LEN;

                tr_debug("Packet len: %u", _packet_len);

                if (_packet_len > sizeof(data)) {
                    tr_error("This packet is too long, won't fit into data buffer");
                    _packet_index = 0;
                    _packet_len = 0;
                    _packet_checksum_index = 0;
                    break;

                } else {
                    _packet_checksum_index = _packet_len - 1;
                }

            } else if (_packet_checksum_index > UBX_HEADER_LEN && i == _packet_checksum_index) {
                tr_debug("Processing this packet: %s", tr_array((uint8_t *)data, (i + 1)));

                _packet_checksum_index = 0;

                uint16_t ck = checksum(data, i + 1 - 2);

                if (ck == (data[i] << 8 | data[i - 1])) {
                    tr_debug("Checksum OK");

                    for (struct oob_t *oob = _oobs; oob; oob = oob->next) {
                        if (oob->class_id == UBX_ACK || (oob->class_id == data[2] && oob->id == data[3])) {
                            oob->cb.call();
                            break;
                        }
                    }

                } else {
                    tr_error("Checksum failed");
                }

                _data_len -= (i + 1);
                _bytes_available -= (i + 1);
                _packet_index = 0;
                _packet_len = 0;

                if (_data_len > 0) {
                    tr_debug("There is more data to process");
                    memmove(data, data + i + 1, _data_len);
                    search_data();  // start again with corrected index
                    return;

                } else {
                    goto END;
                }
            }

            if (packet_increment) {
                _packet_index++;
            }
        }

END:

        if (_bytes_available > 0) {
            tr_debug("There are still more bytes to read");
            get_data();

        } else {
            _flags.set(UBX_FLAGS_SEARCH_DONE);
        }

    } else {
        _bytes_available = 0;
    }
}

void UbxGpsI2C::oob(UbxClassId class_id, char id, Callback<void()> cb) {
    // In case we have it already in, remove it
    remove_oob(class_id, id);

    tr_debug("Registering oob CLASS: %02hhX, ID: %02hhX", class_id, id);

    struct oob_t *oob = new struct oob_t;
    oob->class_id = class_id;
    oob->id = id;
    oob->cb = cb;
    oob->next = _oobs;
    _oobs = oob;
}

void UbxGpsI2C::remove_oob(UbxClassId class_id, char id) {
    struct oob_t *prev = nullptr;
    struct oob_t *oob = _oobs;

    while (oob) {
        if (oob->class_id == class_id && oob->id == id) {
            if (prev) {
                prev->next = oob->next;

            } else {
                _oobs = oob->next;
            }

            tr_debug("Removing oob CLASS: %02hhX, ID: %02hhX", class_id, id);
            delete oob;
            return;
        }

        prev = oob;
        oob = oob->next;
    }
}

void UbxGpsI2C::rx_cb(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        _queue->call(this, &UbxGpsI2C::search_data);
    }
}

void UbxGpsI2C::tx_cb(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        _flags.set(UBX_FLAGS_TRANSFER_DONE);
    }
}

void UbxGpsI2C::ack_cb() {
    if (_ack[0] == data[6] && _ack[1] == data[7]) {  // compare ACK class & id
        tr_debug("Got ACK: %u for class: %02X, id: %02X", data[3], data[6], data[7]);

        if (data[3] == UBX_ACK_NAK) {
            _flags.set(UBX_FLAGS_NAK);
            tr_error("ACK failed");
        }

        _flags.set(UBX_FLAGS_ACK_DONE);

    } else {
        tr_debug("Different UBX ACK class");
    }
}

void UbxGpsI2C::cfg_cb() {
    tr_debug("UBX CFG packet[%u]: %s", _data_len, tr_array((uint8_t *)data, _data_len));

    if ((size_t)(_packet_len - UBX_HEADER_LEN - UBX_CHECKSUM_LEN) > sizeof(_buf)) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_SIZE),
                   "UBX CFG packet will not fit into buffer");
    }

    memcpy(_buf, data + UBX_HEADER_LEN, _packet_len - UBX_HEADER_LEN - UBX_CHECKSUM_LEN);

    _flags.set(UBX_FLAGS_CFG);
}

bool UbxGpsI2C::init(I2C *i2c_obj) {
    cfg_prt_t cfg_prt;

    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    tr_info("Setting up ublox GPS");

    _buf[0] = 0;  // DDC port
    bool ok = send(UBX_CFG, UBX_CFG_PRT, _buf, 1);

    if (ok) {
        // register our packet before polling
        oob(UBX_CFG, UBX_CFG_PRT, callback(this, &UbxGpsI2C::cfg_cb));

        if (poll()) {
            tr_debug("Waiting for UBX CFG packet");

            uint32_t cfg = _flags.wait_all(UBX_FLAGS_CFG | UBX_FLAGS_SEARCH_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT);

            if (!(cfg & UBX_FLAGS_ERROR)) {
                tr_debug("UBX CFG packet received");

                memcpy(&cfg_prt, _buf, sizeof(cfg_prt_t));  // _buf contains only payload

                cfg_prt.outProtoMask &= ~0b110;  // output only UBX

                memcpy(_buf, &cfg_prt, sizeof(cfg_prt_t));

                if (send_ack(UBX_CFG, UBX_CFG_PRT, _buf, sizeof(cfg_prt_t))) {
                    tr_info("Setup OK");

                } else {
                    ok = false;
                }

            } else {
                tr_error("UBX CFG timeout");
                ok = false;
            }

        } else {
            ok = false;
        }

        remove_oob(UBX_CFG, UBX_CFG_PRT);
    }

    return ok;
}

bool UbxGpsI2C::auto_send(UbxClassId class_id, char id, uint8_t rate) {
    tr_info("Request to autosend CLASS: %02hhX, ID: %02hhX, rate: %u", class_id, id, rate);
    _buf[0] = class_id;
    _buf[1] = id;
    _buf[2] = rate;

    if (send_ack(UBX_CFG, UBX_CFG_MSG, _buf, sizeof(cfg_prt_t))) {
        tr_info("Request OK");

        return true;
    }

    return false;
}

bool UbxGpsI2C::set_output_rate(milliseconds ms, uint16_t cycles) {
    cfg_rate_t cfg_rate;
    tr_info("Setting output rate: %llims", duration_cast<milliseconds>(ms).count() * cycles);

    bool ok = send(UBX_CFG, UBX_CFG_RATE);

    if (ok) {
        // register our packet before polling
        oob(UBX_CFG, UBX_CFG_RATE, callback(this, &UbxGpsI2C::cfg_cb));

        if (poll()) {
            tr_debug("Waiting for CFG RATE packet");
            uint32_t cfg = _flags.wait_all(UBX_FLAGS_CFG | UBX_FLAGS_SEARCH_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT);

            if (!(cfg & UBX_FLAGS_ERROR)) {
                tr_debug("CFG RATE packet received");

                memcpy(&cfg_rate, _buf, sizeof(cfg_rate_t)); // _buf contains only payload

                cfg_rate.measRate = duration_cast<milliseconds>(ms).count();
                cfg_rate.navRate = cycles;
                cfg_rate.timeRef = 0;  // UTC

                memcpy(_buf, &cfg_rate, sizeof(cfg_rate_t));

                if (send_ack(UBX_CFG, UBX_CFG_RATE, _buf, sizeof(cfg_rate_t))) {
                    tr_info("Output rate set");

                } else {
                    ok = false;
                }

            } else {
                tr_error("CFG RATE timeout");
                ok = false;
            }

        } else {
            ok = false;
        }

        remove_oob(UBX_CFG, UBX_CFG_RATE);
    }

    return ok;
}

bool UbxGpsI2C::set_odometer(bool enable, UbxOdoProfile profile, uint8_t velocity_filter) {
    odometer_t odo;
    tr_info("%s odometer", enable ? "Setting up" : "Disabling");

    bool ok = send(UBX_CFG, UBX_CFG_ODO);

    if (ok) {
        // register our packet before polling
        oob(UBX_CFG, UBX_CFG_ODO, callback(this, &UbxGpsI2C::cfg_cb));

        if (poll()) {
            tr_debug("Waiting for CFG ODO packet");
            uint32_t cfg = _flags.wait_all(UBX_FLAGS_CFG | UBX_FLAGS_SEARCH_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT);

            if (!(cfg & UBX_FLAGS_ERROR)) {
                tr_debug("CFG ODO packet received");

                memcpy(&odo, _buf, sizeof(odometer_t)); // _buf contains only payload

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

                memcpy(_buf, &odo, sizeof(odometer_t));

                if (send_ack(UBX_CFG, UBX_CFG_ODO, _buf, sizeof(odometer_t))) {
                    tr_info("Odometer set");

                } else {
                    ok = false;
                }

            } else {
                tr_error("CFG ODO timeout");
                ok = false;
            }

        } else {
            ok = false;
        }

        remove_oob(UBX_CFG, UBX_CFG_ODO);
    }

    return ok;
}

bool UbxGpsI2C::set_power_mode(PowerModeValue mode, uint16_t period, uint16_t on_time) {
    cfg_pms_t cfg_pms;
    tr_info("Setting power mode: %u, period: %u, on_time: %u", mode, period, on_time);

    bool ok = send(UBX_CFG, UBX_CFG_PMS);

    if (ok) {
        // register our packet before polling
        oob(UBX_CFG, UBX_CFG_PMS, callback(this, &UbxGpsI2C::cfg_cb));

        if (poll()) {
            tr_debug("Waiting for CFG PMS packet");
            uint32_t cfg = _flags.wait_all(UBX_FLAGS_CFG | UBX_FLAGS_SEARCH_DONE, MBED_CONF_UBXGPSI2C_TIMEOUT, false);

            if (!(cfg & UBX_FLAGS_ERROR)) {
                tr_debug("CFG PMS packet received");

                memcpy(&cfg_pms, _buf, sizeof(cfg_pms_t)); // _buf contains only payload

                cfg_pms.powerSetupValue = (uint8_t)mode;
                cfg_pms.period = (mode == PSV_INTERVAL) ? period : 0;
                cfg_pms.onTime = (mode == PSV_INTERVAL) ? on_time : 0;

                memcpy(_buf, &cfg_pms, sizeof(cfg_pms_t));

                if (send_ack(UBX_CFG, UBX_CFG_PMS, _buf, sizeof(cfg_pms_t))) {
                    tr_info("Power mode set");

                } else {
                    ok = false;
                }

            } else {
                tr_error("CFG PMS timeout");
                ok = false;
            }

        } else {
            ok = false;
        }

        remove_oob(UBX_CFG, UBX_CFG_PMS);
    }

    return ok;
}

bool UbxGpsI2C::reset_odometer() {
    tr_info("Resetting odometer");

    if (send_ack(UBX_NAV, UBX_NAV_RESETODO)) {
        tr_info("Reset OK");

        return true;
    }

    return false;
}
