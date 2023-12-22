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

#ifndef UBXGPSI2C_H
#define UBXGPSI2C_H

#include <chrono>

#include "UbxParser.h"

using namespace std::chrono;

#define UBX_DEFAULT_ADDRESS (0x42 << 1)

#define UBX_FLAGS_DATA_DONE (1 << 1)
#define UBX_FLAGS_ERROR (1 << 31)

class UbxGpsI2C : public UbxParser {
   public:
    UbxGpsI2C(int8_t address = UBX_DEFAULT_ADDRESS);
    UbxGpsI2C(PinName sda, PinName scl, int8_t address = UBX_DEFAULT_ADDRESS, uint32_t frequency = 400000);
    ~UbxGpsI2C();

    bool init(Callback<void()> cb, I2C *i2c_obj = nullptr);
    bool poll();
    void process();
    bool send(UbxClassId class_id, char id, const void *payload = nullptr, uint16_t payload_len = 0);
    bool send_ack(UbxClassId class_id, char id, const void *payload = nullptr, uint16_t payload_len = 0);

    bool auto_send(UbxClassId class_id, char id, uint8_t rate, Callback<void()> cb);
    bool set_output_rate(milliseconds ms, uint16_t cycles = 1);
    bool set_odometer(bool enable, OdoCfgProfile profile, uint8_t velocity_filter = 0);
    bool set_power_mode(PowerSetupValue mode, uint16_t period = 0, uint16_t on_time = 0);
    bool set_psm(bool low_power);
    bool reset(ResetMode mode, uint16_t bbr_mask);
    bool reset_odometer();
    bool permanent_configuration(PermanentConfig type, uint32_t mask, uint8_t device_mask);
    bool set_dynamic_model(DynamicModel model);
    bool wakeup();
    bool protocol_version(char *version);

   protected:
    I2C *_i2c;

    int bytes_available();
    bool read_sync(int length);

   private:
    Callback<void()> _cb;  // callback for required poll
    Mutex _mutex;

    uint32_t _i2c_obj[sizeof(I2C) / sizeof(uint32_t)] = {0};
    uint16_t _rx_buffer_len = 0;
    const int8_t _i2c_addr;
    bool _done_cb_called = false;
    bool _new_cfg = false;
    char _tx_buffer[MBED_CONF_UBXGPS_TX_BUFFER_SIZE] = {0};  // This buffer is also used for reading (internal purpose)
    char _rx_buffer[MBED_CONF_UBXGPS_RX_BUFFER_SIZE] = {0};

    void rx_cb(int event);
    void done_cb();
    bool get(UbxClassId class_id, char id);
};

#endif  // UBXGPSI2C_H
