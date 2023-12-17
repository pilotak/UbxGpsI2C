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

#define UBX_FLAGS_SEARCH_DONE (1 << 1)
#define UBX_FLAGS_CFG (1 << 2)
#define UBX_FLAGS_ACK_DONE (1 << 3)
#define UBX_FLAGS_NAK (1 << 4)
#define UBX_FLAGS_MON (1 << 5)
#define UBX_FLAGS_ERROR (1 << 31)

class UbxGpsI2C : public UbxParser {
   public:
    UbxGpsI2C(int8_t address = UBX_DEFAULT_ADDRESS);
    UbxGpsI2C(PinName sda, PinName scl, int8_t address = UBX_DEFAULT_ADDRESS, uint32_t frequency = 400000);
    ~UbxGpsI2C();

    bool init(Callback<void()> cb);
    bool poll();
    bool auto_send(UbxClassId class_id, char id, uint8_t rate, Callback<void()> cb);
    void read();

   protected:
    I2C *_i2c;
    EventFlags _flags;

    int bytes_available();
    bool read_sync(int length);
    bool send(UbxClassId class_id, char id, const void *payload, uint16_t payload_len);
    bool send_ack(UbxClassId class_id, char id, const void *payload, uint16_t payload_len);

   private:
    const int8_t _i2c_addr;
    uint32_t _i2c_obj[sizeof(I2C) / sizeof(uint32_t)] = {0};
    char _tx_buffer[MBED_CONF_UBXGPS_TX_BUFFER_SIZE] = {0};
    char _rx_buffer[MBED_CONF_UBXGPS_RX_BUFFER_SIZE] = {0};
    char _ack[2] = {0};
    uint16_t _rx_buffer_len = 0;

    Callback<void()> _cb;  // callback for required poll

    void rx_cb(int event);
    void ack_cb();
};

#endif  // UBXGPSI2C_H
