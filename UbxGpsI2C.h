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

#ifndef UBXGPSI2C_H
#define UBXGPSI2C_H

#include "mbed.h"

#define UBX_DEFAULT_ADDRESS (0x42<<1)
#define UBX_HEADER_LEN     6
#define UBX_CHECKSUM_LEN   2

#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

#define UBX_CFG_PRT  0x00
#define UBX_CFG_MSG  0x01
#define UBX_CFG_RST  0x04
#define UBX_CFG_RATE 0x08
#define UBX_CFG_CFG  0x09
#define UBX_CFG_ODO  0x1E

#define UBX_ACK_ACK  0x01

#define UBX_NAV_ODO      0x09
#define UBX_NAV_RESETODO 0x10

#define UBX_NAV_PVT 0x07

class UbxGpsI2C {
  public:
    typedef enum {
        UBX_NAV = 0x01,
        UBX_RXM = 0x02,
        UBX_INF = 0x04,
        UBX_ACK = 0x05,
        UBX_CFG = 0x06,
        UBX_UPD = 0x09,
        UBX_MON = 0x0A,
        UBX_AID = 0x0B,
        UBX_TIM = 0x0D,
        UBX_ESF = 0x10,
        UBX_MGA = 0x13,
        UBX_LOG = 0x21,
        UBX_SEC = 0x27,
        UBX_HNR = 0x28
    } UbxClassId;

    typedef enum {
        ODO_RUNNING = 0,
        ODO_CYCLING,
        ODO_SWIMMING,
        ODO_CAR,
        ODO_CUSTOM
    } UbxOdoProfile;

    UbxGpsI2C(int8_t address = UBX_DEFAULT_ADDRESS);
    UbxGpsI2C(PinName sda, PinName scl, int8_t address = UBX_DEFAULT_ADDRESS, uint32_t frequency = 400000);
    virtual ~UbxGpsI2C(void);

    uint16_t checksum(const char * data, uint16_t payload_len);
    bool     init(event_callback_t cb, I2C * i2c_obj = NULL);
    bool     sendUbx(UbxClassId class_id, uint8_t id, const char * data = NULL, uint16_t data_len = 0);
    uint16_t sendUbxSync(UbxClassId class_id, uint8_t id, const char * data = NULL, uint16_t data_len = 0);
    bool     sendUbxSyncAck(UbxClassId class_id, uint8_t id, const char * data = NULL, uint16_t data_len = 0);

    bool autoSend(UbxClassId class_id, uint8_t id, uint8_t rate = 1);
    bool setOdometer(bool enable, UbxOdoProfile profile, uint8_t velocity_filter = 0);
    bool setOutputRate(uint16_t ms, uint16_t cycles = 1);
    bool poll();
    bool resetOdometer();

    char buffer[MBED_CONF_UBXGPSI2C_RX_SIZE];

  protected:
    struct cfg_prt_t {
        uint8_t port;
        uint8_t reserved1;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudRate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t flags;
        uint8_t reserved2[2];
    };

    struct cfg_rate_t {
        uint16_t measRate;
        uint16_t navRate;
        uint16_t timeRef;
    };

    struct odometer_t {
        uint8_t message_version;
        uint8_t reserved1[3];
        uint8_t flags;
        uint8_t odoCfg;
        uint8_t reserved2[6];
        uint8_t cogMaxSpeed;
        uint8_t cogMaxPosAcc;
        uint8_t reserved3[2];
        uint8_t velLpGain;
        uint8_t cogLpGain;
        uint8_t reserved4[2];
    };

  private:
    I2C * _i2c;
    event_callback_t _cb;
    const int8_t _i2c_addr;
    uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
    char _tx_buf[MBED_CONF_UBXGPSI2C_TX_SIZE];

    bool     correctIndex();
    uint16_t packetBuilder(UbxClassId class_id, uint8_t id, const char * data, uint16_t data_len);
};

#endif  // UBXGPSI2C_H
