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

#ifndef UBXPARSER_H
#define UBXPARSER_H

#include "mbed-trace/mbed_trace.h"
#include "mbed.h"

#ifndef TRACE_GROUP
    #define TRACE_GROUP "UBX "
#endif

#if !defined(MBED_CONF_UBXGPS_DEBUG)
    #define ubx_error(...) \
        {}
    #define ubx_warning(...) \
        {}
    #define ubx_info(...) \
        {}
    #define ubx_debug(...) \
        {}
#else
    #define ubx_error tr_error
    #define ubx_warning tr_warning
    #define ubx_info tr_info
    #define ubx_debug tr_debug
#endif

#define UBX_HEADER_LEN 6
#define UBX_CHECKSUM_LEN 2

#define UBX_SYNC_CHAR1 0xB5
#define UBX_SYNC_CHAR2 0x62

#define UBX_CFG_PRT 0x00
#define UBX_CFG_MSG 0x01
#define UBX_CFG_RST 0x04
#define UBX_CFG_RATE 0x08
#define UBX_CFG_CFG 0x09
#define UBX_CFG_RXM 0x11
#define UBX_CFG_SBAS 0x16
#define UBX_CFG_ODO 0x1E
#define UBX_CFG_NAV5 0x24
#define UBX_CFG_PM2 0x3B
#define UBX_CFG_PMS 0x86

#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01

#define UBX_NAV_PVT 0x07
#define UBX_NAV_ODO 0x09
#define UBX_NAV_RESETODO 0x10

#define UBX_MON_VER 0x04

class UbxParser {
   public:
    struct cfg_msg_t {
        uint8_t classId;
        uint8_t id;
        uint8_t rate;
    };

    struct cfg_prt_t {
        uint8_t portID;
        uint8_t reserved1;
        uint16_t txReady;
        uint32_t mode;
        uint8_t reserved2[4];
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t flags;
        uint8_t reserved3[2];
    };

    struct cfg_prt_usb_t {
        uint8_t portID;
        uint8_t reserved1;
        uint16_t txReady;
        uint8_t reserved2[8];
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint8_t reserved3[2];
        uint8_t reserved4[2];
    };

    struct cfg_prt_uart_t {
        uint8_t portID;
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

    struct cfg_pms_t {
        uint8_t version;
        uint8_t powerSetupValue;
        uint16_t period;
        uint16_t onTime;
        uint8_t reserved[2];
    };

    struct cfg_odo_t {
        uint8_t messageVersion;
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

    struct cfg_rxm_t {
        uint8_t reserved;
        uint8_t lpMode;
    };

    struct cfg_cfg_t {
        uint32_t clearMask;
        uint32_t saveMask;
        uint32_t loadMask;
        uint8_t deviceMask;
    };

    struct cfg_nav5_t {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t dgnssTimeout;
        uint8_t cnoThreshNumSVs;
        uint8_t cnoThresh;
        uint8_t reserved1[2];
        uint16_t staticHoldMaxDist;
        uint8_t utcStandard;
        uint8_t reserved2[5];
    };

    struct cfg_sbas_t {
        uint8_t mode;
        uint8_t usage;
        uint8_t maxSBAS;
        uint8_t scanmode2;
        uint32_t scanmode1;
    };

    struct cfg_pm2_t {
        uint8_t version;
        uint8_t reserved1;
        uint8_t maxStartupStateDur;  // s
        uint8_t reserved2;
        uint32_t flags;
        uint32_t updatePeriod;  // ms
        uint32_t searchPeriod;  // ms
        uint32_t gridOffset;    // ms
        uint16_t onTime;        // s
        uint16_t minAcqTime;    // s
        uint8_t reserved3[20];
    };

    struct cfg_rst_t {
        uint16_t navBbrMask;
        uint8_t resetMode;
        uint8_t reserved1;
    };

    struct message_t {
        uint8_t classId;
        uint8_t id;
        uint16_t length;
        std::unique_ptr<uint8_t[]> data;
        uint16_t checksum;
    };

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

    typedef enum { ODO_RUNNING = 0, ODO_CYCLING, ODO_SWIMMING, ODO_CAR, ODO_CUSTOM } OdoCfgProfile;

    typedef enum {
        PSV_FULL_POWER = 0,  // No compromises on power saves
        PSV_BALANCED,        // Power savings without performance degradation
        PSV_INTERVAL,        // ON OFF mode setup
        PSV_AGGRESSIVE_1HZ,  // Strong power saving setup
        PSV_AGGRESSIVE_2HZ,  // Excellent power saving setup
        PSV_AGGRESSIVE_4HZ,  // Good power saving setup
        PSV_INVALID = 0xFF,
    } PowerSetupValue;

    typedef enum { Clear, Save, Load } PermanentConfig;

    typedef enum {
        DevBBR = 0b1,           // Battery backed RAM
        DevFlash = 0b10,        // Flash
        DevEEPROM = 0b100,      // EEPROM
        DevSpiFlash = 0b10000,  // SPI Flash
        DevAll = 0b10111
    } DeviceMask;

    typedef enum {
        Portable = 0,
        Stationary = 2,
        Pedestrian,
        Automotive,
        Sea,
        Airborne1g,
        Airborne2g,
        Airborne4g,
        WristWornWatch,
        Bike
    } DynamicModel;

    typedef enum {
        HardwareReset = 0,  // immediately
        SoftwareReset,
        GnssReset,
        AfterShutdown = 4,
        ControlledGnssStop = 8,
        ControlledGnssStart,
    } ResetMode;

    UbxParser();
    ~UbxParser(void);

    uint16_t buildMessage(char *buffer, UbxClassId class_id, char id, const void *payload = nullptr,
                          uint16_t payload_len = 0);
    void oob(UbxClassId class_id, char id, Callback<void()> cb);
    void remove_oob(UbxClassId class_id, char id);
    void parse(const char *buffer, uint16_t length);

    message_t msg = {0};

   private:
    struct oob_t {
        char class_id;
        char id;
        Callback<void()> cb;
        oob_t *next;
    };

    typedef enum {
        SYNC_SEARCH_1,
        SYNC_SEARCH_2,
        CLASS_READ,
        ID_READ,
        LENGTH_READ_1,
        LENGTH_READ_2,
        DATA_READ,
        CHECKSUM_READ_1,
        CHECKSUM_READ_2
    } StateMachine;

    oob_t *_oobs = nullptr;
    EventFlags _flags;
    uint8_t _checksum_calc[2] = {0};

    void add_checksum(char b);
};

#endif  // UBXPARSER_H
