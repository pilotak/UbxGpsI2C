# UbxGpsI2C
Synch I2C library for mbed to use Ublox GPS using ubx protocol only

## Example
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

char gps_buffer[112];
UbxGpsI2C gps(PB_7, PB_6, gps_buffer, sizeof(gps_buffer));

struct gps_data_t {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSv;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1;
    int32_t headVeh;
    uint8_t reserved2;
} gps_data;

int main() {
    if (gps.init()) {
        printf("Init OK\n");

        while (1) {
            if (gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07, 92) == 92) {
                gps_data_t gps_data;
                memcpy(&gps_data, gps_buffer, 92);
                printf("fix: %u, lat: %lu lon:%lu\n", gps_data.fixType, gps_data.lat, gps_data.lon);
            }

            ThisThread::sleep_for(1000);
        }

    } else {
        printf("cound not init\n");
        MBED_ASSERT(false);
    }
}
```

## Example pass I2C object
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

char gps_buffer[112];

I2C i2c(PC_9, PA_8);
UbxGpsI2C gps(&i2c, gps_buffer, sizeof(gps_buffer));

struct gps_data_t {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSv;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1;
    int32_t headVeh;
    uint8_t reserved2;
} gps_data;

int main() {
    if (gps.init()) {
        printf("Init OK\n");

        while (1) {
            if (gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07, 92) == 92) {
                gps_data_t gps_data;
                memcpy(&gps_data, gps_buffer, 92);
                printf("fix: %u, lat: %lu lon:%lu\n", gps_data.fixType, gps_data.lat, gps_data.lon);
            }

            ThisThread::sleep_for(1000);
        }

    } else {
        printf("cound not init\n");
        MBED_ASSERT(false);
    }
}
```