# UbxGpsI2C
Asynch I2C library for mbed to use Ublox GPS using ubx protocol only

## Example
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

char buf[112];  // longest requested len + 8 for header + 10 extra space

UbxGpsI2C gps(PC_9, PA_8, buf, sizeof(buf));  // sda, scl, pointer to buffer, size of buffer

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


void data(int len) {
    printf("data[%i]: \n", len);

    for (int i = 0; i < len; ++i) {
        printf("%02X ", buf[i]);
    }

    printf("\n");

    if (len == 92) {
        memcpy(&gps_data, buf, len);
        printf("lat: %lu lon:%lu\n", gps_data.lat, gps_data.lon);
    }
}


int main() {
    if (gps.init()) {
        while (1) {
            // uBlox class, message id, nothing to send, zero tx len, expecting 92 bytes of payload, callback, include header and checksum in buffer
            gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07, NULL, 0, 92, callback(data), false);
            wait_ms(200);
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

I2C i2c(PC_9, PA_8);

char buf[112];  // longest requested len + 8 for header + 10 extra space

UbxGpsI2C gps(buf, sizeof(buf));  // pointer to buffer, size of buffer

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


void data(int len) {
    printf("data[%i]: \n", len);

    for (int i = 0; i < len; ++i) {
        printf("%02X ", buf[i]);
    }

    printf("\n");

    if (len == 92) {
        memcpy(&gps_data, buf, len);
        printf("lat: %lu lon:%lu\n", gps_data.lat, gps_data.lon);
    }
}


int main() {
    if (gps.init(&i2c)) {
        while (1) {
            // uBlox class, message id, nothing to send, zero tx len, expecting 92 bytes of payload, callback, include header and checksum in buffer
            gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07, NULL, 0, 92, callback(data), false);
            wait_ms(200);
        }

    } else {
        printf("cound not init\n");
        MBED_ASSERT(false);
    }
}
```