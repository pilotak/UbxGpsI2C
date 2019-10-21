# UbxGpsI2C
Asynch I2C library for mbed to use Ublox GPS using ubx protocol only

## Example
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

EventQueue eQueue(32 * EVENTS_EVENT_SIZE);
UbxGpsI2C gps(I2C_SDA, I2C_SCL);

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

void print() {
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void data(int payload_len) {  // ISR
    if (gps.buffer[2] == UbxGpsI2C::UBX_NAV && gps.buffer[3] == 0x07) {
        if (payload_len == sizeof(gps_data_t)) {
            memcpy(&gps_data, gps.buffer + UBX_HEADER_LEN, sizeof(gps_data_t));
            eQueue.call(print);
        }
    }
}

int main() {
    Thread eQueueThread;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    // DO NOT try event queue for callback (eQeue.event(callback(data)))
    // gps.buffer must be read in ISR due to DMA access
    if (gps.init(&data) && gps.setOutputRate(500)) {
        while (1) {
            if (gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07)) {  // NAV-PVT
                printf("Request OK\n");

            } else {
                printf("Request failed\n");
            }

            ThisThread::sleep_for(1000);
        }

    } else {
        printf("Cound not init\n");
        MBED_ASSERT(false);
    }
}
```

## Example pass I2C object
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

EventQueue eQueue(32 * EVENTS_EVENT_SIZE);
I2C i2c(I2C_SDA, I2C_SCL);
UbxGpsI2C gps;

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

void print() {
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void data(int payload_len) {  // ISR
    if (gps.buffer[2] == UbxGpsI2C::UBX_NAV && gps.buffer[3] == 0x07) {
        if (payload_len == sizeof(gps_data_t)) {
            memcpy(&gps_data, gps.buffer + UBX_HEADER_LEN, sizeof(gps_data_t));
            eQueue.call(print);
        }
    }
}

int main() {
    Thread eQueueThread;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    // DO NOT try event queue for callback (eQeue.event(callback(data)))
    // gps.buffer must be read in ISR due to DMA access
    if (gps.init(&data, &i2c) && gps.setOutputRate(500)) {
        while (1) {
            if (gps.sendUbx(UbxGpsI2C::UBX_NAV, 0x07)) {  // NAV-PVT
                printf("Request OK\n");

            } else {
                printf("Request failed\n");
            }

            ThisThread::sleep_for(1000);
        }

    } else {
        printf("Cound not init\n");
        MBED_ASSERT(false);
    }
}
```