# UbxGpsI2C
Ublox GPS I2C async library for mbed. All config messages are sent in blocking mode while `poll()` is asynch menthod to get buffer with data from GPS.

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
    uint8_t reserved1[6];
    int32_t headVeh;
    uint8_t reserved2[4];
} gps_data;

struct odo_data_t {
    uint8_t  version;
    uint8_t  reserved1[3];
    uint32_t iTOW;
    uint32_t distance;
    uint32_t totalDistance;
    uint32_t distanceStd;
} odo_data;

void printPVT() {
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void printOdo() {
    printf("Odo: %lu\n", odo_data.distance);
}

void data(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        for (uint16_t i = 0; i < MBED_CONF_UBXGPSI2C_RX_SIZE; i++) {
            if (gps.buffer[i] == UBX_SYNC_CHAR1 && gps.buffer[i + 1] == UBX_SYNC_CHAR2) {  // find index of header
                uint16_t payload_len = ((gps.buffer[5 + i] << 8) | gps.buffer[4 + i]);

                if (payload_len <= (i + payload_len)) {  // fits into this buffer
                    if (gps.buffer[2 + i] == UbxGpsI2C::UBX_NAV) {
                        if (gps.buffer[3 + i] == UBX_NAV_PVT && payload_len == sizeof(gps_data_t)) {
                            memcpy(&gps_data, gps.buffer + i + UBX_HEADER_LEN, sizeof(gps_data_t));
                            eQueue.call(printPVT);

                        } else if (gps.buffer[3 + i] == UBX_NAV_ODO && payload_len == sizeof(odo_data_t)) {
                            memcpy(&odo_data, gps.buffer + i + UBX_HEADER_LEN, sizeof(odo_data_t));
                            eQueue.call(printOdo);
                        }
                    }

                    i += (payload_len + UBX_CHECKSUM_LEN + UBX_HEADER_LEN - 1);

                    if (gps.buffer[i + 1] != UBX_SYNC_CHAR1) {  // no more valid data
                        break;
                    }
                }
            }
        }
    }
}

int main() {
    Thread eQueueThread;;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    // DO NOT try event queue for callback (eQeue.event(callback(data))) gps.buffer must be read in ISR due to DMA access
    if (gps.init(&data)) {
        if (gps.setOdometer(true, UbxGpsI2C::ODO_RUNNING)) {
            if (gps.setOutputRate(1000)) {
                if (gps.autoSend(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, 1)) {
                    if (gps.autoSend(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, 1)) {
                        while (1) {
                            if (!gps.poll()) {
                                printf("Request failed\n");
                            }

                            ThisThread::sleep_for(1000);
                        }

                    } else {
                        printf("Auto PVT FAILED\n");
                    }

                } else {
                    printf("Auto odo FAILED\n");
                }

            } else {
                printf("PVT rate FAILED\n");
            }

        } else {
            printf("Odo FAILED\n");
        }

    } else {
        printf("Cound not init\n");
    }

    MBED_ASSERT(false);
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
    uint8_t reserved1[6];
    int32_t headVeh;
    uint8_t reserved2[4];
} gps_data;

struct odo_data_t {
    uint8_t  version;
    uint8_t  reserved1[3];
    uint32_t iTOW;
    uint32_t distance;
    uint32_t totalDistance;
    uint32_t distanceStd;
} odo_data;

void printPVT() {
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void printOdo() {
    printf("Odo: %lu\n", odo_data.distance);
}

void data(int event) {  // ISR
    if (event > I2C_EVENT_ERROR_NO_SLAVE) {
        for (uint16_t i = 0; i < MBED_CONF_UBXGPSI2C_RX_SIZE; i++) {
            if (gps.buffer[i] == UBX_SYNC_CHAR1 && gps.buffer[i + 1] == UBX_SYNC_CHAR2) {  // find index of header
                uint16_t payload_len = ((gps.buffer[5 + i] << 8) | gps.buffer[4 + i]);

                if (payload_len <= (i + payload_len)) {  // fits into this buffer
                    if (gps.buffer[2 + i] == UbxGpsI2C::UBX_NAV) {
                        if (gps.buffer[3 + i] == UBX_NAV_PVT && payload_len == sizeof(gps_data_t)) {
                            memcpy(&gps_data, gps.buffer + i + UBX_HEADER_LEN, sizeof(gps_data_t));
                            eQueue.call(printPVT);

                        } else if (gps.buffer[3 + i] == UBX_NAV_ODO && payload_len == sizeof(odo_data_t)) {
                            memcpy(&odo_data, gps.buffer + i + UBX_HEADER_LEN, sizeof(odo_data_t));
                            eQueue.call(printOdo);
                        }
                    }

                    i += (payload_len + UBX_CHECKSUM_LEN + UBX_HEADER_LEN - 1);

                    if (gps.buffer[i + 1] != UBX_SYNC_CHAR1) {  // no more valid data
                        break;
                    }
                }
            }
        }
    }
}

int main() {
    Thread eQueueThread;;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    // DO NOT try event queue for callback (eQeue.event(callback(data))) gps.buffer must be read in ISR due to DMA access
    if (gps.init(&data, &i2c)) {
        if (gps.setOdometer(true, UbxGpsI2C::ODO_RUNNING)) {
            if (gps.setOutputRate(1000)) {
                if (gps.autoSend(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, 1)) {
                    if (gps.autoSend(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, 1)) {
                        while (1) {
                            if (!gps.poll()) {
                                printf("Request failed\n");
                            }

                            ThisThread::sleep_for(1000);
                        }

                    } else {
                        printf("Auto PVT FAILED\n");
                    }

                } else {
                    printf("Auto odo FAILED\n");
                }

            } else {
                printf("PVT rate FAILED\n");
            }

        } else {
            printf("Odo FAILED\n");
        }

    } else {
        printf("Cound not init\n");
    }

    MBED_ASSERT(false);
}
```