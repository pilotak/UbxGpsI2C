# UbxGpsI2C
[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)

Ublox GPS I2C async library for mbed.

## Example
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

#define GPS_READ_INTERVAL 1000

EventQueue eQueue(2 * EVENTS_EVENT_SIZE);
UbxGpsI2C gps(I2C_SDA, I2C_SCL, &eQueue);

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

void gpsPVT() {
    memcpy(&gps_data, gps.data + UBX_HEADER_LEN, sizeof(gps_data_t));
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void gpsOdo() {
    memcpy(&odo_data, gps.data + UBX_HEADER_LEN, sizeof(odo_data_t));
    printf("Odo: %lu\n", odo_data.distance);
}

int main() {
    Thread eQueueThread;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
        return 0;
    }

    if (!gps.init()) {
        printf("Cound not init\n");
        return 0;
    }

    if (!gps.set_output_rate(GPS_READ_INTERVAL)) {
        printf("PVT rate FAILED\n");
        return 0;
    }

    if (!gps.set_odometer(true, UbxGpsI2C::ODO_RUNNING)) {
        printf("Odo FAILED\n");
        return 0;
    }

    if (!gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, 1)) {
        printf("Auto pvt FAILED\n");
        return 0;
    }

    UbxGpsI2C::cfg_sbas_t cfg_sbas;
    char *buffer = new char[sizeof(UbxGpsI2C::cfg_sbas_t)];

    cfg_sbas.mode = 1; // enable
    cfg_sbas.usage = 0b111; // integrity, diffCorr, range
    cfg_sbas.maxSBAS = 3;
    cfg_sbas.scanmode2 = 0;
    cfg_sbas.scanmode1 = 0b100001011001; // EGNOS: PRN131, PRN126, PRN124, PRN123, PRN120

    memcpy(buffer, &cfg_sbas, sizeof(UbxGpsI2C::cfg_sbas_t));

    if (!gps.send_ack(UbxGpsI2C::UBX_CFG, UBX_CFG_SBAS, buffer, sizeof(UbxGpsI2C::cfg_sbas_t))) {
        printf("SBAS FAILED\n");
        return 0;
    }

    delete[] buffer;

    if (!gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, 1)) {
         printf("Auto odo FAILED\n");
         return 0;
    }

    gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, gpsPVT);
    gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, gpsOdo);

    printf("OK\n");

    while (1) {
        ThisThread::sleep_for(GPS_READ_INTERVAL);

        if (!gps.read()) {
            printf("Request failed\n");
        }
    }

    MBED_ASSERT(false);
}
```

## Example pass I2C object
```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

#define GPS_READ_INTERVAL 1000

EventQueue eQueue(1 * EVENTS_EVENT_SIZE);
I2C i2c(I2C_SDA, I2C_SCL);
UbxGpsI2C gps(&eQueue);

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

void gpsPVT() {
    memcpy(&gps_data, gps.data + UBX_HEADER_LEN, sizeof(gps_data_t));
    printf("fix: %u lat: %li lon: %li\n", gps_data.fixType, gps_data.lat, gps_data.lon);
}

void gpsOdo() {
    memcpy(&odo_data, gps.data + UBX_HEADER_LEN, sizeof(odo_data_t));
    printf("Odo: %lu\n", odo_data.distance);
}

int main() {
    Thread eQueueThread;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    i2c.frequency(400000);

    if (gps.init(&i2c)) {
        if (gps.set_odometer(true, UbxGpsI2C::ODO_RUNNING)) {
            if (gps.set_output_rate(GPS_READ_INTERVAL)) {
                if (gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, 1)) {
                    if (gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, 1)) {

                        gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, gpsPVT);
                        gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, gpsOdo);

                        while (1) {
                            ThisThread::sleep_for(GPS_READ_INTERVAL);

                            if (!gps.read()) {
                                printf("Request failed\n");
                            }
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

## Example with enabled debug
`mbed_app.json`
```json
{
    "config": {
        "trace-level": {
            "help": "Options are TRACE_LEVEL_ERROR,TRACE_LEVEL_WARN,TRACE_LEVEL_INFO,TRACE_LEVEL_DEBUG",
            "macro_name": "MBED_TRACE_MAX_LEVEL",
            "value": "TRACE_LEVEL_DEBUG"
        }
    },
    "target_overrides": {
        "*": {
            "mbed-trace.enable": true,
            "ubxgpsi2c.debug": true
        }
    }
}
```

```cpp
#include "mbed.h"
#include "UbxGpsI2C.h"

#if MBED_CONF_MBED_TRACE_ENABLE
#include "mbed-trace/mbed_trace.h"
static Mutex trace_mutex;

static void trace_wait() {
    trace_mutex.lock();
}

static void trace_release() {
    trace_mutex.unlock();
}

void trace_init() {
    mbed_trace_init();
    // mbed_trace_exclude_filters_set(const_cast<char*>("UBX "));

    mbed_trace_mutex_wait_function_set(trace_wait);
    mbed_trace_mutex_release_function_set(trace_release);
}
#endif

#define GPS_READ_INTERVAL 1000

EventQueue eQueue(1 * EVENTS_EVENT_SIZE);
UbxGpsI2C gps(I2C_SDA, I2C_SCL, &eQueue);

void gpsPVT() {
    printf("PVT data\n");
}

void gpsOdo() {
    printf("Odo data\n");
}

int main() {
#if MBED_CONF_MBED_TRACE_ENABLE
    trace_init();
#endif

    Thread eQueueThread;

    if (eQueueThread.start(callback(&eQueue, &EventQueue::dispatch_forever)) != osOK) {
        printf("eQueueThread error\n");
    }

    if (gps.init()) {
        if (gps.set_odometer(true, UbxGpsI2C::ODO_RUNNING)) {
            if (gps.set_output_rate(GPS_READ_INTERVAL)) {
                if (gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, 1)) {
                    if (gps.auto_send(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, 1)) {

                        gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_PVT, gpsPVT);
                        gps.oob(UbxGpsI2C::UBX_NAV, UBX_NAV_ODO, gpsOdo);

                        while (1) {
                            ThisThread::sleep_for(GPS_READ_INTERVAL);

                            if (!gps.read()) {
                                printf("Request failed\n");
                            }
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