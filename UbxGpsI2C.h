/*
MIT License

Copyright (c) 2018 Pavel Slama

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

#ifndef DEBOUNCEIN_H
#define DEBOUNCEIN_H

#include "mbed.h"

#define DEFAULT_ADDRESS (0x42<<1)
#define DEFAULT_TIMEOUT 2000  // ms
#define TX_BUFFER_SIZE 48

#define SYNC_CHAR1 0xB5
#define SYNC_CHAR2 0x62

#define CFG_PRT 0x00
#define ACK_ACK 0x01

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

  UbxGpsI2C(char * buf, uint16_t buf_size, int8_t address = DEFAULT_ADDRESS);
  UbxGpsI2C(PinName sda, PinName scl, char * buf, uint16_t buf_size, int8_t address = DEFAULT_ADDRESS, uint32_t frequency = 400000);
  bool init(I2C * i2c_obj = NULL);
  bool sendUbxAck(UbxClassId class_id, uint8_t id, const char * data, uint16_t tx_len);
  bool sendUbx(UbxClassId class_id, uint8_t id, const char * data, uint16_t tx_len, uint16_t rx_len = 0, event_callback_t function = NULL,
               bool include_header_checksum = true);

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

  bool send(uint8_t tx_size, uint16_t rx_size);
  uint16_t checksum(const char * data, uint16_t len, uint16_t offset = 0);

 private:
  I2C * _i2c;
  event_callback_t _done_cb;
  Semaphore _semaphore;

  const uint16_t _buf_size;
  const int8_t _i2c_addr;

  char * _rx_buf;
  char _tx_buf[TX_BUFFER_SIZE];
  uint16_t _req_len;
  bool _got_ubx_data;
  bool _include_header_checksum;

  void internalCb(int event);
};

#endif
