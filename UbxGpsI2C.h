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
#include "mbed_events.h"

#define DEFAULT_ADDRESS (0x42<<1)
#define DEFAULT_REPEAT_TIMEOUT 100  // ms
#define TX_BUFFER_SIZE 48

#define UBX_ACK 0x05
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00

class UbxGpsI2C {
 public:
  UbxGpsI2C(char * buf, uint16_t buf_size, int8_t address = DEFAULT_ADDRESS);
  bool init(I2C * i2c_obj, EventQueue * queue);
  void get(event_callback_t function, uint8_t repeat_timeout = DEFAULT_REPEAT_TIMEOUT);
  bool sendUbxAck(uint8_t class_id, uint8_t id, const char * data, uint16_t len);
  bool sendUbx(uint8_t class_id, uint8_t id, const char * data, uint16_t len, uint16_t rx_len = 0);

 private:
  typedef enum {
    start,
    ready,
    getLen,
    getData,
    sendOk,
    getAck,
    getUbx
  } Stage_t;

  I2C * _i2c;
  EventQueue * _queue;
  event_callback_t _done_cb;
  EventFlags _event;

  const uint16_t _buf_size;
  const int8_t _i2c_addr;

  int _queue_id;
  bool _initialized;
  Stage_t _step;
  char * _rx_buf;
  char _tx_buf[TX_BUFFER_SIZE];
  uint16_t _rx_len;
  uint8_t _repeat_timeout;
  bool _send_status;

  void internalCb(int event);
  bool send(uint8_t tx_size, uint16_t rx_size);
  void getLenCb();
};

#endif
