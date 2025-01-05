#ifndef _Logger_h
#define _Logger_h

#include  "Arduino.h"

namespace logger{
  struct ALOG{
    uint32_t stamp;
    uint8_t mode;
    uint8_t duty;
    uint8_t cmd;
    uint16_t interval;
    uint16_t latency;
    uint16_t omega;
    int16_t beta;
    int16_t sigma;
    int16_t eval;
  };
  extern ALOG stage;
  extern ALOG *data;
  extern int size;
  void start();
  void latch();
  void dump();
  int length();
  int limit();
}

#endif
