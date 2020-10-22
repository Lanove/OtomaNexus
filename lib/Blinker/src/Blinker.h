#ifndef BLINKER_H
#define BLINKER_H

#define BLINKER_MODE_PATTERN 0
#define BLINKER_MODE_BLINK 1
#define BLINKER_STATUS_INACTIVE 0
#define BLINKER_STATUS_ACTIVE 1

#include "Arduino.h"

class Blinker
{
private:
  void (*bitWriter)(uint8_t, uint8_t);
  unsigned long blinkedAt, disableAfterDuration, startDisableAt, interval;
  uint8_t ledPin,
      ledState,
      pattern[20],
      patternSize,
      patternIndex,
      mode,
      status = BLINKER_STATUS_INACTIVE,
      disableAfterFlag = false;
  void init_pattern();
  void init_led();

public:
  Blinker(uint8_t pin, void (*bitWriterPointer)(uint8_t, uint8_t));
  Blinker(uint8_t pin, int ms_interval, void (*bitWriterPointer)(uint8_t, uint8_t));
  void setInterval(unsigned long);
  void setPattern(unsigned long ms_interval, uint8_t *p, uint8_t s);
  void update();
  void on();
  void off();
  void toggle();
  void disable();
  uint8_t getMode();
  uint8_t getStatus();
  void disableAfter(unsigned long duration);
};

#endif