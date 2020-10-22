#include "Blinker.h"

Blinker::Blinker(uint8_t pin, void (*bitWriterPointer)(uint8_t, uint8_t))
{
  bitWriter = bitWriterPointer;
  ledPin = pin;
}

Blinker::Blinker(uint8_t pin, int ms_interval, void (*bitWriterPointer)(uint8_t, uint8_t))
{
  bitWriter = bitWriterPointer;
  ledPin = pin;
  interval = ms_interval;
}

void Blinker::setInterval(unsigned long ms_interval)
{
  on();
  blinkedAt = millis();
  interval = ms_interval;
  mode = BLINKER_MODE_BLINK;
  status = BLINKER_STATUS_ACTIVE;
  disableAfterFlag = false;
}

void Blinker::setPattern(unsigned long ms_interval, uint8_t *p, uint8_t s)
{
  (p[0] == LOW) ? off() : on();
  patternIndex = 0;
  blinkedAt = millis();
  interval = ms_interval;
  mode = BLINKER_MODE_PATTERN;
  status = BLINKER_STATUS_ACTIVE;
  patternSize = s;
  disableAfterFlag = false;

  for (uint8_t i = 0; i < patternSize; ++i)
    pattern[i] = p[i];
}

uint8_t Blinker::getMode()
{
  return mode;
}

void Blinker::update()
{
  if (status == BLINKER_STATUS_ACTIVE)
  {
    unsigned long uptime = millis();
    if (uptime - blinkedAt >= interval)
    {
      blinkedAt = uptime;
      if (mode == BLINKER_MODE_BLINK)
        toggle();
      else
      {
        pattern[patternIndex++] == 1 ? on() : off();
        if (patternIndex >= patternSize)
          patternIndex = 0;
      }
    }
    if (uptime - startDisableAt >= disableAfterDuration && disableAfterFlag)
    {
      status = BLINKER_STATUS_INACTIVE;
      disableAfterFlag = false;
    }
  }
}

void Blinker::on()
{
  ledState = HIGH;
  bitWriter(ledPin, ledState);
}

void Blinker::off()
{
  ledState = LOW;
  bitWriter(ledPin, ledState);
}

void Blinker::toggle()
{
  ledState = (ledState == LOW) ? HIGH : LOW;
  bitWriter(ledPin, ledState);
}

uint8_t Blinker::getStatus()
{
  return status;
}

void Blinker::disable()
{
  status = BLINKER_STATUS_INACTIVE;
  disableAfterFlag = false;
}

void Blinker::disableAfter(unsigned long duration)
{
  disableAfterFlag = true;
  startDisableAt = millis();
  disableAfterDuration = duration;
}