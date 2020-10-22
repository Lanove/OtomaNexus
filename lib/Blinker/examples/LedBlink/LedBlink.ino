#include <LEDBlinker.h>

LEDBlinker builtinLED(LED_BUILTIN);

void setup() {
  builtinLED.set_interval(250);
}

void loop()
{
  builtinLED.blink();
}
