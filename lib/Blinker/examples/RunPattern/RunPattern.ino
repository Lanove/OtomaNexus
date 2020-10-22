#include <LEDBlinker.h>

LEDBlinker builtinLED(LED_BUILTIN, 100);

void setup() {
  int p[] = {1,1,1,1,1,0,1,0,1,0,1,0,1,0};
  builtinLED.set_pattern(p, sizeof(p)/sizeof(p[0]));
}

void loop()
{
  builtinLED.run_pattern();
}