Arduino LED Blinker
======

*Simple library for Arduino that offers control over LEDs without the use of delay functions.*

Turn LEDs on/off, toggle them, make them blink or even run custom patterns! All without the use of delay functions, so your firmware can run continuously smooth!

Features
========
 - Easy control of LEDs
 - Simple blinking
 - Custom pattern support
 - No delay functions

Installation
============
 - Download the [ZIP archive](https://github.com/fulf/arduino-led-blinker/archive/master.zip) of the project (from GitHub's Clone or download menu).
 - Open the Arduino IDE 
 - Go to *Sketch* -> *Include Library* -> *Add .ZIP Library...* 
 - Select the ZIP you just downloaded

Quick start
===========
Include the library in your project
```c++
#include <LEDBlinker.h>
```


To create an LED Blinker you must specify the LED's pin
```c++
LEDBlinker led(1);
```


The constructor also accepts a second argument describing the interval for the *blink()* and *run_pattern()* methods, in **miliseconds**
```c++
LEDBlinker led(1, 250);
```


Setting the blinking interval:
```c++
led.set_interval(500);
```


Setting a pattern (necesarry for *run_pattern()*):
```c++
int pattern[] = {1,1,1,0,1,0,1,0};
int size = sizeof(p)/sizeof(p[0]); //8

led.set_pattern(pattern, size);
```


Turning the LED ON/OFF:
```c++
led.on();
led.off();
```


Making te LED blink (should be in a looping method):
```c++
led.blink();
```

Running the LED pattern (should be in a looping method):
```c++
// the pattern defaults to a simple blinking pattern unless otherwise set
led.run_pattern();
```