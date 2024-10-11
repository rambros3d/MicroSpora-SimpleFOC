/*
  Microspora - Blink example
  
  This code blinks the inbuilt LED connected to pin PC6
  The LED will turn on for 1 second, then turn off for 1 second, repeatedly.
*/

#include "Arduino.h"

#define BLINK_DELAY 1000
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(BLINK_DELAY);
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(BLINK_DELAY);
}
