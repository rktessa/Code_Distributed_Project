
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//MeRGBLed rgbled_7(7, 2);
int ledpin =7;
void setup() {
  pinMode(A7, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledpin, OUTPUT);
  //rgbled_7.fillPixelsBak(0, 2, 1);
  while(!((0 ^ (analogRead(A7) > 10 ? 0 : 1))))
  {
    _loop();
  }
  while(1) {
      digitalWrite(LED_BUILTIN, HIGH); //led blu piccolo nominato su scheda L
      _loop();
  }

}

void _loop() {
}

void loop() {
  _loop();
}
