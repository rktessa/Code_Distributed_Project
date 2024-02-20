

//VEDI SE APPENA PREMI SI ACCENDE LED BLU E RIMANE ACCESO
//VEDI SE PREMENDO RESET SI SPEGNE 
//TOGLI COMMENTO E PROVA SE RIPREMENDO PULSATE SI SPEGNE

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//MeRGBLed rgbled_7(7, 2);
int ledpin =7;
int buttonstate =0;

const int analogPin = A7;
int analogValue = analogRead(analogPin);

void setup() {
  pinMode(analogPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledpin, OUTPUT);
  //rgbled_7.fillPixelsBak(0, 2, 1);
Serial.begin(9600);

while(buttonstate ==0){
  int analogValue = analogRead(analogPin);
  if(analogValue < 10){
    buttonstate = 1;
    }
  }
}


void loop() {
//attivazione pulsante
Serial.println(analogValue);
  delay(1);     

  Serial.println(buttonstate);
  delay(1);     
//esecuzione
  while(buttonstate == 1){
    digitalWrite(LED_BUILTIN, HIGH);
     delay(1000);
    int analogValue = analogRead(analogPin);
  //arresto pulsante
    if(analogValue < 10){
    buttonstate = 0;
    digitalWrite(LED_BUILTIN, LOW);
    }
   }
}
    
   
