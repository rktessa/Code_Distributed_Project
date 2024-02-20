#include <Wire.h>
#include <EEPROM.h>
#define pi 3.1415926535897932384626433
unsigned long time;
long t;
double s;

void setup()
{  Serial.begin(9600); }

void loop() 
{  //Serial.print("Time: ");
   t = millis();
   s = sin(2*3.141*(t));

   //Serial.println(t);
   //Serial.print("Sine: ");
   Serial.print(2);
   Serial.print(" ");
   Serial.print(-2);
   Serial.print(" ");
   Serial.println(s);
  // delay(1000);   
   }
