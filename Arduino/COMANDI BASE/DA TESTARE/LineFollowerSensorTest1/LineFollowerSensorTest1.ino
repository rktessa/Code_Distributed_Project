
//#include "MeOrion.h"

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeLineFollower lineFinder(PORT_2); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

void setup()
{
  Serial.begin(9600);
}

void loop()
{
    int sensorState = lineFinder.readSensors(); //RILEVA SOLO BIANCO O NERO
  switch(sensorState)
  {
    case S1_IN_S2_IN: Serial.println("Sensor 1 and 2 are inside of black line"); break; //ENTRAMBI FUORI DAL PAVIMENTO
    case S1_IN_S2_OUT: Serial.println("Sensor 2 is outside of black line"); break; //LATO DE00STRO SUL PAVIMENTO
    case S1_OUT_S2_IN: Serial.println("Sensor 1 is outside of black line"); break; //LATO SINISTRO SUL PAVIMENTO
    case S1_OUT_S2_OUT: Serial.println("Sensor 1 and 2 are outside of black line"); break; // ENTRAMBI SUL PAVIMENTO
    default: break;
  }
  delay(200);
  
}
