
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
    int sensorState = lineFinder.readSensors();
    Serial.println(sensorState);
    //sensorState = 0 => FUORI DAL PAVIMENTO TUTTI E DUE
    //sensorState = 1 => FUORI DAL PAVIMENTO LATO SINISTRO
    //sensorState = 2 => FUORI DAL PAVIMENTO LATO DESTRO
    //sensorState = 3 => DENTRO DAL PAVIMENTO ENTRAMBI
  delay(200);
  
}
