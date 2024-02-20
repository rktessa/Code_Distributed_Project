

#include "MeOrion.h"
//#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "InterpolationLib.h"

const int numValues = 10;
double xValues[10] = {   5,  12,  30,  50,  60,  70,  74,  84,  92, 100 };
double yValues[10] = { 150, 200, 200, 200, 180, 100, 100, 150, 220, 320 };

///////////////////////////////////////////////////////////////

void setup(void) {
  Serial.begin(115200);
  delay(1000);
    for (float xValue = 0; xValue <= 110; xValue += .25)
	{
	  Serial.print(Interpolation::Step(xValues, yValues, numValues, xValue, 0.0));
		Serial.print(',');
		Serial.print(Interpolation::Step(xValues, yValues, numValues, xValue, 0.5));
		Serial.print(',');
		Serial.print(Interpolation::Step(xValues, yValues, numValues, xValue, 1.0));
		Serial.print(',');
		Serial.print(Interpolation::SmoothStep(xValues, yValues, numValues, xValue));
		Serial.print(',');
		Serial.print(Interpolation::Linear(xValues, yValues, numValues, xValue, false));
		Serial.print(',');
		Serial.print(Interpolation::Linear(xValues, yValues, numValues, xValue, true));
		Serial.print(',');
		Serial.print(Interpolation::CatmullSpline(xValues, yValues, numValues, xValue));
		Serial.print(',');
		Serial.println(Interpolation::ConstrainedSpline(xValues, yValues, numValues, xValue));
	}
  
}

////////////////////////////////////////////////////////////
void loop() {
 
}
