/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    PIRMotionSensorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/10
 * @brief   Description: this file is sample program for PIR Motion module.
 *
 * Function List:
 * 1. void    MePIRMotionSensor::SetPirMotionMode(uint8_t ModePin)
 * 2. bool    MePIRMotionSensor::isHumanDetected();
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/07/24     1.0.0            Rebuild the old lib.
 * Rafael Lee       2015/09/10     1.0.1            Added some comments and macros.
 * </pre>
 */

/* Includes ------------------------------------------------------------------*/
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

/* Private variables ---------------------------------------------------------*/
MePIRMotionSensor myPIRsensor(PORT_3);

void(* resetFunc) (void) = 0;

void setup()
{
  Serial.begin(115200);
  myPIRsensor.SetPirMotionMode(1);   //Continuous Trigger mode
  Serial.println("Setup");
  pinMode(A7, INPUT);
}

void loop()
{
  Serial.println("People Motion NON Detected");
  if(myPIRsensor.isHumanDetected() )
  {
    Serial.println("People Motion Detected");
  }
  delay(100);

  if(analogRead(A7) < 10){
    resetFunc();
  }
}
