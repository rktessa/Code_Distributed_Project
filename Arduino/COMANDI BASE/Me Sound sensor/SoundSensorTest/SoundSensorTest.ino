/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    SoundSensorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/01
 * @brief   Description: this file is sample code for Me sound sensor device.
 *
 * Function List:
 * 1. int16_t MeSoundSensor::strength()
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/01    1.0.0          rebuild the old lib
 * </pre>
 */
//#include "MeOrion.h"
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeSoundSensor mySound(PORT_4);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.print("value=");
  Serial.println(mySound.strength() );
  delay(100);
}
