/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    EncoderMotorTestMoveTo.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/11/19
 * @brief   Description: this file is sample code for Encoder Motor  device.
 *
 * Function List:
 *
 *    1. void MeEncoderMotor::begin();
 *    2. boolean MeEncoderMotor::moveTo(float angle, float speed);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * forfish      2015/11/19    1.0.0          add some descriptions
 * </pre>
 */

#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>

MeEncoderMotor motor1(0x09, SLOT1);   //  Motor at slot1
MeEncoderMotor motor2(0x09, SLOT2);   //  motor at slot2

int n = 0;
int cycleTime=100;

void setup()
{
  motor1.begin();
  motor2.begin();
  //motor1.setRatio(45);
  Serial.begin(9600);
}

void loop(){

  if(n<4000/cycleTime){
    motor1.runSpeed(50);
    motor2.runSpeed(50);
  }else{
    motor1.runSpeed(100);
    motor2.runSpeed(100);
  }

  if(n>8000/cycleTime){
    n = 0;
  }
  /*
  Serial.print(motor1.getCurrentPosition());
  Serial.print(' ');
  Serial.print(motor2.getCurrentPosition());
  Serial.print(' ');
  */
  Serial.print(motor1.getCurrentSpeed());
  Serial.print(' ');
  Serial.println(motor2.getCurrentSpeed());
  /*
  Serial.print("POS_1: ");
  Serial.print(motor1.getCurrentPosition());//angle
  Serial.print(" POS_2: ");
  Serial.print(motor2.getCurrentPosition());//angle
  Serial.println(' ');
  Serial.print("SPE_1: ");
  Serial.print(motor1.getCurrentSpeed());
  Serial.print(" SPE_2: ");
  Serial.print(motor2.getCurrentSpeed());
  Serial.println(' ');
  /*
  Serial.print("RT_1: ");
  Serial.print(motor1.getRatio());
  Serial.print(" RT_2: ");
  Serial.print(motor2.getRatio());
  Serial.println(' ');
  */
  
  delay(cycleTime);
  n++;
}
/*
void loop()
{
  motor2.moveTo(360, 200);
  motor1.moveTo(360, 200);
  delay(2000);
  motor2.moveTo(0, 100);
  motor1.moveTo(0, 100);
  delay(2000);
}

*/
