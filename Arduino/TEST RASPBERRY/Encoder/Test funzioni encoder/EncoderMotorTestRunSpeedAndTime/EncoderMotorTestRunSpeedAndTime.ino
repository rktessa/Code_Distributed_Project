/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    EncoderMotorTestRunSpeedAndTime.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/11/19
 * @brief   Description: this file is sample code for Encoder Motor  device.
 *
 * Function List:
 *
 *    1. void MeEncoderMotor::begin();
 *    2. boolean MeEncoderMotor::runSpeedAndTime(float speed, float time);
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

MeEncoderNew motor1(0x09, SLOT1);
MeEncoderNew motor2(0x09, SLOT2);   //  motor at slot2

int n=0;
void setup()
{
  motor1.begin();
  motor2.begin();
  motor1.setRatio(45.0); //Rapporto di riduzione motore
  motor2.setRatio(45.0);

  motor1.setPulse(13); // Numero di pulse che legge encoder in un giro del motore
  motor2.setPulse(13);
  Serial.begin(115200);
  motor1.setPWM(0);
  motor2.setPWM(0);
}

void loop(){
  runtime();
  Serial.println(n);
  n++;
}

void turns(){ //non si capisce cosa deve fare
  motor1.runTurns(2,50);
  delay(1000);
  motor1.setPWM(0);
  motor2.setPWM(0);
}

void angletunrs1()
//dovrebbe fare 360 gradi la ruota ma ne fa 330 circa
//accelerando e decelerando
{
  motor2.moveTo(360, 100);//ang deg e pwm
  motor1.moveTo(360, 100);
  delay(2000);
  motor2.moveTo(0, 50);
  motor1.moveTo(0, 50);
  delay(2000);
}

void angletunrs2()
{
  motor2.move(360, 100);
  motor1.move(360, 100);
  delay(2000);
  motor2.move(0, 100);
  motor1.move(0, 100);
  delay(2000);
}
void runtime()
{
  motor1.runSpeedAndTime(200,5);//pwm and seconds 
  delay(1000);  //il delay comanda il tempo
  motor1.runSpeedAndTime(0,5);
  delay(1000);
}
