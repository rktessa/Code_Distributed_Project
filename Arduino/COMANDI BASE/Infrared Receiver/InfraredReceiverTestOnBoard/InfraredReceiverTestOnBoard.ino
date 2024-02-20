/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    InfraredReceiverTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/01
 * @brief   Description: this file is sample code for Me Infrared Receiver device.
 *
 * Function List:
 * 1. void MeInfraredReceiver::begin(void)
 * 2. int16_t MeInfraredReceiver::read(void)
 * 3. int16_t MeInfraredReceiver::available(void)
 * 4. bool MeInfraredReceiver::buttonState(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/01    1.0.0          rebuild the old lib
 * </pre>
 */
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeIR ir;
MeRGBLed rgbled_7(7, 2);

/*  CODICE TASTI
 *  A   69
 *  B   70
 *  C   71
 *  D   68
 *  E   67
 *  F   13
 *  su  64
 *  giu 25
 *  sx  7
 *  dx  9
 *  imp 21
 *  0   22
 *  1   12
 *  2   24
 *  3   94
 *  4   8
 *  5   28
 *  6   90
 *  7   66
 *  8   82
 *  9   74
 */


void setup()
{
  ir.begin();
  Serial.begin(115200);
  Serial.println("InfraredReceiverDecode Start!");

  rgbled_7.fillPixelsBak(0, 2, 1);
  rgbled_7.setColor(0,0,0,0);
          rgbled_7.show();
}

void loop()
{
  if(ir.keyPressed(69)){//tasto A
          rgbled_7.setColor(0,255,0,0);
          rgbled_7.show();
      }

  if(ir.keyPressed(70)){//tasto B
          rgbled_7.setColor(0,0,0,0);
          rgbled_7.show();
     }
}
