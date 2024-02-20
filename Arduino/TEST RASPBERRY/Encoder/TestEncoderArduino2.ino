//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
// http://docs.makeblock.com/diy-platform/en/electronic-modules/main-control-boards/makeblock-orion.html
// http://docs.makeblock.com/diy-platform/en/electronic-modules/adapters/me-rj25-adapter.html
// https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide

// QUESTA VERSIONE VA:
// Inserendo RJ25 Adapter in porta 4 solo, e l'interrupt digital del pin 0
// Articolo interessante anche per come si implementa una lettura dell'encoder https://www.embeddedrelated.com/showarticle/158.php


#include "MeOrion.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>

// Definizione porta con Libreria per il modulo "ME RJ25 Adapter"
//MePort port(PORT_4);
// l'altro alla porta seriale 2 con doppino modificato

// Connessione al motore in DC
MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI


/////////CODE FROM ARDUINO EXAMPLES
// Loro assumono di usare il TB6612FNG che è lo stesso modulo che abbiamo 
// a bordo della Orion
#define ENCODER_A_PIN  3 
#define ENCODER_B_PIN  9

#define ENCODER_A2_PIN  2 
#define ENCODER_B2_PIN  8


long pulse_number=0;   
int rpm;

long pulse_number2=0;   
int rpm2;
 

void setup()
{
  MsTimer2::set(500, send);     
  MsTimer2::start();  

  

  pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT); 
    attachInterrupt(1, read_quadrature, FALLING);  

   pinMode(ENCODER_A2_PIN, INPUT);
    pinMode(ENCODER_B2_PIN, INPUT); 
    attachInterrupt(0, read_quadrature2, FALLING);  
    
    
    Serial.begin(115200);    
    
}

void loop()
{
   
    for (int a=80;a<=200;a++)
     {
       

        motor1.run(a);
        motor2.run(a);
        
        
        delay(200);

        
     }
    motor1.stop();
    motor2.stop();
    delay(1000);
}



void send()    
{
     rpm=int(pulse_number/5.3);
     Serial.print("rpm: ");
     Serial.print(rpm, DEC);
     pulse_number = 0;
    rpm2=int(pulse_number2/5.3);
     Serial.print(" rpm2: ");
     Serial.println(rpm2, DEC);
     pulse_number2 = 0;
}




void read_quadrature()    
{
  //Serial.print("rpm: PIPPO\r\n ");
  if (digitalRead(ENCODER_A_PIN) == LOW)
  {
    if (digitalRead(ENCODER_B_PIN) == LOW)
    { pulse_number --; }
    if (digitalRead(ENCODER_B_PIN) == HIGH)   
    { pulse_number ++; }
  }
  //return pulse_number;
}


void read_quadrature2()    
{
  //Serial.print("rpm: PIPPO\r\n ");
  if (digitalRead(ENCODER_A2_PIN) == LOW)
  {
    if (digitalRead(ENCODER_B2_PIN) == LOW)
    { pulse_number2 --; }
    if (digitalRead(ENCODER_B2_PIN) == HIGH)   
    { pulse_number2 ++; }
  }
  //return pulse_number;
}
