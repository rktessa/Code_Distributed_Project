//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include "MeOrion.h"
//#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//Adafruit_MPU6050 mpu;
//MeGyro gyro;
//MeRGBLed rgbled_7(7, 2);
//MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
//MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI



/// CODE FROM DATASHEET


double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
//MeEncoderOnBoard Encoder_1(SLOT1);
//MeEncoderOnBoard Encoder_2(SLOT2);

MeEncoderNew Encoder_1(0x09, SLOT1);   //  Motor at slot1
MeEncoderNew Encoder_2(0x09, SLOT2);   //  motor at slot2

void isr_process_encoder1(void)
{
    if(digitalRead(Encoder_1.getPortB()) == 0){
        Encoder_1.pulsePosMinus();
    }else{
        Encoder_1.pulsePosPlus();
    }
}

void isr_process_encoder2(void)
{
    if(digitalRead(Encoder_2.getPortB()) == 0){
        Encoder_2.pulsePosMinus();
    }else{
        Encoder_2.pulsePosPlus();
    }
}

void setup(){
    TCCR1A = _BV(WGM10);//PIN12
    TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
    TCCR2B = _BV(CS22);
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
}

void loop(){
    Encoder_1.setTarPWM(100);
    Encoder_2.setTarPWM(100);
    delay(1);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    delay(1);
    loop();
}
