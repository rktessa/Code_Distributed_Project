#include <MeMCore.h>
#include <Arduino.h>
//#include <Wire.h>
#include <SoftwareSerial.h>
//#include "MeOrion.h"
#include "myIMU.h"
#include <Wire.h>

#define CAL_NUM 500       //number of measures to calibrate the accx and accy
double accXrob, accYrob, accZrob;           //accelerations in the robot frame
double gyroXrob, gyroYrob, gyroZrob;           //accelerations in the robot frame
double accXOffset, accYOffset,accZOffset, angleOffset; // offset of the accelerations
double gyroXOffset, gyroYOffset,gyroZOffset;
double aSensitivity = 16384;               // gyro class return acc as LSB at +-2g: with this sensitivity factor get the value in g
double gSensitivity = 131;               // gyro class return acc as LSB at +-2g: with this sensitivity factor get the value in g

IMU imu;

//calibrate gyro
void eval_Acc() 
{   
    double xSum = 0, ySum = 0, zSum = 0, angSum = 0;
    for (int i = 0; i < CAL_NUM; i++) 
    {   imu.fast_update();
        xSum += imu.getAccX();
        ySum += imu.getAccY();
        zSum += imu.getAccZ();
        angSum += imu.getAngleX();   }
        
    accXOffset = xSum / CAL_NUM;
    accYOffset = ySum / CAL_NUM;
    accZOffset = zSum / CAL_NUM;
    angleOffset = angSum / CAL_NUM;   }

void eval_Gyro() 
{   
    double gxSum = 0, gySum = 0, gzSum = 0;
    for (int i = 0; i < CAL_NUM; i++) 
    {   imu.fast_update();
        gxSum += imu.getGyroX1();
        gySum += imu.getGyroY1();
        gzSum += imu.getGyroZ1();   }
        
    gyroXOffset = gxSum / CAL_NUM;
    gyroYOffset = gySum / CAL_NUM;
    gyroZOffset = gzSum / CAL_NUM;   }

    
void setup() {
  Serial.begin(115200);
  imu.begin();
  eval_Acc();             //calibrate sensor
  eval_Gyro();
  Serial.print("accX");
  Serial.print(",accY");
  Serial.print(",accZ");
  Serial.print(",accX_cal");
  Serial.print(",accY_cal");
  Serial.print(",accZ_cal");
  Serial.print(",gyroX");
  Serial.print(",gyroY");
  Serial.print(",gyroZ");
  Serial.print(",yroX_cal");
  Serial.print(",gyroX_cal");
  Serial.println(",gyroX_cal");
  Serial.print(gyroXOffset);
  Serial.print(",");
  Serial.print(gyroYOffset);
  Serial.print(",");
  Serial.println(gyroZOffset);

}

void loop() {
  imu.fast_update();
  Serial.read();
  //print acceleration
  Serial.print("accXOffset:");
  Serial.print(accXOffset);
  Serial.print("   accYOffset:");
  Serial.print(accYOffset);
  Serial.print("   accZOffset:");
  Serial.println(accZOffset);
  Serial.print("accX:");
  Serial.print(imu.getAccX() );
  Serial.print("   accY:");
  Serial.print(",");
  Serial.print(imu.getAccY() );
  Serial.print("   accZ:");
  Serial.print(",");
  Serial.print(imu.getAccZ() );
  Serial.print("accX1:");
  Serial.print(",");
  Serial.print(imu.getAccX1() );
  Serial.print("   accY1:");
  Serial.print(imu.getAccY1() );
  Serial.print("   accZ1:");
  Serial.println(imu.getAccZ1() );
  accXrob = (imu.getAccX() - accXOffset) / aSensitivity;
  accYrob = (imu.getAccY() - accYOffset) / aSensitivity;
  accZrob = (imu.getAccZ() - accZOffset) / aSensitivity; 
  Serial.print("accX_cal:");
  Serial.print(accXrob);
  Serial.print("   accY_cal:");
  Serial.print(",");
  Serial.print(accYrob);
  Serial.print("   accZ_cal:");
  Serial.print(",");
  Serial.println(accZrob);
    Serial.print(gyroXOffset);
  Serial.print(",");
  Serial.print(gyroYOffset);
  Serial.print(",");
  Serial.println(gyroZOffset);
  Serial.print("gyroX:");
  Serial.print(",");
  Serial.print(imu.getGyroX1());
  Serial.print("   gyroY:");
  Serial.print(",");
  Serial.print(imu.getGyroY1());
  Serial.print("   gyroZ:");
  Serial.print(",");
  Serial.println(imu.getGyroZ1());
  gyroXrob = (imu.getGyroX1() - gyroXOffset);
  gyroYrob = (imu.getGyroY1() - gyroYOffset);
  gyroZrob = (imu.getGyroZ1() - gyroZOffset);
  Serial.print(gyroXrob);
  Serial.print("   gyroY_cal:");
  Serial.print(",");
  Serial.print(gyroYrob);
  Serial.print("   gyroZ_cal:");
  Serial.print(",");
  Serial.println(gyroZrob);
  Serial.print("gyroX_cal:");
  Serial.print(",");
  Serial.print(imu.getGyroX());
  Serial.print("   gyroY_cal:");
  Serial.print(",");
  Serial.print(imu.getGyroY());
  Serial.print("   gyroZ_cal:");
  Serial.print(",");
  Serial.println(imu.getGyroZ());
  Serial.println("-------------------------------");
  delay(1000);
}
