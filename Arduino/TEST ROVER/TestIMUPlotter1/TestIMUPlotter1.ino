//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
MeGyro gyro;

float accX=0;
float accY=0;
float accZ=0;

float speX=0;
float speY=0;
float speZ=0;

float rotX=0;
float rotY=0;
float rotZ=0;

long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  gyro.begin();
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /*
   * Acceleration X: 0.35, Y: 1.41, Z: 10.54 m/s^2
    Rotation X: -0.02, Y: 0.01, Z: 0.01 rad/s
    Temperature: 22.44 degC

   */
/*
  accX = accX - 0.35;
  accY = accY - 1.41;
  accZ = accZ - 10.54;

  speX = speX + accX*0.5;
*/
start=millis();
  
  accX = accX + a.acceleration.x*100;
  accY = accY + a.acceleration.y*100;
  accZ = accZ + a.acceleration.z*100;

  speX = speX + accX*cycleTime/100000;
  speY = speY + accY*cycleTime/100000;
  speZ = speZ + accZ*cycleTime/100000;
  
 Serial.print(accY/100);
  Serial.print(' ');
  Serial.println(speY);
  /* Print out the values */
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
*/

  delay(100);
  accX =- a.acceleration.x*100;
  accY =- a.acceleration.y*100;
  accZ =- a.acceleration.z*100;
  
  cycleTime=millis()-start;
  //Serial.print("cycleTime: ");
 // Serial.println(cycleTime);
}
