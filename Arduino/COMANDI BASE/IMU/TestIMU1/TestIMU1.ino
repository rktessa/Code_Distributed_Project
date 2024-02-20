//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
MeGyro gyro;

const int sampTime = 10;

float accError=1;
float speError=1;


float accX=0;
float accY=0;
float accZ=0;
float accT=0;

float speX=0;
float speY=0;
float speZ=0;
float speT=0;

float rotX=0;
float rotY=0;
float rotZ=0;

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
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  gyro.begin();
  
  delay(100);
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
  /* Print out the values */

  gyro.update();
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

  Serial.print("Angle X: ");
  Serial.print(gyro.getAngleX());
  Serial.print(" Y: ");
  Serial.print(gyro.getAngleY());
  Serial.print(" Z: ");
  Serial.println(gyro.getAngleZ());

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(100);
}

//////////////////////////////////////////////////////////
void calibrationStop(){

  sensors_event_t a, g, temp;

int num=500;
  for(int i=0;i<num;i++){

  mpu.getEvent(&a, &g, &temp);

  gyro.update();
  accX += a.acceleration.x;
  accY += a.acceleration.y;
  accZ += a.acceleration.z;

   delay(sampTime); 

  }
  accX/=num;
  accY/=num;
  accZ/=num;

  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));
  accError = 2-accT/9.81;

  Serial.print("ACC ");
  Serial.print(accX);
  Serial.print(' ');
  Serial.print(accY);
  Serial.print(' ');
  Serial.print(accZ);
  Serial.print(' ');
  Serial.println(accT);
  Serial.print("accError ");
  Serial.println(accError);


}
