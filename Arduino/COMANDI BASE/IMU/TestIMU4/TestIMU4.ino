//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include "MeOrion.h"
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
MeGyro gyro;
MeRGBLed rgbled_7(7, 2);

const int sampTime = 20;

long start=0;
float cycleTime=0;
int n = 0;

const int dim = 10;

float AaccX[dim]; //array accX
float AaccY[dim]; //array accX
float AaccZ[dim]; //array accX

float accError=1;
float speError=1;

float accXError=0;
float accYError=0;
float accZError=0;

float rotXError=0;
float rotYError=0;
float rotZError=0;

float angXError=0;
float angYError=0;
float angZError=0;

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

float angX=0;//positivo su fianco destro
float angY=0;//positivo in discesa
float angZ=0;//positivo giro a sinistra

void(* resetFunc) (void) = 0;

void setup(void) {
  Serial.begin(115200);
    rgbled_7.fillPixelsBak(0, 2, 1);
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
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  gyro.begin();

  pinMode(A7, INPUT);
  
  for(int i=0;i<dim;i++){
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
   }

   while(analogRead(A7) > 10){
    delay(10);
    
    rgbled_7.setColor(1,255,0,0);//destro
    rgbled_7.setColor(2,255,0,0);//sinistro
    rgbled_7.show();
   }

   calibrationStop();

   rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

}

void loop() {

  start=millis();

    rgbled_7.setColor(1,0,0,0);//destro
rgbled_7.setColor(2,0,0,0);//sinistro
rgbled_7.show();

  /* Get new sensor events with the readings */
  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angX = gyro.getAngleX() - angXError;
  angY = gyro.getAngleY() - angYError;
  angZ = gyro.getAngleZ() - angZError;

  AaccX[0] = a.acceleration.x - accXError;
  AaccY[0] = a.acceleration.y - accYError;
  AaccZ[0] = a.acceleration.z - accZError;

  AaccX[0] *= accError;
  AaccY[0] *= accError;
  AaccZ[0] *= accError;

  accX = AaccX[dim-1]/(dim-1) - AaccX[0];
  accY = AaccY[dim-1]/(dim-1) - AaccY[0];
  accZ = AaccZ[dim-1]/(dim-1) - AaccZ[0];

  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  if(n>20){
  speX += accX*cycleTime/1000;
  speY += accY*cycleTime/1000;
  speZ += accZ*cycleTime/1000;

  speX *= speError;
  speY *= speError;
  speZ *= speError;
  
  speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));

  }
  
  if(analogRead(A7) < 10){
    resetFunc();
  }
 
  Serial.print("Acceleration X: ");
  Serial.print(accX);
  Serial.print(", Y: ");
  Serial.print(accY);
  Serial.print(", Z: ");
  Serial.print(accZ);
  Serial.print(", T: ");
  Serial.print(accT);
  Serial.println(" m/s^2");

  Serial.print("Speed X: ");
  Serial.print(speX);
  Serial.print(", Y: ");
  Serial.print(speY);
  Serial.print(", Z: ");
  Serial.print(speZ);
  Serial.print(", T: ");
  Serial.print(speT);
  Serial.println(" m/s");

  Serial.print("Rotation X: ");
  Serial.print(rotX);
  Serial.print(", Y: ");
  Serial.print(rotY);
  Serial.print(", Z: ");
  Serial.print(rotZ);
  Serial.println(" rad/s");

  Serial.print("Angle X: ");
  Serial.print(angX);
  Serial.print(" Y: ");
  Serial.print(angY);
  Serial.print(" Z: ");
  Serial.print(angZ);
  Serial.println(" deg");
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(sampTime);

  n++;

  cycleTime=millis()-start;
}

//////////////////////////////////////////////////////////
void calibrationStop(){

  rgbled_7.setColor(1,0,0,0);//destro
rgbled_7.setColor(2,0,0,0);//sinistro
rgbled_7.show();

delay(1500);

rgbled_7.setColor(1,0,255,0);//destro
rgbled_7.setColor(2,0,255,0);//sinistro
rgbled_7.show();


delay(1500);

int num=200;
  for(int i=0;i<num;i++){

  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accXError += a.acceleration.x;
  accYError += a.acceleration.y;
  accZError += a.acceleration.z;

  rotXError += g.gyro.x;
  rotYError += g.gyro.y;
  rotZError += g.gyro.z;

  angXError += gyro.getAngleX();
  angYError += gyro.getAngleY();
  angZError += gyro.getAngleZ();

  
   delay(sampTime); 

  }
  accXError/=num;
  accYError/=num;
  accZError/=num;

  rotXError/=num;
  rotYError/=num;
  rotZError/=num;

  angXError/=num;
  angYError/=num;
  angZError/=num;

  accT = sqrt(pow(accXError,2)+pow(accYError,2)+pow(accZError,2));
  accError = 2-accT/9.81;
  
  Serial.print("ACC ");
  Serial.print(accXError);
  Serial.print(' ');
  Serial.print(accYError);
  Serial.print(' ');
  Serial.print(accZError);
  Serial.print(' ');
  Serial.println(accT);
  Serial.print("accError ");
  Serial.println(accError);
  Serial.print("rotError X: ");
  Serial.print(rotXError);
  Serial.print(" Y: ");
  Serial.print(rotYError);
  Serial.print(" Z: ");
  Serial.println(rotZError);
  Serial.print("angError X: ");
  Serial.print(angXError);
  Serial.print(" Y: ");
  Serial.print(angYError);
  Serial.print(" Z: ");
  Serial.println(angZError);
  


}
