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

MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

const int sampTime = 50;

long start=0;
float cycleTime=0;
int n = 0;

float filt = 0;
const int dim = 10;

float AaccX[dim]; //array accX
float AaccY[dim]; //array accX
float AaccZ[dim]; //array accX

float accXA=0;
float accYA=0;
float accZA=0;
float accTA=0;

float accXl=0;
float accYl=0;
float accZl=0;

float accX=0;
float accY=0;
float accZ=0;
float accT=0;

float speXA=0;
float speYA=0;
float speZA=0;
float speTA=0;

float speX=0;
float speY=0;
float speZ=0;
float speT=0;

float rotX=0;
float rotY=0;
float rotZ=0;

float angXdeg=0;
float angYdeg=0;
float angZdeg=0;

float angXrad=0;
float angYrad=0;
float angZrad=0;

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
/*
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
*/
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

  //motor1.run(-120);
  //motor2.run(120);

   rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

}

void loop() {
 filt = 0.2;
 
  start=millis();
if(n>200){
    motor1.stop();
  motor2.stop();
}
    rgbled_7.setColor(1,0,0,0);//destro
rgbled_7.setColor(2,0,0,0);//sinistro
rgbled_7.show();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angXdeg = gyro.getAngleX() - angXError;
  angYdeg = gyro.getAngleY() - angYError;
  angZdeg = gyro.getAngleZ() - angZError;

  accXl = a.acceleration.x - accXError;
  accYl = a.acceleration.y - accYError;
  accZl = a.acceleration.z - accZError;

  accX = accXl*filt + accX*(1-filt);
  accY = accYl*filt + accY*(1-filt);
  accZ = accZl*filt + accZ*(1-filt);

  accX *= accError;
  accY *= accError;
  accZ *= accError;
  
  AaccX[0] = a.acceleration.x - accXError;
  AaccY[0] = a.acceleration.y - accYError;
  AaccZ[0] = a.acceleration.z - accZError;

  AaccX[0] *= accError;
  AaccY[0] *= accError;
  AaccZ[0] *= accError;
  
  accXA = AaccX[dim-1]/(dim-1) - AaccX[0];
  accYA = AaccY[dim-1]/(dim-1) - AaccY[0];
  accZA = AaccZ[dim-1]/(dim-1) - AaccZ[0];

  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

  accTA = sqrt(pow(accXA,2)+pow(accYA,2)+pow(accZA,2));

  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  if(n>20){
  speXA += accXA*cycleTime/1000;
  speYA += accYA*cycleTime/1000;
  speZA += accZA*cycleTime/1000;

  speX += accX*cycleTime/1000;
  speY += accY*cycleTime/1000;
  speZ += accZ*cycleTime/1000;

  speXA *= speError;
  speYA *= speError;
  speZA *= speError;

  speX *= speError;
  speY *= speError;
  speZ *= speError;
  
  speTA = sqrt(pow(speXA,2)+pow(speYA,2)+pow(speZA,2));

  speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));

  }
  
  angXrad += rotX*cycleTime/1000;
  angYrad += rotY*cycleTime/1000;
  angZrad += rotZ*cycleTime/1000;

  if(analogRead(A7) < 10){
    resetFunc();
  }
  gyro.update();
  Serial.print("Accel_array X: ");
  Serial.print(accXA);
  Serial.print(", Y: ");
  Serial.print(accYA);
  Serial.print(", Z: ");
  Serial.print(accZA);
  Serial.print(", T: ");
  Serial.print(accTA);
  Serial.println(" m/s^2");

  Serial.print("Accel_stand X: ");
  Serial.print(accX);
  Serial.print(", Y: ");
  Serial.print(accY);
  Serial.print(", Z: ");
  Serial.print(accZ);
  Serial.print(", T: ");
  Serial.print(accT);
  Serial.println(" m/s^2");

  Serial.print("Speed_array X: ");
  Serial.print(speXA);
  Serial.print(", Y: ");
  Serial.print(speYA);
  Serial.print(", Z: ");
  Serial.print(speZA);
  Serial.print(", T: ");
  Serial.print(speTA);
  Serial.println(" m/s");

  Serial.print("Speed_stand X: ");
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

  Serial.print("Angle_func X: ");
  Serial.print(angXdeg*3.1415/180);
  Serial.print(" Y: ");
  Serial.print(angYdeg*3.1415/180);
  Serial.print(" Z: ");
  Serial.print(angZdeg*3.1415/180);
  Serial.println(" rad");
  
  Serial.print("Angle_calc X: ");
  Serial.print(angXrad);
  Serial.print(" Y: ");
  Serial.print(angYrad);
  Serial.print(" Z: ");
  Serial.print(angZrad);
  Serial.println(" rad");
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.print("cycleTime: ");
  Serial.print(cycleTime);

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
  sensors_event_t a, g, temp;

int num=500;
  for(int i=0;i<num;i++){

  mpu.getEvent(&a, &g, &temp);

  gyro.update();
  accXError += a.acceleration.x;
  accYError += a.acceleration.y;
  accZError += a.acceleration.z;

  rotXError += g.gyro.x;
  rotYError += g.gyro.y;
  rotZError += g.gyro.z;

  angXError += gyro.getAngleX();
  angYError += gyro.getAngleY();
  angZError += gyro.getAngleZ();
  
   delay(sampTime/10); 

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
  Serial.print(accX);
  Serial.print(' ');
  Serial.print(accY);
  Serial.print(' ');
  Serial.print(accZ);
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
