//https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include "MeOrion.h"
//#include <MeMCore.h>
#include "MeOrion.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
MeGyro gyro;
MeRGBLed rgbled_7(7, 2);
MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

const int sampTime = 10;

long start=0;
float cycleTime=0;
int n = 0;

const int dim = 20;

float AaccX[dim]; //array accX
float AaccY[dim]; //array accX
float AaccZ[dim]; //array accX

float accError=1;
float speError=1;
float d = 0;

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
    //SoftwareSerial mySerial(0, 1); // RX, TX
  rgbled_7.fillPixelsBak(0, 2, 1);
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

   // Try to initialize!
   mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
   // Accelerometer range set to:+-8G(+-2G,+-4G,+-16G)
   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
   //Gyro range set to: +- 500 deg/s (+- 250 deg/s, +- 1000 deg/s, +- 2000 deg/s) 
   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
   //Filter bandwidth set to: 21 Hz (260 Hz, 184 Hz, 94 Hz, 44 Hz, 10 Hz, 5 Hz)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  gyro.begin();
  
  pinMode(A7, INPUT);
  motor1.stop();
  motor2.stop();

  delay(500);
  
  for(int i=0;i<dim;i++){
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
   }

   while(analogRead(A7) > 10){
    if(n%2==0){
    rgbled_7.setColor(1,255,0,0);//destro
    rgbled_7.setColor(2,0,0,0);//sinistro
    rgbled_7.show();
    }else{
    rgbled_7.setColor(2,255,0,0);
    rgbled_7.setColor(1,0,0,0);
    rgbled_7.show();
    }
    n++;
    delay(100);
    }
n=0;

   calibrationStop();

   rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

}

void loop() {

  start=millis();

    motor1.run(-120);
    motor2.run(120);
    
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
  
/*
  d = 0;
  for(int i=1;i<(dim-2);i++){
    d += pow((AaccX[i]/i - AaccX[dim-1]/(dim-1)),2);
  }
  d += pow((AaccX[0] - AaccX[dim-1]/(dim-1)),2);
  d = sqrt(d/(dim-2));//  /AaccY[dim-1]/(dim-1);

  //d = 2.58*d/sqrt((dim-2));
  /*
  Serial.print("DEV: ");
  Serial.println(d);
  */
  /*
  AaccY[0] = 0;
  
  for(int i=0;i<10;i++){
    sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  AaccX[0] = (a.acceleration.x - accXError) - accError*sin(angX*3.1415/180);
  AaccY[0] += (a.acceleration.y - accYError) - 9.81*sin(angY*3.1415/180);
  AaccZ[0] = (a.acceleration.z - accZError) - accError*cos(angX*3.1415/180)*cos(angY*3.1415/180);
  
 }

  AaccY[0]/=10;
  */
  
  AaccX[0] = (a.acceleration.x - accXError) - accError*sin(angX*3.1415/180);
  AaccY[0] = (a.acceleration.y - accYError) - accError*sin(angY*3.1415/180);
  AaccZ[0] = (a.acceleration.z - accZError) - accError*cos(angX*3.1415/180)*cos(angY*3.1415/180);
 
  accX = AaccY[0] - AaccY[1] - AaccY[dim-1]/(dim-1);
  accY = AaccY[0]-AaccY[3]/3;//AaccY[0] - AaccY[1];// - AaccY[dim-1]/(dim-1);
  accZ = AaccY[0] - AaccY[dim-1]/(dim-1);// - AaccZ[dim-1]/(dim-1);

  //0.1*(AaccY[0]-AaccY[1])+0.9*(AaccY[dim-2]/(dim-2)-AaccY[dim-1]/(dim-1))
//*/
  //accX *= (1-d);
  //accY *= (1-d);
  //accZ *= (1-d);
  
  
  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

/*
  for(int i=1;i<(dim-1);i++){
  accX += (AaccX[i]/(i)-AaccX[i+1]/(i+1));//-AaccX[1];
  accY += (AaccY[i]/(i)-AaccY[i+1]/(i+1));//(i)* /(dim-2) /(dim-i)
  accZ += (AaccZ[i]/(i)-AaccZ[i+1]/(i+1));//-AaccZ[1];
  }

  accX/=(dim-2);
  accY/=(dim-2);
  accZ/=(dim-2);
*/

  
  
  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  speX += accX*cycleTime/10;
  speY += accY*cycleTime/10;
  speZ += accZ*cycleTime/10;

  speT = sqrt(pow(speX,2)+pow(speY,2));//+pow(speZ,2));

  //Serial.print(AaccY[0]*10);
  //Serial.print(' ');
  //Serial.print(AaccY[dim-1]*10/(dim-1));
  //Serial.print(' ');
  Serial.print(speY);
  Serial.print(' ');
  Serial.println(accY);


/*
  Serial.print(speY);
  Serial.print(' ');
  Serial.print(AaccY[dim-1]/(dim-1));
  Serial.print(' ');
  Serial.println(accY);
//*/
     
  


  
       
 if(analogRead(A7) < 10){
    motor1.stop();
  motor2.stop();
  delay(100);
    resetFunc();
  }
  
 /*
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

  */
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

  accError = sqrt(pow(accXError,2)+pow(accYError,2)+pow(accZError,2));
  //accError = 9.81/accT;
  /*
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
  */


}
