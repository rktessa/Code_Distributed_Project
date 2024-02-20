
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//#include "MeOrion.h"
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

MeUltrasonicSensor ultraSensor(PORT_3);
MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;
//MeBluetooth bluetooth(PORT_2);
//Me7SegmentDisplay disp(PORT_2);

Adafruit_MPU6050 mpu;
MeGyro gyro;

void(* resetFunc) (void) = 0; //FUNZIONE RESET LOOP

const int resamotori = 90;
float compsx = 100*(200-resamotori)/100;
float compdx = 98*(200-resamotori)/100;
float power = 60.0/100; // 50 % potenza
const int maxSpeed = 255;
uint8_t motorSpeed = maxSpeed*power; /* value: between -255 and 255. */

//float travel=100; //mm percorsi

//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
// 7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%

 
const float steeringTime = 30000.0/(280*power); //tempo di rotazione a 30 gradi
const float backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
const float stopTime = 400*power; //TEMPO TRA STOP E RIAVVIO MOTORI
//int exitTime = 300*1000.0/(362*power); // USCITA INDIETRO DI 300 mm 2200
const int sampTime = 10; //tempo in millis tra una lettura e l'altra


float start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento


float distStop =60*power;
float velocity = 36*power;
float accSensitivity = 0.23;//0.16
float speSensitivity = 0.08;//0.12
//float limitaccSensit = 0.16;
int collision=0;
float speStop=0; 

int nStop=0;
int n = 0;
int s = 0;
int movement=0;

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

float angX=0;
float angY=0;
float angZ=0;


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

  if(motorSpeed*compdx/100>maxSpeed||motorSpeed*compsx/100>maxSpeed){//check per non andare oltre 255
    motorSpeed=maxSpeed*100/max(compdx,compsx);
  }

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

//UTILIZZARE ALIMENTAZIONE DA COMPUTER CON CAVO PER ESEGUIRE
calibrationStop();
//calibrationMove();


rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

}

void loop() {
    motion_loop();

   if(analogRead(A7) < 10){
     motor1.stop();
     motor2.stop();
    resetFunc();
  }
}

void motion_loop(){
  
  start=millis();

  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angX = gyro.getAngleX() - angXError;
  angY = gyro.getAngleY() - angYError;
  angZ = gyro.getAngleZ() - angZError;

  AaccX[0] = a.acceleration.x - accXError - accError*sin(angX*3.1415/180);
  AaccY[0] = a.acceleration.y - accYError - accError*sin(angY*3.1415/180);
  AaccZ[0] = a.acceleration.z - accZError - accError*cos(angX*3.1415/180)*cos(angY*3.1415/180);

  AaccX[0] *= accError;
  AaccY[0] *= accError;
  AaccZ[0] *= accError;

  accX = AaccX[0] - AaccX[dim-1]/(dim-1);
  accY = AaccY[0] - AaccY[dim-1]/(dim-1);
  accZ = AaccZ[0] - AaccZ[dim-1]/(dim-1);

  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

 
  speX += accX*cycleTime/10;
  speY += accY*cycleTime/10;
  speZ += accZ*cycleTime/10;
/*
  speX *= speError;
  speY *= speError;
  speZ *= speError;
  */
  speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));

    if(sqrt((speStop-speT)*(speStop-speT))<speSensitivity*velocity){
      nStop++;
      }else{
      speStop=speT;
      nStop=0;
    }

  
/*
  if(speT>velocity){
      //Serial.println(speT);
      speError = 2 - speT/velocity;
    }
*/

if(collision==0){

    if(sqrt(angY*angY)>30){
    collision=1;
      n=0;
      buzzer.tone(500, 0.1 * 1000);
  }
    if(accT<accSensitivity*power*resamotori||nStop>800/sampTime){//accSensitivity*power*resamotori

      if(nStop>800/sampTime){
        collision=1;
        n=0;
         buzzer.tone(950, 0.1 * 1000);
         buzzer.tone(700, 0.1 * 1000);
      }
    }else{
      /*
      if(speY>0){
        collision=1;
      }else{
        collision =-1;
      }
      */
      collision=1;
      n=0;
      buzzer.tone(700, 0.1 * 1000);
    }
  }

    
if(collision==-1){
   if(n*cycleTime>backTime){
      movement =2;
      collision=0;
      n=0;
      }else{
      movement =1;
    }
  }


if(collision==1){
  if(n*cycleTime>backTime){//backward
    if((n*cycleTime-backTime)>stopTime){//stop
        if((n*cycleTime-backTime-stopTime)>steeringTime){//steering
            collision=0;
            n=0;
        }else{
          movement =2;
          nStop=0;
        }
     }else{
       movement=0;   
     }
  }else{
     movement =3;
  }
}else{ //collision==0   
  if(ultraSensor.distanceCm() <= distStop){
    if(n*cycleTime>stopTime){
    movement =2;
    }else{
      movement =0;
    }
  }else{
    movement =1;
    n=0;
  }
}


  switch (movement) {
    case 0:    //Serial.println("stop");
      motor1.stop();
      motor2.stop();
      /*
      speX=0;
      speY=0;
      speZ=0;
      */
      break;
    case 1:   // Serial.println("forward");
      motor1.run(-motorSpeed*compsx/100);
      motor2.run(motorSpeed*compdx/100);
      speY=abs(speY);
      distStop =60*power;
      break;
    case 2:   // Serial.println("right");
      motor1.run(-motorSpeed*0.6*compsx/100);
      motor2.run(-motorSpeed*0.6*compdx/100);
      //speX=abs(speX);
      distStop = 120*power;
      break;
    case 3:    // Serial.println("backward");
      motor1.run(motorSpeed*compsx/100);
      motor2.run(-motorSpeed*compdx/100);
      speY=-abs(speY);
      distStop =60*power;
      break;
  }
  
  Serial.print(speY);
  Serial.print(' ');
  Serial.println(accY);
  /*
  
// ##########################################
  // PARTE PER LA STAMPA
  Serial.print("Inizio");
  Serial.print(",");
  Serial.print(start/1000);
  Serial.print(",");
  Serial.print(accX);
  Serial.print(",");
  Serial.print(accY);
  Serial.print(",");
  Serial.print(accZ);
  Serial.print(",");
  Serial.print(speX);
  Serial.print(",");
  Serial.print(speY);
  Serial.print(",");
  Serial.print(speZ);
  Serial.print(",");
  Serial.print(rotX);
  Serial.print(",");
  Serial.print(rotY);
  Serial.print(",");
  Serial.print(rotZ);
  Serial.print(",");
  Serial.print(angX);
  Serial.print(",");
  Serial.print(angY);
  Serial.print(",");
  Serial.print(angZ);
  Serial.print(",");
  Serial.print(accSensitivity);
  Serial.print(",");
  Serial.print(speSensitivity);
  Serial.print(",");
  Serial.println("Fine");
  */
  delay(sampTime);       // delay in between reads for stability

  n++;
  s++;
  cycleTime=millis()-start;
  //Serial.println("cycle ");
  //Serial.println(cycleTime);
}

void calibrationStop(){

sensors_event_t a, g, temp;

rgbled_7.setColor(1,0,0,0);//destro
rgbled_7.setColor(2,0,0,0);//sinistro

delay(1500);

rgbled_7.setColor(1,0,255,0);//destro
rgbled_7.setColor(2,0,255,0);//sinistro
rgbled_7.show();

int num=500;
  for(int i=0;i<num;i++){

  gyro.update();
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
/*
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
*/

}


void calibrationMove(){

 accSensitivity=0.05;
 speSensitivity=0.2;
 speError =0;

 rgbled_7.setColor(1,0,0,0);//destro
rgbled_7.setColor(2,0,0,0);//sinistro
rgbled_7.show();

 delay(500);

 rgbled_7.setColor(1,0,0,60);//destro
rgbled_7.setColor(2,0,0,60);//sinistro
rgbled_7.show();
           
while(n<250){
  start=millis();
  n++;

  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(ultraSensor.distanceCm() <= distStop){
     motor1.stop();
     motor2.stop();
     break;
  }

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angX = gyro.getAngleX() - angXError;
  angY = gyro.getAngleY() - angYError;
  angZ = gyro.getAngleZ() - angZError;
  
  AaccX[0] = a.acceleration.x - accXError - accError*sin(angX*3.1415/180);
  AaccY[0] = a.acceleration.y - accYError - accError*sin(angY*3.1415/180);
  AaccZ[0] = a.acceleration.z - accZError - accError*cos(angX*3.1415/180)*cos(angY*3.1415/180);

  AaccX[0] *= accError;
  AaccY[0] *= accError;
  AaccZ[0] *= accError;

  accX = AaccX[0] - AaccX[dim-1]/(dim-1);
  accY = AaccY[0] - AaccY[dim-1]/(dim-1);
  accZ = AaccZ[0] - AaccZ[dim-1]/(dim-1);

  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];
  
  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  if(n<15||n>240){
    motor1.stop();
    motor2.stop();
  }else{
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(motorSpeed*compdx/100);

/*
if(n<15||n>240){
    motor1.stop();
    motor2.stop();
  }else{
    if(n<180||n>210){
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(motorSpeed*compdx/100);
    }else{
    motor1.run(motorSpeed*compsx/100);
    motor2.run(-motorSpeed*compdx/100);
    }
    */
    if(n>30){
    speX += accX*cycleTime/10;
    speY += accY*cycleTime/10;
    speZ += accZ*cycleTime/10;
    
    
    speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));
    
    speError += speT;

    if(sqrt((speStop-speT)*(speStop-speT))<speSensitivity*velocity){
      nStop++;
    }else{
      speStop=speT;
      nStop=0;
    }
    }
/*
      if(angZ>0){
        compsx /= cos(angZ*3.1415/180);
      }else{
        compdx /= cos(angZ*3.1415/180);
      }
      */
  }


  if(n>9&&n<245){
    
     if(accT>accSensitivity*power*resamotori&&accT<0.16*power*resamotori){
       accSensitivity = accT/(power*resamotori);
    }
/*
    if(accT>20){
      motor1.stop();
      motor2.stop();
      Serial.print("TEST NON VALIDO, RIESEGUIRE!!  ");
      Serial.println(accT);
      delay(200);
      resetFunc();
    }
*/
    if(nStop>100/sampTime){
          speSensitivity = sqrt((speStop-speT)*(speStop-speT))/velocity;
      nStop=0;
    }
  }

  delay(sampTime);

   cycleTime=millis()-start;
}

  speError /= (n-40);
  speError = 2 - speError/velocity;
/*
  if(speSensitivity<0.06){
    speSensitivity = 0.06;
  }
*/
  if(accSensitivity>0.16){
    accSensitivity += 0.05;
  }else{
  accSensitivity *=1.6;
  }

  speSensitivity *=1.4;
  /*
  Serial.print("accSensitivity: ");
  Serial.println(accSensitivity);
  Serial.print("speSensitivity: ");
  Serial.println(speSensitivity);
  Serial.print("speError ");
  Serial.println(speError);
  Serial.print("cycleTime ");
  Serial.println(cycleTime);
  Serial.print("Time travel ");
  Serial.println(cycleTime*225);
*/
n=0;
nStop=0;


motor1.stop();
motor2.stop();
delay(1500);

  for(int i=0;i<dim;i++){
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
   }

speX=0;
speY=0;
speZ=0;
accX=0;
accY=0;
accZ=0;

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();
}
