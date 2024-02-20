// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 98*(200-resamotori)/100;
float power = 70.0/100; // 50 % potenza
uint8_t motorSpeed = 255*power; /* value: between -255 and 255. */
float travel=100; //mm percorsi

//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
// 7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%
MeUltrasonicSensor ultraSensor(PORT_3);

int steeringAngle = 30; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle*1000.0/(280*power); //DEVE ESSERE INTERO E PARI
int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 400*power; //TEMPO TRA STOP E RIAVVIO MOTORI
int exitTime = 300*1000.0/(362*power); // USCITA INDIETRO DI 300 mm 2200
float sampTime = 50.00; //tempo in millis tra una lettura e l'altra
long startSteer =0;
float steBlockTime =0;
float obstacleTime = 4000/power;

long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento

int obstacle =1;
int nObstacle=5;
int resto = 0;
int rnd = 1;

float distStop =60*power;
float velocity = 0.36*power;
float accSensitivity = 0.16;//0.16
float limitaccSensit = 0.1;
float speStop=0; 

int nStop=0;
int nSteer =1;
int n = 0;


Adafruit_MPU6050 mpu;
MeGyro gyro;


float accX=0;
float accY=0;
float accZ=0;
float accT=0;

float accX1=0;
float accY1=0;
float accZ1=0;

float accX2=0;
float accY2=0;
float accZ2=0;

float accX3=0;
float accY3=0;
float accZ3=0;

float speX=0;
float speY=0;
float speZ=0;
float speT=0;

float rotX=0;
float rotY=0;
float rotZ=0;

void setup(void) {
  rgbled_7.fillPixelsBak(0, 2, 1);
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

   // Try to initialize!
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

  while(analogRead(A7) > 10){
    rgbled_7.setColor(1,255,0,0);//destro
    rgbled_7.setColor(2,0,0,0);//sinistro
    rgbled_7.show();
    delay(100);
    rgbled_7.setColor(2,255,0,0);
    rgbled_7.setColor(1,0,0,0);
    rgbled_7.show();
    delay(100);
    }

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
      rgbled_7.show();
      
  motor1.run(-motorSpeed*compsx/100);
  motor2.run(motorSpeed*compdx/100);
  delay(300);
         
while(n<200){
  n++;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(ultraSensor.distanceCm() <= distStop){
     motor1.stop();
     motor2.stop();
     break;
  }

  accX = accX + a.acceleration.x*100;
  accY = accY + a.acceleration.y*100;
  accZ = accZ + a.acceleration.z*100;
  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

  if(n>10){

  speX = speX + accX*sampTime/100000;
  speY = speY + accY*sampTime/100000;
  speZ = speZ + accZ*sampTime/100000;
  speT = sqrt(speX*speX+speY*speY+speZ*speZ);
  
    if(sqrt((speStop-speT)*(speStop-speT))<accSensitivity*velocity){//&&speT<(velocity/2)){
      nStop++;
      }else{
      speStop=speT;
      nStop=0;
    }

    if(accT/100>accSensitivity*power*resamotori||nStop==6){
      accSensitivity = accSensitivity*1.1;
      /*
      if(accSensitivity>limitaccSensit*2){
        accSensitivity = limitaccSensit*2;
      }
      */
    }else{
      accSensitivity = accSensitivity*0.995;
      /*
      if(accSensitivity<limitaccSensit){
        accSensitivity = limitaccSensit;
      }
      */
    }
  }

/*
      Serial.print("X ");
      Serial.print(accX/100);
      Serial.print(' ');
      Serial.print("Y ");
      Serial.print(accY/100);
      Serial.print(' ');
      Serial.print("Z ");
      Serial.println(accZ/100);
      */
      /*
  Serial.print(accX/100);
  Serial.print(' ');
  Serial.print(accY/100);
  Serial.print(' ');
  Serial.print(accZ/100);
  Serial.print(' ');
  */
  Serial.print(accT/100);
  Serial.print(' ');
  Serial.println(speT);
  /*
  Serial.print(' ');
  Serial.println(accSensitivity);
  */
  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene

  accX =- a.acceleration.x*100;
  accY =- a.acceleration.y*100;
  accZ =- a.acceleration.z*100;

       
}

/*
  Serial.print("accSensitivity: ");
  Serial.println(accSensitivity);
  */

  if(accSensitivity>0.07&&accSensitivity<limitaccSensit){
    limitaccSensit=accSensitivity;
  }
n=0;
nStop=0;

motor1.stop();
motor2.stop();
delay(1500);
/*
  while(analogRead(A7) > 10){
    rgbled_7.setColor(1,0,0,60);//destro
      rgbled_7.show();
      rgbled_7.setColor(2,0,0,0);//sinistro
      rgbled_7.show();
    delay(200);
    rgbled_7.setColor(2,0,0,60);
      rgbled_7.show();
    rgbled_7.setColor(1,0,0,0);
      rgbled_7.show();
      delay(200);
    }

*/
}



void loop() {
  // Get new sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 
  
if(ultraSensor.distanceCm() <= distStop){
          nStop=0;
          
          if(n==0){
          startSteer =millis();
          }
          n++;
          /*
          Serial.println("----------STOP ROTAZIONE--------- ");
          Serial.println(n);
          Serial.print("distance : ");
          Serial.println(ultraSensor.distanceCm() );
          */
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          //tentativo rotazione
          // obstacle = obstacle + random(1,2);

          speX = 0;
          speY = 0;
          speZ = 0;
/*
          accX = 0;
          accY = 0;
          accZ = 0;
    */
          if(obstacle ==1){
            resto =  random(1,5)%2;
          }else{
            resto = (obstacle)%2;
          }
        
          distStop = distStop + 20;
         
          //Serial.print("resto : ");
          //Serial.println(resto);
          
          if(resto == 0){
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(backTime/obstacle);
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(-motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(obstacle*steeringTime);
          }else{
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(backTime/obstacle);
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(motorSpeed*compdx/100);
          delay(obstacle*steeringTime);
          }
          obstacle++;
          rnd = random(2,4);

          //Serial.print("obstacle : ");
          //Serial.println(obstacle);
          //riposizionamento, indietro e rotazione
          if(obstacle == 6){
            motor1.stop();
            motor2.stop();
            delay(stopTime);
            motor1.run(-motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(obstacle*steeringTime/2);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(exitTime);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(rnd*steeringTime);
            obstacle =1;
            distStop = 60*power;

            if(ultraSensor.distanceCm() <= distStop){
              motor1.run(-motorSpeed*compsx/100);
              motor2.run(motorSpeed*compdx/100);
              delay(100);
              motor1.run(-motorSpeed*compsx/100);
              motor2.run(-motorSpeed*compdx/100);
              delay(2*rnd*steeringTime);
            }
            }
            nSteer = 1;
          }else{
         //avanti
         
        // Serial.print("distance : ");
        // Serial.println(ultraSensor.distanceCm() );

         
         motor1.run(-motorSpeed*compsx/100);
         motor2.run(motorSpeed*compdx/100);
         delay(100);
         obstacle =1;
         distStop = 60*power;
         nSteer = 0;
    }


if(n==nObstacle){
  n=0;
  steBlockTime = millis()-startSteer;
  
  //Serial.print("steBlockTime ");
  //Serial.println(steBlockTime);
  
  if(steBlockTime<nObstacle*obstacleTime){
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(motorSpeed*compdx/100);
    delay(100);
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(-motorSpeed*compdx/100);
    delay(2*rnd*steeringTime);
    nStop=0;
    nSteer = 1;
    obstacle =1;
    distStop = 60*power;
    }
 }

  start=millis();

if( nSteer !=1){

  
 
  accX = accX + a.acceleration.x*100;
  accY = accY + a.acceleration.y*100;
  accZ = accZ + a.acceleration.z*100;
  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

  speX = speX + accX*cycleTime/100000;
  speY = speY + accY*cycleTime/100000;
  speZ = speZ + accZ*cycleTime/100000;
  speT = sqrt(speX*speX+speY*speY+speZ*speZ);
  
   //Serial.print(accY/100);
  //Serial.print(' ');
  
  //Serial.print("X ");
  //Serial.print(speX);
  //Serial.print(' ');
  //Serial.print("Y ");
  //Serial.print(speY);
  //Serial.print(' ');
  //Serial.print("Z ");
  //Serial.print(speZ);
  //Serial.print(' ');
  //Serial.print("T ");
  //Serial.println(speT);
  /*
  Serial.print(accX/100);
  Serial.print(' ');
  Serial.print(accY/100);
  Serial.print(' ');
  Serial.print(accZ/100);
  Serial.print(' ');
  */
  Serial.print(accT/100);
  Serial.print(' ');
  Serial.print(speT); //sopra 0.6 = urto
  Serial.print(' ');

  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene
 
  accX =- a.acceleration.x*100;
  accY =- a.acceleration.y*100;
  accZ =- a.acceleration.z*100;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene

  accX = accX + a.acceleration.x*100;
  accY = accY + a.acceleration.y*100;
  accZ = accZ + a.acceleration.z*100;
  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

  speX = speX + accX*cycleTime/100000;
  speY = speY + accY*cycleTime/100000;
  speZ = speZ + accZ*cycleTime/100000;
  speT = sqrt(speX*speX+speY*speY+speZ*speZ);

  Serial.print(accT/100);
  Serial.print(' ');
  Serial.println(speT); //sopra 0.6 = urto

  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene
 
  accX =- a.acceleration.x*100;
  accY =- a.acceleration.y*100;
  accZ =- a.acceleration.z*100;

}


  cycleTime=millis()-start;

  /*
  Serial.print("cycleTime: ");
  Serial.print(cycleTime);
*/

    if(sqrt((speStop-speT)*(speStop-speT))<accSensitivity*velocity){//&&speT<(velocity/2)){
      nStop++;
      obstacle =1;
      distStop =60*power;
       nSteer=0;
    }else{
      speStop=speT;
      nStop=0;
    }

    if(accT/100>accSensitivity*power*resamotori||nStop==8){
      /*
      Serial.print("nStop : ");
      Serial.println(nStop);
      */
/*
      Serial.print("X ");
      Serial.print(accX/100);
      Serial.print(' ');
      Serial.print("Y ");
      Serial.print(accY/100);
      Serial.print(' ');
      Serial.print("Z ");
      Serial.println(accZ/100);
      */
      if(nStop==8){
      buzzer.tone(950, 0.1 * 1000);
      }
      buzzer.tone(700, 0.1 * 1000);
      
      accSensitivity = accSensitivity*1.1;
      if(accSensitivity>limitaccSensit*2){
        accSensitivity = limitaccSensit*2;
      }
        motor1.stop();
        motor2.stop();
        delay(stopTime);
        motor1.run(motorSpeed*compsx/100);
        motor2.run(-motorSpeed*compdx/100);
        //delay(backTime);
        delay(exitTime);
        motor1.stop();
        motor2.stop();
        delay(stopTime);
        if(random(1,2)==1){
        motor1.run(-motorSpeed*compsx/100);
        motor2.run(-motorSpeed*compdx/100);
        delay(steeringTime);
        }else{
        motor1.run(motorSpeed*compsx/100);
        motor2.run(motorSpeed*compdx/100);
        delay(steeringTime);
        nStop=0;
        nSteer = 0;
        obstacle =1;
        distStop =60*power;
    }
    }else{
      accSensitivity = accSensitivity*0.995;
      
      if(accSensitivity<limitaccSensit){
        accSensitivity = limitaccSensit;
      }
      
    }
/*
    Serial.print(' ');
  Serial.println(accSensitivity);
  */
}
