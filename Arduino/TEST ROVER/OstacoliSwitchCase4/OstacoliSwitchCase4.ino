
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

MeUltrasonicSensor ultraSensor(PORT_3);
MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;
MeBluetooth bluetooth(PORT_2);

Adafruit_MPU6050 mpu;
MeGyro gyro;

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 98*(200-resamotori)/100;
float power = 50.0/100; // 50 % potenza
const int maxSpeed = 255;
uint8_t motorSpeed = 255*power; /* value: between -255 and 255. */

//float travel=100; //mm percorsi

//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
// 7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%

int steeringAngle = 30; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle*1000.0/(280*power); //DEVE ESSERE INTERO E PARI
int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 400*power; //TEMPO TRA STOP E RIAVVIO MOTORI
//int exitTime = 300*1000.0/(362*power); // USCITA INDIETRO DI 300 mm 2200
int sampTime = 4; //tempo in millis tra una lettura e l'altra
//long startSteer =0;
//float steBlockTime =0;
//float obstacleTime = 4000/power;

long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento

//int obstacle =1;
//int nObstacle=5;
//int resto = 0;
//int rnd = 1;

float distStop =60*power;
float velocity = 0.36*power;
float accSensitivity = 0.16;//0.16
//float limitaccSensit = 0.16;
int collision=0;
float speStop=0; 

int nStop=0;
int n = 0;
int movement=0;

float accX=0;
float accY=0;
float accZ=0;
float accT=0;

float accX0=0;
float accY0=0;
float accZ0=0;

float accX1=0;
float accY1=0;
float accZ1=0;

float accX2=0;
float accY2=0;
float accZ2=0;

float accX3=0;
float accY3=0;
float accZ3=0;

float accX4=0;
float accY4=0;
float accZ4=0;

float accX5=0;
float accY5=0;
float accZ5=0;

float speX=0;
float speY=0;
float speZ=0;
float speT=0;

float rotX=0;
float rotY=0;
float rotZ=0;


void setup(void) {
  
  //SoftwareSerial mySerial(0, 1); // RX, TX
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

  if(motorSpeed>maxSpeed){
    motorSpeed=maxSpeed;
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
rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

//calibration();

//calibration1();


}

void loop() {

  start=millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX1 = -a.acceleration.x*100;
  accY1 = -a.acceleration.y*100;
  accZ1 = -a.acceleration.z*100;
/*
  accX = accX0 - accX1;
  accY = accY0 - accY1;
  accZ = accZ0 - accZ1;
*/

  accX = accX5/4 - accX1;
  accY = accY5/4 - accY1;
  accZ = accZ5/4 - accZ1;


/*
  accX = accX4/3 - accX1;
  accY = accY4/3 - accY1;
  accZ = accZ4/3 - accZ1;
*/

/*
  accX = accX3/2 - accX1;
  accY = accY3/2 - accY1;
  accZ = accZ3/2 - accZ1;
*/

/*
  accX = accX2 - accX1;
  accY = accY1 - accY1;
  accZ = accZ2 - accZ1;
*/

  accX5 = accX4 + accX1;
  accY5 = accY4 + accY1;
  accZ5 = accZ4 + accZ1;

  accX4 = accX3 + accX1;
  accY4 = accY3 + accY1;
  accZ4 = accZ3 + accZ1;

  accX3 = accX2 + accX1;
  accY3 = accY2 + accY1;
  accZ3 = accZ2 + accZ1;

  accX2 = accX1;
  accY2 = accY1;
  accZ2 = accZ1; 
 
  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

  speX = speX + accX*cycleTime/100000;
  speY = speY + accY*cycleTime/100000;
  speZ = speZ + accZ*cycleTime/100000;
  speT = sqrt(speX*speX+speY*speY+speZ*speZ);


  if(sqrt((speStop-speT)*(speStop-speT))<accSensitivity*velocity){
      nStop++;
      }else{
      speStop=speT;
      nStop=0;
    }

if(collision==0){
    if(accT/100<accSensitivity*power*resamotori||nStop==800/sampTime){//accSensitivity*power*resamotori

      if(nStop==800/sampTime){
        collision=1;
        n=0;
         buzzer.tone(950, 0.1 * 1000);
         buzzer.tone(700, 0.1 * 1000);
      }/*
      accSensitivity = accSensitivity*0.995;
      
      if(accSensitivity<limitaccSensit){
        accSensitivity = limitaccSensit;
      } 
*/
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
      /*
      accSensitivity = accSensitivity*1.75;
      if(accSensitivity>limitaccSensit*2){
        accSensitivity = limitaccSensit*2;
      }
      */
    }
  }

//Serial.print("accSensitivity ");
//Serial.print(' ');
//Serial.println(accSensitivity);//*power*resamotori);

//Serial.print("collision ");
//Serial.println(collision);
//Serial.print("n ");
//Serial.println(n);

 
    
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
  if(n*cycleTime>backTime){
    if((n*cycleTime-backTime)>steeringTime){
        collision=0;
        n=0;
    }else{
      speX=0;
      speY=0;
      speZ=0;
      movement =2;
    }
   }else{
      movement =3;
   }
}

if(collision==0){    
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
      speX=0;
      speY=0;
      speZ=0;
      break;
    case 1:   // Serial.println("forward");
      motor1.run(-motorSpeed*compsx/100);
      motor2.run(motorSpeed*compdx/100);
      speY=abs(speY);
      distStop =60*power;
      break;
    case 2:   // Serial.println("right");
      motor1.run(-motorSpeed*compsx/100);
      motor2.run(-motorSpeed*compdx/100);
      //speX=abs(speX);
      distStop = distStop + 0.5*sampTime;
      break;
    case 3:    // Serial.println("backward");
      motor1.run(motorSpeed*compsx/100);
      motor2.run(-motorSpeed*compdx/100);
      speY=-abs(speY);
      distStop =60*power;
      break;
  }

  /*
  //Serial.print("ACC ");
  Serial.print(accX/100);
  Serial.print(' ');
  Serial.print(accY/100);
  Serial.print(' ');
  Serial.print(accZ/100);
  Serial.print(' ');
  Serial.println(accT/100);
*/
  //Serial.print("SPE ");
  //Serial.print(speX);
  //Serial.print(' ');
  //Serial.println(speY);
  //Serial.print(' ');
  //Serial.print(speZ);
  //Serial.print(' ');
  //Serial.println(speT);


  Serial.print(accT/100);
  Serial.print(' ');
  Serial.println(speT);
  //Serial.print("distance : ");
  //Serial.println(ultraSensor.distanceCm() );
  
  delay(sampTime);       // delay in between reads for stability

  n++;
  cycleTime=millis()-start;
  //Serial.println("cycle ");
  //Serial.println(cycleTime);
}

void calibration1(){

sensors_event_t a, g, temp;

delay(1000);

  for(int i=0;i<1000;i++){

  mpu.getEvent(&a, &g, &temp);
  
  accX = -a.acceleration.x*100;
  accY = -a.acceleration.y*100;
  accZ = -a.acceleration.z*100;

  accX0 += accX;
  accY0 += accY;
  accZ0 += accZ;

   delay(sampTime); 

  }
  accX0/=1000;
  accY0/=1000;
  accZ0/=1000;
/*
  Serial.print("ACC ");
  Serial.print(accX0/100);
  Serial.print(' ');
  Serial.print(accY0/100);
  Serial.print(' ');
  Serial.println(accZ0/100);
  */
}


void calibration(void){

 accSensitivity=0.1;
 
motor1.run(-motorSpeed*compsx/100);
  motor2.run(motorSpeed*compdx/100);
  delay(300);
         
while(n<2000/sampTime){
  n++;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(ultraSensor.distanceCm() <= distStop){
     motor1.stop();
     motor2.stop();
     break;
  }

  accX1 = -a.acceleration.x*100;
  accY1 = -a.acceleration.y*100;
  accZ1 = -a.acceleration.z*100;

  accX = accX5/4 - accX1;
  accY = accY5/4 - accY1;
  accZ = accZ5/4 - accZ1;


/*
  accX = accX4/3 - accX1;
  accY = accY4/3 - accY1;
  accZ = accZ4/3 - accZ1;
*/

/*
  accX = accX3/2 - accX1;
  accY = accY3/2 - accY1;
  accZ = accZ3/2 - accZ1;
*/

/*
  accX = accX2 - accX1;
  accY = accY1 - accY1;
  accZ = accZ2 - accZ1;
*/

  accX5 = accX4 + accX1;
  accY5 = accY4 + accY1;
  accZ5 = accZ4 + accZ1;

  accX4 = accX3 + accX1;
  accY4 = accY3 + accY1;
  accZ4 = accZ3 + accZ1;

  accX3 = accX2 + accX1;
  accY3 = accY2 + accY1;
  accZ3 = accZ2 + accZ1;

  accX2 = accX1;
  accY2 = accY1;
  accZ2 = accZ1; 
  
  if(n>5){

  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

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

    if(nStop==800/sampTime){
      
      accSensitivity = accSensitivity*0.5;
      
      }
      if(accT/100>accSensitivity*power*resamotori){
        
      accSensitivity = accSensitivity*1.5;
     
    }
  }

  //Serial.print("X ");
  //Serial.print(speX);
  //Serial.print(' ');
  //Serial.print("Y ");
  //Serial.print(speY);
  //Serial.print(' ');
  //Serial.print("Z ");
  //Serial.print(speZ);
  //Serial.print(' ');
      /*
  Serial.print(accX/100);
  Serial.print(' ');
  Serial.print(accY/100);
  Serial.print(' ');
  Serial.print(accZ/100);
  Serial.print(' ');
  */
  
  //Serial.print(accT/100);
  //Serial.print(' ');
  //Serial.println(speT);
  
  //Serial.print(' ');
 Serial.println(accSensitivity);
  
  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene
       
}

  //Serial.print("accSensitivity: ");
  Serial.println(accSensitivity);
  /*
  if(accSensitivity>0.08&&accSensitivity<limitaccSensit){
    limitaccSensit=accSensitivity;
  }
*/
  //limitaccSensit=accSensitivity;
n=0;
nStop=0;

motor1.stop();
motor2.stop();
delay(1500);

speX=0;
speY=0;
speZ=0;
accX=0;
accY=0;
accZ=0;

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

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();
}
