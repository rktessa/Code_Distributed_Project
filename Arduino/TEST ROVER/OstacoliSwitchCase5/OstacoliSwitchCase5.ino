
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
//MeBluetooth bluetooth(PORT_2);
Me7SegmentDisplay disp(PORT_2);

Adafruit_MPU6050 mpu;
MeGyro gyro;

const int resamotori = 90;
const int compsx = 100*(200-resamotori)/100;
const int compdx = 98*(200-resamotori)/100;
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

 
const int steeringTime = 30*1000.0/(280*power); //tempo di rotazione a 30 gradi
const int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
const int stopTime = 400*power; //TEMPO TRA STOP E RIAVVIO MOTORI
//int exitTime = 300*1000.0/(362*power); // USCITA INDIETRO DI 300 mm 2200
int sampTime = 4; //tempo in millis tra una lettura e l'altra


long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento


float distStop =60*power;
float velocity = 0.36*power;
float accSensitivity = 0.16;//0.16
//float limitaccSensit = 0.16;
int collision=0;
float speStop=0; 

int nStop=0;
int n = 0;
int nReg = 0;
int movement=0;

const int dim = 10;

float AaccX[dim]; //array accX
float AaccY[dim]; //array accX
float AaccZ[dim]; //array accX

float accXerror=0;
float accYerror=0;
float accZerror=0;

float speXerror=0;
float speYerror=0;
float speZerror=0;

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
rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

calibration1();
calibration();


}

void loop() {

  start=millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  AaccX[0] = -a.acceleration.x - accXerror;
  AaccY[0] = -a.acceleration.y - accYerror;
  AaccZ[0] = -a.acceleration.z - accZerror;
/*
  accX = accXerror - AaccX[0];
  accY = accYerror - AaccY[0];
  accZ = accZerror - AaccZ[0];
*/

  accX = AaccX[dim-1]/(dim-1) - AaccX[0];
  accY = AaccY[dim-1]/(dim-1) - AaccY[0];
  accZ = AaccZ[dim-1]/(dim-1) - AaccZ[0];
/*
  Serial.print("Acc[DIM] ");
  Serial.print(AaccX[dim-1]/(dim-1));
  Serial.print(' ');
  Serial.print(AaccY[dim-1]/(dim-1));
  Serial.print(' ');
  Serial.print(AaccZ[dim-1]/(dim-1));
  Serial.println(' ');
*/
  for(int i=0;i<(dim-2);i++){
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
   }
/*
  Serial.print("AccX ");
  for(int i=0;i<dim;i++){
    Serial.print(AaccX[i]);
    Serial.print(' ');
  }Serial.println(' ');

  Serial.print("AccY ");
  for(int i=0;i<dim;i++){
    Serial.print(AaccY[i]);
    Serial.print(' ');
  }Serial.println(' ');

  Serial.print("AccZ ");
  for(int i=0;i<dim;i++){
    Serial.print(AaccZ[i]);
    Serial.print(' ');
  }Serial.println(' ');
*/
  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

  accT = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2));

  speX = speX + accX*cycleTime/1000;
  speY = speY + accY*cycleTime/1000;
  speZ = speZ + accZ*cycleTime/1000;
  speT = sqrt(pow(speX,2)+pow(speY,2)+pow(speZ,2));

  if((n-nReg)>100/cycleTime){
    disp.display((float)speT);
    nReg = n;
  }else if(n==0){
    nReg = n;   
  }

  if(sqrt((speStop-speT)*(speStop-speT))<accSensitivity*velocity){
      nStop++;
      }else{
      speStop=speT;
      nStop=0;
    }

if(collision==0){
    if(accT<accSensitivity*power*resamotori||nStop==800/sampTime){//accSensitivity*power*resamotori

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
      /*
      speX=0;
      speY=0;
      speZ=0;
      */
      movement =2;
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

  
  Serial.print("ACC ");
  Serial.print(accX);
  Serial.print(' ');
  Serial.print(accY);
  Serial.print(' ');
  Serial.print(accZ);
  Serial.print(' ');
  Serial.println(accT);

  Serial.print("SPE ");
  Serial.print(speX);
  Serial.print(' ');
  Serial.print(speY);
  Serial.print(' ');
  Serial.print(speZ);
  Serial.print(' ');
  Serial.print(speT);
  Serial.println(' ');


  //Serial.print(accT);
  //Serial.print(' ');
  //Serial.print(speT);
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
int num=1000;
  for(int i=0;i<num;i++){

  mpu.getEvent(&a, &g, &temp);
  
  accXerror += -a.acceleration.x;
  accYerror += -a.acceleration.y;
  accZerror += -a.acceleration.z;

   delay(sampTime); 

  }
  accXerror/=num;
  accYerror/=num;
  accZerror/=num;

  accT = sqrt(pow(accXerror,2)+pow(accYerror,2)+pow(accZerror,2));

  Serial.print("ACC ");
  Serial.print(accXerror);
  Serial.print(' ');
  Serial.print(accYerror);
  Serial.print(' ');
  Serial.print(accZerror);
  Serial.print(' ');
  Serial.println(accT);

  accXerror *= (accT/9.81 -1);
  accYerror *= (accT/9.81 -1);
  accZerror *= (accT/9.81 -1);

  Serial.print("ACC ");
  Serial.print(accXerror);
  Serial.print(' ');
  Serial.print(accYerror);
  Serial.print(' ');
  Serial.println(accZerror);

}


void calibration(){

 accSensitivity=0.1;
 
           
while(n<2000/sampTime){
  n++;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(ultraSensor.distanceCm() <= distStop){
     motor1.stop();
     motor2.stop();
     break;
  }

  AaccX[0] = -a.acceleration.x - accXerror;
  AaccY[0] = -a.acceleration.y - accYerror;
  AaccZ[0] = -a.acceleration.z - accZerror;
/*
  accX = accXerror - AaccX[0];
  accY = accYerror - AaccY[0];
  accZ = accZerror - AaccZ[0];
*/

  
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
  
  accT = sqrt(accX*accX+accY*accY+accZ*accZ);

  speX = speX + accX*sampTime/1000 - speXerror;
  speY = speY + accY*sampTime/1000 - speYerror;
  speZ = speZ + accZ*sampTime/1000 - speZerror;
  speT = sqrt(speX*speX+speY*speY+speZ*speZ);
  /*
    if(sqrt((speStop-speT)*(speStop-speT))<accSensitivity*velocity){//&&speT<(velocity/2)){
      nStop++;
      }else{
      speStop=speT;
      nStop=0;
    }

    if(nStop==800/sampTime){
       accSensitivity = accSensitivity*0.5;
    }
    if(accT>accSensitivity*power*resamotori){
       accSensitivity = accSensitivity*1.5;
    }
  */

  if(n<15||n>1500/sampTime){
    motor1.stop();
    motor2.stop();
  }else{
    motor1.run(-motorSpeed*compsx/100);
    motor2.run(motorSpeed*compdx/100);
    Serial.println(speT);
    if(speT>velocity){
      Serial.println(speT);
      speXerror *=(speT/velocity -1);
      speYerror *=(speT/velocity -1);
      speZerror *=(speT/velocity -1);
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
  Serial.print(accX);
  Serial.print(' ');
  Serial.print(accY);
  Serial.print(' ');
  Serial.print(accZ);
  Serial.print(' ');
  */
  
  //Serial.print(accT);
  //Serial.print(' ');
  //Serial.println(speT);
  
  //Serial.print(' ');
 //Serial.println(accSensitivity);
  
  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene
       
}

  //Serial.print("accSensitivity: ");
  //Serial.println(accSensitivity);
  /*
  if(accSensitivity>0.08&&accSensitivity<limitaccSensit){
    limitaccSensit=accSensitivity;
  }
*/
  //limitaccSensit=accSensitivity;
n=0;
nStop=0;

speXerror = speX;
speYerror = speY;
speZerror = speZ;

  Serial.print("speError ")
  Serial.print(speX);
  Serial.print(' ');
  Serial.print(speY);
  Serial.print(' ');
  Serial.print(speZ);
  Serial.println(' ');

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

*/

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();
}
