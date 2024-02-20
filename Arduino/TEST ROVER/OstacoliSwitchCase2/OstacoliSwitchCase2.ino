
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

Adafruit_MPU6050 mpu;
MeGyro gyro;

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 98*(200-resamotori)/100;
float power = 50.0/100; // 50 % potenza
uint8_t motorSpeed = 255*power; /* value: between -255 and 255. */
float travel=100; //mm percorsi

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
int i = 0;
int movement=0;


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

float accX4=0;
float accY4=0;
float accZ4=0;

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
}

void loop() {

  start=millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX1 = -a.acceleration.x*100;
  accY1 = -a.acceleration.y*100;
  accZ1 = -a.acceleration.z*100;

  accX = accX4/3 - accX1;// - accX3/2;//accX2;
  accY = accY4/3 - accY1;// - accY3/2;//accY1;
  accZ = accZ4/3 - accZ1;// - accZ3/2;//accZ2;

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
  Serial.print(speX);
  Serial.print(' ');
  Serial.print(speY);
  Serial.print(' ');
  Serial.print(speZ);
  Serial.print(' ');
  Serial.println(speT);
//*/
  //Serial.print("distance : ");
  //Serial.println(ultraSensor.distanceCm() );

  
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
  switch (movement) {
    case 0:    // your hand is on the sensor
    //Serial.println("stop");
      motor1.stop();
      motor2.stop();
      break;
    case 1:    // your hand is close to the sensor
    //Serial.println("forward");
      motor1.run(-motorSpeed*compsx/100);
      motor2.run(motorSpeed*compdx/100);
      speY=abs(speY);
      break;
    case 2:    // your hand is a few inches from the sensor
    /*
      resto =  random(1,5)%2;
      if(resto == 0){
         Serial.println("right");
          motor1.run(-motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
      }else{
        Serial.println("left");
          motor1.run(motorSpeed*compsx/100);
          motor2.run(motorSpeed*compdx/100);
      }
      */
     // Serial.println("right");
          motor1.run(-motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          speX=abs(speX);
          
      break;
    case 3:    // your hand is nowhere near the sensor
   // Serial.println("backward");
      motor1.run(motorSpeed*compsx/100);
      motor2.run(-motorSpeed*compdx/100);
      speY=-abs(speY);
      break;
  }
  
  delay(20);        // delay in between reads for stability

  n++;
  cycleTime=millis()-start;
  //Serial.print("cycle ");
  //Serial.println(cycleTime);
}
