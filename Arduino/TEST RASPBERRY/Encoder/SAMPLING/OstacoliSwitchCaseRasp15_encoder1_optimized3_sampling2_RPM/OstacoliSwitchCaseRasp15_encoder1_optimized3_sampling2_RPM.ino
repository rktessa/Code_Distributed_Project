
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "MeOrion.h"
//#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeEncoderNew motor1(0x09, SLOT1);  //  motor at slot1
MeEncoderNew motor2(0x09, SLOT2);  //  motor at slot2

//MeTouchSensor MeTouchSensor(PORT_3);
MeUltrasonicSensor ultraSensor(PORT_8);


Adafruit_MPU6050 mpu;
MeGyro gyro;

void (*resetFunc)(void) = 0;  //FUNZIONE RESET LOOP

//VARIABLES
float compsx = 100/100; //SEGNO MENO PERCHE MOTORE INVERTITO
float compdx = -100/100;
float power = 50.0/100; // PRIMO TENTATIVO VEDI VELOCITY PER CORREGGERE VELOCITA'
uint8_t motorSpeed = 200*power;//255*power; /* value: between -255 and 255. */
const float wheel = 64.0*3.14; //circonference

/*MBOT MCORE
    4.4 GIRI AL 60% PER 4 SECONDI = 1.83
    3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
    1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
    DIAMETRO RUOTA 64 mm = CIRC 201 mm
    const float halfinteraxis = 112.0/2;
    362 mm/ secondo al 100 %
    test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
     7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%
  */
float steeringTime = 9000;        //3*360 in 6 s                     //30000.0*1.13/(280*power); //tempo di rotazione a 30 gradi
float backTime = 100 * 1000.0 / (750 * power);   //RITORNO INDIETRO DI 100 mm 550
float stopTime = 400 * 1.13 * power;                    //TEMPO TRA STOP E RIAVVIO MOTORI
float forwTime = 6000;//500 * 1000.0 / (750 * power);  //AVANTI di 1000 mm
const int sampTime = 10;                                      //tempo in millis tra una lettura e l'altra

long start = 0;
long cycleTime = 0;  //tempo per compiere un ciclo loop o tempo di campionatemento

float distStop = 100 * power;       //cm
const float velocity = 30;  //cm/s 122 cm/s al 100% con 7.89V
const float accSensitivity = 0.35;  //0.23;//0.16
const float speSensitivity = 0.16;  //0.12
int collision = 0;
int nStop = 0;
int n = 0;
int s = 0;
int movement = 0;

const int dim = 10;
const int num = 200;

float AaccX[dim];  //array accX
float AaccY[dim];  //array accY
float AaccZ[dim];  //array accZ

float accError = 9.81;

float accXError = 0;
float accYError = 0;
float accZError = 0;

float rotXError = 0;
float rotYError = 0;
float rotZError = 0;

float angXError = 0;
float angYError = 0;
float angZError = 0;

float accX = 0;
float accY = 0;
float accZ = 0;
float accT = 0;

float speX = 0;
float speY = 0;
float speZ = 0;
float speT = 0;

float rotX = 0;
float rotY = 0;
float rotZ = 0;

float angX = 0;
float angY = 0;
float angZ = 0;

float speM1 = 0;  //rpm
float speM2 = 0;  //rpm
float speMY = 0;  //cm/s

String data, Start, Stop, Reset, Calibrate;

///////////////////////////////////////////////////////////////

void setup(void) {

  // Definition of the parameters of Our motor
  motor1.begin();
  motor2.begin();

  motor1.setRatio(45.0);  //Rapporto di riduzione motore
  motor2.setRatio(45.0);

  motor1.setPulse(13);  // Numero di pulse che legge encoder in un giro del motore
  motor2.setPulse(13);


  //MeTouchSensor.SetTogMode(0);


  Serial.begin(115200);
  //Serial.flush();
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

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

      motor1.runSpeed(0);//motor1.setPWM(0);
      motor2.runSpeed(0);//motor2.setPWM(0);

  for (int i = 0; i < dim; i++) {
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
  }

  Start = String("Start");          //avvio motionloop
  Stop = String("Stop");            //ferma il programma e basta ridare Start
  Calibrate = String("Calibrate");  //avvia calibrazione solo se prima Reset
  Reset = String("Reset");          //ferma e resetta il programma

  Serial.println("Waiting serial input: Calibrate or Start");

  while (1) {
    if (Serial.available() > 0) {
      data = Serial.readStringUntil('\n');
      if (data == Calibrate) {
        calibrationStop();
        Serial.println("Calibration done");
        Serial.println("Waiting serial input: Calibrate or Start");
      }
      if (data == Start) {
        Serial.println("During the run, you can give serial input: Stop or Reset");
        Serial.println("The run starts in 2 seconds");
        delay(2000);

        Serial.println(data);
        delay(10);
        break;
      }
    }
    delay(10);
  }

  motor1.runSpeed(motorSpeed * compsx);//motor1.setPWM(motorSpeed * compsx);
  motor2.runSpeed(motorSpeed * compdx);//motor2.setPWM(motorSpeed * compdx);
  delay(2000);

  while (s < num) {

    speM1 = motor1.getCurrentSpeed();  //*wheel/600;
    speM2 = -motor2.getCurrentSpeed();   //*wheel/600;
    speMY += (speM1 + speM2) * (wheel / 600) / 2;
    s++;
    if (ultraSensor.distanceCm() < distStop) {
      break;
    }
    delay(sampTime);
  }
  speMY /= s; 
  Serial.println(speMY);                                     //mean speed
  power = velocity * power / speMY;                // correzione potenza in base a speed
  motorSpeed = 200 * power;                        //255*power /* value: between -255 and 255. */
  backTime = 100 * 1000.0 * 1.13 / (750 * power);  //RITORNO INDIETRO DI 100 mm 550
  stopTime = 6000 * 1.13 * power;                   //TEMPO TRA STOP E RIAVVIO MOTORI
  forwTime = 500 * 1000.0 / (750 * power);
  distStop = 150 * power;                          //cm
  s = 0;
  Serial.print("power ");
  Serial.println(power);
  motor1.runSpeed(0);//motor1.setPWM(0);
  motor2.runSpeed(0);//motor2.setPWM(0);
  
  delay(12000);
}

////////////////////////////////////////////////////////////
void loop() {
  motion_loop();

  if (Serial.available() > 0) {

    if (Serial.readStringUntil('\n') == Reset) {
      //Serial.println(data);
      Serial.println("Reset");
      motor1.runSpeed(0);//motor1.setPWM(0);
      motor2.runSpeed(0);//motor2.setPWM(0);
      delay(100);
      resetFunc();
    }

    /*
    if(Serial.readStringUntil('\n') == Stop){
      //Serial.println(data);
      Serial.println("Stop");
      Serial.println("Waiting serial input: Reset or Start");
      motor1.setPWM(0);
      motor2.setPWM(0);
      delay(200);
      while(1){
        if (Serial.available() > 0) {
          if(Serial.readStringUntil('\n') == Start){
            //Serial.println(data);
            Serial.println("Start");
            delay(100);
            break;
          }
          if(Serial.readStringUntil('\n') == Reset){
            //Serial.println(data);
            Serial.println("Reset");
            delay(100);
            resetFunc();
          }
        }
        delay(10);
      }
    }
    */
  }
}

//////////////////////////////////////////////////////////
void motion_loop() {

  start = millis();

  gyro.update();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rotX = g.gyro.x - rotXError;
  rotY = g.gyro.y - rotYError;
  rotZ = g.gyro.z - rotZError;

  angX = gyro.getAngleX() - angXError;
  angY = gyro.getAngleY() - angYError;
  angZ = gyro.getAngleZ() - angZError;

  AaccX[0] = a.acceleration.x - accXError - accError * sin(angX * 3.1415 / 180);
  AaccY[0] = a.acceleration.y - accYError - accError * sin(angY * 3.1415 / 180);
  AaccZ[0] = a.acceleration.z - accZError - accError * cos(angX * 3.1415 / 180) * cos(angY * 3.1415 / 180);

  accX = AaccX[0] - AaccX[dim - 1] / (dim - 1);
  accY = AaccY[0] - AaccY[dim - 1] / (dim - 1);
  accZ = AaccZ[0] - AaccZ[dim - 1] / (dim - 1);


  for (int i = 0; i < (dim - 2); i++) {
    AaccX[dim - 1 - i] = AaccX[dim - 2 - i] + AaccX[0];
    AaccY[dim - 1 - i] = AaccY[dim - 2 - i] + AaccY[0];
    AaccZ[dim - 1 - i] = AaccZ[dim - 2 - i] + AaccZ[0];
  }

  AaccX[1] = AaccX[0];
  AaccY[1] = AaccY[0];
  AaccZ[1] = AaccZ[0];

  accT = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));

  //if(s*cycleTime >=100){ 
  speM1 = motor1.getCurrentSpeed();  //*wheel/600;
  speM2 = -motor2.getCurrentSpeed();   //*wheel/600;
  s=0;
  //}
  /*
  if (n * cycleTime > forwTime) {                                  //forward
    if ((n * cycleTime - forwTime) > stopTime) {                   //stop
      if ((n * cycleTime - forwTime - stopTime) > steeringTime) {  //steering
        movement = 0;
      } else {
        movement = 4;
      }
    } else {
      movement = 0;
    }
  } else {
    movement = 1;
  }
  */
  
  if (n * cycleTime > forwTime) {                                  //forward
    if ((n * cycleTime - forwTime) > steeringTime) {                   //right
      movement = 0;
      /*
      if ((n * cycleTime - forwTime - steeringTime)>10*cycleTime) {   
        delay(100);
        resetFunc();
      }
      */
      
    } else {
      movement = 2;
    }
  } else {
    movement = 1;
  }


  switch (movement) {
    case 0:  //Serial.println("stop");
      motor1.runSpeed(0);//motor1.setPWM(0);
      motor2.runSpeed(0);//motor2.setPWM(0);
      break;
    case 1:  // Serial.println("forward");
      motor1.runSpeed(motorSpeed * compsx);//motor1.setPWM(motorSpeed * compsx);
      motor2.runSpeed(motorSpeed * compdx);//motor2.setPWM(motorSpeed * compdx);
      distStop = 150 * power;
      break;
    case 2:  // Serial.println("right");
      motor1.runSpeed(motorSpeed * 0.6 * compsx);//motor1.setPWM(-motorSpeed * 0.6 * compsx);
      motor2.runSpeed(-motorSpeed * 0.6 * compdx);//motor2.setPWM(motorSpeed * 0.6 * compdx);
      distStop = 300 * power;
      break;
    case 3:  // Serial.println("backward");
      motor1.runSpeed(-motorSpeed * compsx);//motor1.setPWM(-motorSpeed * compsx);
      motor1.runSpeed(-motorSpeed * compdx);//motor2.setPWM(-motorSpeed * compdx);
      distStop = 150 * power;
      break;
    case 4:  // Serial.println("left");
      motor1.runSpeed(-motorSpeed * 0.6 * compsx);//motor1.setPWM(motorSpeed * 0.6 * compsx);
      motor2.runSpeed(motorSpeed * 0.6 * compdx);//motor2.setPWM(-motorSpeed * 0.6 * compdx);
      distStop = 300 * power;
      break;
  }
  
  // ##########################################
  // PARTE PER LA STAMPA
  Serial.print("Inizio");
  Serial.print(",");
  Serial.print(start / 1000);
  Serial.print(",");
  Serial.print(accX);  //accX
  Serial.print(",");
  Serial.print(accY);
  Serial.print(",");
  Serial.print(accZ);
  Serial.print(",");
  Serial.print(rotX);  //rotX
  Serial.print(",");
  Serial.print(rotY);  //rotY
  Serial.print(",");
  Serial.print(rotZ);
  Serial.print(",");
  Serial.print(speM1);
  Serial.print(",");
  Serial.print(speM2);
  Serial.print(",");
  Serial.println("Fine");

  delay(sampTime);  // delay in between reads for stability

  n++;
  s++;
  cycleTime = millis() - start;
  //Serial.println("cycle ");
  //Serial.println(cycleTime);
}

/////////////////////////////////////////////////////////

void calibrationStop() {
  Serial.println("Calibration execution");
  sensors_event_t a, g, temp;

  delay(1500);

  for (int i = 0; i < num; i++) {

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
  accXError /= num;
  accYError /= num;
  accZError /= num;

  rotXError /= num;
  rotYError /= num;
  rotZError /= num;

  angXError /= num;
  angYError /= num;
  angZError /= num;

  accError = sqrt(pow(accXError, 2) + pow(accYError, 2) + pow(accZError, 2));
}
