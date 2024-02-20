
#include <Adafruit_MPU6050.h>  // accelerometer sensor library
#include <Adafruit_Sensor.h>

#include "MeOrion.h"  // Mbot microcontroller library
//#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>  //serial comunication library


MeEncoderNew motor1(0x09, SLOT1);  //  motor connects at slot1
MeEncoderNew motor2(0x09, SLOT2);  //  motor connects at slot2

MeUltrasonicSensor ultraSensor(PORT_8);


Adafruit_MPU6050 mpu;  //recall accelerometer reading
MeGyro gyro;          //recall gyroscope reading

void (*resetFunc)(void) = 0;  //RESET LOOP, when the function is called exit from loop, reset variable buffer and restart the code

//VARIABLES
const float compsx = -100 / 100;  //compensate left motor speed, use sign minus because the motor is fixed inverted
const float compdx = 100 / 100;   //compensate right motor speed
const float wheel = 64.0 * 3.1415;  //circonference of wheel, diameter of 64 mm

float power = 30.0 / 100;                              // variable of control speed 
uint8_t motorSpeed = 255 * power;                      // value: between -255 and 255
float steeringTime = 45000.0 * 1.13 / (280 * power);   //time to steer a 30 gradi
float backTime = 100 * 1000.0 * 1.13 / (750 * power);  //time to go backward of 100 mm
float stopTime = 400 * 1.13 * power;                   //time between change direction to avoid rear up
float distStop = 150 * power;                          //set distance before stopping in cm, influenced by power setting

const int sampTime = 70;  //sampling time between measurement readings in milliseconds
long start = 0;       //variable for calculating cycle time
long cycleTime = 0;  //variable which measure time to perform a loop cycle, and is used for integratedacceleration

float dist; //variable which store measument distance in each cycle
float distnow; //variable which store last distance to compare with reading
const float velocity = 30;          //set velocity of rover in cm/s
const float accSensitivity = 0.65;  //set confident level to capture acceleration peaks
const float speSensitivity = 0.12;  //set confident level to capture if rover is stuck
int nStop = 0;  //counter variable to measure time when speed is close to zero
int steer = 2;  //define fisrt side of steering 2=LEFT 3=RIGHT
int n = 0;     
int s = 0;
int movement = 0; //integer variable which expreses mode of movement, it starts going forward

const int dim = 6;  //dimension of acceleration vector
const int num = 200; //number of iteration when sensor calibration is required

float AaccX[dim];  //array accX
float AaccY[dim];  //array accY
float AaccZ[dim];  //array accZ

float accError = 9.81;

//DYNAMIC VARIABLES

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

String data, Start, Reset, Calibrate;

///////////////////////////////////////////////////////////////

void setup(void) {

  // Definition of the parameters of Our motor
  motor1.begin();
  motor2.begin();

  motor1.setRatio(45.0);  //gear ratio of motor transmition
  motor2.setRatio(45.0);

  motor1.setPulse(13);  // number of pulse per revolution of encoder motor
  motor2.setPulse(13);


  Serial.begin(115200); //baudrate of serial comunication
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
    Serial.println("Failed to find MPU6050 chip");  //check IMU device is ready
    while (1) {
      delay(10);
    }
  }

  gyro.begin();

  motor1.setPWM(0);  //set zero velocity of motor
  motor2.setPWM(0);

  for (int i = 0; i < dim; i++) { //inizializing acceleration vectors to zero
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
  }

  Start = String("Start");          //serial command to start motionloop function 
  Calibrate = String("Calibrate");  //serial command to start calibration function
  Reset = String("Reset");          //serial command to stop rover and reset code

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
        Serial.println("The run starts in 10 seconds");
        delay(10000);

        Serial.println(data);
        delay(10);
        break;
      }
    }
    delay(10);
  }

  //POWER CALIBRATION start a litle run, measure the actual speed and set power to compensate the battery charge level

  motor1.setPWM(motorSpeed * compsx);  //set velocity of motors
  motor2.setPWM(motorSpeed * compdx);
  delay(2000);

  while (s < num) {

    speM1 = -motor1.getCurrentSpeed();
    speM2 = motor2.getCurrentSpeed(); 
    speMY += (speM1 + speM2) * (wheel / 600) / 2; //calculate 
    s++;
    if (ultraSensor.distanceCm() < distStop) {
      break;
    }
    delay(sampTime);
  }
  speMY /= s;                                      //mean speed
  power = velocity * power / speMY;                // correzione potenza in base a speed
  motorSpeed = 255 * power;                        /* value: between -255 and 255. */
  steeringTime = 45000.0 * 1.13 / (280 * power);   //tempo di rotazione a 30 gradi
  backTime = 100 * 1000.0 * 1.13 / (750 * power);  //RITORNO INDIETRO DI 100 mm 550
  stopTime = 400 * 1.13 * power;                   //TEMPO TRA STOP E RIAVVIO MOTORI
  distStop = 150 * power;                          //cm
  s = 0;

  //Serial.println(power);

  distnow = ultraSensor.distanceCm();
}


////////////////////////////////////////////////////////////
void loop() {
  motion_loop();

  if (Serial.available() > 0) {

    if (Serial.readStringUntil('\n') == Reset) {
      //Serial.println(data);
      Serial.println("Reset");
      motor1.setPWM(0);
      motor2.setPWM(0);
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

  if ((s - dim) * cycleTime > 100) {
    distnow = ultraSensor.distanceCm();
    s = dim + 1;
  }

  if (s > dim) {
    speX += accX * cycleTime / 10;
    speY += accY * cycleTime / 10;
    speZ += accZ * cycleTime / 10;
    speT = sqrt(pow(speX, 2) + pow(speY, 2) + pow(speZ, 2));

    speM1 = -motor1.getCurrentSpeed();  //*wheel/600;
    speM2 = motor2.getCurrentSpeed();   //*wheel/600;
    speMY = (speM1 + speM2) * (wheel / 600) / 2;


    if (movement != 0) {  //accT < 3 &&
      if ((abs(speM1) * (wheel / 600) < speSensitivity * velocity || abs(speM2) * (wheel / 600) < speSensitivity * velocity || (movement == 1 && speMY <= (1 - speSensitivity) * velocity) || abs(speT) < speSensitivity * velocity)) {
        nStop++;
      } else {
        nStop = 0;
      }
    }

    if (sqrt(angY * angY) > 30 || accT > ((1.5 - accSensitivity) * 10 + power * 2) && abs(accY) > 3 || nStop > 600 / sampTime || distnow <= 75 * power) {
      collision = 1;
      n = 0;
      if (rotZ > 0) {
        steer = 3;
      } else {
        steer = 2;
      }
    }
  }

  if (collision == 1) {
    if (n * cycleTime > backTime) {                                  //backward
      if ((n * cycleTime - backTime) > stopTime) {                   //stop
        if ((n * cycleTime - backTime - stopTime) > steeringTime) {  //steering
          collision = 0;
          n = 0;
        } else {
          movement = steer;
          nStop = 0;
        }
      } else {
        movement = 0;
      }
    } else {
      movement = 4;
    }
  }

  if (collision == 2) {
    if (distnow >= dist && distnow < 380) {  //if(n*cycleTime>stopTime){
      dist = distnow;
    } else {
      collision = 0;
    }
  }

  if (collision == 0) {
    if (distnow <= distStop) {
      collision = 2;
      dist = 5;
      movement = 3;  //random(3, 2);
    } else {
      movement = 1;
    }
  }


  switch (movement) {
    case 0:  //Serial.println("stop");
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    case 1:  // Serial.println("forward");
      motor1.setPWM(motorSpeed * compsx);
      motor2.setPWM(motorSpeed * compdx);
      distStop = 150 * power;
      break;
    case 2:  // Serial.println("right");
      motor1.setPWM(-motorSpeed * 0.8 * compsx);
      motor2.setPWM(motorSpeed * 0.8 * compdx);
      distStop = 300 * power;
      break;
    case 3:  // Serial.println("left");
      motor1.setPWM(motorSpeed * 0.8 * compsx);
      motor2.setPWM(-motorSpeed * 0.8 * compdx);
      distStop = 300 * power;
      break;
    case 4:  // Serial.println("backward");
      motor1.setPWM(-motorSpeed * compsx);
      motor2.setPWM(-motorSpeed * compdx);
      distStop = 150 * power;
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
  Serial.print(accT);  //rotX
  Serial.print(",");
  Serial.print(speT);  //rotY
  Serial.print(",");
  Serial.print(speMY);  //rotZ
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
