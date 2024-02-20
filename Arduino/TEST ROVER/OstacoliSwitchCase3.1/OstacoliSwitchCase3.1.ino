
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

Adafruit_MPU6050 mpu;
MeGyro gyro;

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 98*(200-resamotori)/100;
float power = 50.0/100; // 50 % potenza
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
int sampTime = 16; //tempo in millis tra una lettura e l'altra
//long startSteer =0;
//float steBlockTime =0;
//float obstacleTime = 4000/power;

long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento

//int obstacle =1;
//int nObstacle=5;
//int resto = 0;
//int rnd = 1;
float dist;
float distnow;
float distStop =60*power;
const float velocity = 30;          //75 * power;  //cm/s
const float accSensitivity = 0.65;  //0.23;//0.16   0.0 - 1.0
const float speSensitivity = 0.12;  //0.12  % della velocity
int collision=0;
float speStop=0; 

int nStop=0;
int steer =2;
int n = 0;
int s = 0;
int movement = 0;

const int dim = 6;
const int num = 100;

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


void setup(void) {
  
  //SoftwareSerial mySerial(0, 1); // RX, TX
  rgbled_7.fillPixelsBak(0, 2, 1);
  Serial.begin(115200);
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
  
  pinMode(A7, INPUT);
  motor1.stop();
  motor2.stop();

  for (int i = 0; i < dim; i++) {
    AaccX[i] = 0.00;
    AaccY[i] = 0.00;
    AaccZ[i] = 0.00;
  }

  while(analogRead(A7) < 10){
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

calibration();

distnow = ultraSensor.distanceCm();

}

void loop() {

  start=millis();

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

  if ((s - dim) * cycleTime >= 100) {
    distnow = ultraSensor.distanceCm();
    s = dim + 1;
  }

  if (s > dim) {
    speX += accX * cycleTime / 10;
    speY += accY * cycleTime / 10;
    speZ += accZ * cycleTime / 10;
    speT = sqrt(pow(speX, 2) + pow(speY, 2) + pow(speZ, 2));

    if (movement != 0) {  //accT < 3 &&
      if ( abs(speT) < speSensitivity * velocity) {
        nStop++;
      } else {
        nStop = 0;
      }
    }
  
    if (sqrt(angY * angY) > 30 || accT > ((1.5 - accSensitivity) * 10 + power * 2) && abs(accY) > 3 || nStop > 600 / sampTime || distnow <= 75 * power) {
      collision = 1;
      n = 0;
      if(rotZ>0){
        steer =3;
      }else{
        steer =2;
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
      motor1.stop();
      motor2.stop();
      break;
    case 1:  // Serial.println("forward");
      motor1.run(motorSpeed * compsx);
      motor2.run(motorSpeed * compdx);
      distStop = 150 * power;
      break;
    case 2:  // Serial.println("right");
      motor1.run(motorSpeed * 0.8 * compsx);
      motor2.run(-motorSpeed * 0.8 * compdx);
      distStop = 300 * power;
      break;
    case 3:  // Serial.println("left");
      motor1.run(-motorSpeed * 0.8 * compsx);
      motor2.run(motorSpeed * 0.8 * compdx);
      distStop = 300 * power;
      break;
    case 4:  // Serial.println("backward");
      motor1.run(-motorSpeed * compsx);
      motor2.run(-motorSpeed * compdx);
      distStop = 150 * power;
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


  Serial.print(accT);
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

void calibration(void){

  delay(300);
         
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

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();
}
