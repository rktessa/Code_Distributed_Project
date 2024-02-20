
//TESTATO OK

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;

MeDCMotor motor1(M1);     //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI
MeRGBLed rgbled_7(7, 2);


int resamotori = 90;
int compsx = 100 * (200 - resamotori) / 100;
int compdx = 98 * (200 - resamotori) / 100;
float power = 50.0 / 100; // 50 % potenza
uint8_t motorSpeed = 255 * power; /* value: between -255 and 255. */
float travel = 100; //mm percorsi

//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
// 7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%
MeUltrasonicSensor ultraSensor(PORT_3);

int steeringAngle = 20; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle * 1000.0 / (280 * power); //DEVE ESSERE INTERO E PARI
int backTime = 150 * 1000.0 / (362 * power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 200; //TEMPO TRA STOP E RIAVVIO MOTORI
int exitTime = 400 * 1000.0 / (362 * power); // USCITA INDIETRO DI 400 mm 2200
int tempo = 0;
int obstacle = 1;
int distStop = 30;
int resto = 0;
int rnd = 1;
boolean on_motore = false;
int distanceR = 0;
int distanceL =  0;

float ultSens = 0;

float accX=0;
float accY=0;
float accZ=0;
float accT=0;

float speX=0;
float speY=0;
float speZ=0;
float speT=0;

long start=0;
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento
float sampTime = 50.00; //tempo in millis tra una lettura e l'altra

void setup() {


  
  rgbled_7.fillPixelsBak(0, 2, 1);
  pinMode(A7, INPUT);
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
   
  motor1.stop();
  motor2.stop();

  while (analogRead(A7) > 10) {
    rgbled_7.setColor(1, 255, 0, 0); //destro
    rgbled_7.setColor(2, 0, 0, 0); //sinistro
    rgbled_7.show();
    delay(100);
    rgbled_7.setColor(2, 255, 0, 0);
    rgbled_7.setColor(1, 0, 0, 0);
    rgbled_7.show();
    delay(100);
  }

rgbled_7.setColor(1, 0, 0, 0);
rgbled_7.setColor(2, 0, 0, 0);
rgbled_7.show();

}


int lookRight(int gradi)
{

  turnRight(gradi);
  moveStop();

  int distance = ultraSensor.distanceCm();


  return distance;

}

int lookLeft(int gradi)
{



  turnLeft(gradi);
  moveStop();

  int distance = ultraSensor.distanceCm();
  return distance;

}

void moveStop() {

  motor1.stop();
  motor2.stop();
  on_motore = false;
}

void moveForward() {

  motor1.run(-motorSpeed * compsx / 100);
  motor2.run(motorSpeed * compdx / 100);
  on_motore = true;
}

void moveBackward() {

  motor1.run(motorSpeed * compsx / 100);
  motor2.run(-motorSpeed * compdx / 100);
  on_motore = true;
}

void turnRight(int gradi) {

  motor1.run(-motorSpeed * compsx / 100);
  motor2.run(-motorSpeed * compdx / 100);
  delay(gradi * 1000.0 / (280 * power));
  on_motore = true;

}



void turnLeft(int gradi) {

  motor1.run(motorSpeed * compsx / 100);
  motor2.run(motorSpeed * compdx / 100);
  delay(gradi * 1000.0 / (280 * power));
  on_motore = true;

}

//int16_t accY, accZ;
float accAngle;


void loop() {

/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //delay(500);
    
  if (ultraSensor.distanceCm() > distStop) {
    //avanti
    //Serial.print("distance : ");
    //Serial.println(ultraSensor.distanceCm() );
    moveForward();

  } else {

start=millis();

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
  Serial.print(speT); //sopra 0.6 = urto
  Serial.print(' ');

  delay(sampTime);//serve per distanziare i due campionamenti, meno di 40 non va bene

  accX =- a.acceleration.x*100;
  accY =- a.acceleration.y*100;
  accZ =- a.acceleration.z*100;

cycleTime=millis()-start;
    /*
      // Print out the values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");

  Serial.print("Gyroscope ");

Serial.print("X: ");

Serial.print(g.gyro.x, 1);

Serial.print(" rps, ");

Serial.print("Y: ");

Serial.print(g.gyro.y, 1);

Serial.print(" rps, ");

Serial.print("Z: ");

Serial.print(g.gyro.z, 1);

Serial.println(" rps");
Serial.println("Gyroscope – rps");

Serial.print(g.gyro.x, 1);

Serial.print(", ");

Serial.print(g.gyro.y, 1);

Serial.print(", ");

Serial.print(g.gyro.z, 1);

Serial.println("");
*/
    // Serial.print("distance : ");
    // Serial.println(ultraSensor.distanceCm() );
    if (on_motore)
      moveStop();

    //delay(200);

    distanceR = lookRight(30);
    //Serial.print("distance R: ");
    //Serial.println(distanceR );

    //delay(200);

    distanceL = lookLeft(60);
    //Serial.print("distance L: ");
    //Serial.println(distanceL );

    //delay(200);

    if (distanceR > distStop)
    {
      moveForward();

    } else {
      if (distanceL > distStop)
        moveForward();
      else if (distanceR >= distanceL)
      {

        moveBackward();
        delay(200);
        moveStop();
        turnRight(30);

        moveStop();

      } else {

        moveBackward();
        delay(200);
        moveStop();
        turnLeft(30);

        moveStop();
      }

    }

  }
}
