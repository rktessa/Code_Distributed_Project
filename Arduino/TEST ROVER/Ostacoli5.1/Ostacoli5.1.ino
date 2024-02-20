
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

void setup() {


  
  rgbled_7.fillPixelsBak(0, 2, 1);
  pinMode(A7, INPUT);
  Serial.begin(115200);

  



   
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

int16_t accY, accZ;
float accAngle;


void loop() {


    
  if (ultraSensor.distanceCm() > distStop) {
    //avanti
    //Serial.print("distance : ");
    //Serial.println(ultraSensor.distanceCm() );
    moveForward();

  } else {
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
