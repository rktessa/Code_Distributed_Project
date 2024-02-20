

//TESTATO OK
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeDCMotor motor1(M1);     //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

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
MeUltrasonicSensor ultraSensor(PORT_3);

int steeringAngle = 30; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle*1000.0/(280*power); //DEVE ESSERE INTERO E PARI
int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 200; //TEMPO TRA STOP E RIAVVIO MOTORI
int exitTime = 400*1000.0/(362*power); // USCITA INDIETRO DI 400 mm 2200
int obstacle =1;
int resto = 0;
int rnd = 1;
 
void setup() {
  pinMode(A7, INPUT);
  Serial.begin(9600);
  motor1.stop();
  motor2.stop();

//attivazione pulsante
  while(analogRead(A7) > 10){
    _loop();
    }  
    
}

void _loop() {
}

void loop() {
  // read the sensor:
  int range=1;
    if(ultraSensor.distanceCm()<20){
    range=0;
    }
  rnd = random(1,3);

  if(obstacle == 8){
    range =2;
    }
  
  // do something different depending on the range value:
  switch (range) {
    case 0:    // your hand is on the sensor
      motor1.stop();
      motor2.stop();
      delay(stopTime);
      //tentativo rotazione
      resto = obstacle%2;

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
      break;
    case 1:    // your hand is close to the sensor
      Serial.println("dim");
      motor1.run(-motorSpeed*compsx/100);
      motor2.run(motorSpeed*compdx/100);
      obstacle =1;
      break;
    case 2:    // your hand is a few inches from the sensor
            motor1.stop();
            motor2.stop();
            delay(stopTime);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(obstacle*steeringTime/2);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(exitTime);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(rnd*steeringTime);
            obstacle =1;
      break;
    case 3:    // your hand is nowhere near the sensor
      Serial.println("bright");
      break;
  }
  delay(1);        // delay in between reads for stability
}
