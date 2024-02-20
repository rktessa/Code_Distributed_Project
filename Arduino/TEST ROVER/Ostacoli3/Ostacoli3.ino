

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
float power = 60.0/100; // 50 % potenza
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

int steeringAngle = 45; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle*1000.0/(280*power); //DEVE ESSERE INTERO E PARI
int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 200; //TEMPO TRA STOP E RIAVVIO MOTORI
int exitTime = 400*1000.0/(362*power); // USCITA INDIETRO DI 400 mm 2200
int tempo = 0;
int obstacle =1;
int distStop =30;
int resto = 0;
int rnd = 1;

float sens1=0;

 
void setup() {
  pinMode(A7, INPUT);
  Serial.begin(9600);
  motor1.stop();
  motor2.stop();

  while(analogRead(A7) > 10){
    delay(100);
    }
}


void loop() {
    if(ultraSensor.distanceCm() > 60&&ultraSensor.distanceCm() < 200){
      Serial.print("distance : ");
      Serial.println(ultraSensor.distanceCm() );
      tempo = 0;
      while(tempo < 30&&){
        delay(100);
        tempo++;
        if(ultraSensor.distanceCm()<50&&ultraSensor.distanceCm()>210){
          break;
      }
      }
      Serial.println(tempo);
      if(tempo < 32){
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
      }
            
    }
motor1.run(-motorSpeed*compsx/100);
motor2.run(motorSpeed*compdx/100);
    
      
}


void evita(){
}
