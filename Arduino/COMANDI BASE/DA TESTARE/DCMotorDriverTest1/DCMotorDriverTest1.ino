
 //DEVI CAPIRE QUALI PORTE SONO ASSEGNATE AI MOTORI
 
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeDCMotor motor1(M1); //SINISTRO -motorSpeed == AVANTI

MeDCMotor motor2(M2); //DESTRO motorSpeed == AVANTI

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 96*(200-resamotori)/100;
float power = 50.0/100; // 50 % potenza
float travel=100; //mm percorsi

uint8_t motorSpeed = 255*power; /* value: between -255 and 255. */
//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//travel*1000/(362*power)
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
void setup()
{
  pinMode(M1, INPUT);
  pinMode(M2, INPUT);
  Serial.begin(9600);
}

void goback(){
  motor1.run(-motorSpeed*compsx/100);
  motor2.run(motorSpeed*compdx/100);
  delay(2000);
  motor1.stop();
  motor2.stop();
  delay(100);
  motor1.run(motorSpeed*compsx/100);
  motor2.run(-motorSpeed*compdx/100);
  delay(2000);
  motor1.stop();
  motor2.stop();
  delay(2000);
}

void test(){
  motor1.run(-motorSpeed*compsx/100);
  motor2.run(motorSpeed*compdx/100);
  Serial.print("M1 : ");
  Serial.println(analogRead(M1));
  Serial.print("M2 : ");
  Serial.println(analogRead(M2));
  delay(2000);
  motor1.stop();
  motor2.stop();
   Serial.print("M1 : ");
  Serial.println(analogRead(M1));
  Serial.print("M2 : ");
  Serial.println(analogRead(M2));
  delay(2000);
  }
void loop()
{
test(); //CONTINUA A CICLARE
}
