

//TESTATO OK
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

int buttonstate =0;
uint8_t motorSpeed = 100; /* value: between -255 and 255. */
int compsx = 100;
int compdx = 110;
int steeringAngle = 30; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int obstacle =1;
int resto = 0;
int rnd = 1;

MeDCMotor motor1(M1);//CONTROLLA PORTA CON TEST PRECEDENTE
                         //DEVE ESSERE MOTORE SINISTRO
MeDCMotor motor2(M2);//CONTROLLA PORTA CON TEST PRECEDENTE
                         //DEVE ESSERE MOTORE DESTRO
MeUltrasonicSensor ultraSensor(PORT_3); //CONTROLLA PORTA CON TEST PRECEDENTE

/*
 * 
int anArray[20];  //an array capable of holding 20 entries numbered 0 to 19
byte arrayIndex = 0;
anArray[arrayIndex] = 123;  //put a value in entry 0
arrayIndex++;  //increment the array index
anArray[arrayIndex] = 456;  //put a value in entry 1
//etc, etc
 */

 
void setup() {
  pinMode(A7, INPUT);
  motor1.stop();
  motor2.stop();
}


void loop() {
//attivazione pulsante
  if(analogRead(A7) < 10){
    buttonstate = 1;
    }

//esecuzione
  while(buttonstate == 1){

    if(ultraSensor.distanceCm() > 20){
         //avanti
         motor1.run(-motorSpeed*compsx/100);
         motor2.run(motorSpeed*compdx/100);
         obstacle =1;
    }else{
          motor1.stop();
          motor2.stop();
          delay(500);
          //tentativo rotazione
          resto = obstacle%2;
          if(resto == 0){
          motor1.run(-motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(10*steeringAngle*obstacle);
          }else{
          motor1.run(motorSpeed*compsx/100);
          motor2.run(motorSpeed*compdx/100);
          delay(10*steeringAngle*obstacle);
          }
          obstacle++;

          //riposizionamento, indietro e rotazione
          if(obstacle == 8){
            motor1.stop();
            motor2.stop();
            delay(200);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(10*steeringAngle*obstacle/2);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(2000);
            rnd = random(1,3);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(10*steeringAngle*rnd);
            obstacle =1;
            }
          }
      
   }
}
    
   
