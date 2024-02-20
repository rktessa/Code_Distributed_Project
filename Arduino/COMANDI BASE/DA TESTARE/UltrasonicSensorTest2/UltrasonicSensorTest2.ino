

//VERIFICARE PRIMA Start_ButtonTest2

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

int buttonstate =0;

MeUltrasonicSensor ultraSensor(PORT_3);

void setup() {
  pinMode(A7, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);


}


void loop() {
//attivazione pulsante
  if(analogRead(A7) < 10){
    buttonstate = 1;
    }

//esecuzione
  while(buttonstate == 1){

    if(ultraSensor.distanceCm() > 10){//CONTROLLA CHE SIA IL LIMITE CORRETTO in teoria sono 10 cm

          digitalWrite(LED_BUILTIN, HIGH);

    }else{
          for(int i = 2; i < 4; i++) {
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          }
    }
      
    }
}
    
   
