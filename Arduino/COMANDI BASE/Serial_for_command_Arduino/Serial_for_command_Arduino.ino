
#include "MeOrion.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// The string read and use for the comparison
String data, Start, Stop, Reset; 
int n;
void setup(){
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial.flush();
  while (!Serial) {
    delay(10);  // wait for serial port to connect. Needed for native USB port only
  }

  Start = String("Start");
  Stop = String("Stop");
  Reset = String("Reset");
  n = 0;
}

void loop() { // run over and over
   
  while(1){
    if (Serial.available() > 0) {
      break;
    }
  }
  
  // The Arduino attend an instruction from the raspberry 
  if (Serial.available() > 0) {
    // read the incoming byte:
    data = Serial.readStringUntil('\n'); // Read all the data and stops when there is a new line character
    Serial.print(data);
    Serial.print(" ");
    Serial.print(n);
    Serial.print(" ");
    // 
    if(data == Calibrate){
      
    }

    if (data == Start) {
      Serial.println("dentro al if");
    } else if (data == Stop){ 
      Serial.println(data);
    } else if (data == Reset){
      Serial.println(data);
    }
  n++;
}
 
 

}



















