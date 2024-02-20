

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

    /*  CODICE TASTI
 *  A   69
 *  B   70
 *  C   71
 *  D   68
 *  E   67
 *  F   13
 *  su  64
 *  giu 25
 *  sx  7
 *  dx  9
 *  imp 21
 *  0   22
 *  1   12
 *  2   24
 *  3   94
 *  4   8
 *  5   28
 *  6   90
 *  7   66
 *  8   82
 *  9   74
 */
 
MeDCMotor motor1(M1);    //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

MeUltrasonicSensor ultraSensor(PORT_3);
MeRGBLed rgbled_7(7, 2);
MeBuzzer buzzer;
MeIR ir;
Me7SegmentDisplay disp(PORT_2);


void setup() {
  rgbled_7.fillPixelsBak(0, 2, 1);
  ir.begin();
  
  Serial.begin(115200);

rgbled_7.setColor(1,0,0,0);
rgbled_7.setColor(2,0,0,0);
rgbled_7.show();

}

void loop() {
    rgbled_7.setColor(1,255,0,0);//destro
    rgbled_7.setColor(2,0,0,0);//sinistro
    rgbled_7.show();
    delay(200);

if(ir.keyPressed(69)){//PREMENDO TAST A ENTRI IN MANUALE
  int movement = 0;
  while(1){
    if(ir.keyPressed(70)){//PREMENDO TAST B ESCI DA MANUALE
      break;
    }
    if(ir.keyPressed(22)){//PREMENDO TAST 0 STOP MOTORI
      movement = 0;
      }
    if(ir.keyPressed(64)){//PREMENDO TAST su AVANTI
      movement = 1;
       }
    if(ir.keyPressed(25)){//PREMENDO TAST giu INDIETRO
      movement = 2;
       }     

     switch (movement) {
        case 0:
          rgbled_7.setColor(1,0,0,0);
          rgbled_7.setColor(2,0,0,0);
          rgbled_7.show();
          break;
        case 1:
          rgbled_7.setColor(1,255,0,0);
          rgbled_7.setColor(2,255,0,0);
          rgbled_7.show();
          break; 
        case 2:
          rgbled_7.setColor(1,0,0,60);
          rgbled_7.setColor(2,0,0,60);
          rgbled_7.show();
          break;
     }         
   }
  }
}
