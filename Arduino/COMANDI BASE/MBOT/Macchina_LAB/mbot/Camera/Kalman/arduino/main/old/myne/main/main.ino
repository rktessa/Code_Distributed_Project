#include "MeOrion.h"
#include "gyroAcc.h"
#include <Wire.h>
#include <EEPROM.h>

#define CAL_NUM 500       //number of measures to calibrate the accx and accy
#define BUFF_L 52
#define GYRO_TIME 30      //ms of the gyro period (1/f)
#define OMEGA_FACTOR 1    //multiply the value from serial to adjust the motors speed

//define obj to br used
MeRGBLed led(0, 30);
MeDCMotor motors;
GyroAcc gyro;

//main variables
String name = "Brazorf";
long time_counter = 0;
byte buff[52];                    //serial read buffer
byte rec_byte;
bool start_s_buff = false;        //buffer parser flag
int  nbuf = 0;                    //position in the buffer
bool echo = false;                //echo received data to the serial (flag)
long gyro_time;
bool send_gyro_period = false;    //send the h command with the period of the update function
bool send_gyro = false;           //send gyro data
bool send_idle = false;
double accXrob, accYrob, accZrob; //accelerations in the robot frame
double aSensitivity =  16384;               // gyro class return acc as LSB at +-2g: with this sensitivity factor get the value in g
double accXOffset, accYOffset, angleOffset; // offset of the accelerations

//main structs--------------------------------
struct RGBLedStruct {
  int r;
  int g;
  int b;
} s_RGBLed = {0, 0, 0};

struct MotorStruct {
  int vr; //right wheel speed
  int vl; //left wheel speed
  int v;  //forward speed
  int w;  //angular speed
} s_Motors = {0, 0};

//conversion from byte to float/long
union {
  byte byteVal[4];
  float floatVal;
  long longVal;
}val ;

//initialization functions------------------------

//save led colors to apply
void setRGB(int r, int g, int b) { 
  s_RGBLed.r = r;
  s_RGBLed.g = g;
  s_RGBLed.b = b;
}
//set motor values to apply
void setMotors(int r, int l) { 
  s_Motors.vr = r;
  s_Motors.vl = l;
  s_Motors.v = 0;
  s_Motors.w = 0;
}

//initialization function for setup
void init_f() {
  setRGB(0, 100, 0);
  setMotors(0, 0);
}

//send error code
void sendErrorCode(char r) { 
  Serial.print("o");
  Serial.println(r);
}

//-------setup functions-------------------------

//calibrate gyro
void calAcc() {
  gyro.begin(); 
  double xSum = 0, ySum = 0, angSum = 0;
  for (int i = 0; i < CAL_NUM; i++) {
    gyro.fast_update();
    xSum += gyro.getAccX();
    ySum += gyro.getAccY();
    angSum += gyro.getAngleX();
  }
  accXOffset = xSum / CAL_NUM;
  accYOffset = ySum / CAL_NUM;
  angleOffset = angSum / CAL_NUM;
}

void updateLed() { 
  led.setColor(s_RGBLed.r, s_RGBLed.g, s_RGBLed.b);
  led.show();
}

//update the motors speeds
void updateMotors() { 
  motors.reset(M1);
  motors.run(-s_Motors.vl);
  motors.reset(M2);
  motors.run(s_Motors.vr);
}

//send by serial the bot name (retrieved from eeprom)
void sendName() { 
  Serial.print('n');
  for (int i = 0; i < name.length(); i++)
    Serial.print(char(name[i]));
  setRGB(100, 100, 0);
  led.show();
  Serial.println(); // 'n'+ name -> n is the command identifier
  delay(5);
}

//----main aquisition functions----------------------

//send float value with its identifier
void sendFloat(char c, float value) { 
  Serial.write('&'); // identify is a float and not a string
  Serial.write(c);
  val.floatVal = value;
  Serial.write(val.byteVal[0]);
  Serial.write(val.byteVal[1]);
  Serial.write(val.byteVal[2]);
  Serial.write(val.byteVal[3]);
  Serial.println();
}

// send identifier and 3 floats (1+1+12 bytes)
void send3Floats(char c, float value1, float value2, float value3) { 
  Serial.write('&');// identify is a float and not a string
  Serial.write(c);
  val.floatVal = value1;
  Serial.write(val.byteVal[0]);
  Serial.write(val.byteVal[1]);
  Serial.write(val.byteVal[2]);
  Serial.write(val.byteVal[3]);
  val.floatVal = value2;
  Serial.write(val.byteVal[0]);
  Serial.write(val.byteVal[1]);
  Serial.write(val.byteVal[2]);
  Serial.write(val.byteVal[3]);
  val.floatVal = value3;
  Serial.write(val.byteVal[0]);
  Serial.write(val.byteVal[1]);
  Serial.write(val.byteVal[2]);
  Serial.write(val.byteVal[3]);
  Serial.println();
}

//return a float given the start position in the buffer
float readFloat(int idx) { 
  val.byteVal[0] = buff[idx];
  val.byteVal[1] = buff[idx + 1];
  val.byteVal[2] = buff[idx + 2];
  val.byteVal[3] = buff[idx + 3];
  return val.floatVal;
}

//acquire and preelabotare the data
void getAcc() { 
  gyro.update();
  accXrob = (gyro.getAccX() - accXOffset) / aSensitivity;
  accYrob = (gyro.getAccY() - accYOffset) / aSensitivity;
  accZrob = (gyro.getAccZ()) / aSensitivity;
}

//send the gyro data
void sendGyro() { 
  getAcc();
  send3Floats('d', accYrob , gyro.getGyroZ(), gyro.getAngleX());
  //send3Floats('d', accYrob, gyro.getAngleZ(), gyro.getAngleX()-angleOffset);
}

//calculare motors value
void calcMotors() { 
  s_Motors.vl = s_Motors.v - s_Motors.w * OMEGA_FACTOR;
  s_Motors.vr = s_Motors.v + s_Motors.w * OMEGA_FACTOR;
}

//---serial parser---------------------------------

//clear serial buffer
void clearBuff() { 
  for (int i = 0; i < nbuf; i++)
    buff[i] = 0;
  nbuf = 0;
}

// parse the serial buffer at the end of a command
void parseBuff() { 
  if (echo) { // send back what has received if echo is true (usefull for debug purpose)
    for (int i = 0; i < nbuf; i++) {
      Serial.print(char(buff[i]));
    }
    Serial.println('_'); //used to distinguish from echo and not echo messages
  }

  /*
    buff[0]-> command identifier
    buff[1-4] -> float bytes if necessary
  */

  switch (buff[0]) {
    case 'c': // calibrate acc
      calAcc();
      break;
    case 'e': // enable or disable serial echo
      if (buff[1] == '1')
        echo = true;
      else if (buff[1] == '0')
        echo = false;
      else sendErrorCode('e');// cannot say if need to set echo on or off
      break;
    case 'h': // send or not the gyro freq
      send_gyro_period = !send_gyro_period;
      break;
    case 'l': // toggle send gyro
      send_gyro = !send_gyro;
      break;
    case 'n': // send the name if requested
      sendName();
      break;
    case 'g':
      setRGB(100, 0, 0);
      led.show();
      break;
    case 'p': // reply to the server with timestamp
      Serial.println('p');
      break;
    case 't':
      {
        float w = readFloat(1);
        //remove values out of a reasonable range (means that there is a communication error). Keep the 0 value
        if ((abs(w) >= 0.01 && abs(w) <= 500) ) { 
          //sendFloat('w', w);
          //needed to impose 500 as 0 because sometimes get 0 even if is not 0 the value wanted. In this way is more robust to comunication errors
          if (w == 500) 
            w = 0;
          s_Motors.w = w;
          calcMotors();
        } else //sendErrorCode('t');
          Serial.println('otout');
      }
      break;
    case 'v':
      {
        float v = readFloat(1);
        if ((abs(v) >= 0.01 && abs(v) <= 500)) { //remove values out of a reasonable range (means that there is a communication error). Keep the 0 value
          //sendFloat('v', v);
          if (v == 500) //needed to impose 500 as 0 because sometimes get 0 even if is not 0 the value wanted. In this way is more robust to comunication errors
            v = 0;
          s_Motors.v = v;
          calcMotors();
        } else //sendErrorCode('v');
          Serial.println('ovout');
        //send_idle = true;
      }
      break;
    default:
      sendErrorCode(' ');

  }
  clearBuff();
}

void parseSerial() {
  /*
    message composition
        1   2   3-4-5-6                     3-n                 last
    byte    '%'   letter  bytes                     bytes               '$'
    meaning   Start   Command Float bytes if the command is linked to a value ASCII of chars if message is string End command
  */
  while (Serial.available() > 0 && nbuf < BUFF_L) {
    rec_byte = Serial.read();
    //Serial.println(rec_byte); // the error is in the transmission or the pc side because here the buffer is used correctly

    if (rec_byte == (byte)'$') { //check for end of command
      start_s_buff = false;
      parseBuff();
    }

    if (start_s_buff) { // add byte to the buffer
      if (rec_byte != (byte)'\n' && rec_byte != (byte)'\r') // filter \n and \r from the command
        buff[nbuf++] = rec_byte;
    }

    if (rec_byte == (byte)'%') { // check for start of command
      if (start_s_buff) // if receive a new command erase the last command
        clearBuff();
      start_s_buff = true;
    }
  }
}


//-------------AQUISITION SETUP---------------------- 
void setup() {
  init_f();
  Serial.begin(9600);
  gyro.begin();
  calAcc();             //calibrate sensor
  led.setpin(13);
  updateLed();
  gyro_time = millis();
  
}

//-------------Aquisition LOOP-----------------------
void loop() {
  if (millis() - gyro_time >= GYRO_TIME) {  //sync acquisition
    if (send_gyro_period ) {
      sendFloat('h', millis() - gyro_time);
    }
    gyro_time = millis();
    if (send_gyro) sendGyro();
  }
  updateLed();
  updateMotors();
  
  parseSerial();
}
