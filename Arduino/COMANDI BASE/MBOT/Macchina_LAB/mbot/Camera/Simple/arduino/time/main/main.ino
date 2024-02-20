#include "MeOrion.h"
#include "myIMU.h"
#include <Wire.h>
#include <EEPROM.h>

#define CAL_NUM 500       //number of measures to calibrate the accx and accy
#define BUFF_L 52
#define IMU_TIME 30      //ms of the gyro period (1/f)
#define OMEGA_FACTOR 1    //multiply the value from serial to adjust the motors speed

//define obj to br used
MeRGBLed led(0, 30);
MeDCMotor motor_l(M1);
MeDCMotor motor_r(M2);
IMU imu;

//-----------main variables-----------------------------------------------
String name = "Brazorf";
long time_counter = 0;
byte buff[52];                    //serial read buffer
byte rec_byte;
bool start_s_buff = false;        //buffer parser flag
int  nbuf = 0;                    //position in the buffer
bool echo = false;                //echo received data to the serial (flag)
long imu_time;
bool send_imu_period = false;    //send the h command with the period of the update function
bool send_imu = false;           //send gyro data
bool send_idle = false;
double accXrob, accYrob, accZrob; //accelerations in the robot frame
double aSensitivity =  16384;               // gyro class return acc as LSB at +-2g: with this sensitivity factor get the value in g
double accXOffset, accYOffset, angleOffset; // offset of the accelerations

int k = 0;
int a = 0;
int s = 0; 
int c = 0; 

//----------------main structs------------------------------------------------
struct RGBLedStruct 
{   int r;
    int g;
    int b;   } s_RGBLed = {0, 0, 0};

//(v: forwars speed, w: angular speed)
struct MotorStruct 
{   int vr; 
    int vl; 
    int v_abs;  
    int v_rel;   } s_Motors = {0, 0};

//conversion from byte to float/long
union 
{   byte byteVal[4];
    float floatVal;
    long longVal;   } val;

//---------------initialization functions-------------------------------------

//save led colors to apply
void set_RGB(int r, int g, int b) 
{   s_RGBLed.r = r;
    s_RGBLed.g = g;
    s_RGBLed.b = b;   }

//set motor values to apply
void set_Motors(int r, int l) 
{   s_Motors.vr = r;
    s_Motors.vl = l;
    s_Motors.v_abs = 0;
    s_Motors.v_rel = 0;   }

//initialization function for setup
void init_f() 
{   set_RGB(0, 100, 0);
    set_Motors(0, 0);
    update_Motors();   }

//send error code
void send_ErrorCode(char r) 
{   Serial.print("o");
    Serial.println(r);   }


//-----------------setup functions--------------------------------------------

//calibrate gyro
void eval_Acc() 
{   imu.begin(); 
    double xSum = 0, ySum = 0, angSum = 0;
    for (int i = 0; i < CAL_NUM; i++) 
    {   imu.fast_update();
        xSum += imu.getAccX();
        ySum += imu.getAccY();
        angSum += imu.getAngleX();   }
        
    accXOffset = xSum / CAL_NUM;
    accYOffset = ySum / CAL_NUM;
    angleOffset = angSum / CAL_NUM;   }

void update_Led() 
{   led.setColor(s_RGBLed.r, s_RGBLed.g, s_RGBLed.b);
    led.show();   }

//update the motors speeds
void update_Motors() 
{   set_RGB(0, 100, 100);
    motor_l.run(-s_Motors.vl);
    motor_r.run(s_Motors.vr);   }

//send by serial the bot name (retrieved from eeprom)
void send_Name() 
{   Serial.print('n');
    for (int i = 0; i < name.length(); i++)
    {   Serial.print(char(name[i]));   }
    set_RGB(50, 100, 0);
    led.show();
    Serial.println(); // 'n'+ name -> n is the command identifier
    delay(5);   }


//----------------main aquisition functions-------------------------------------

//send float value with its identifier
void send_Float(char c, float value) 
{   Serial.write('&'); //identify is a float and not a string
    Serial.write(c);
    val.floatVal = value;
    Serial.write(val.byteVal[0]);
    Serial.write(val.byteVal[1]);
    Serial.write(val.byteVal[2]);
    Serial.write(val.byteVal[3]);
    Serial.println();   }

// send identifier and 3 floats (1+1+12 bytes)
void send_3Floats(char c, float value1, float value2, float value3) 
{   Serial.write('&'); //identify is a float and not a string
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
    Serial.println();   }

//return a float given the start position in the buffer
float read_Float(int idx) 
{   val.byteVal[0] = buff[idx];
    val.byteVal[1] = buff[idx + 1];
    val.byteVal[2] = buff[idx + 2];
    return val.floatVal;   }
    
float read_Float3(int idx) 
{   val.byteVal[0] = buff[idx];
    return val.floatVal;   }

float read_Float2(int idx) 
{   val.byteVal[0] = buff[idx];
    val.byteVal[1] = buff[idx + 1];
    val.byteVal[2] = buff[idx + 2];
    val.byteVal[3] = buff[idx + 3];
    Serial.print(val.floatVal);   }

//acquire and preelabotare the data
void get_Acc() 
{   imu.update();
    accXrob = (imu.getAccX() - accXOffset) / aSensitivity;
    accYrob = (imu.getAccY() - accYOffset) / aSensitivity;
    accZrob = (imu.getAccZ()) / aSensitivity;   }

//send the gyro data
void send_IMU() 
{   get_Acc();
    send_3Floats('d', accYrob , imu.getGyroZ(), imu.getAngleZ());   }
 
//--------------serial parser functions----------------------------------------

//clear serial buffer
void clear_Buff() 
{   for (int i = 0; i < nbuf; i++)
    {   buff[i] = 0;  }
    nbuf = 0;   }

// parse the serial buffer at the end of a command
void parse_Buff() 
{   if (echo) { // send back what has received if echo is true (usefull for debug purpose)
        for (int i = 0; i < nbuf; i++) 
        {   Serial.print(char(buff[i]));   }
    //used to distinguish from echo and not echo messages    
    Serial.println('_');   }
    
    //buff[0]-> command identifier, buff[1-4] -> float bytes if necessary
    switch (buff[0]) 
    {   case 'c': // calibrate acc
            eval_Acc();
            break;
            
        case 'e': // enable or disable serial echo
            if (buff[1] == '1')
                echo = true;
            else if (buff[1] == '0')
                echo = false;
            else send_ErrorCode('e');// cannot say if need to set echo on or off
            break;
            
        case 'h': // send or not the gyro freq
            send_imu_period = !send_imu_period;
            break;
            
        case 'l': // toggle send gyro
            send_imu = !send_imu;
            break;
            
        case 'n': // send the name if requested
            send_Name();
            break;
            
        case 'g': //change color led(debug
            set_RGB(100, 0, 0);
            led.show();
            break;
            
        case 'p': // reply to the server with timestamp
            Serial.println('p');
            break;
        
        case 's': // stop bot motion
            init_f();
            break;

        case 'r': // run bot motion
            update_Motors();
            break;

        case 't': 
            {   k = read_Float(1);
                s = read_Float(4);
                c = read_Float(7);
                a = read_Float3(10);
                if( k == 100 or a == 3 or s == 127 or c == 200)
                {   set_RGB(0, 50, 100);  }
                float v_left = k*pow(t,a)+s*sin(DEG_TO_RAD*t)+c*cos(DEG_TO_RAD*t);
                if (abs(v_left)>=1 && abs(v_left)<=250)
                {   set_RGB(100, 0, 100);
                    s_Motors.vl = v_left;  }
                else
                {   Serial.println();   }
                break;   }
      
        case 'v': 
            {   k = read_Float(1);
                s = read_Float(4);
                c = read_Float(7);
                a = read_Float3(10);
                if( k == 100 or a == 3 or s == 127 or c == 200)
                {   set_RGB(0, 50, 100);  }             
                float v_right = k*pow(t,a)+s*sin(DEG_TO_RAD*t)+c*cos(DEG_TO_RAD*t);
                if (abs(v_right)>=1 && abs(v_right)<=250)
                {   set_RGB(50, 10, 50);
                    s_Motors.vr = v_right;  }                    
                else
                {   Serial.println();  }
                break;  }

        default:
            send_ErrorCode(' ');   }
            
    clear_Buff();   }

/*
  message composition:
    1        2       3-4-5-6                  3-n                 last
  byte      '%'     letter  bytes             bytes               '$'
  meaning   Start   Command Float bytes if the command is linked to a value ASCII of chars if message is string End command
*/

  
// parse the serial coomunication data 
void parse_Serial() 
{   while (Serial.available() > 0 & nbuf < BUFF_L) 
    {   rec_byte = Serial.read();
        //Serial.println(rec_byte); // the error is in the transmission or the pc side because here the buffer is used correctly

        if (rec_byte == (byte)'$')  //check for end of command
        {   start_s_buff = false;
            parse_Buff();   }

        if (start_s_buff)  // add byte to the buffer
        {   if (rec_byte != (byte)'\n' && rec_byte != (byte)'\r') // filter \n and \r from the command
            {   buff[nbuf++] = rec_byte;   }   }

        if (rec_byte == (byte)'%')  // check for start of command
        {   if (start_s_buff)       // if receive a new command erase the last command
            {   clear_Buff();    }
            start_s_buff = true;    }   }   }



//-------------- AQUISITION SETUP AND LOOP ----------------------------------------
            
void setup() 
{   init_f();
    Serial.begin(2400);
    imu.begin();
    eval_Acc();             //calibrate sensor
    led.setpin(13);
    update_Led();
    update_Motors();
    imu_time = millis();   }


void loop() 
{   if (millis() - imu_time >= IMU_TIME)   //sync acquisition
    {   if (send_imu_period ) 
        {   send_Float('h', millis() - imu_time);   }
        
        imu_time = millis();
        t = millis()/1000;
        if (send_imu) 
        {   send_IMU();   }    }
   
    update_Led();
    parse_Serial();    }
