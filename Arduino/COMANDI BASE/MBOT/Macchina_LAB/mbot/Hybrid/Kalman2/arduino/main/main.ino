#include "MeOrion.h"
#include "myIMU.h"
#include <Wire.h>
#include <EEPROM.h>

#define CAL_NUM 500       //number of measures to calibrate the accx and accy
#define BUFF_L 52
#define IMU_TIME 20      //ms of the gyro period (1/f)
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
long t;
bool send_imu_period = false;    //send the h command with the period of the update function
bool send_imu = false;           //send gyro data
bool send_idle = false;
bool SET_RUN = false;
bool SET_LEFT = false;
bool SET_RIGHT = false;
float start_time = 0;
float end_time = 0;
double accXrob, accYrob, accZrob; //accelerations in the robot frame
double aSensitivity =  16384;               // gyro class return acc as LSB at +-2g: with this sensitivity factor get the value in g
double accXOffset, accYOffset, angleOffset; // offset of the accelerations
float count_cmd; 
float val1, val2;
//----------------main structs------------------------------------------------
struct MotorStruct 
{   int vr; 
    int vl;   } s_Motors = {0, 0};

struct ParamStruct
{   int k;  
    int s;
    int c;
    int a;   } s_Param = {0, 0, 0, 0};
    
struct RightVelStruct
{   int kr;  
    int sr;
    int cr;
    int ar;   } s_Right = {0, 0, 0, 0};
    
struct LeftVelStruct
{   int kl;  
    int sl;
    int cl;
    int al;   } s_Left = {0, 0, 0, 0};
    
//conversion from byte to float/long
union 
{   byte byteVal[4];
    float floatVal;
    long longVal;   } val;

//---------------initialization functions-------------------------------------
void set_Parameters(int k, int s, int c, int a) 
{   s_Param.k = k;
    s_Param.s = s;
    s_Param.c = c;   
    s_Param.a = a;   }
    
void set_RightVel(int k, int s, int c, int a) 
{   s_Right.kr = k;
    s_Right.sr = s;
    s_Right.cr = c;   
    s_Right.ar = a;   }
    
void set_LeftVel(int k, int s, int c, int a) 
{   s_Left.kl = k;
    s_Left.sl = s;
    s_Left.cl = c;   
    s_Left.al = a;   }
    
void set_RGB(int r, int g, int b) 
{   led.setColor(r,g,b);
    led.show();   }
    
//set motor values to apply
void set_Motors(float r, float l) 
{   s_Motors.vr = r;
    s_Motors.vl = l;  }

//initialization function for setup
void init_f() 
{   set_RGB(100, 0, 0);
    set_Motors(0, 0);
    set_Parameters(0, 0, 0, 0);
    set_RightVel(0, 0, 0, 0);
    set_LeftVel(0, 0, 0, 0);
    update_Motors();   }

//send error code
void send_ErrorCode(char r) 
{   Serial.print("o");
    Serial.println(r);   }


//-----------------setup functions--------------------------------------------

//calibrate gyro
void eval_Acc() 
{   //imu.begin();
    set_RGB(100, 100, 0);
    double xSum = 0, ySum = 0, angSum = 0;
    for (int i = 0; i < CAL_NUM; i++) 
    {   imu.fast_update();
        xSum += imu.getAccX();
        ySum += imu.getAccY();
        angSum += imu.getAngleX();   }
        
    accXOffset = xSum / CAL_NUM;
    accYOffset = ySum / CAL_NUM;
    angleOffset = angSum / CAL_NUM;   }

//update the motors speeds
void update_Motors() 
{   set_RGB(50, 100, 0);
    motor_l.run(-s_Motors.vl);
    motor_r.run(s_Motors.vr);   }

//send by serial the bot name (retrieved from eeprom)
void send_Name() 
{   Serial.print('n');
    for (int i = 0; i < name.length(); i++)
    {   Serial.print(char(name[i]));   }
    //Serial.println(); // 'n'+ name -> n is the command identifier
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
    val.byteVal[3] = buff[idx + 3];
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
    send_3Floats('d', accYrob , imu.getGyroZ(), accXrob);   }
 
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
    {   case 'a':
            count_cmd = count_cmd + 1;
            s_Param.k = read_Float(1);
            set_RightVel(s_Param.k, s_Param.s, s_Param.c, s_Param.a);
            break;
            
        case 'b':
            s_Param.k = read_Float(1);
            set_LeftVel(s_Param.k, s_Param.s, s_Param.c, s_Param.a);
            break;
            
        case 'd': 
            if (buff[1] == 'a')   
            {   s_Param.a = read_Float(2);
                set_RGB(0, 0, 25);
                break;   }
            if (buff[1] == 'k')   
            {   s_Param.k = read_Float(2);
                set_RGB(0, 0, 50);
                break;   }
            if (buff[1] == 's')   
            {   s_Param.s = read_Float(2);
                set_RGB(0, 0, 75);
                break;   }
            if (buff[1] == 'c')   
            {   s_Param.c = read_Float(2); 
                set_RGB(0, 0, 100);
                break;   }
     
        case 'c': // calibrate acc
            imu.begin();
            eval_Acc();
            start_time = millis();
            set_RGB(0, 0, 100);
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
            //Serial.println('n'+name);
            break;
            
        case 'g': //change color led(debug
            set_RGB(100, 0, 0);
            send_imu_period = true;
            break;
            
        case 'p': // reply to the server with timestamp
            Serial.println('p');
            set_RGB(100, 0, 100);
            break;
        
        case 's': // stop bot motion (reset motor to zero)
            init_f();
            SET_RUN = false;
            break;
            
        case 'm': //send acquisition interval 
            if (buff[1] == 's') 
            {   start_time = millis();
                count_cmd = 0;  }
            else if (buff[1] == 'e') 
            {    end_time = millis();
                send_Float('t', end_time - start_time);
                send_Float('w', count_cmd);   }
            break;
            
        case 'r': // run bot motion
            SET_RUN = true;
            break;

        case 't': //set left wheel speed
            set_RGB(50, 50, 0);
            set_LeftVel(s_Param.k, s_Param.s, s_Param.c, s_Param.a);
            break;   
      
        case 'v': //set right wheel speed
            set_RGB(50, 50, 0);
            set_RightVel(s_Param.k, s_Param.s, s_Param.c, s_Param.a);
            break;   

        default:
            send_ErrorCode(' ');   }
            
    clear_Buff();   }


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


//-------------- AQUISITION SETUP AND LOOP ----------------------------------------------------------------------       
void setup() 
{   led.setpin(13);  
    init_f();
    Serial.begin(9600);
    imu.begin();
    eval_Acc();             //calibrate sensor
    update_Motors();
    imu_time = millis();
    set_RGB(0, 0, 0);   }

void loop() 
{   if (millis() - imu_time >= IMU_TIME)   //sync acquisition
    {   if (send_imu_period ) 
        {   send_Float('h', millis() - imu_time); 
            send_imu_period = false;  }
        
        imu_time = millis();
        if (send_imu) 
        {   send_IMU();   }    }
        
    t = millis()/5;
    if (SET_RUN == true)
    {   //float v_left = s_Left.kl*pow(t,s_Left.al)+s_Left.sl*sin(2*3.14*t)+s_Left.cl*cos(2*3.14*t);  
        //float v_right = s_Right.kr*pow(t,s_Right.ar)-s_Right.sr*sin(2*3.14*t)+s_Right.cr*cos(2*3.14*t);  
        set_Motors(s_Right.kr,s_Left.kl);
        update_Motors();   }
    
    parse_Serial();    }
