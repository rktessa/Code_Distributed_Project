#include "myIMU.h"

//constructor list methods
#ifdef ME_PORT_DEFINED
    IMU::IMU(void) : MePort(0)
    {   Device_Address = IMU_DEFAULT_ADDRESS;  }

    IMU::IMU(uint8_t port) : MePort(port)
    {   Device_Address = IMU_DEFAULT_ADDRESS;  }

    IMU::IMU(uint8_t port, uint8_t address) : MePort(port)
    {   Device_Address = address;  }
    
#else
    IMU::IMU(uint8_t AD0, uint8_t INT)
    {   Device_Address = IMU_DEFAULT_ADDRESS;
        _AD0 = AD0;
        _INT = INT;  }

    IMU::IMU(uint8_t AD0, uint8_t INT, uint8_t address)
    {   Device_Address = address;
        _AD0 = AD0;
        _INT = INT;  }
#endif

void IMU::setpin(uint8_t AD0, uint8_t INT)
{   _AD0 = AD0;
    _INT = INT;
#ifdef ME_PORT_DEFINED
     s1 = AD0;
     s2 = INT;
#endif  
}

//-------------main methods------------------------------------------------------
void IMU::begin(void)
{   gSensitivity = 131; //for 500 deg/s, check data sheet
    gx = 0;
    gy = 0;
    gz = 0;
    gyrX = 0;
    gyrY = 0;
    gyrZ = 0;
    accX = 0;
    accY = 0;
    accZ = 0;
    gyrXoffs = 0;
    gyrYoffs = 0;
    gyrZoffs = 0;
    Wire.begin();
    delay(800);
    writeReg(0x6b, 0x00); //close the sleep mode
    writeReg(0x1a, 0x01); //configurate the digital low pass filter
    //writeReg(0x1b, 0x08); //set the gyro scale to 500 deg/s
  
    deviceCalibration();  }

void IMU::update(void)
{   static unsigned long  last_time = 0;
    int8_t return_value;
    double dt, filter_coefficient;
    double ax, ay, az;
    
    //read imu data 
    return_value = readData(0x3b, i2cData, 14);
    if(return_value != 0)
    {  return;  }
    
    // assemble 16 bit sensor data 
    accX = ( (i2cData[0] << 8) | i2cData[1] );
    accY = ( (i2cData[2] << 8) | i2cData[3] );
    accZ = ( (i2cData[4] << 8) | i2cData[5] );  
    gyrX = ( ( (i2cData[8] << 8) | i2cData[9] ) - gyrXoffs) / gSensitivity;
    gyrY = ( ( (i2cData[10] << 8) | i2cData[11] ) - gyrYoffs) / gSensitivity;
    gyrZ = ( ( (i2cData[12] << 8) | i2cData[13] ) - gyrZoffs) / gSensitivity;  
    ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;
    ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;  

    dt = (double)(millis() - last_time) / 1000;
    last_time = millis();

    if(accZ > 0)
    {   gx = gx - gyrY * dt;
        gy = gy + gyrX * dt;  }
    else
    {   gx = gx + gyrY * dt;
        gy = gy - gyrX * dt;  }
        
    gz += gyrZ * dt;
    gz = gz - 360 * floor(gz / 360);
    
    if(gz > 180)
    {   gz = gz - 360;  }
    
    // complementary filter-> set 0.5sec = tau = dt * A / (1 - A)
    // A = tau / (tau + dt)
    filter_coefficient = 0.5 / (0.5 + dt);
    gx = gx * filter_coefficient + ax * (1 - filter_coefficient);
    gy = gy * filter_coefficient + ay * (1 - filter_coefficient);   }


//alternative update method with static filter coeff instead of dynamic
void IMU::fast_update(void)
{   static unsigned long  last_time = 0;
    int8_t return_value;
    double dt, filter_coefficient;
    double ax, ay, az;
    
    dt = (double)(millis() - last_time) / 1000.0;
    last_time = millis();
  
    /* read imu data */
    return_value = readData(0x3b, i2cData, 14);
    if(return_value != 0)
    {  return;  }
  
    /* assemble 16 bit sensor data */
    accX = ( (i2cData[0] << 8) | i2cData[1] );
    accY = ( (i2cData[2] << 8) | i2cData[3] );
    accZ = ( (i2cData[4] << 8) | i2cData[5] );  
    gyrX = ( ( (i2cData[8] << 8) | i2cData[9] ) - gyrXoffs) / gSensitivity;
    gyrY = ( ( (i2cData[10] << 8) | i2cData[11] ) - gyrYoffs) / gSensitivity;
    gyrZ = ( ( (i2cData[12] << 8) | i2cData[13] ) - gyrZoffs) / gSensitivity;  
    ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;
    ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;  
  
    if(accZ > 0)
    {   gx = gx - gyrY * dt;
        gy = gy + gyrX * dt;  }
    else
    {   gx = gx + gyrY * dt;
        gy = gy - gyrX * dt;  }
        
    gz += gyrZ * dt;
    gz = gz - 360 * floor(gz / 360);
    
    if(gz > 180)
    {  gz = gz - 360;  }
    
    //use a fixed coeff for evaluation
    gy = 0.98 * gy + 0.02 * ay;
    gx = 0.98 * gx + 0.02 * ax;  }



uint8_t IMU::getDevAddr(void)
{   return Device_Address;   }

//---------gyro angle evaluation methods------------------------------------------ 
double IMU::getAngleX(void)
{   return gx;   }

double IMU::getAngleY(void)
{   return gy;   }

double IMU::getAngleZ(void)
{   return gz;   }

double IMU::getGyroX(void)
{   return gyrX;   }

double IMU::getGyroY(void)
{   return gyrY;   }

double IMU::getGyroZ(void)
{   return gyrZ;   } 

double IMU::getAngle(uint8_t index)
{   update();
    if(index == 1)
    {   return gx;   }
    else if(index == 2)
    {   return gy;   }
    else if(index == 3)
    {   return gz;   }   } 


//---------accelerometer acc evaluation methods-----------------------------------
int16_t IMU::getAccX(void)
{   return accX;   }
    
int16_t IMU::getAccY(void)
{   return accY;   }
    
int16_t IMU::getAccZ(void)
{   return accZ;   }

//gyro calibration method
void IMU::deviceCalibration(void)
{   int8_t return_value;
    uint16_t x = 0;
    uint16_t num = 500;
    long xSum = 0, ySum = 0, zSum = 0;
    for(x = 0; x < num; x++)
    {   return_value = readData(0x43, i2cData, 6);
        xSum += ( (i2cData[0] << 8) | i2cData[1] );
        ySum += ( (i2cData[2] << 8) | i2cData[3] );
        zSum += ( (i2cData[4] << 8) | i2cData[5] );  }
        
    gyrXoffs = xSum / num;
    gyrYoffs = ySum / num;
    gyrZoffs = zSum / num;   }


//-------rea/write serial methods--------------------------------------------------
int8_t IMU::writeReg(int16_t reg, uint8_t data)
{   int8_t return_value = 0;
    return_value = writeData(reg, &data, 1);
    return(return_value);   }

int8_t IMU::readData(uint8_t start, uint8_t *buffer, uint8_t size)
{   int16_t i = 0;
    int8_t return_value = 0;
    Wire.beginTransmission(Device_Address);
    return_value = Wire.write(start);
    
    if(return_value != 1)
    {   return(I2C_ERROR);   }
    
    return_value = Wire.endTransmission(false);
    
    if(return_value != 0)
    {   return(return_value);   }
    
    delayMicroseconds(1);
    //Third parameter is true: relase I2C-bus after data is read
    Wire.requestFrom(Device_Address, size, (uint8_t)true);
    
    while(Wire.available() && i < size)
    {   buffer[i++] = Wire.read();   }
    delayMicroseconds(1);
    
    if(i != size)
    {   return(I2C_ERROR);   }
    return(0);   }   //return(0): no error 

int8_t IMU::writeData(uint8_t start, const uint8_t *pData, uint8_t size)
{   int8_t return_value = 0;
    Wire.beginTransmission(Device_Address);
    return_value = Wire.write(start); 
    
    if(return_value != 1)
    {   return(I2C_ERROR);   }
    Wire.write(pData, size);  
    
    return_value = Wire.endTransmission(true); 
    return(return_value);   }//return: no error                     
