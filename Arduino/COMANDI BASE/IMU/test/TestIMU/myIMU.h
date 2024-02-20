//Define to prevent recursive inclusion
#ifndef myIMU_h
    #define myIMU_h

    //include libraries
    #include <stdint.h>
    #include <stdbool.h>
    #include <Arduino.h>
    #include <MeConfig.h>
    
    #ifdef ME_PORT_DEFINED
        #include <MePort.h>
    #endif //

    //Exported macro
    #define I2C_ERROR                  (-1)
    #define IMU_DEFAULT_ADDRESS       (0x68)


    #ifndef ME_PORT_DEFINED
        class IMU
    #else // 
        class IMU : public MePort
    #endif // 

    {
        public:
            //possible different constructors
            #ifdef ME_PORT_DEFINED
                //function to map the MeCompass to arduino port
                IMU(void);
                //PWM frequency set to 976 Hz, (port - RJ25 port from PORT_1 to M2)
                IMU(uint8_t port);
                //change the i2c device address, (address - the i2c address you want to set)
                IMU(uint8_t port, uint8_t address);
            #else
                //function to map the _AD0 and _INT to arduino port (_AD0 - arduino gpio number,_INT - arduino gpio number)
                IMU(uint8_t AD0, uint8_t INT);
                //change the i2c device address, address - (the i2c address you want to set)
                IMU(uint8_t AD0, uint8_t INT, uint8_t address)
            #endif  
            
            //Set the PIN of the button module.(AD0 - pin mapping for arduino,INT - pin mapping for arduino)
            void setpin(uint8_t AD0, uint8_t INT);

            //Initialize a new IMU instance
            void begin();
            
            //Update some calculated angle values to the variable
            //The angle values are calculated by complementary filter.
            //time constant of filter is set to 0.5 second, but period dt is not a constant, filter coefficient will be calculated dynamically.
            void update(void);

            //Fast update some calculated angle values to the variable.
            void fast_update(void);

            //Get the device address of Gyro.
            uint8_t getDevAddr(void);

//----------gyro angles evaluation (calculated by complementary filter)----------------------------------------

            //Get the angle value of X-axis.
            double getAngleX(void);
            
            //Get the angle value of Y-axis.
            double getAngleY(void);

            //Get the angle value of Z-axi(Z-axis angle value is integral of Z-axis angular velocity)
            double getAngleZ(void);

            //Get the data of gyroXrate
            double getGyroX(void);

            //Get the data of gyroYrate
            double getGyroY(void);

            //Get the data of gyroZrate
            double getGyroZ(void);

            //Get the data of gyroXrate
            double getGyroX1(void);

            //Get the data of gyroYrate
            double getGyroY1(void);

            //Get the data of gyroZrate
            double getGyroZ1(void);

            //Get the angle value of specified setting axis (index --> Axis settings(1:X-axis, 2:Y-axis, 3:Z-axis))
            double getAngle(uint8_t index);

//----------accelerometer acc evaluation------------------------------------------------------------------------

            //Get the acc value of X-axis
            int16_t getAccX(void);
            
            //Get the acc value of Y-axis
            int16_t getAccY(void);

            //Get the acc value of Z-axis
            int16_t getAccZ(void);
            
                        //Get the acc value of X-axis
            int16_t getAccX1(void);
            
            //Get the acc value of Y-axis
            int16_t getAccY1(void);

            //Get the acc value of Z-axis
            int16_t getAccZ1(void);
        
        private:
            volatile uint8_t  _AD0;
            volatile uint8_t  _INT;
            double  gSensitivity;      /* for 500 deg/s, check data sheet */
            double  gx, gy, gz;
            double  gyrX, gyrY, gyrZ;
            double  gyrX1, gyrY1, gyrZ1;
            int16_t accX, accY, accZ;
            int16_t accX1, accY1, accZ1;
            double  gyrXoffs, gyrYoffs, gyrZoffs;
            uint8_t i2cData[14];
            uint8_t i2cData2[14];
            uint8_t Device_Address;

            //Calibration function for the IMU
            void deviceCalibration(void);
            
            //Write the registor of i2c device. (reg - the address of registor,data - the written to the registor)
            //Return the error code:the definition of the value of variable return_value:
            //0:success
            //1:BUFFER_LENGTH is shorter than size
            //2:address send, nack received
            //3:data send, nack received
            //4:other twi error
            int8_t writeReg(int16_t reg, uint8_t data);

            //Read the data from i2c serial device
            //(start - address which will read the data from, size - set the number of data will be red from buffer)
            int8_t readData(uint8_t start, uint8_t *buffer, uint8_t size);

            //Write the data to i2c serial device.
            //(start - address which will write the data to, pData -head address of data array, size - set the number of data will be written)
            //Calling the official i2c library to write data
            int8_t writeData(uint8_t start, const uint8_t *pData, uint8_t size);
            
    };
    
#endif //  MeGyro_H
        
