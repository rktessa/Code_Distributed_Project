// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>


MeDCMotor motor1(M1);     //SINISTRO -motorSpeed == AVANTI
MeDCMotor motor2(M2);    //DESTRO motorSpeed == AVANTI

int resamotori = 90;
int compsx = 100*(200-resamotori)/100;
int compdx = 99*(200-resamotori)/100;
float power = 50.0/100; // 50 % potenza
uint8_t motorSpeed = 255*power; /* value: between -255 and 255. */
float travel=100; //mm percorsi

//4.4 GIRI AL 60% PER 4 SECONDI = 1.83
//3.6 GIRI AL 50% PER 4 SECONDI = 1.8 giri/seconod al 100%
//1.8 GIRI AL 25 % PER 4 SECONDI = 1.8 giri/secondo al 100%
//DIAMETRO RUOTA 64 mm = CIRC 201 mm

// 362 mm/ secondo al 100 %
//test pavimento con peso batteria 1300 mm = 325 mm/secondo con resa 90%
// 7 IMPULSI DA 0.36 SECONDI PER COMPLETARE 360 GRADI AL 50% = 285 gradi/secondo al 100%
MeUltrasonicSensor ultraSensor(PORT_3);

int steeringAngle = 20; //gradi di rotazione DEVI VERIFICARE QUANTO CI METTE
int steeringTime = steeringAngle*1000.0/(280*power); //DEVE ESSERE INTERO E PARI
int backTime = 150*1000.0/(362*power); //RITORNO INDIETRO DI 100 mm 550
int stopTime = 200; //TEMPO TRA STOP E RIAVVIO MOTORI
int exitTime = 300*1000.0/(362*power); // USCITA INDIETRO DI 300 mm 2200
float cycleTime=0; //tempo per compiere un ciclo loop o tempo di campionatemento
int tempo = 0;
int obstacle =1;
int distStop =30;
int resto = 0;
int rnd = 1;

int n = 0;
long start=0;


Adafruit_MPU6050 mpu;
MeGyro gyro;

float accX1=0;
float accY1=0;
float accZ1=0;

float accX2=0;
float accY2=0;
float accZ2=0;

float accX3=0;
float accY3=0;
float accZ3=0;

float accX=0;
float accY=0;
float accZ=0;

float speX=0;
float speY=0;
float speZ=0;

float rotX=0;
float rotY=0;
float rotZ=0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  gyro.begin();
  
  pinMode(A7, INPUT);
  motor1.stop();
  motor2.stop();

  while(analogRead(A7) > 10){
    delay(100);
    }
}

/* FERMO
   *Acceleration X: 0.32, Y: 0.11, Z: 8.42 m/s^2
    Rotation X: -0.01, Y: -0.03, Z: -0.01 rad/s
    X:2.19 Y:0.78 Z:0.76
    Temperature: 25.79 degC


Acceleration X: 0.33, Y: 1.38, Z: 10.52 m/s^2
Rotation X: -0.02, Y: 0.01, Z: 0.01 rad/s
X:1.97 Y:7.73 Z:0.58
Temperature: 22.57 degC

   */

void loop() {
  // Get new sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 
  
if(ultraSensor.distanceCm() <= distStop){
          n=0;
          Serial.println("----------STOP ROTAZIONE--------- ");
          Serial.println(n);
          Serial.print("distance : ");
          Serial.println(ultraSensor.distanceCm() );
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          //tentativo rotazione
          // obstacle = obstacle + random(1,2);

          speX = 0;
          speY = 0;
          speZ = 0;

          accX = 0;
          accY = 0;
          accZ = 0;

          Serial.print("speX:");
          Serial.print(speX );
          Serial.print(" speY:");
          Serial.print(speY );
          Serial.print(" speZ:");
          Serial.println(speZ );
    
          if(obstacle ==1){
            resto =  random(1,5)%2;
          }else{
            resto = (obstacle)%2;
          }
          //*/
          distStop = distStop + 10;
          //resto = (obstacle)%2;

          Serial.print("resto : ");
          Serial.println(resto);
          
          if(resto == 0){
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(backTime/obstacle);
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(-motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(obstacle*steeringTime);
          }else{
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(-motorSpeed*compdx/100);
          delay(backTime/obstacle);
          motor1.stop();
          motor2.stop();
          delay(stopTime);
          motor1.run(motorSpeed*compsx/100);
          motor2.run(motorSpeed*compdx/100);
          delay(obstacle*steeringTime);
          }
          obstacle++;
          rnd = random(1,3);

          Serial.print("obstacle : ");
          Serial.println(obstacle);
          //riposizionamento, indietro e rotazione
          if(obstacle == 6){
            motor1.stop();
            motor2.stop();
            delay(stopTime);
            motor1.run(-motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(obstacle*steeringTime/2);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(-motorSpeed*compdx/100);
            delay(exitTime);
            motor1.run(motorSpeed*compsx/100);
            motor2.run(motorSpeed*compdx/100);
            delay(rnd*steeringTime);
            obstacle =1;
            distStop =30;
            }
          }else{
         //avanti
         Serial.print("distance : ");
         Serial.println(ultraSensor.distanceCm() );
         motor1.run(-motorSpeed*compsx/100);
         motor2.run(motorSpeed*compdx/100);
         obstacle =1;
         distStop = 30;
    }

  start=millis();
  
  accX = accX - a.acceleration.x*100;
  accY = accY - a.acceleration.y*100;
  accZ = accZ - a.acceleration.z*100;

  speX = speX + accX*cycleTime/100000;
  speY = speY + accY*cycleTime/100000;
  speZ = speZ + accZ*cycleTime/100000;

  rotX = rotX - gyro.getAngleX();
  rotY = rotY - gyro.getAngleY();
  rotZ = rotZ - gyro.getAngleZ();

  
  // Print out the values 
  
  Serial.println("----------PARTENZA AVANTI--------- ");
  Serial.println(n);
  Serial.print("Acceleration X: ");
  Serial.print(accX/100);
  Serial.print(", Y: ");
  Serial.print(accY/100);
  Serial.print(", Z: ");
  Serial.print(accZ/100);
  Serial.println(" m/s^2");
/*
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
*/
  Serial.print("speX:");
  Serial.print(speX );
  Serial.print(" speY:");
  Serial.print(speY );
  Serial.print(" speZ:");
  Serial.println(speZ );
  
  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(100);
  n++;
/*
  accX = -a.acceleration.x + 0.33;
  accY = -a.acceleration.y + 1.38;
  accZ = -a.acceleration.z + 10.52;

  speX = speX + accX*cycleTime/1000;
  speY = speY + accY*cycleTime/1000;
  speZ = speZ + accZ*cycleTime/1000;

  rotX = gyro.getAngleX() -1.97;
  rotY = gyro.getAngleY() -7.73;
  rotZ = gyro.getAngleZ() -0.58;
  */

  accX1 = a.acceleration.x*100;
  accY1 = a.acceleration.y*100;
  accZ1 = a.acceleration.z*100;
  delay(50);

  accX2 = a.acceleration.x*100;
  accY2 = a.acceleration.y*100;
  accZ2 = a.acceleration.z*100;
  delay(50);

  accX3 = a.acceleration.x*100;
  accY3 = a.acceleration.y*100;
  accZ3 = a.acceleration.z*100;
  delay(50);

  accX = (accX1 + accX2 + accX3)/3;
  accY = (accY1 + accY2 + accY3)/3;
  accZ = (accZ1 + accZ2 + accZ3)/3;

  rotX = gyro.getAngleX();
  rotY = gyro.getAngleY();
  rotZ = gyro.getAngleZ();
  

  cycleTime=millis()-start;
  Serial.print("cycleTime: ");
  Serial.println(cycleTime);

}
/*
void loop() {
  // Get new sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
   

  // Print out the values 
  
  motor1.run(-motorSpeed*compsx/100);
  motor2.run(motorSpeed*compdx/100);
  //delay(100);

  n=1;
  while(n<20){
  Serial.println("----------PARTENZA AVANTI--------- ");
  Serial.println(n);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(50);
  n++;
  }



  
  Serial.println("----------AVANTI----------- ");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);

  motor1.stop();
  motor2.stop();
 // delay(100);
  
  Serial.println("----------STOP----------- ");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);

  motor1.run(motorSpeed*compsx/100);
  motor2.run(-motorSpeed*compdx/100);
  //delay(100);
  
  Serial.println("----------PARTENZA INDIETRO----------- ");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(1000);

  Serial.println("----------INDIETRO----------- ");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);

  motor1.stop();
  motor2.stop();
  //delay(100);
  
  Serial.println("----------STOP----------- ");
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}
*/
