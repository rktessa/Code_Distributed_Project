
#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MeGyro gyro;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gyro.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX());
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY());
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ());
  delay(1000);
}
