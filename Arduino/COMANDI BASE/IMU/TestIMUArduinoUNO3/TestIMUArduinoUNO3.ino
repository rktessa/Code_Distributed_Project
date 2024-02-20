#include <MKRIMU.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);

    if (x > 0.9 || x < -0.9) {
      Serial.println("X threshold met");
      delay(1000);
    }
    if (y > 0.9 || y < -0.9) {
      Serial.println("Y threshold met");
      delay(1000);
    }
    if (z > 0.9 || z < -0.9) {
      Serial.println("Z threshold met");
      delay(1000);
    }
  }
  delay(500);
}
