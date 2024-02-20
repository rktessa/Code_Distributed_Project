
int range=0;

void setup() {
  // initialize serial communication:
  Serial.begin(115200);
}

void loop() {
  
  // do something different depending on the range value:
  switch (range) {
    case 0:    // your hand is on the sensor
      Serial.println("dark");
      range =2;
      break;
    case 1:    // your hand is close to the sensor
      Serial.println("dim");
      break;
    case 2:    // your hand is a few inches from the sensor
      Serial.println("medium");
      range =0;
      break;
    case 3:    // your hand is nowhere near the sensor
      Serial.println("bright");
      break;
  }
  Serial.println("cycle");
  delay(100);        // delay in between reads for stability
}
