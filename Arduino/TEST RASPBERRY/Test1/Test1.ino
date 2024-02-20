
float accX =0;
String rasp;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  accX = accX +0.01;
  rasp = String(accX,3);
  Serial.println(rasp);
  delay(100);
}
