void setup() {
  char buf[6];

  Serial.begin(115200);

  float v = 1111.23;
  dtostrf(v,5,3,buf);
  Serial.print(buf);
  //Serial.println("<")  ;
}

void loop() {

}
