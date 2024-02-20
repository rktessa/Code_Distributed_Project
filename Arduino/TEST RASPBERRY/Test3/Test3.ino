void setup() {
  int i;
  float val;
  char buff[10];
  
  Serial.begin(115200);
  val = 0.0;
  for (i = 0; i < 10; i++) {
    snprintf (buff, sizeof(buff), "%f", val);
    Serial.print("val: ");
    Serial.println(val);
    val += 5.0;
  }
  
  Serial.println("Contents of Buffer: ");
  for (int a=0; a<10; a++) {
    Serial.println(buff[a]);
  }
  
}

void loop() {
}
