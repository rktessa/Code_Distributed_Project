void setup() {
  int i;
  float val;
  char buff[10];
  
  Serial.begin(115200);
  val = 0.0;
  for (i = 0; i < 10; i++) {
    snprintf (buff, sizeof(buff), "%f", val);
    Serial.println(val);
    val += 5.0;
  }
}

void loop() {}
