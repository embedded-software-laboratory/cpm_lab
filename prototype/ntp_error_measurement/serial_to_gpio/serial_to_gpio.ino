void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  int chr = Serial.read();
  if(chr == '1') digitalWrite(LED_BUILTIN, HIGH);
  else if(chr == '0') digitalWrite(LED_BUILTIN, LOW); 

}
