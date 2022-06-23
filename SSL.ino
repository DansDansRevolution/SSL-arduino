void setup() {
  Serial.begin(115200);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
 
}

void loop() {
  Serial.print(analogRead(A3)-800);
  Serial.print(",");
  Serial.print(analogRead(A4)-800);
  Serial.print(",");
  Serial.println(analogRead(A5)-800);
}
