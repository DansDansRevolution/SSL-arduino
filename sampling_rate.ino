

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void loop() {
  float dt = 0;
  int n = 0;
  float t1 = micros();
  while (n < 1000){
    analogRead(A3);
    analogRead(A4);
    analogRead(A5);
    n++;
  }
  float t2 = micros();
  dt = t2 - t1;
  t1 = t2;

  float avgTime = dt / 1000;
  Serial.println(avgTime);

}
