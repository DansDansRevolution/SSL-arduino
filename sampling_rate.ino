int sampleSize = 1024;
int t2 = 0;
int t1=0;
int sumDT = 0;
int dt = 0;

float micro = 10e-6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);


  
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |    
                   ADC_CTRLB_RESSEL_10BIT;         

  ADC->SAMPCTRL.reg = 0x00;
  
}

void loop() {
  sumDT = 0;
  int n = 0;
  t1 = micros();
  while (n < sampleSize){
    analogRead(A3);
    analogRead(A4);
    analogRead(A5);
    t2 = micros();
    dt = t2 - t1;
    sumDT= sumDT + dt;
    t1 = t2;
    n++;
  }
  

  int avgTime = sumDT / sampleSize;
  Serial.println(avgTime);

}
