#include "arduinoFFT.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

arduinoFFT FFT = arduinoFFT(); 

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2

int sampling_period_us = 90;
int t1;
int t2;
int dt;
int sumDT = 0;
int k;

double v1Real[samples];
double v1Imag[samples];
double v2Real[samples];
double v2Imag[samples];
double v3Real[samples];
double v3Imag[samples];

double SampleDummySig2[samples];
double SampleDummySig3[samples];
double DPmult12[samples];
double DPmult23[samples];
double DPmult13[samples];
double DPmaxArray12[samples];
double DPmaxArray23[samples];
double DPmaxArray13[samples];
int DPmax12 = 0;
int DPmax23 = 0;
int DPmax13 = 0;
int DPmaxArraymax12 = 0;
int DPmaxArraymax23 = 0;
int DPmaxArraymax13 = 0;

int sigMax1 = 0;
int sigMax2 = 0;
int sigMax3 = 0;

double dt12 = 0;
double dt23 = 0;
double dt13 = 0;
int s12 = 0;
int s23 = 0;
int s13 = 0;

int vSound = 343;
double d = 0.165;

// Sampling frequency
double samplingFrequency = 1000000 / sampling_period_us; // Hz
//  high cut-off frequency (-3 dB)
double const f_c = 500; // Hz
// Normalized cut-off frequency
double const f_n = 2 * f_c / samplingFrequency;
auto filter = butter<4>(f_n);


#define MIC1 A3
#define MIC2 A4
#define MIC3 A5
#define LED1 7
#define LED2 8
#define LED3 9

#define SCL_FREQUENCY 0x02

void setup()
{
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |    
                   ADC_CTRLB_RESSEL_10BIT;         

  ADC->SAMPCTRL.reg = 0x00;
  
  Serial.begin(115200);
  // while the serial stream is not open, do nothing:
   while (!Serial) ;
}



void loop()
{  
  getData(micros());
  lowPass();
  dcAverage();
  findFFT();
  crossCorrelation();
  //printFFT();
}

void getData (unsigned long microseconds){
  for(int i = 0; i < samples; i++){

    v1Real[i] = analogRead(A3);
    v1Imag[i] = 0;
    v2Real[i] = analogRead(A4);
    v2Imag[i] = 0;
    v3Real[i] = analogRead(A5);
    v3Imag[i] = 0;
    while(micros() - microseconds < sampling_period_us){
        
    }
    microseconds += sampling_period_us;
  }
}

void lowPass (){
  for(int n = 0; n < samples; n++){
    v1Real[n] = filter(v1Real[n]);
    v2Real[n] = filter(v2Real[n]);
    v3Real[n] = filter(v3Real[n]);
  }

}

void dcAverage(){
  double addSig1 = 0;
  double addSig2 = 0;
  double addSig3 = 0;
  
  double meanSig1 = 0;
  double meanSig2 = 0;
  double meanSig3 = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    addSig1 = addSig1 + v1Real[i];
    addSig2 = addSig2 + v2Real[i];
    addSig3 = addSig3 + v3Real[i];
  }
  meanSig1 = addSig1 / samples;
  meanSig2 = addSig2 / samples;
  meanSig3 = addSig3 / samples;
  
  for (uint16_t i = 0; i < samples; i++) {
    v1Real[i] -= meanSig1;
    v2Real[i] -= meanSig2;
    v3Real[i] -= meanSig3;
  }
  
}

void crossCorrelation(){
  for (int i=0;i<samples;i++) {
    Shift12(i);
    Shift23(i);
    Shift13(i);
    DotProduct12(i);
    DotProduct23(i);
    DotProduct13(i);
    DPmax12 = 0;
    DPmax23 = 0;
    DPmax13 = 0;
  }
  for (int i=0;i<samples;i++) {
    if (DPmaxArray12[i] > DPmaxArraymax12) {
      DPmaxArraymax12 = DPmaxArray12[i];
    }
    if (DPmaxArray23[i] > DPmaxArraymax23) {
      DPmaxArraymax23 = DPmaxArray23[i];
    }
    if (DPmaxArray13[i] > DPmaxArraymax13) {
      DPmaxArraymax13 = DPmaxArray13[i];
    }
  }
  
  for (int i=0;i<samples;i++) {
    if (DPmaxArray12[i] == DPmaxArraymax12) {
      if (i > (samples / 2)) {
        dt12 = (i - samples) / samplingFrequency;
        s12 = i;
        PrintData();
        break;
      }
      dt12 = i / samplingFrequency;
      s12 = i;
      PrintData();
      break;
    }
  }
  for (int i=0;i<samples;i++) {
    if (DPmaxArray23[i] == DPmaxArraymax23) {
      if (i > (samples / 2)) {
        dt23 = (i - samples) / samplingFrequency;
        s23 = i;
        PrintData();
        break;
      }
      dt23 = i / samplingFrequency;
      s23 = i;
      PrintData();
      break;
    }
  }
  for (int i=0;i<samples;i++) {
    if (DPmaxArray13[i] == DPmaxArraymax13) {
      if (i > (samples / 2)) {
        dt13 = (i - samples) / samplingFrequency;
        s13 = i;
        PrintData();
        break;
      }
      dt13 = i / samplingFrequency;
      s13 = i;
      PrintData();
      break;
    }
  }
  DPmax12 = 0;
  DPmax23 = 0;
  DPmax13 = 0;
  DPmaxArraymax12 = 0;
  DPmaxArraymax23 = 0;
  DPmaxArraymax13 = 0;
}

void Shift12(int s) {
  for (int i=0;i<samples-s;i++) {
    SampleDummySig2[i] = v2Real[s+i];
  }
  for (int i=0;i<s;i++) {
    SampleDummySig2[(samples-s)+i] = v2Real[i];
  }
}

void Shift23(int s) {
  for (int i=0;i<samples-s;i++) {
    SampleDummySig3[i] = v3Real[s+i];
  }
  for (int i=0;i<s;i++) {
    SampleDummySig3[(samples-s)+i] = v3Real[i];
  }
}

void Shift13(int s) {
  for (int i=0;i<samples-s;i++) {
    SampleDummySig3[i] = v3Real[s+i];
  }
  for (int i=0;i<s;i++) {
    SampleDummySig3[(samples-s)+i] = v3Real[i];
  }
}

void DotProduct12(int DPi) {
  for (int i=0;i<samples;i++)
  {
    DPmult12[i] = v1Real[i]*SampleDummySig2[i];
    if (DPmult12[i] > DPmax12) {
      DPmax12 = DPmult12[i];
    }
  }
  DPmaxArray12[DPi] = DPmax12;
}

void DotProduct23(int DPi) {
  for (int i=0;i<samples;i++)
  {
    DPmult23[i] = v2Real[i]*SampleDummySig3[i];
    if (DPmult23[i] > DPmax23) {
      DPmax23 = DPmult23[i];
    }
  }
  DPmaxArray23[DPi] = DPmax23;
}

void DotProduct13(int DPi) {
  for (int i=0;i<samples;i++)
  {
    DPmult13[i] = v1Real[i]*SampleDummySig3[i];
    if (DPmult13[i] > DPmax13) {
      DPmax13 = DPmult13[i];
    }
  }
  DPmaxArray13[DPi] = DPmax13;
}
void findFFT(){
  FFT.Windowing(v1Real, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(v1Real, v1Imag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(v1Real, v1Imag, samples); /* Compute magnitudes */
  FFT.Windowing(v2Real, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(v2Real, v2Imag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(v2Real, v2Imag, samples); /* Compute magnitudes */
  FFT.Windowing(v3Real, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(v3Real, v3Imag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(v3Real, v3Imag, samples); /* Compute magnitudes */

  for(int j = 0; j < 2; j++){
      v1Real[j] = 0;
      v2Real[j] = 0;
      v3Real[j] = 0;
  }
}

void PrintData() {
  if (abs(dt12) < (d / vSound) && abs(dt23) < (d / vSound)) {
    Serial.print("MIC12:");
    Serial.print("  ");
    Serial.print(s12);
    Serial.print("  ");
    Serial.print(dt12, 7);
    Serial.print("  ");
    double theta12 = acos((dt12 * vSound) / d);
    Serial.println(theta12 * 180 / M_PI);
    
    Serial.print("MIC23:");
    Serial.print("  ");
    Serial.print(s23);
    Serial.print("  ");
    Serial.println(dt23, 7);



    Serial.print("MIC13:");
    Serial.print("  ");
    Serial.print(s13);
    Serial.print("  ");
    Serial.println(dt13, 7);
    double theta23 = M_PI - (acos((dt23 * vSound) / d));
    Serial.println(theta23 * 180 / M_PI);
    
    Serial.println("  ");
  } else {
    loop();
  }
}

void printFFT(){
  for (int i = 0; i<(samples/2); i++){
    int currF = i * ((1000000 / sampling_period_us)/samples);
    int currA = v1Real[i];
    Serial.print(i);Serial.print(",");Serial.print(currF);
    for (int j=0; j<(currA/100); j++){
    Serial.print("*");
    }
    Serial.print(",");
    Serial.println(currA);
   }
}

void checkTime (int t1){
  t2 = micros();
  dt = t2 - t1;
  sumDT = sumDT + dt;
}
