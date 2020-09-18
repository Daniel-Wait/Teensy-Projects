
#include <stdint.h> 
#define ARM_MATH_CM7
#define __FPU_PRESENT
#include "arm_math.h"

float barker[] = {+1,+1,+1,+1,+1,-1,-1,+1,+1,-1,+1,-1,+1};
int bark_len = 13;
float autocorr[26];

void setup()   {                
  Serial.begin(38400);
}

void loop()                     
{
  Serial.println("Barker autocorrelation:");
  arm_correlate_f32(barker, bark_len, barker, bark_len, autocorr);
  arrprint(autocorr, 26);
  delay(1000);
}



void arrprint(const float* src, int len)
{
  for(int i = 0; i < len; i++){
    Serial.print(src[i]);
    Serial.print("  ");
  }
  Serial.println();
}


void userDelay(int period)
{
  unsigned long previousMillis = millis(); //time now
  while(millis() - previousMillis < period)
  {
    
  }
}
