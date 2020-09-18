
#include <stdint.h> 
#define ARM_MATH_CM7
#define __FPU_PRESENT
#include "arm_math.h"

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=212.80625915527344,299.8250045776367
AudioInputI2S            i2s1;           //xy=227.80625915527344,370.8249969482422
AudioMixer4              mixer1;         //xy=473.99996185302734,193.99994659423828
AudioMixer4              mixer2;         //xy=478.9999237060547,492.0000400543213
AudioRecordQueue         queue1;         //xy=652.8062324523926,194.82497024536133
AudioOutputI2S           i2s2;           //xy=652.806266784668,334.8250255584717
AudioAnalyzeFFT256       fft256_1;       //xy=654.806266784668,261.82501220703125
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav1, 1, mixer2, 2);
AudioConnection          patchCord3(i2s1, 0, mixer1, 1);
AudioConnection          patchCord4(i2s1, 1, mixer2, 3);
AudioConnection          patchCord5(mixer1, 0, i2s2, 0);
AudioConnection          patchCord6(mixer1, queue1);
AudioConnection          patchCord7(mixer1, fft256_1);
AudioConnection          patchCord8(mixer2, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=522.9999923706055,98.00000381469727
// GUItool: end automatically generated code

float arrfft[256];
char* wavFile = "Shallow.WAV";
float temp = 0.0;

void setup()   {                
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial.println("Start serial");
  
  AudioMemory(12);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(50);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.5);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if(!(SD.begin(SDCARD_CS_PIN))){
    Serial.println("Unable to access SD card. Program halted.");
    while(1);  
  }
  Serial.println("Done.");
  Serial.println("Waiting for control input.");

  fft256_1.windowFunction(AudioWindowHamming256);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  playFile(wavFile);
}

void loop()                     
{
  Serial.println("Barker autocorrelation:");
  
  if (fft256_1.available()){
    
    for(int i = 0; i < 256; i++){  
      arrfft[i] = (float)fft256_1.read(i);
    }
    for(int i = 0; i < 256; i++){
      temp = arrfft[i];
      Serial.println(20*temp);
      Serial.print("  ");
      delay(5);
    }  
  }
}

void arrPrint(const float* src, int len)
{
  for(int i = 0; i < len; i++){
    Serial.print(src[i]);
    Serial.print("  ");
  }
  Serial.println();
}


void playFile(const char* file)
{
  Serial.print("Start playing: ");
  playSdWav1.play(file);
  Serial.println(file);
  delay(10);
}
