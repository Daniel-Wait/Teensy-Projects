// Advanced Microcontroller-based Audio Workshop
//
// http://www.pjrc.com/store/audio_tutorial_kit.html
// https://hackaday.io/project/8292-microcontroller-audio-workshop-had-supercon-2015
// 
// Part 2-3: Playing Samples

// WAV files converted to code by wav2sketch
#include "AudioSampleSnare.h"        // http://www.freesound.org/people/KEVOY/sounds/82583/
#include "AudioSampleTomtom.h"       // http://www.freesound.org/people/zgump/sounds/86334/
#include "AudioSampleHihat.h"        // http://www.freesound.org/people/mhc/sounds/102790/
#include "AudioSampleKick.h"         // http://www.freesound.org/people/DWSD/sounds/171104/
#include "AudioSampleGong.h"         // http://www.freesound.org/people/juskiddink/sounds/86773/
#include "AudioSampleCashregister.h" // http://www.freesound.org/people/kiddpark/sounds/201159/
#include <Bounce.h>


///////////////////////////////////
// copy the Design Tool code here
///////////////////////////////////
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlayMemory          playMem1;       //xy=100.80625915527344,190.82500457763672
AudioMixer4              mixer1;         //xy=301.1937026977539,213.9999485015869
AudioAnalyzePeak         peak1;          //xy=461.80625915527344,252.82500457763672
AudioOutputI2S           i2s1;           //xy=508.80625915527344,182.82500457763672
AudioConnection          patchCord1(playMem1, 0, mixer1, 0);
AudioConnection          patchCord2(mixer1, peak1);
AudioConnection          patchCord3(mixer1, 0, i2s1, 0);
AudioConnection          patchCord4(mixer1, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=350.19373321533203,118.0000057220459
// GUItool: end automatically generated code



void setup() {
  AudioMemory(10);
  while(!Serial){}
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);
  playMem1.play(AudioSampleSnare);
}

void loop() { 
  if (peak1.available() ) {
    Serial.println(peak1.read());
  }
}
  
