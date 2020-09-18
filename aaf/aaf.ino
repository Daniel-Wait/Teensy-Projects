
#include <stdint.h> 

#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=55,390.8249988555908
AudioSynthWaveformSine   sine1;          //xy=70.80625915527344,324.8250045776367
AudioMixer4              mixer1;         //xy=301.1937026977539,213.9999485015869
AudioMixer4              mixer2;         //xy=306.19366455078125,512.0000419616699
AudioMixer4              mixer3;         //xy=510.00002098083496,417.82500076293945
AudioOutputI2S           i2s2;           //xy=510.8062286376953,318.82502365112305
AudioAnalyzePeak         peak1;          //xy=607.0000534057617,527.8250112533569
AudioConnection          patchCord1(i2s1, 0, mixer1, 1);
AudioConnection          patchCord2(i2s1, 1, mixer2, 3);
AudioConnection          patchCord3(sine1, 0, mixer1, 0);
AudioConnection          patchCord4(sine1, 0, mixer2, 2);
AudioConnection          patchCord5(mixer1, 0, mixer3, 0);
AudioConnection          patchCord6(mixer1, 0, i2s2, 0);
AudioConnection          patchCord7(mixer2, 0, mixer3, 1);
AudioConnection          patchCord8(mixer2, 0, i2s2, 1);
AudioConnection          patchCord9(mixer3, peak1);
AudioControlSGTL5000     sgtl5000_1;     //xy=350.19373321533203,118.0000057220459
// GUItool: end automatically generated code

void setup()   {                
  // put your setup code here, to run once:
  Serial.begin(38400);
  
  AudioMemory(12);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(0);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.5);

  //sine1.amplitude(0.25);
  //sine1.frequency(22000);

  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();

  delay(1000);
}
                 
void loop() {
  float n;
  int i;  

  if (peak1.available() ) {
    Serial.println(peak1.read());
    delay(5);
  }
}
  
