#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=215,285
AudioInputI2S            i2s1;           //xy=233,387
AudioMixer4              mixer1;         //xy=609,234
AudioMixer4              mixer2;         //xy=610,424
AudioOutputI2S           i2s2;           //xy=815,331
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav1, 1, mixer2, 2);
AudioConnection          patchCord3(i2s1, 0, mixer1, 1);
AudioConnection          patchCord4(i2s1, 1, mixer2, 3);
AudioConnection          patchCord5(mixer1, 0, i2s2, 0);
AudioConnection          patchCord6(mixer2, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=393,172
// GUItool: end automatically generated code
char* wavFile = "Shallow.WAV";
byte wavNum = 0;
bool wavPlay = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial.println("Start serial");
  
  AudioMemory(8);
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
  
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  playFile(wavFile);
}

void loop() {
  /*
  // put your main code here, to run repeatedly:
  if(!(playSdWav1.isPlaying()) && (wavPlay)){
    wavNum++;
    if(wavNum > 0){
      wavNum = 0;
    }
    playFile(wavFiles[wavNum]);
  } 
  */
}

void playFile(const char* file)
{
  Serial.print("Start playing: ");
  playSdWav1.play(file);
  Serial.println(file);
  delay(10);
}
