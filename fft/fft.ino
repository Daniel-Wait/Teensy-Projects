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
AudioSynthWaveformSine   sine1;          //xy=216.80626678466797,333.82499504089355
AudioInputI2S            i2s1;           //xy=227.80625915527344,370.8249969482422
AudioMixer4              mixer1;         //xy=473.99996185302734,193.99994659423828
AudioMixer4              mixer2;         //xy=478.9999237060547,492.0000400543213
AudioRecordQueue         queue1;         //xy=652.8062324523926,194.82497024536133
AudioOutputI2S           i2s2;           //xy=652.806266784668,334.8250255584717
AudioAnalyzeFFT256       fft256_1;       //xy=654.806266784668,261.82501220703125
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav1, 1, mixer2, 2);
AudioConnection          patchCord3(sine1, 0, mixer1, 2);
AudioConnection          patchCord4(sine1, 0, mixer2, 1);
AudioConnection          patchCord5(i2s1, 0, mixer1, 1);
AudioConnection          patchCord6(i2s1, 1, mixer2, 3);
AudioConnection          patchCord7(mixer1, 0, i2s2, 0);
AudioConnection          patchCord8(mixer1, queue1);
AudioConnection          patchCord9(mixer1, fft256_1);
AudioConnection          patchCord10(mixer2, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=522.9999923706055,98.00000381469727
// GUItool: end automatically generated code

float arrfft[256];

char* wavFile = "Shallow.WAV";
float temp = 0.0;

void setup()   {                
  // put your setup code here, to run once:
  Serial.begin(38400);
  
  AudioMemory(12);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(55);
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

  sine1.amplitude(0);
  sine1.frequency(10000);
  
  fft256_1.windowFunction(AudioWindowHamming256);
  AudioProcessorUsageMaxReset();
  AudioMemoryUsageMaxReset();
  playFile(wavFile);
}
                  
void loop() {
  float n;
  int i;  

  if (fft256_1.available()) {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    for (i=0; i<256; i ++) {
      n = fft256_1.read(i);
      Serial.println(n*20);
      //Serial.print("\t");

      //if(n*20 > 0.01){
        //Serial.println(i);
      //}
      
      delay(5);
    }
    //Serial.println();
  }
}

void playFile(const char* file)
{
  playSdWav1.play(file);
  Serial.println(file);
  delay(10);
}
  
