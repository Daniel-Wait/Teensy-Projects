
#include <stdint.h> 
#include <stdbool.h>

#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

#define PACKETS 16

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
AudioOutputI2S           i2s2;           //xy=510.8062286376953,318.82502365112305
AudioMixer4              mixer3;         //xy=512.0000305175781,414.82501220703125
AudioRecordQueue         queue1;         //xy=639.0000343322754,414
AudioConnection          patchCord1(i2s1, 0, mixer1, 1);
AudioConnection          patchCord2(i2s1, 1, mixer2, 3);
AudioConnection          patchCord3(sine1, 0, mixer1, 0);
AudioConnection          patchCord5(mixer1, 0, mixer3, 0);
AudioConnection          patchCord6(mixer1, 0, i2s2, 0);
AudioConnection          patchCord7(mixer2, 0, mixer3, 1);
AudioConnection          patchCord8(mixer2, 0, i2s2, 1);
AudioConnection          patchCord9(mixer3, queue1);
AudioControlSGTL5000     sgtl5000_1;     //xy=350.19373321533203,118.0000057220459
// GUItool: end automatically generated code

uint8_t L = PACKETS*128;
int16_t arr[PACKETS][128] = {0};
uint8_t flag = 1;
int arrival = 0;
int prev;
int curr;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  AudioMemory(208);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(0);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.2);

  //AudioProcessorUsageMaxReset();
  //AudioMemoryUsageMaxReset();
  
  while(!Serial){}
  delay(3000);
  Serial.println("BEGIN");
  sine1.amplitude(0.5);
  sine1.frequency(344.5);
  queue1.begin();
/*  
  while(queue1.available()<2){
    
  }
  
  for(int k = 0; k < 2; k++)
  {
    memcpy(arr, queue1.readBuffer(), 128);
    queue1.freeBuffer();
    Serial.println(arr[0]);
    Serial.println(arr[4]);
  }
  queue1.end();
  queue1.clear();
 */
 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (flag == 1){
    if (queue1.available() >= PACKETS)
    {
      //Serial.println(millis()-prev);
      //prev = millis();
      //arrival = millis();
      for(int k = 0; k < PACKETS; k++)
      {
        memcpy(arr[k], queue1.readBuffer(), 256);
        queue1.freeBuffer();
      }
      //Serial.println(millis()- arrival);
      flag = 0;
       
      //queue1.end();
      //queue1.clear();
      //queue1.begin();
    }
  }
  
  if(flag == 0)
  {
    flag = 1;
   
    for(int h = 0; h < PACKETS; h++ )
    {
      for(int i = 0; i < 128; i++)
      {
        Serial.println(arr[h][i]);
      }
      Serial.println(50000);
    }
   
  }
}
