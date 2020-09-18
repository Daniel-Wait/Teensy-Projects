#include <stdint.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#define F_SAMP 44100
#define M_LEN 128
#define L_LEN 128
#define N_LEN 256
#define B_BLOCKS 3
#define PACKETS 1

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=126.8062629699707,234.8249797821045
AudioSynthWaveformSine   sine1;          //xy=130,193.00000190734863
AudioMixer4              mixer1;         //xy=301.1937026977539,213.9999485015869
AudioOutputI2S           i2s1;           //xy=532.8062629699707,211.82499313354492
AudioRecordQueue         queue1;         //xy=533.8062591552734,260.8250045776367
AudioConnection          patchCord1(i2s2, 0, mixer1, 1);
AudioConnection          patchCord2(i2s2, 1, mixer1, 2);
AudioConnection          patchCord3(sine1, 0, mixer1, 0);
AudioConnection          patchCord4(mixer1, 0, i2s1, 0);
AudioConnection          patchCord5(mixer1, 0, i2s1, 1);
AudioConnection          patchCord6(mixer1, queue1);
AudioControlSGTL5000     sgtl5000_1;     //xy=350.19373321533203,118.0000057220459
// GUItool: end automatically generated code

typedef struct F32_Data{
    float32_t msg[B_BLOCKS][N_LEN] = {{0}};
    float32_t fft[B_BLOCKS][2 * N_LEN] = {{0}};
} fdata;
int fdata_head = 0;

uint8_t flag = 1;

fdata mf;
fdata rx;

q15_t q_rx_msg[N_LEN] = {0};

float32_t y_msg[B_BLOCKS][N_LEN] = {0};
float32_t y_fft[B_BLOCKS][2 * N_LEN] = {0};

const int ola_max = N_LEN + L_LEN*(B_BLOCKS - 1);
float32_t ola_buffer[ola_max] = {0};
int ola_head = N_LEN-L_LEN;
const int ola_overlap = N_LEN-L_LEN;


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  
  AudioMemory(100);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(40);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.3);

  //AudioProcessorUsageMaxReset();
  //AudioMemoryUsageMaxReset();

  while (!Serial) {}

  sine1.amplitude(0.8);
  sine1.frequency(882);

  

  const int bit_len = 1;
  uint8_t bitseq[bit_len] = {1};

  generateFSK2D(mf.msg, bitseq, bit_len, F_SAMP, M_LEN*B_BLOCKS/bit_len, 5000, 882);
  
  /*
  for(int p = 0; p < B_BLOCKS; p++)
  {
    for(int k = 0; k < N_LEN; k++)
    {
      Serial.println( mf.msg[p][k] );
      delay(2);
    }
    delay(1000);
  }
  */
  
  for(int p = 0; p < B_BLOCKS; p++)
  {
    f32tRealFFT(mf.fft[p], mf.msg[p], N_LEN);
  }

  
  queue1.begin();
}


void loop() 
{
  
  // put your main code here, to run repeatedly:
  if (flag == 1) 
  {
    if (queue1.available() >= PACKETS)
    {    
      for (int k = 0; k < PACKETS; k++)
      {
        memcpy(q_rx_msg + 128*k, queue1.readBuffer(), 256);
        queue1.freeBuffer();
      }
      flag = 0;
    }
  }

  if (flag == 0)
  {
    float32_t sum_msg[N_LEN] = {0};
    //float32_t sum_fft[2*N_LEN] = {0};
 
    flag = 1;
      
    arm_q15_to_float(q_rx_msg, rx.msg[fdata_head], N_LEN);

    /* 
    for(int k = 0; k < N_LEN; k++)
    {
      Serial.println( rx.msg[fdata_head][k] );
    }
    */
    
        
    f32tRealFFT(rx.fft[fdata_head], rx.msg[fdata_head], N_LEN);
    
    /*
    for(int k = 0; k < N_LEN; k++)
    {
      Serial.println( rx.fft[fdata_head][k] );
    }
    */
        
    int index;
    for(int k = 0; k < B_BLOCKS; k++)
    {
      index = k + fdata_head;
     
      if(index >= B_BLOCKS)
      {
        index -= B_BLOCKS;
      }
            
      cmplx_mult(rx.fft[index], mf.fft[B_BLOCKS-k], y_fft[index], N_LEN);
      f32tRealIFFT(y_msg[index], y_fft[index], N_LEN);
      /*
      for(int k = 0; k < N_LEN; k++)
      {
        Serial.println( y_fft[index][k] );
      }
      */
      
      for(int j = 0; j < N_LEN; j++)
      {
        sum_msg[j] += y_msg[index][j];
      }
    }
    
      
    //f32tRealIFFT(sum_msg, sum_fft, N_LEN);

    
    for(int k = 0; k < N_LEN; k++)
    {
      Serial.println( sum_msg[k] );
    }
    
    
    if(fdata_head >= B_BLOCKS-1)
    {
      fdata_head = 0;
    }
    else
    {
      fdata_head++;
    }
    
    //write_ola_buffer(sum_msg);
    
    
    /*     
    for (int j = 0; j < ola_max; j++)
    {
      Serial.println(ola_buffer[j]);
    }
    */
    
  }
}
    


void generateFSK2D(float32_t out_fsk_modulated[][N_LEN], uint8_t* in_bit_sequence, int num_bits, float fs, int samples_per_bit, float f0, float f1)
{
  float f_mux;
  int modu_len = M_LEN*B_BLOCKS;
  float32_t pre_modulated[ modu_len ] = {0};
  float32_t reversed_modu[ modu_len ] = {0};
  
  for (int i = 0; i < num_bits; i++)
  {
    int index = (int)(i * samples_per_bit);

    if (in_bit_sequence[i] == 0)
    {
      f_mux = f0;
    }
    else
    {
      f_mux = f1;
    }

    for (int cnt = 0; cnt < samples_per_bit; cnt++)
    {
      pre_modulated[index + cnt] = arm_sin_f32( (float32_t)(2 * PI * f_mux * cnt) / fs );
    }
  }
  
  for(int k = 0; k < modu_len; k++)
  {
    reversed_modu[k] = pre_modulated[modu_len - k - 1];
  }

  for (int p = 0; p < B_BLOCKS; p++)
  {
    for (int j = 0; j < M_LEN; j++)
    {
      int indx = p*M_LEN;
      out_fsk_modulated[p][j] = reversed_modu[indx + j];
    }
  }
  
}



void f32tRealFFT(float32_t* out_fsk_fft, float32_t* in_fsk_msg, int in_len)
{
  arm_rfft_fast_instance_f32 fastfft32;

  arm_rfft_fast_init_f32(&fastfft32, in_len);

  arm_rfft_fast_f32(&fastfft32, in_fsk_msg, out_fsk_fft, 0);
}



void f32tRealIFFT(float32_t* out_fsk_msg, float32_t* in_fsk_fft, int out_len)
{
  arm_rfft_fast_instance_f32 fastfft32;

  arm_rfft_fast_init_f32(&fastfft32, out_len);

  arm_rfft_fast_f32(&fastfft32, in_fsk_fft, out_fsk_msg, 1);
}



void cmplx_mult( const float32_t* pSrcA, const float32_t* pSrcB, float32_t* pDst, int  numSamples)
{
  for (int n = 0; n < numSamples; n++)
  {
    pDst[(2 * n) + 0] = pSrcA[(2 * n) + 0] * pSrcB[(2 * n) + 0]  - pSrcA[(2 * n) + 1] * pSrcB[(2 * n) + 1] ;
    pDst[(2 * n) + 1] = pSrcA[(2 * n) + 0] * pSrcB[(2 * n) + 1]  + pSrcA[(2 * n) + 1]  * pSrcB[(2 * n) + 0] ;
  }
}



void write_ola_buffer(float32_t* input)
{

  ola_head -= ola_overlap;
  
  if(ola_head < 0)
  {
    ola_head += ola_max;
  }
    
  for(int i = 0; i < ola_overlap; i++)
  {
    ola_buffer[ola_head++] +=  input[i];

    if(ola_buffer[ola_head]>60)
    {
      //Serial.println("THRESHOLD"); 
    }
    
    if(ola_head >= ola_max)
    {
      ola_head = 0;
    }
  }
    
  for(int i = ola_overlap; i < N_LEN; i++)
  {
    ola_buffer[ola_head++] = input[i];
    
    if(ola_buffer[ola_head]>60)
    {
      //Serial.println("THRESHOLD"); 
    }
    
    if(ola_head >= ola_max)
    {
      ola_head = 0;
    } 
  }

}
