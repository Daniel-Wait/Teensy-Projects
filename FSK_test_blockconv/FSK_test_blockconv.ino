#include <stdint.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#define DOWN_SAMP 2
#define L_LEN 1024 //8 PACKETS
#define N_LEN 2048
#define B_BLOCKS 6
#define BIT_LEN 441
#define PACKETS 8 //8 PACKETS * 1/DOWNSAMP

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

uint8_t bstart = 1;

uint8_t flag = 1;

int16_t fdata_head = 0;

const float f1 = 400;
const float f0 = 200;

float32_t mf_fft[B_BLOCKS][2*N_LEN] = {{0}};
float32_t rx_fft[B_BLOCKS][2*N_LEN] = {{0}};


const int ola_max = N_LEN+11*L_LEN; //+ L_LEN*(B_BLOCKS - 1);
float32_t ola_buffer[ola_max] = {0};
int ola_head = N_LEN-L_LEN;
const int ola_overlap = N_LEN-L_LEN;

int8_t loops = 0;

void setup() {
  // put your setup code here, to run once:
  float32_t mf_msg[B_BLOCKS][N_LEN] = {{0}};
  
  Serial.begin(9600);
   
  AudioMemory(100);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(0);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.3);

  //AudioProcessorUsageMaxReset();  
  //AudioMemoryUsageMaxReset();

  while (!Serial) {}

  const int bit_num = 12;
  
  int bitseq[bit_num] = {1,0,1,0,1,0,1,0,1,0,1,0};
  //int bitseq[bit_num] = {1,1,0,1,0,0,1,1,1,1,0,0};
  //int bitseq[bit_num] = {1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0};
  
  float space = f0*DOWN_SAMP;
  float mark = f1*DOWN_SAMP;
  generateFSK2D(mf_msg, bitseq, bit_num, 44100, BIT_LEN , space, mark);

  /*
  for(int p = 0; p < B_BLOCKS; p++)
  {
    for(int k = 0; k < L_LEN; k++)
    {
      Serial.println( mf_msg[p][k] );
      delay(5);
    }
  }
  */
  
  for(int p = 0; p < B_BLOCKS; p++)
  {
    f32tRealFFT(mf_fft[p], mf_msg[p], N_LEN);
  }

  /*
  for(int p = 0; p < B_BLOCKS; p++)
  {
    for(int k = 0; k < N_LEN; k++)
    {
      Serial.println( mf_fft[p][k] );
      delay(2);
    }
    delay(1000);
  }
  */
  
  queue1.begin();
  //sine1.amplitude(1.0);
  //sine1.frequency(f1);
}


void loop() 
{
  // put your main code here, to run repeatedly:
  if(bstart == 1)
  {
    if(loops < 12)
    {
      q15_t q_rx_msg[L_LEN*DOWN_SAMP] = {0};
       
      if (flag == 1) 
      {
        if (queue1.available() >= PACKETS*DOWN_SAMP)
        {    
          for (int k = 0; k < PACKETS*DOWN_SAMP; k++)
          {
            memcpy(q_rx_msg + 128*k, queue1.readBuffer(), 256);
            queue1.freeBuffer();
          }

          /*
          for(int k = 0; k < L_LEN*DOWN_SAMP; k++)  
          {
            Serial.println( q_rx_msg[k] );
          }
          */
          
          flag = 0;
        }
      }
    
      if (flag == 0)
      {
        //int curr = micros();
        
        flag = 1;
        block_convolver(q_rx_msg);
        loops++;
        
        //int diff = micros()-curr;
        /*
        Serial.println("#");
        Serial.print("Calculation Time 24MHz\t");
        Serial.println( diff );
        Serial.print("Packets Available\t");
        Serial.println( queue1.available() );
        */
      }
    }
    else
    {
      bstart = 0;
      
      
      for(int m = 0; m < ola_max; m++)
      {
        int indx = ola_head + m;
        if(indx >= ola_max)
        {
          indx -= ola_max;
        }            
        Serial.println( ola_buffer[indx] );
      }
      
            
      queue1.clear();
      queue1.end();      
    }
  }
}
    


void generateFSK2D(float32_t out_fsk_modulated[][N_LEN], int* in_bit_sequence, int num_bits, float fs, int samples_per_bit, float f0, float f1)
{
  float f_mux;
  float f_chirp = 50;
  float f_add = 50*DOWN_SAMP;
 
  const int fhss_num = 2;
  float fhss_0[ fhss_num ] = {0};
  float fhss_1[ fhss_num ] = {0};

  int tally_0 = 0;
  int tally_1 = 0;

  for(int m = fhss_num; m >= 0; m--)
  {
    fhss_0[m] = f0 - m*f_add;
    fhss_1[m] = f1 + m*f_add;
  }

  
  int modu_len = samples_per_bit*num_bits;
  float32_t pre_modulated[ modu_len ] = {0};
  float32_t reversed_modu[ modu_len ] = {0};
  
  for (int i = 0; i < num_bits; i++)
  {
    int index = (int)(i * samples_per_bit);
    
    if (in_bit_sequence[i] == 0)
    {
      for (int cnt = 0; cnt < samples_per_bit; cnt++)
      {
        //f_mux = fhss_0[ 0 ] - DOWN_SAMP*50*cnt/samples_per_bit;
        f_mux = fhss_0[ tally_0 % fhss_num ] - DOWN_SAMP*f_chirp*cnt/samples_per_bit;       
        pre_modulated[index + cnt] = arm_sin_f32( (float32_t)(2 * PI * f_mux * cnt ) / fs );
        //Serial.println( 10*pre_modulated[index + cnt] );
        //delay(3);
      }
      tally_0++;
    }
    else
    {
      for (int cnt = 0; cnt < samples_per_bit; cnt++)
      {
        //f_mux = fhss_1[ 0 ] + DOWN_SAMP*50*cnt/samples_per_bit;
        f_mux = fhss_1[ tally_1 % fhss_num ] + DOWN_SAMP*f_chirp*cnt/samples_per_bit;
        pre_modulated[index + cnt] = arm_sin_f32( (float32_t)(2 * PI * f_mux * cnt ) / fs );
        //Serial.println( pre_modulated[index + cnt] );
        //delay(3);
      }
      tally_1++;
    }
    
    /*
    for (int cnt = 0; cnt < samples_per_bit; cnt++)
    {
      float fchirp = 50;
      if(f_mux == f0)
      {
        fchirp = -1 * fchirp;
      }

      fchirp = fchirp*DOWN_SAMP*cnt/samples_per_bit;
      
      pre_modulated[index + cnt] = arm_sin_f32( (float32_t)(2 * PI * (f_mux + fchirp) * cnt ) / fs );
    }
    */

  }
  
  for(int k = 0; k < modu_len; k++)
  {
    reversed_modu[k] = pre_modulated[modu_len - k - 1];
    //Serial.println( reversed_modu[ k ] );
    //delay(3);
  }

  for (int p = 0; p < B_BLOCKS; p++)
  {
    for (int j = 0; j < L_LEN; j++)
    {
      int indx = p*L_LEN;

      if(indx+j < modu_len)
      {
        out_fsk_modulated[p][j] = reversed_modu[indx + j];
      }
      else
      {
        out_fsk_modulated[p][j] = 0.0;
      }
      
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
    
    if(ola_head >= ola_max)
    {
      ola_head = 0;
    }
  }
    
  for(int i = ola_overlap; i < N_LEN; i++)
  {
    ola_buffer[ola_head++] = input[i];
    
    if(ola_head >= ola_max)
    {
      ola_head = 0;
    } 
  }

}



void downsample(float32_t* out_arr, float32_t* in_arr, int len)
{
  int cnt = 0;
  for(int k = 0; k < len; k++)
  {
    out_arr[k] = in_arr[cnt];
    cnt += DOWN_SAMP;
  }
}



void block_convolver(q15_t* q15_data)
{
  float32_t y_fft[2*N_LEN] = {0};   
  float32_t rx_msg[L_LEN*DOWN_SAMP] = {0};
  float32_t rx_down[N_LEN] = {0};
  
  arm_q15_to_float(q15_data, rx_msg, L_LEN*DOWN_SAMP);
  downsample(rx_down, rx_msg, L_LEN);

  /*
  for(int k = 0; k < L_LEN*DOWN_SAMP; k++)  
  {
    Serial.println( q15_data[k] );
    delay(5);
  }
  
  
  for(int k = 0; k < L_LEN*DOWN_SAMP; k++)
  {
    Serial.println( 10*rx_msg[k] );
  }
  */
  
  for(int k = 0; k < L_LEN; k++)  
  {
    Serial.println( rx_down[k] );
  }
    
  
      
  f32tRealFFT(rx_fft[fdata_head], rx_down, N_LEN);


  float32_t sum_msg[N_LEN] = {0};
  float32_t sum_fft[2*N_LEN] = {0};

  
  int index;
  for(int k = 0; k < B_BLOCKS; k++)
  {
    index = fdata_head-k;
    
    if(index < 0)
    {
      index += B_BLOCKS;
    }
    
    arm_cmplx_mult_cmplx_f32(rx_fft[index], mf_fft[k], y_fft, N_LEN);
    
    for(int j = 0; j < 2*N_LEN; j++)
    {
      sum_fft[j] += y_fft[j];
    }
  }
  f32tRealIFFT(sum_msg, sum_fft, N_LEN);



  if(fdata_head >= B_BLOCKS-1)
  {
    fdata_head = 0;
  }
  else
  {
    fdata_head++;
  }

  write_ola_buffer(sum_msg);
}
