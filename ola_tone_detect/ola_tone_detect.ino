#include <stdint.h>
//#define ARM_MATH_CM7
//#define __FPU_PRESENT
#include "arm_math.h"
#include "arm_const_structs.h"

#define F_SAMP 44100
#define M_LEN 2048
#define L_LEN 2048
#define N_LEN 4096
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


/*Setup

   choose lengths M, L, and N
   choose # blocks (B) to ovelap

   mf_msg:    init. w/ N elems; modulate bit seq to fsk and insert fsk samples in elems 0:M-1
   mf_fft:    init. w/ 2*N elems; =FFT{mf_msg}
   rx_aud:    init. w/ N elems;
   rx_fft:    init. w/ 2*N elems;
   y:         init. w/ N elems;
   ola:       init. w/ N+L*(B-1) elems;

*/

/*Loop

   WAIT until L audio samples available
   rx_aud:    insert audio samples in elems 0:L-1
   rx_fft:    =FFT{rx_aud}

   y:          = IFFT{ rx_fft*mf_fft }
   ola:        add newest samples w/ y[0:N-L]; overwrite oldest samples w/ y[N-L:N]; insert threshold check

   if(threshold): toggle pin/LED/print

   rx_aud:    memset/memalloc to all zeroes
   rx_fft:    memset/memalloc to all zeroes
   y:    memset/memalloc to all zeroes

*/

typedef struct F32_Data{
    float32_t msg[N_LEN] = {0};
    float32_t fft[2 * N_LEN] = {0};
} fdata;
int fdata_head = 0;

uint8_t flag = 1;

fdata mf;
fdata rx;
fdata y;

q15_t q_rx_msg[N_LEN] = {0};

const int ola_max = N_LEN + L_LEN*(B_BLOCKS-1);
float32_t ola_buffer[ola_max] = {0};
int ola_head = N_LEN-L_LEN;
const int ola_overlap = N_LEN-L_LEN;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(58824);
  
  AudioMemory(100);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(60);
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.audioPreProcessorEnable();
  sgtl5000_1.audioPostProcessorEnable();
  sgtl5000_1.volume(0.3);

  //AudioProcessorUsageMaxReset();
  //AudioMemoryUsageMaxReset();

  while (!Serial) {}

  sine1.amplitude(1.0);
  sine1.frequency(8820);

  const int bit_len = 1;
  uint8_t bitseq[bit_len] = {1};
  generateFSK(mf.msg, bitseq, bit_len, F_SAMP, M_LEN / bit_len, 5000, 8820);
  //generateFSK2D(mf.msg, bitseq, bit_len, F_SAMP, M_LEN / bit_len, 5000, 8820);
  /*for(int k = 0; k < N_LEN; k++)
  {
    Serial.println( mf.msg[k] );
    delay(2);
  }*/
  //Serial.println( ola_max );

  f32tRealFFT(mf.fft, mf.msg, N_LEN);
  
  /*for(int k = 0; k < 2*N_LEN; k++)
    {
    Serial.println( mf_fft[k] );
    delay(2);
    }*/

  /*for(int k = 0; k < N_LEN*2; k+=2)
    {
    float re = (float)mf_fft[k];
    float im = (float)mf_fft[k+1];
    Serial.print( re );
    Serial.print("\t");
    Serial.println( im );
    delay(5);
    }*/
    
  queue1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (flag == 1) {
    if (queue1.available() >= PACKETS)
    {
      for (int k = 0; k < PACKETS; k++)
      {
        memcpy(q_rx_msg + 128 * k, queue1.readBuffer(), 256);
        queue1.freeBuffer();
      }
      flag = 0;
    }
  }

  if (flag == 0)
  {
    flag = 1;
    queue1.clear();
    
    /*
    for(int i = 0; i < N_LEN; i++)
    {
      Serial.println(q_rx_msg[i]);
    }
    */
    
    int curr = micros();
    arm_q15_to_float(q_rx_msg, rx.msg, N_LEN);
    f32tRealFFT(rx.fft, rx.msg, N_LEN);
    
    // C[k] + jC[k+1]   =   ( A[k] + jA[k+1] ) * ( B[k] + jB[k+1] )
    // C[k]             =   A[k]B[k] - A[k+1]B[k+1]
    // C[k+1]           =   A[k]B[k+1] + A[k+1]B[k]

    arm_cmplx_mult_cmplx_f32(rx.fft, mf.fft, y.fft, N_LEN);
    

    arm_cmplx_mult_cmplx_f32(rx.fft, mf.fft, y.fft, N_LEN);
    
    f32tRealIFFT(y.msg, y.fft, N_LEN);
    int diff = micros() - curr;

    /*
    for (int j = 0; j < N_LEN; j++)
    {
      Serial.println(y.msg[j]);
    }
    */


    write_ola_buffer(y.msg);
    
    /*  
    for (int j = 0; j < ola_max; j++)
    {
      Serial.println(ola_buffer[j]);
    }
    */
   
    //if(queue1.available() > PACKETS)
    //{
      Serial.println("#");
      Serial.print("Calculation Time 600MHz\t");
      Serial.println( diff );
      Serial.print("Packets Available\t");
      Serial.println( queue1.available() );
    //}
   
  }
}

    /*
    for(int j = 0; j < N_LEN; j++)
    {
      Serial.println(rx.msg[j]);
    }
    */

    
    /*
    for (int j = 0; j < 2 * N_LEN; j++)
    {
      Serial.println(rx.fft[j]);
    }
    */

    /*
    for(int k = 0; k < N_LEN*2; k+=2)
    {
      q15_t re = rx.fft[k];
      q15_t im = rx.fft[k+1];
      Serial.print( re );
      Serial.print("\t");
      Serial.println( im );
    }
    */


    
    /*
    for(int j = 45; j < 55; j++)
    {
      Serial.print(j);
      Serial.print("\t");
      Serial.print(y_f[2*j]);
      Serial.print("\t");
      Serial.println(y_f[2*j+1]);
    }
    */
    /*
    for(int k = 0; k < B_BLOCKS; k++)
    {
      f32tRealIFFT(y.msg, y.fft, N_LEN);
    }
    */
    
    /*
    float32_t a[N_LEN] = {0};
    for(int k = 0; k < B_BLOCKS; k++)
    {
      for(int j = 0; j < N_LEN; j++)
      {
        a[j] += y.fft[j];
      }
    }
    */
    
    /*     
    for(int k = 0; k < N_LEN*2; k+=2)
    {
      float re = y.fft[k];
      float im = y.fft[k+1];
      Serial.print( re );
      Serial.print("\t");
      Serial.println( im );
    }
    */

    
    /*
    ola_head -= ola_overlap;
    if(ola_head < 0)
    {
      ola_head += ola_max;
      //Serial.println('#');
    }
    for(int i = 0; i < ola_overlap; i++)
    {
      ola_buffer[ola_head++] +=  y.msg[i];
      if(ola_head >= ola_max)
      {
        ola_head = 0;
        //Serial.println('#');
      }
    }
    
    for(int i = ola_overlap; i < N_LEN; i++)
    {
      ola_buffer[ola_head++] = y.msg[i];
      
      if(ola_head >= ola_max)
      {
        ola_head = 0;
        //Serial.println('#');
      } 
    }
    */

    
    /*
    for (int j = 0; j < ola_max; j++)
    {
      Serial.println(ola_buffer[j]);
    }
    */



void generateFSK(float32_t* out_fsk_modulated, uint8_t* in_bit_sequence, int num_bits, float fs, int samples_per_bit, float f0, float f1)
{
  float f_mux;
  float32_t pre_modulated[num_bits * samples_per_bit] = {0};

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

  for (int j = 0; j < M_LEN; j++)
  {
    out_fsk_modulated[j] = pre_modulated[M_LEN - j];
  }
}


void q15tRealFFT(q15_t* out_fsk_fft, q15_t* in_fsk_msg, int in_len)
{
  arm_rfft_instance_q15 real_fft;

  arm_rfft_init_q15( &real_fft, in_len, 0, 1);

  arm_rfft_q15( &real_fft, in_fsk_msg, out_fsk_fft);
}



void q15tRealIFFT(q15_t* out_fsk_msg, q15_t* in_fsk_fft, int out_len)
{
  arm_rfft_instance_q15 real_ifft;

  arm_rfft_init_q15( &real_ifft, out_len, 1, 1);

  arm_rfft_q15( &real_ifft, in_fsk_fft, out_fsk_msg);
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



void q_cmplx_mult( const q15_t* pSrcA, const q15_t* pSrcB, q15_t* pDst, int  numSamples)
{
  for (int n = 0; n < numSamples; n++)
  {
    pDst[(2 * n) + 0] = ( (pSrcA[(2 * n) + 0]) * (pSrcB[(2 * n) + 0]) ) / 16384  -  ( 0.5*(pSrcA[(2 * n) + 1]) * (pSrcB[(2 * n) + 1]) ) / 16384 ;
    pDst[(2 * n) + 1] = ( (pSrcA[(2 * n) + 0]) * (pSrcB[(2 * n) + 1]) ) / 16384  +  ( 0.5*(pSrcA[(2 * n) + 1]) * (pSrcB[(2 * n) + 0]) ) / 16384 ;
  }
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
