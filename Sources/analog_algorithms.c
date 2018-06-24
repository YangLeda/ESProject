/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Leda Yang
 *  @date 2018-06-25
 */

#include "analog_algorithms.h"

// One step Newton's method - RMS(n+1) = (RMS(n) + mean_rms_squares / RMS(n)) / 2
void Algorithm_RMS(uint8_t channelNb, int16_t realVoltage)
{
  // Put new voltage square into array
  AnalogThreadData[channelNb].voltage[AnalogThreadData[channelNb].sample_count] = realVoltage;
  AnalogThreadData[channelNb].voltage_squares[AnalogThreadData[channelNb].sample_count] = (uint32_t) realVoltage * realVoltage;
  AnalogThreadData[channelNb].sample_count ++;



  // Calculate and update new rms each cycle
  if (AnalogThreadData[channelNb].sample_count > 15)
  {
    uint32_t sumvoltageSquares = 0;
    for (uint8_t i = 0; i < 16; i++)
      sumvoltageSquares += AnalogThreadData[channelNb].voltage_squares[i];

    // Avoid divide by 0
    if (AnalogThreadData[channelNb].rms == 0)
      AnalogThreadData[channelNb].rms = 1;

    AnalogThreadData[channelNb].rms = (AnalogThreadData[channelNb].rms + (sumvoltageSquares >> 4) / AnalogThreadData[channelNb].rms) >> 1;

    AnalogThreadData[channelNb].sample_count = 0;

    // Signal cycle semaphore when 16 samples are collected on ch0
    if (channelNb == 0)
    {

      OS_SemaphoreSignal(CycleSem);
    }

  }
}

void Algorithm_Frequency(int16_t realVoltage)
{
  // Zero crossing - low to high
  if (AnalogThreadData[0].last_sample < 0 && realVoltage > 0)
  {
    switch (AnalogThreadData[0].zero_crossing_count)
    {
      case 0: // First zero crossing
        // Calculate left time offset
        AnalogThreadData[0].left_fix_time = realVoltage * AnalogThreadData[0].sample_period / (realVoltage - AnalogThreadData[0].last_sample);
        AnalogThreadData[0].frequency_tracking_sample_count = 0;
        AnalogThreadData[0].zero_crossing_count = 1;
        break;
      case 1: // Second zero crossing
        // Calculate right time offset
        AnalogThreadData[0].right_fix_time = realVoltage * AnalogThreadData[0].sample_period / (realVoltage - AnalogThreadData[0].last_sample);
        AnalogThreadData[0].zero_crossing_count = 0;
        // In nano second
        uint64_t new_cycle_period = AnalogThreadData[0].frequency_tracking_sample_count * AnalogThreadData[0].sample_period + AnalogThreadData[0].left_fix_time - AnalogThreadData[0].right_fix_time;
        uint16_t new_frequency = 1e10 / new_cycle_period; // Calculate frequency

        // Update frequency
        if (AnalogThreadData[0].rms >= 150)
        {
          AnalogThreadData[0].frequency = new_frequency;
          AnalogThreadData[0].sample_period = new_cycle_period >> 4;
        }
        else // Below 1.5V then 50Hz assumed
        {
          AnalogThreadData[0].frequency = 500;
          AnalogThreadData[0].sample_period = INITIAL_SAMPLE_PERIOD;
        }

        PIT_Set(AnalogThreadData[0].sample_period, FALSE);
        break;
      default:
        break;
    }
  }
  AnalogThreadData[0].last_sample = realVoltage;
  AnalogThreadData[0].frequency_tracking_sample_count ++;
}


static void FFT_Bit_Reverse(double complex *X)
{
  for (uint8_t i = 0; i < 16; ++i)
  {
    uint8_t n = i;
    uint8_t a = i;
    uint8_t count = 3;

    n >>= 1;
    while (n > 0)
    {
        a = (a << 1) | (n & 1);
        count--;
        n >>= 1;
    }
    n = (a << count) & ((1 << 4) - 1);

    if (n > i)
    {
      double complex tmp = X[i];
      X[i] = X[n];
      X[n] = tmp;
    }
  }
}

void FFT_Cooley_Tukey(double complex *X)
{
  FFT_Bit_Reverse(X);
  for (uint8_t i = 0; i < 4; i ++)
  {
    uint8_t stride = pow(2, i);
    double complex w = cexp(-2.0 * I * M_PI / stride);
    for (uint8_t j = 0; j < 16; j += stride)
    {
      double complex v = 1.0;
      for (uint8_t k = 0; k < stride / 2; ++k)
      {
        X[k + j + stride / 2] = X[k + j] - v * X[k + j + stride / 2];
        X[k + j] -= (X[k + j + stride / 2] - X[k + j]);
        v *= w;
      }
    }
  }
}
