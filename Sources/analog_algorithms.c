/*! @file
 *
 *  @brief Routines and algorithms for analog VRR on the TWR-K70F120M.
 *
 *  This contains the functions for analog VRR algorithms.
 *
 *  @author Leda Yang
 *  @date 2018-06-26
 */

#include "analog_algorithms.h"

/*! @brief Handle RMS calculation.
 *
 *  Using one step Newton's method.
 *  @param ch The channel number.
 *  @param realVoltage The converted sample voltage.
 *  @return void
 */
void Algorithm_RMS(uint8_t channelNb, int16_t realVoltage)
{
  // Put new voltage square into array
  AnalogThreadData[channelNb].voltageSquares[AnalogThreadData[channelNb].sampleCount] = (uint32_t) realVoltage * (uint32_t) realVoltage;
  AnalogThreadData[channelNb].sampleCount ++;

  // Calculate and update new rms each cycle
  if (AnalogThreadData[channelNb].sampleCount > 15)
  {
    uint32_t sumvoltageSquares = 0;
    for (uint8_t i = 0; i < 16; i++)
      sumvoltageSquares += AnalogThreadData[channelNb].voltageSquares[i];

    // Avoid divide by 0
    if (AnalogThreadData[channelNb].rms == 0)
      AnalogThreadData[channelNb].rms = 1;

    AnalogThreadData[channelNb].rms = (AnalogThreadData[channelNb].rms + (sumvoltageSquares >> 4) / AnalogThreadData[channelNb].rms) >> 1;

    AnalogThreadData[channelNb].sampleCount = 0;

    // Signal cycle semaphore when 16 samples are collected on ch0
    if (channelNb == 0)
      OS_SemaphoreSignal(CycleSem);
  }
}

/*! @brief Handle frequency calculation and update timer.
 *
 *  @param realVoltage The converted sample voltage.
 *  @return void
 */
void Algorithm_Frequency(int16_t realVoltage)
{
  // Zero crossing - low to high
  if (AnalogThreadData[0].sample < 0 && realVoltage > 0)
  {
    switch (AnalogThreadData[0].frequencyTrackingCrossingCount)
    {
      case 0: // First zero crossing
        // Calculate left time offset
        AnalogThreadData[0].frequencyTrackingLeftFixTime = realVoltage * AnalogThreadData[0].samplePeriod / (realVoltage - AnalogThreadData[0].sample);
        AnalogThreadData[0].frequencyTrackingSampleCount = 0;
        AnalogThreadData[0].frequencyTrackingCrossingCount = 1;
        break;
      case 1: // Second zero crossing
        // Calculate right time offset
        AnalogThreadData[0].frequencyTrackingRightFixTime = realVoltage * AnalogThreadData[0].samplePeriod / (realVoltage - AnalogThreadData[0].sample);
        AnalogThreadData[0].frequencyTrackingCrossingCount = 0;

        uint64_t new_cycle_period = AnalogThreadData[0].frequencyTrackingSampleCount * AnalogThreadData[0].samplePeriod + AnalogThreadData[0].frequencyTrackingLeftFixTime - AnalogThreadData[0].frequencyTrackingRightFixTime;
        // Calculate frequency
        uint16_t new_frequency = 1e10 / new_cycle_period;

        // Update frequency
        if (AnalogThreadData[0].rms >= 150)
        {
          AnalogThreadData[0].frequency = new_frequency;
          AnalogThreadData[0].samplePeriod = new_cycle_period >> 4;
        }
        else // Below 1.5V then 50Hz assumed
        {
          AnalogThreadData[0].frequency = 500;
          AnalogThreadData[0].samplePeriod = INITIAL_SAMPLE_PERIOD;
        }

        // Update timer
        PIT_Set(AnalogThreadData[0].samplePeriod, FALSE);
        break;
      default:
        break;
    }
  }
  AnalogThreadData[0].sample = realVoltage;
  AnalogThreadData[0].frequencyTrackingSampleCount ++;
}

/*! @brief 16 samples Fast Fourier Transform by recursive Cooley-Tukey method.
 *
 *  Reference: https://www.algorithm-archive.org/chapters/FFT/cooley_tukey.html
 *  @param sample The 16 samples array.
 *  @return void
 */
void FFT_Cooley_Tukey(double complex *sample)
{
  // Bit reverse
  for (uint8_t i = 0; i < 16; i ++)
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
      double complex tmp = sample[i];
      sample[i] = sample[n];
      sample[n] = tmp;
    }
  }

  for (uint8_t i = 0; i < 4; i ++)
  {
    uint8_t stride = pow(2, i);
    double complex w = cexp(-2.0 * I * M_PI / stride);
    for (uint8_t j = 0; j < 16; j += stride)
    {
      double complex v = 1.0;
      for (uint8_t k = 0; k < stride / 2; ++k)
      {
        sample[k + j + stride / 2] = sample[k + j] - v * sample[k + j + stride / 2];
        sample[k + j] -= (sample[k + j + stride / 2] - sample[k + j]);
        v *= w;
      }
    }
  }
}

/*! @brief Calculate the magnitude of a complex number by Newton's method.
 *
 *  @return uint16_t The magnitude result.
 */
uint16_t Magnitude(int16_t real, int16_t image)
{
  uint32_t squareMagnitude;
  uint16_t magnitude;

  squareMagnitude = (uint32_t) ((int32_t) real * (int32_t) real + (int32_t) image * (int32_t) image);

  // Guess
  magnitude = (uint16_t) (squareMagnitude / 2);

  // Five steps
  for (uint8_t i = 0; i < 5; i ++)
    if (magnitude != 0) // Avoid devision by zero
      magnitude = (uint16_t) ((squareMagnitude / 2 + magnitude) / 2);

  return magnitude;
}
