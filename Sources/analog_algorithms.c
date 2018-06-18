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
void Algorithm_RMS(uint8_t ch, int16_t realVoltage)
{
  // Put new voltage square into array
  AnalogThreadData[ch].voltage_squares[AnalogThreadData[ch].sample_count] = (uint32_t) realVoltage * realVoltage;
  AnalogThreadData[ch].sample_count ++;

  // Calculate and update new rms each cycle
  if (AnalogThreadData[ch].sample_count > 15)
  {
    uint32_t sumvoltageSquares = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
      sumvoltageSquares += AnalogThreadData[ch].voltage_squares[i];
    }

    // Avoid divide by 0
    if (AnalogThreadData[ch].rms == 0)
      AnalogThreadData[ch].rms = 1;

    AnalogThreadData[ch].rms = (AnalogThreadData[ch].rms + (sumvoltageSquares >> 4) / AnalogThreadData[ch].rms) >> 1;

    AnalogThreadData[ch].sample_count = 0;

    // Signal cycle semaphore when 16 samples are collected on ch0
    if (ch == 0)
      OS_SemaphoreSignal(CycleSem);
  }
}

float calculateTimeOffset(int16_t sample1, int16_t sample2)
{
  float gradient = (sample2 - sample1)/(1);
  float timeOffset = ((float)(-sample1) / gradient);
  return timeOffset;
}

void Algorithm_Frequency(int16_t realVoltage)
{
  // Zero crossing - low to high
  if (AnalogThreadData[0].last_sample <= 0 && realVoltage >= 0)
  {

    switch (AnalogThreadData[0].crossingNb)
    {
      // First zero crossing
      case 1:
        // Calculate time offset (fraction of a sample) between samples[count] and the zero crossing
        AnalogThreadData[0].offset1 = calculateTimeOffset(AnalogThreadData[0].last_sample, realVoltage);
        AnalogThreadData[0].frequency_tracking_sample_count = 0; // Reset sample offset
        AnalogThreadData[0].crossingNb = 2; // We've found the first zero crossing, find the next..
        break;

      // Second zero crossing
      case 2:
        // Calculate time offset (fraction of a sample) between samples[count] and the zero crossing
        AnalogThreadData[0].offset2 = calculateTimeOffset(AnalogThreadData[0].last_sample, realVoltage);
        // Number of samples between the first zero crossing and the second zero crossing
        // Minus the time offset of the first zero crossing
        // Plus the time offset of the second zero crossing
        // Multiplied by the sample period..
        //float period_s =  // Convert Period in ns to period in s .. Is there a better way?
        double new_period = (AnalogThreadData[0].frequency_tracking_sample_count - AnalogThreadData[0].offset1 + AnalogThreadData[0].offset2) * ((float) 125e4 / 1000000000); // Period of wave in s
        uint16_t frequency = (1 / (new_period)) ; // Calculate frequency
        AnalogThreadData[0].frequency = frequency;



        AnalogThreadData[0].crossingNb = 1;
        break;

    } // End switch

    // Increment sample offset
    AnalogThreadData[0].frequency_tracking_sample_count ++;
  }


}


