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

#define PERIOD_OF_PIT0 125e4

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

void Algorithm_Frequency(int16_t realVoltage)
{
  // Zero crossing - low to high
  if (AnalogThreadData[0].last_sample <= 0 && realVoltage >= 0)
  {
    switch (AnalogThreadData[0].zero_crossing_count)
    {
      // First zero crossing
      case 0:
        // Calculate left time offset
        AnalogThreadData[0].left_fix_time = realVoltage * PERIOD_OF_PIT0 / (realVoltage - AnalogThreadData[0].last_sample);
        AnalogThreadData[0].frequency_tracking_sample_count = 0; // Reset sample offset
        AnalogThreadData[0].zero_crossing_count = 1; // We've found the first zero crossing, find the next..
        break;

      // Second zero crossing
      case 1:
        // Calculate right time offset
        AnalogThreadData[0].right_fix_time = AnalogThreadData[0].last_sample * PERIOD_OF_PIT0 / (AnalogThreadData[0].last_sample - realVoltage);
        // In nano second
        uint32_t new_period = AnalogThreadData[0].frequency_tracking_sample_count * PERIOD_OF_PIT0 + AnalogThreadData[0].left_fix_time - AnalogThreadData[0].right_fix_time
        uint16_t frequency = 1e9 / new_period ; // Calculate frequency
        
        // Update frequency
        if (AnalogThreadData[0].rms >= 150)
        {
          AnalogThreadData[0].frequency = frequency;
          // set PIT0
        }
        else // Below 1.5V then 50Hz assumed
        {
          AnalogThreadData[0].frequency = 500;
          // set PIT0
        }
   
        AnalogThreadData[0].zero_crossing_count = 0;
        break;
    } // End switch
  } // End if
  // Increment sample offset
  AnalogThreadData[0].frequency_tracking_sample_count ++;
}
