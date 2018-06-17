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
  }

}
