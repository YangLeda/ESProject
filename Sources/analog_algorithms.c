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

// Newton's method
// Sample number is 16
// " / 16" == " >> 4", " / 2" == " >> 1"
void Algorithm_RMS(uint8_t channelNb, int16_t realVoltage)
{
  // Subtract one sample
  AnalogChannelData[channelNb].sum_rms_squares -= AnalogChannelData[channelNb].sum_rms_squares >> 4;
  // Add new RMS square
  AnalogChannelData[channelNb].sum_rms_squares += (uint32_t) realVoltage * realVoltage;
  
  // Avoid divide by 0
  if (AnalogChannelData[channelNb].rms == 0)
    AnalogChannelData[channelNb].rms = 1;
  
  // New rms
  AnalogChannelData[channelNb].rms = (AnalogChannelData[channelNb].rms + (AnalogChannelData[channelNb].sum_rms_squares >> 4) / AnalogChannelData[channelNb].rms) >> 1;
  
}
