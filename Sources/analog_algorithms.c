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
uint16_t Algorithm_RMS(uint16_t *rms, uint32_t *sum_rms_squares, int16_t realVoltage)
{
  // Subtract one sample
  *sum_rms_squares -= *sum_rms_squares >> 4;
  // Add new RMS square
  *sum_rms_squares += (uint32_t) realVoltage * realVoltage;
  
  // Avoid divide by 0
  if (*rms == 0)
    *rms = 1;
  
  // New rms
  *rms = (*rms + (*sum_rms_squares >> 4) / *rms) >> 1;

  
}
