/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Leda Yang
 *  @date 2018-06-25
 */

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// new types
#include "types.h"
#include "PE_Types.h"
#include "MK70F12.h"
#include "OS.h"

// Number of analog channels
#define NB_ANALOG_CHANNELS 3


// Struct to store data of an analog channel
typedef struct
{
  uint16_t rms;
  uint32_t sum_rms_squares;
} TAnalogChannelData;

extern TAnalogChannelData AnalogChannelData[NB_ANALOG_CHANNELS];

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
void Algorithm_RMS(uint8_t channelNb, int16_t realVoltage);



#endif
