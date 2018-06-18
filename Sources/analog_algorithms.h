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


/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  uint32_t voltage_squares[16];
  uint8_t sample_count; // 0-15
  uint16_t rms;
  uint8_t voltage_status_code; // 0 - In boundary; 1 - Too high; 2 - Too low
  uint8_t tapping_status_code; // 0 - Not tapping; 1 - Lower; 2 - Raise
  bool timing;
  uint16_t target_timing_count;
  uint16_t current_timing_count;
  uint16_t frequency; // Hz*10
} TAnalogThreadData;

extern TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS];

// Semaphores of cycle
OS_ECB* CycleSem;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
void Algorithm_RMS(uint8_t ch, int16_t realVoltage);



#endif
