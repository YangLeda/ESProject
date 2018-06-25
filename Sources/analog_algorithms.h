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
#include "PIT.h"

#include "complex.h"
#include "math.h"
#define M_PI 3.14159265358979323846264338327950288



// Number of analog channels
#define NB_ANALOG_CHANNELS 3
// Initial sample period in nanosecond is 1 / 50 / 16 * 1000000000 = 125e4
#define INITIAL_SAMPLE_PERIOD 125e4


/*! @brief Data structure used to pass analog data to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  uint32_t voltageSquares[16]; // Square voltage of 16 samples
  uint8_t sampleCount; // Index of next sample in the array
  uint16_t rms; // Voltage RMS
  uint8_t tappingStatusCode; // 0 - Not tapping; 1 - Lower tapping; 2 - Raise tapping
  uint8_t timingStatusCode; // 0 - Not timing; 1 - Definite timing; 2 - Inverse timing; 3 - Already tapping
  uint16_t targetTimingCount; // Number of counts before tapping
  uint16_t currentTimingCount; // Current timing count
  uint16_t frequency; // Hz*10
  int16_t sample; // Last voltage sample
  uint8_t frequencyTrackingSampleCount; // Number of samples between zero crossings
  uint8_t frequencyTrackingCrossingCount; // Number zero crossings found
  uint32_t frequencyTrackingLeftFixTime; // Nano second
  uint32_t frequencyTrackingRightFixTime; // Nano second
  uint32_t samplePeriod; // Sampling period
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

void Algorithm_Frequency(int16_t realVoltage);

void FFT_Cooley_Tukey(double complex *sample);

uint16_t Magnitude(int16_t real, int16_t image);

#endif
