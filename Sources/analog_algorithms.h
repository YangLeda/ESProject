/*! @file
 *
 *  @brief Routines and algorithms for analog VRR on the TWR-K70F120M.
 *
 *  This contains the functions for analog VRR algorithms.
 *
 *  @author Leda Yang
 *  @date 2018-06-26
 */

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// Include modules
#include "types.h"
#include "PE_Types.h"
#include "MK70F12.h"
#include "OS.h"
#include "PIT.h"
// Include for spectrum calculations
#include "complex.h"
#include "math.h"
#define M_PI 3.14159265358979323846264338327950288

// Number of analog channels
#define NB_ANALOG_CHANNELS 3
// Initial sample period is 1 / 50 / 16 * 1000000000 = 125e4 nanosecond
#define INITIAL_SAMPLE_PERIOD 125e4
// Analog output channels
#define ANALOG_RAISE_CHANNEL 0
#define ANALOG_LOWER_CHANNEL 1
#define ANALOG_ALARM_CHANNEL 2
// Analog 5V
#define ANALOG_5V 16384
// 1 - Definite mode; 2 - Inverse mode
#define INITIAL_TIMING_MODE 1
// Definite time is 5s
#define TIME_DEFINITE 5e9

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

/*! @brief Handle RMS calculation.
 *
 *  Using one step Newton's method.
 *  @param ch The channel number.
 *  @param realVoltage The converted sample voltage.
 *  @return void
 */
void Algorithm_RMS(uint8_t ch, int16_t realVoltage);

/*! @brief Handle frequency calculation and update timer.
 *
 *  @param realVoltage The converted sample voltage.
 *  @return void
 */
void Algorithm_Frequency(int16_t realVoltage);

/*! @brief 16 samples Fast Fourier Transform by recursive Cooley-Tukey method.
 *
 *  Reference: https://www.algorithm-archive.org/chapters/FFT/cooley_tukey.html
 *  @param sample The 16 samples array.
 *  @return void
 */
void FFT_Cooley_Tukey(double complex *sample);

/*! @brief Calculate the magnitude of a complex number by Newton's method.
 *
 *  @return uint16_t The magnitude result.
 */
uint16_t Magnitude(int16_t real, int16_t image);

#endif
