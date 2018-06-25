/* ###################################################################
 **     Filename    : main.c
 **     Project     : Project
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

// CPU module contains low level hardware initialization routines
#include "Cpu.h"
// Simple OS
#include "OS.h"
// Analog functions
#include "analog.h"
// LPTimer functions
#include "PIT.h"
// LED functions
#include "LEDs.h"
// Packet functions
#include "Packet.h"
// Flash functions
#include "Flash.h"
// Algorithms
#include "analog_algorithms.h"



// Arbitrary thread stack size - big enough for stacking of interrupts and OS use
#define THREAD_STACK_SIZE 100
// Baud rate
#define BAUD 115200


#define ANALOG_RAISE_CHANNEL 0
#define ANALOG_LOWER_CHANNEL 1
#define ANALOG_ALARM_CHANNEL 2

#define ANALOG_5V 16384

#define INITIAL_TIMING_MODE 1
#define TIME_DEFINITE 5e9


// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
OS_THREAD_STACK(PIT0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(CycleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTReceiveThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTTransmitThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(SpectrumThreadStack, THREAD_STACK_SIZE);

// Thread priorities - 0 is highest priority
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};


// Flash WritePhrase is in critical section
// Non-volatile variables for numbers of raise/lower events
uint16union_t* volatile NumOfRaise;
uint16union_t* volatile NumOfLower;
// 1 - definite timing mode; 2 - inverse timing mode
uint8_t* volatile TimingMode;


void UpadateAnalogOutput(void);
void UpadateTiming(void);




/*! @brief Analog thread configuration data
 *
 */
TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .voltageSquares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500},
    .sampleCount = 15,
    .rms = 250,
    .tappingStatusCode = 0,
    .timingStatusCode = 0,
    .targetTimingCount = 0,
    .currentTimingCount = 0,
    .frequency = 500,
    .sample = 0,
    .frequencyTrackingSampleCount = 0,
    .frequencyTrackingCrossingCount = 0,
    .frequencyTrackingLeftFixTime = 0,
    .frequencyTrackingRightFixTime = 0,
    .samplePeriod = INITIAL_SAMPLE_PERIOD
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .voltageSquares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500},
    .sampleCount = 15,
    .rms = 250,
    .tappingStatusCode = 0,
    .timingStatusCode = 0,
    .targetTimingCount = 0,
    .currentTimingCount = 0,
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .voltageSquares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500},
    .sampleCount = 15,
    .rms = 250,
    .tappingStatusCode = 0,
    .timingStatusCode = 0,
    .targetTimingCount = 0,
    .currentTimingCount = 0,
  }
};

void Tap(uint8_t channelNb)
{
  if (AnalogThreadData[channelNb].rms > 300)
  {
    AnalogThreadData[channelNb].tappingStatusCode = 1;
    // Update event number to flash
    Flash_Write16((uint16_t*) NumOfLower, NumOfLower->l + 1);
  }
  else if (AnalogThreadData[channelNb].rms < 200)
  {
    AnalogThreadData[channelNb].tappingStatusCode = 2;
    // Update event number to flash
    Flash_Write16((uint16_t*) NumOfRaise, NumOfRaise->l + 1);
  }
}

/*! @brief Initializes modules and send a startup packet.
 *
 */
static void InitModulesThread(void* pData)
{
  // Initialize modules
  bool ledsInitResult = LEDs_Init();
  bool packetInitResult = Packet_Init(BAUD, CPU_BUS_CLK_HZ);
  bool flashInitResult = Flash_Init();
  bool analogInitResult = Analog_Init(CPU_BUS_CLK_HZ);
  bool pitInitResults = PIT_Init(CPU_BUS_CLK_HZ);
  // Sampling PIT timers - PIT0 for analog channel 0, PIT1 for analog channel 1 & 2
  PIT_Set(INITIAL_SAMPLE_PERIOD, FALSE);
  PIT_Enable(TRUE);

  // Initialize cycle semaphore
  CycleSem = OS_SemaphoreCreate(0);
  SpectrumSem = OS_SemaphoreCreate(0);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Initialize number of raise/lower events to 0
  Flash_AllocateVar((volatile void**) &NumOfRaise, sizeof(*NumOfRaise));
  Flash_AllocateVar((volatile void**) &NumOfLower, sizeof(*NumOfLower));
  Flash_AllocateVar((volatile void**) &TimingMode, sizeof(*TimingMode));
  Flash_Write16((uint16_t*) NumOfRaise, 0);
  Flash_Write16((uint16_t*) NumOfLower, 0);
  Flash_Write8((uint8_t*) TimingMode, INITIAL_TIMING_MODE);

  // Send startup packets
  Packet_Put(0x04, 0, 0, 0);


  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}


/*! @brief Sample rate 16 per cycle.
 *
 */
void PIT0Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT0Sem, 0);

    // Signal each analog channel to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
  }
}


/*! @brief Each cycle. Check each channel's RMS. Set and update 3 timers. Control 3 outputs.
 *
 */
void CycleThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(CycleSem, 0);

    UpadateTiming();

    UpadateAnalogOutput();

  }
}

/*! @brief
 *
 */
void UpadateTiming(void)
{
  // Check each channel's RMS and update timers
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    if (AnalogThreadData[analogNb].rms < 200 || AnalogThreadData[analogNb].rms > 300) // RMS out of range
    {
      if (*TimingMode == 1) // Definite mode
      {
        switch (AnalogThreadData[analogNb].timingStatusCode)
        {
          case 0:// Not timing
          case 2: //or is inverse timing
            AnalogThreadData[analogNb].targetTimingCount = (uint16_t) (TIME_DEFINITE / AnalogThreadData[0].samplePeriod) >> 4;
            AnalogThreadData[analogNb].currentTimingCount = 0;
            AnalogThreadData[analogNb].timingStatusCode = 1;
            break;
          case 1:
            // Count Time
            AnalogThreadData[analogNb].currentTimingCount ++;
            if (AnalogThreadData[analogNb].currentTimingCount >= AnalogThreadData[analogNb].targetTimingCount) // Time out
            {
              // Already tap, stop timing count
              AnalogThreadData[analogNb].timingStatusCode = 3;
              Tap(analogNb);
            }
            break;
        }

      }
      else // Inverse mode
      {
        // Deviation of voltage RMS
        int16_t deviation = AnalogThreadData[analogNb].rms - 250;
        if (deviation < 0)
          deviation = - deviation;

        // Target number of counts before time out
        uint8_t deviationTimingCount;
        deviationTimingCount = 25 * AnalogThreadData[0].frequency / deviation;

        switch (AnalogThreadData[analogNb].timingStatusCode)
        {
          case 0:// Not timing
          case 1: // Not timing or is definite timing, start inverse timing
            AnalogThreadData[analogNb].targetTimingCount = deviationTimingCount;
            AnalogThreadData[analogNb].currentTimingCount = 0;
            AnalogThreadData[analogNb].timingStatusCode = 2;
            break;
          case 2:
            // Count Time
            AnalogThreadData[analogNb].currentTimingCount ++;

            if (AnalogThreadData[analogNb].currentTimingCount >= AnalogThreadData[analogNb].targetTimingCount) // Time out
            {
              // Already tap, stop timing count
              AnalogThreadData[analogNb].timingStatusCode = 3;
              Tap(analogNb);
            }

            // New target count according to remaining rate
            uint8_t newTargetTimingCount;
            newTargetTimingCount = (1 - (float) AnalogThreadData[analogNb].currentTimingCount / AnalogThreadData[analogNb].targetTimingCount) * deviationTimingCount + AnalogThreadData[analogNb].currentTimingCount;
            AnalogThreadData[analogNb].targetTimingCount = newTargetTimingCount;
            break;
        }


      }
    }
    else // RMS in range
    {
      AnalogThreadData[analogNb].tappingStatusCode = 0;
      AnalogThreadData[analogNb].timingStatusCode = 0;
    }
  }

}




/*! @brief
 *
 */
void UpadateAnalogOutput(void)
{
  bool hasAlarming = FALSE;
  bool hasRaiseTapping = FALSE;
  bool hasLowerTapping = FALSE;

  // Check each analog channel status
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    if (AnalogThreadData[analogNb].rms > 300 || AnalogThreadData[analogNb].rms < 200)
      hasAlarming = TRUE;
    if (AnalogThreadData[analogNb].tappingStatusCode == 1)
      hasLowerTapping = TRUE;
    else if (AnalogThreadData[analogNb].tappingStatusCode == 2)
      hasRaiseTapping = TRUE;
  }

  // If one or more channel is alarming
  if (hasAlarming)
  {
    LEDs_On(LED_BLUE);
    Analog_Put(ANALOG_ALARM_CHANNEL, ANALOG_5V);
  }
  else
  {
    LEDs_Off(LED_BLUE);
    Analog_Put(ANALOG_ALARM_CHANNEL, 0);
  }

  // If one or more channel is raise tapping
  if (hasRaiseTapping)
  {
    LEDs_On(LED_YELLOW);
    Analog_Put(ANALOG_RAISE_CHANNEL, ANALOG_5V);
  }
  else
  {
    LEDs_Off(LED_YELLOW);
    Analog_Put(ANALOG_RAISE_CHANNEL, 0);
  }

  // If one or more channel is lower tapping
  if (hasLowerTapping)
  {
    LEDs_On(LED_GREEN);
    Analog_Put(ANALOG_LOWER_CHANNEL, ANALOG_5V);
  }
  else
  {
    LEDs_Off(LED_GREEN);
    Analog_Put(ANALOG_LOWER_CHANNEL, 0);
  }
}

/*! @brief Samples a value on an ADC channel
 *
 */
void AnalogThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;
    int16_t realVoltage;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);

    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);

    // Real voltage = analogInputValue / 3276.7 (Result unit: 10^-2V)
    realVoltage = analogInputValue * 305 / 1e4;



    // Calculate RMS Voltage - rms updated per cycle
    Algorithm_RMS(analogData->channelNb, realVoltage);

    // Find zero crossing
    if (analogData->channelNb == 0)
    {
      ///
      Analog_Put(3, analogInputValue);

      Algorithm_Frequency(realVoltage);
    }

  }
}


/*! @brief Routine to receive and handle packets.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 */
void PacketThread(void* data)
{
  for (;;)
  {
    if (Packet_Get())
      Handle_Packet();
  }
}


/*! @brief
 *
 *  @param pData is not used but is required by the OS to create a thread.
 */
void SpectrumThread(void* data)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(SpectrumSem, 0);

    Packet_Put(0x19, 0, 0, 0);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialize low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, TRUE);



  // Create InitModules thread
  error = OS_ThreadCreate(InitModulesThread, NULL, &InitModulesThreadStack[THREAD_STACK_SIZE - 1], 0);
  // Create UARTReceive thread
  error = OS_ThreadCreate(UARTReceiveThread, NULL, &UARTReceiveThreadStack[THREAD_STACK_SIZE - 1], 1);
  // Create UARTTransmit thread
  error = OS_ThreadCreate(UARTTransmitThread, NULL, &UARTTransmitThreadStack[THREAD_STACK_SIZE - 1], 2);
  // Create Analog threads
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
    error = OS_ThreadCreate(AnalogThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  // Create PIT0 thread
  error = OS_ThreadCreate(PIT0Thread, NULL, &PIT0ThreadStack[THREAD_STACK_SIZE - 1], 6);
  // Create Cycle thread
  error = OS_ThreadCreate(CycleThread, NULL, &CycleThreadStack[THREAD_STACK_SIZE - 1], 7);
  // Create Packet thread
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE - 1], 8);
  // Create Packet thread
  error = OS_ThreadCreate(SpectrumThread, NULL, &SpectrumThreadStack[THREAD_STACK_SIZE - 1], 9);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
