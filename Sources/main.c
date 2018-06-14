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

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
// Simple OS
#include "OS.h"
// Analog functions
#include "analog.h"
// LPTimer functions
#include "LPTMR.h"
// LED functions
#include "LEDs.h"

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use
#define THREAD_STACK_SIZE 100
// Number of analog channels
#define NB_ANALOG_CHANNELS 2


// Thread stacks
// Init modules thread stack
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
// Analog thread stacks
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
// LPTMR thread stack
OS_THREAD_STACK(LPTMRThreadStack, THREAD_STACK_SIZE);

// Thread priorities. 0 = highest priority
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {1, 2};

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  }
};


/*! @brief Routine to use LPTMR.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 */
void LPTMRThread(void* data)
{
  for (;;)
  {
    OS_SemaphoreWait(LPTMRSem, 0);

    LEDs_Toggle(LED_GREEN);
    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
  }
}



/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  //
  bool ledsInitResult = LEDs_Init();

  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Initialise the low power timer to tick every 10 ms
  LPTMR_Init(500);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority

  // Create threads for analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  //
  error = OS_ThreadCreate(LPTMRThread, NULL, &LPTMRThreadStack[THREAD_STACK_SIZE-1], 4);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
