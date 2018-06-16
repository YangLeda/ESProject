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
#define THREAD_STACK_SIZE 1000
// Baud rate
#define BAUD 115200
// Tower number
#define TOWER_NUMBER 2324



// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
OS_THREAD_STACK(PIT0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT1ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTReceiveThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTTransmitThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);

// Thread priorities - 0 is highest priority
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};


// Non-volatile variables for tower number and tower mode
uint16union_t* volatile NvTowerNb;
uint16union_t* volatile NvTowerMd;

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  uint16_t rms;
  uint32_t sum_rms_squares;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .rms = 2.5,
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .rms = 2.5,
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .rms = 2.5,
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16
  }
};


/*! @brief Initialises modules.
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

  // PIT0 - period in nanosecond is 1 / 50 / 16 * 1000000000 = 125e4
  PIT_Set(0, 125e6 , FALSE);
  PIT_Enable(0, TRUE);
  // PIT1 - period in nanosecond is 500e6
  PIT_Set(1, 500e6 , FALSE);
  PIT_Enable(1, TRUE);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Orange LED on if all init success
  if (ledsInitResult && packetInitResult && flashInitResult && analogInitResult && pitInitResults)
    LEDs_On(LED_BLUE);

  // Set tower number and tower mode
  Flash_AllocateVar((volatile void**) &NvTowerNb, sizeof(*NvTowerNb));
  Flash_AllocateVar((volatile void**) &NvTowerMd, sizeof(*NvTowerMd));
  if (NvTowerNb->l == 0xFFFF || NvTowerMd->l == 0xFFFF) // Not set before
  {
    Flash_Write16((uint16*) NvTowerNb, TOWER_NUMBER);
    Flash_Write16((uint16*) NvTowerMd, 0x01);
  }

  // Send startup packets
  Packet_Put(0x04, 0, 0, 0); // Special - Startup
  Packet_Put(0x09, 'v', 1, 0); // Special - Version number
  Packet_Put(0x0B, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi); // Tower Number
  Packet_Put(0x0D, 1, NvTowerMd->s.Lo, NvTowerMd->s.Hi); // Tower Mode


  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}


/*! @brief 16 samples per cycle
 *
 */
void PIT0Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT0Sem, 0);

    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
  }
}


/*! @brief Alarm Timing
 *
 */
void PIT1Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT1Sem, 0);

    LEDs_Toggle(LED_GREEN);
  }
}


/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
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

    // Real voltage = analogInputValue / 3276.7
    realVoltage = analogInputValue * 305 / 1e3;

    // Calculate RMS Voltage
    Algorithm_RMS(&(analogData->rms), &(analogData->sum_rms_squares), realVoltage);

    // test put
    Packet_Put(0xff, analogData->channelNb, analogData->rms >> 8, analogData->rms);

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

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialize low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create InitModules thread
  error = OS_ThreadCreate(InitModulesThread, NULL, &InitModulesThreadStack[THREAD_STACK_SIZE - 1], 0);
  // Create UARTReceive thread
  error = OS_ThreadCreate(UARTReceiveThread, NULL, &UARTReceiveThreadStack[THREAD_STACK_SIZE-1], 1);
  // Create UARTTransmit thread
  error = OS_ThreadCreate(UARTTransmitThread, NULL, &UARTTransmitThreadStack[THREAD_STACK_SIZE-1], 2);
  // Create Analog threads
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
    error = OS_ThreadCreate(AnalogThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  // Create PIT0 thread
  error = OS_ThreadCreate(PIT0Thread, NULL, &PIT0ThreadStack[THREAD_STACK_SIZE-1], 6);
  // Create PIT1 thread
  error = OS_ThreadCreate(PIT1Thread, NULL, &PIT1ThreadStack[THREAD_STACK_SIZE-1], 7);
  // Create Packet thread
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 8);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
