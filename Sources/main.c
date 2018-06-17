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

#define ANALOG_RAISE_CHANNEL 0
#define ANALOG_LOWER_CHANNEL 1
#define ANALOG_ALARM_CHANNEL 2

#define ANALOG_5V 16384
#define TIME_DEFINITE 1e9

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
OS_THREAD_STACK(PIT0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT1ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT2ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT3ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTReceiveThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTTransmitThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);

// Thread priorities - 0 is highest priority
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};


// Non-volatile variables for ...
uint16union_t* volatile NumOfRaise;
uint16union_t* volatile NumOfLower;


/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  uint16_t rms;
  uint32_t sum_rms_squares;
  uint8_t voltage_status_code; // 0 - In boundary; 1 - Too high; 2 - Too low
  uint8_t tapping_status_code; // 0 - Not tapping; 1 - Lower; 2 - Raise
  uint16_t timing;
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
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16,
    .voltage_status_code = 0,
    .tapping_status_code = 0,
    .timing = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .rms = 2.5,
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16,
    .voltage_status_code = 0,
    .tapping_status_code = 0,
    .timing = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .rms = 2.5,
    .sum_rms_squares = 1UL * 2.5 * 2.5 * 16,
    .voltage_status_code = 0,
    .tapping_status_code = 0,
    .timing = 0
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
  PIT_Set(0, 125e4, FALSE);
  PIT_Enable(0, TRUE);


  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Set tower number and tower mode
  Flash_AllocateVar((volatile void**) &NumOfRaise, sizeof(*NumOfRaise));
  Flash_AllocateVar((volatile void**) &NumOfLower, sizeof(*NumOfLower));
  // init to 0
  Flash_Write16((uint16*) NumOfRaise, 0);
  Flash_Write16((uint16*) NumOfLower, 0);


  // Send startup packets
  Packet_Put(0x04, 0, 0, 0); // Special - Startup
  Packet_Put(0x09, 'v', 6, 0); // Special - Version number

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}


/*! @brief 16 samples per cycle & control 3 outputs
 *
 */
void PIT0Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT0Sem, 0);

    // Signal each analog channel to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
    }
    
    
    bool hasAlarming = FALSE;
    bool hasRaiseTapping = FALSE;
    bool hasLowerTapping = FALSE;

    // Check each analog channel status
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      if (AnalogThreadData[analogNb].voltage_status_code > 0)
        hasAlarming = TRUE;

      if (AnalogThreadData[analogNb].tapping_status_code == 1)
        hasLowerTapping = TRUE;
      else if (AnalogThreadData[analogNb].tapping_status_code == 2)
        hasRaiseTapping = TRUE;
    }


    // If one or more channel is alarming
    if (hasAlarming)
    {
      LEDs_On(LED_YELLOW);
      Analog_Put(ANALOG_ALARM_CHANNEL, ANALOG_5V);
    }
    else
    {
      LEDs_Off(LED_YELLOW);
      Analog_Put(ANALOG_ALARM_CHANNEL, 0);
    }

    // If one or more channel is raise tapping
    if (hasRaiseTapping)
    {
      LEDs_On(LED_BLUE);
      Analog_Put(ANALOG_RAISE_CHANNEL, ANALOG_5V);
    }
    else
    {
      LEDs_Off(LED_BLUE);
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
}


/*! @brief channel 0 time out
 *
 */
// Global static status bools(alarming) are writen in other threads and read here. So access is put into critical section.
void PIT1Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT1Sem, 0);

    PIT_Enable(1, FALSE);

    AnalogThreadData[0].tapping_status_code = AnalogThreadData[0].voltage_status_code;

    // save to flash
    if (AnalogThreadData[0].tapping_status_code == 1)
    {
      // Lower
      Flash_Write16((uint16*) NumOfLower, NumOfLower + 1);
    }
    else if (AnalogThreadData[0].tapping_status_code == 2)
    {
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise + 1);
    }

    Packet_Put(0xff, 0, NumOfLower->s.Lo, NumOfLower->s.Hi);
    Packet_Put(0xff, 0, NumOfRaise->s.Lo, NumOfRaise->s.Hi);

  }
}

/*! @brief channel 0 time out
 *
 */
// Global static status bools(alarming) are writen in other threads and read here. So access is put into critical section.
void PIT2Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT2Sem, 0);


    PIT_Enable(2, FALSE);

    AnalogThreadData[1].tapping_status_code = AnalogThreadData[1].voltage_status_code;

    // save to flash
    if (AnalogThreadData[0].tapping_status_code == 1)
    {
      // Lower
      Flash_Write16((uint16*) NumOfLower, NumOfLower + 1);
    }
    else if (AnalogThreadData[0].tapping_status_code == 2)
    {
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise + 1);
    }

    Packet_Put(0xff, 1, NumOfLower->s.Lo, NumOfLower->s.Hi);
    Packet_Put(0xff, 1, NumOfRaise->s.Lo, NumOfRaise->s.Hi);

  }
}


/*! @brief channel 0 time out
 *
 */
// Global static status bools(alarming) are writen in other threads and read here. So access is put into critical section.
void PIT3Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT3Sem, 0);

    Packet_Put(0xff, 3, 0, 0);

    PIT_Enable(3, FALSE);

    AnalogThreadData[2].tapping_status_code = AnalogThreadData[2].voltage_status_code;

    // save to flash
    if (AnalogThreadData[0].tapping_status_code == 1)
    {
      // Lower
      Flash_Write16((uint16*) NumOfLower, NumOfLower + 1);
    }
    else if (AnalogThreadData[0].tapping_status_code == 2)
    {
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise + 1);
    }

    Packet_Put(0xff, 2, NumOfLower->s.Lo, NumOfLower->s.Hi);
    Packet_Put(0xff, 2, NumOfRaise->s.Lo, NumOfRaise->s.Hi);

  }
}


/*! @brief Samples a value on an ADC channel & start timer
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

    if (analogData->rms < 2000)
    {
      // PIT1 - period 3s
      if (analogData->timing == 0)
      {
        PIT_Set(analogData->channelNb + 1, TIME_DEFINITE, TRUE);
        analogData->timing = 1;
      }

      analogData->voltage_status_code = 2;

    }
    else if (analogData->rms > 3000)
    {
      // PIT - period 3s
      if (analogData->timing == 0)
      {
        PIT_Set(analogData->channelNb + 1, TIME_DEFINITE, TRUE);
        analogData->timing = 1;
      }

      analogData->voltage_status_code = 1;
    }
    else if (analogData->rms >= 2000 && analogData->rms <= 3000)
    {
      analogData->voltage_status_code = 0;
      analogData->tapping_status_code = 0;

      // PIT - stop timing
      PIT_Enable(analogData->channelNb + 1, FALSE);
      analogData->timing = 0;
    }

    // test put
    //Packet_Put(0xff, analogData->channelNb, analogData->rms >> 8, analogData->rms);

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
  OS_Init(CPU_CORE_CLK_HZ, TRUE);

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
  // Create PIT0 thread
  error = OS_ThreadCreate(PIT2Thread, NULL, &PIT2ThreadStack[THREAD_STACK_SIZE-1], 8);
  // Create PIT1 thread
  error = OS_ThreadCreate(PIT3Thread, NULL, &PIT3ThreadStack[THREAD_STACK_SIZE-1], 9);
  // Create Packet thread
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 10);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
