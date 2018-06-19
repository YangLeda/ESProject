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
// Initial sample period in nanosecond is 1 / 50 / 16 * 1000000000 = 125e4
#define INITIAL_SAMPLE_PERIOD 125e4
#define INITIAL_TIMING_MODE 1
#define TIME_DEFINITE 1e9


// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
OS_THREAD_STACK(PIT0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT1ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT2ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PIT3ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(CycleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTReceiveThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTTransmitThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);

// Thread priorities - 0 is highest priority
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};


// Flash WritePhrase is in critical section
// Non-volatile variables for numbers of raise/lower events
uint16union_t* volatile NumOfRaise;
uint16union_t* volatile NumOfLower;
// 1 - definite timing mode; 2 - inverse timing mode
uint8_t* volatile TimingMode;


/*! @brief Analog thread configuration data
 *
 */
TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .voltage_squares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500}, // Assume 2.5V
    .sample_count = 15,
    .rms = 250,
    .tapping_status_code = 0,
    .timing_status = 0,
    .target_timing_count = 0,
    .current_timing_count = 0,
    .frequency = 500,
    .last_sample = 0,
    .frequency_tracking_sample_count = 0,
    .zero_crossing_count = 0,
    .left_fix_time = 0,
    .right_fix_time = 0,
    .last_deviation_count = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .voltage_squares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500}, // Assume 2.5V
    .sample_count = 15,
    .rms = 250,
    .tapping_status_code = 0,
    .timing_status = 0,
    .target_timing_count = 0,
    .current_timing_count = 0,
    .frequency = 500,
    .last_sample = 0,
    .frequency_tracking_sample_count = 0,
    .zero_crossing_count = 0,
    .left_fix_time = 0,
    .right_fix_time = 0,
    .last_deviation_count = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .voltage_squares = {62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500, 62500}, // Assume 2.5V
    .sample_count = 15,
    .rms = 250,
    .tapping_status_code = 0,
    .timing_status = 0,
    .target_timing_count = 0,
    .current_timing_count = 0,
    .frequency = 500,
    .last_sample = 0,
    .frequency_tracking_sample_count = 0,
    .zero_crossing_count = 0,
    .left_fix_time = 0,
    .right_fix_time = 0,
    .last_deviation_count = 0
  }
};

void Tap(uint8_t channelNb)
{

  if (AnalogThreadData[channelNb].rms > 300)
    AnalogThreadData[channelNb].tapping_status_code = 1;
  else if (AnalogThreadData[channelNb].rms < 200)
    AnalogThreadData[channelNb].tapping_status_code = 2;

  // Update event number to flash
  if (AnalogThreadData[channelNb].tapping_status_code == 1)
    Flash_Write16((uint16*) NumOfLower, NumOfLower->l + 1);
  else if (AnalogThreadData[channelNb].tapping_status_code == 2)
    Flash_Write16((uint16*) NumOfRaise, NumOfRaise->l + 1);
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

  // Initialize cycle semaphore
  CycleSem = OS_SemaphoreCreate(0);

  // PIT0
  PIT_Set(0, INITIAL_SAMPLE_PERIOD, FALSE);
  PIT_Enable(0, TRUE);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Initialize number of raise/lower events to 0
  Flash_AllocateVar((volatile void**) &NumOfRaise, sizeof(*NumOfRaise));
  Flash_AllocateVar((volatile void**) &NumOfLower, sizeof(*NumOfLower));
  Flash_AllocateVar((volatile void**) &TimingMode, sizeof(*TimingMode));
  Flash_Write16((uint16*) NumOfRaise, 0);
  Flash_Write16((uint16*) NumOfLower, 0);
  Flash_Write8(TimingMode, INITIAL_TIMING_MODE);

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
    {
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
    }

  }
}


/*! @brief Analog channel 0 alarm time out.
 *
 */
void PIT1Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT1Sem, 0);

    // Disable PIT
    PIT_Enable(1, FALSE);

    if (AnalogThreadData[0].rms > 300)
      AnalogThreadData[0].tapping_status_code = 1;
    else if (AnalogThreadData[0].rms < 200)
      AnalogThreadData[0].tapping_status_code = 2;

    // Update event number to flash
    if (AnalogThreadData[0].tapping_status_code == 1)
      Flash_Write16((uint16*) NumOfLower, NumOfLower->l + 1);
    else if (AnalogThreadData[0].tapping_status_code == 2)
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise->l + 1);
  }
}

/*! @brief Analog channel 1 alarm time out.
 *
 */
void PIT2Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT2Sem, 0);

    // Disable PIT
    PIT_Enable(2, FALSE);

    if (AnalogThreadData[1].rms > 300)
      AnalogThreadData[1].tapping_status_code = 1;
    else if (AnalogThreadData[1].rms < 200)
      AnalogThreadData[1].tapping_status_code = 2;

    // Update event number to flash
    if (AnalogThreadData[1].tapping_status_code == 1)
      Flash_Write16((uint16*) NumOfLower, NumOfLower->l + 1);
    else if (AnalogThreadData[1].tapping_status_code == 2)
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise->l + 1);
  }
}

/*! @brief Analog channel 2 alarm time out.
 *
 */
void PIT3Thread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(PIT3Sem, 0);

    // Disable PIT
    PIT_Enable(3, FALSE);

    if (AnalogThreadData[2].rms > 300)
      AnalogThreadData[2].tapping_status_code = 1;
    else if (AnalogThreadData[2].rms < 200)
      AnalogThreadData[2].tapping_status_code = 2;

    // Update event number to flash
    if (AnalogThreadData[2].tapping_status_code == 1)
      Flash_Write16((uint16*) NumOfLower, NumOfLower->l + 1);
    else if (AnalogThreadData[2].tapping_status_code == 2)
      Flash_Write16((uint16*) NumOfRaise, NumOfRaise->l + 1);
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

    // Check each channel's RMS and update timers
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      if (AnalogThreadData[analogNb].rms < 200 || AnalogThreadData[analogNb].rms > 300) // RMS out of range
      {
        // Count Time
        AnalogThreadData[analogNb].current_timing_count ++;
        
        if (*TimingMode == 1) // Definite mode
        {
          if (AnalogThreadData[analogNb].timing_status != 1) // Not timing or is inverse timing
          {
            PIT_Set(AnalogThreadData[analogNb].channelNb + 1, TIME_DEFINITE, TRUE);
            AnalogThreadData[analogNb].timing_status = 1;
          }
        }
        else // Inverse mode
        {
          uint16_t deviation; // Deviation of RMS
          if (AnalogThreadData[analogNb].rms < 200)
            deviation = 200 - AnalogThreadData[analogNb].rms;
          else
            deviation = AnalogThreadData[analogNb].rms - 200;

          uint8_t deviationTimingCount; // Target number of counts before time out
          ///////////////////////////frequency hardcoded. Use PITo time in nanosecond?
          deviationTimingCount = 16 * 25 * AnalogThreadData[2].frequency / deviation;


          if (AnalogThreadData[analogNb].timing_status != 2) // Not timing or is definite timing, start inverse timing
          {
            AnalogThreadData[analogNb].last_deviation_count = deviationTimingCount;
            AnalogThreadData[analogNb].target_timing_count = deviationTimingCount;
            AnalogThreadData[analogNb].current_timing_count = 0;
            AnalogThreadData[analogNb].timing_status = 2;
            ///
            Packet_Put(0x01, 0, AnalogThreadData[analogNb].target_timing_count, AnalogThreadData[analogNb].target_timing_count >> 8);
          }
          else // Already inverse timing
          {
            if (AnalogThreadData[analogNb].current_timing_count >= AnalogThreadData[analogNb].target_timing_count) // Time out
              Tap(analogNb);
                        
            if (AnalogThreadData[analogNb].last_deviation_count != deviationTimingCount) // RMS deviation count fluctuated
            {
              AnalogThreadData[analogNb].last_deviation_count = deviationTimingCount;
              // New target count according to remaining rate
              uint8_t newTargetTimingCount; 
              //targetTimingCount *= 1 - (float)AnalogThreadData[analogNb].current_timing_count / AnalogThreadData[analogNb].target_timing_count;
              newTargetTimingCount = AnalogThreadData[analogNb].current_timing_count + （1 - (float)AnalogThreadData[analogNb].current_timing_count / AnalogThreadData[analogNb].target_timing_count）* deviationTimingCount;
              AnalogThreadData[analogNb].target_timing_count = newTargetTimingCount;
              //AnalogThreadData[analogNb].current_timing_count = 0;
              ///
              Packet_Put(0x02, 0, AnalogThreadData[analogNb].target_timing_count, AnalogThreadData[analogNb].target_timing_count >> 8);
            }

          }
        }
      }
      else // RMS in range
      {
        AnalogThreadData[analogNb].tapping_status_code = 0;
        // PIT - stop timing
        PIT_Enable(AnalogThreadData[analogNb].channelNb + 1, FALSE);
        AnalogThreadData[analogNb].timing_status = 0;
      }
    }


    bool hasAlarming = FALSE;
    bool hasRaiseTapping = FALSE;
    bool hasLowerTapping = FALSE;

    // Check each analog channel status
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      if (AnalogThreadData[analogNb].rms > 300 || AnalogThreadData[analogNb].rms < 200)
        hasAlarming = TRUE;

      if (AnalogThreadData[analogNb].tapping_status_code == 1)
        hasLowerTapping = TRUE;
      else if (AnalogThreadData[analogNb].tapping_status_code == 2)
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

    // Real voltage = analogInputValue / 3276.7 V
    // 1e-2 V
    realVoltage = analogInputValue * 305 / 1e4;

    // Calculate RMS Voltage - rms updated per cycle
    Algorithm_RMS(analogData->channelNb, realVoltage);

    // Find zero crossing
    if (analogData->channelNb == 0)
    {
      ///
      Analog_Put(3, analogInputValue);

      Algorithm_Frequency(realVoltage);
      
      analogData->last_sample = realVoltage;
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
  // Create Cycle thread
  error = OS_ThreadCreate(CycleThread, NULL, &CycleThreadStack[THREAD_STACK_SIZE-1], 7);
  // Create PIT1 thread
  error = OS_ThreadCreate(PIT1Thread, NULL, &PIT1ThreadStack[THREAD_STACK_SIZE-1], 8);
  // Create PIT2 thread
  error = OS_ThreadCreate(PIT2Thread, NULL, &PIT2ThreadStack[THREAD_STACK_SIZE-1], 9);
  // Create PIT3 thread
  error = OS_ThreadCreate(PIT3Thread, NULL, &PIT3ThreadStack[THREAD_STACK_SIZE-1], 10);
  // Create Packet thread
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 11);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
