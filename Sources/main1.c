/*!
** @file main.c
** @version 1.0
** @brief  Main module.
**
**   This file contains the high-level code for the project.
**   It initializes appropriate hardware subsystems,
**   creates application threads, and then starts the OS.
**
**   The program implements the same functionality as Lab 4.
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
#include "PE_Types.h"
#include "types.h"
#include "IO_Map.h"
#include "LEDs.h"
#include "packet.h"
#include "Flash.h"
#include "PIT.h"

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100

// Baud rate
const static uint32_t BAUD = 115200;

// Tower number
const static uint32_t TOWER_NUMBER = 2324;

// Non-volatile variables for tower number and tower mode
uint16union_t* volatile NvTowerNb;
uint16union_t* volatile NvTowerMd;

// Thread stacks
static uint32_t InitModulesThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t UARTReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t UARTTransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t INITMODULES_THREAD_PRIORITY = 0;

/*! @brief Routine to initialize modules and turns on orange LED if successful.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  for (;;)
  {
    bool ledsInitResult = LEDs_Init();
    bool packetInitResult = Packet_Init(BAUD, CPU_BUS_CLK_HZ);
    bool flashInitResult = Flash_Init();

    bool pitInitResults = PIT_Init(CPU_BUS_CLK_HZ);
    PIT_Set(500e6 , FALSE);
    PIT_Enable(TRUE);

    // Orange LED on if init success
    if (packetInitResult && ledsInitResult && flashInitResult && pitInitResults)
      LEDs_On(LED_ORANGE);

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

/*! @brief Routine to use PIT to toggle the green LED.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 */
void PITThread(void* data)
{
  for (;;)
  {
    OS_SemaphoreWait(PITSem, 0);
    LEDs_Toggle(LED_GREEN);
  }
}

/*! @brief Initializes the hardware, sets up threads, and starts the OS.
 *
 */
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialize low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, FALSE);

  // Create threads
  error = OS_ThreadCreate(InitModulesThread, NULL, &InitModulesThreadStack[THREAD_STACK_SIZE - 1], 0);
  error = OS_ThreadCreate(UARTReceiveThread, NULL, &UARTReceiveThreadStack[THREAD_STACK_SIZE-1], 1);
  error = OS_ThreadCreate(UARTTransmitThread, NULL, &UARTTransmitThreadStack[THREAD_STACK_SIZE-1], 2);
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 3);
  error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 4);

  // Start multithreading - never returns!
  OS_Start();
}

/*!
** @}
*/
