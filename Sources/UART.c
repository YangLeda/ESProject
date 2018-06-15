/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Leda Yang, Jiayi Li
 *  @date 2018-05-30
 */

#include "UART.h"
#include "MK70F12.h"

// FIFOs
static TFIFO RxFIFO;
static TFIFO TxFIFO;

// Semaphores
static OS_ECB *RxSem;
static OS_ECB *TxSem;

// Data read from UART2_D
static uint8_t rxData;

/*! @brief Sets up the UART interface before first use.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the UART was successfully initialized.
 */
bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  TxSem = OS_SemaphoreCreate(0);
  RxSem = OS_SemaphoreCreate(0);

  uint16_t sbr, brfa;
  uint8_t temp;

  // Enable clock for UART2
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;

  // Enable clock for PORTE
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  UART2_C2 &= ~UART_C2_TE_MASK;
  UART2_C2 &= ~UART_C2_RE_MASK;

  // Enable UART2_TXD function on PTE16, UART is ALT3 function for this pin
  PORTE_PCR16 = PORT_PCR_MUX(0x3);

  // Enable UART2_RXD function on PTE17, UART is ALT3 function for this pin
  PORTE_PCR17 = PORT_PCR_MUX(0x3);

  // Configure the UART for 8-bit mode
  UART2_C1 &= ~ UART_C1_M_MASK;
  // Configure the UART for no parity
  UART2_C1 &= ~ UART_C1_PE_MASK;

  // Calculate baud settings
  sbr = (uint16_t)(moduleClk/(baudRate * 16));

  // Save off the current value of the UARTx_BDH except for the SBR field
  temp = UART2_BDH & ~(UART_BDH_SBR(0x1F));
  UART2_BDH = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
  UART2_BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

  // Determine if a fractional divider is needed to get closer to the baud rate
  brfa = ((((uint32_t)moduleClk * 32) / (baudRate * 16)) - (sbr * 32));

  // Save off the current value of the UARTx_C4 register except for the BRFA field
  temp = UART2_C4 & ~(UART_C4_BRFA(0x1F));
  UART2_C4 = temp |  UART_C4_BRFA(brfa);

  // Enable receive and transmit interrupts
  UART2_C2 |= UART_C2_TIE_MASK;
  UART2_C2 |= UART_C2_RIE_MASK;

  //Initialize NVIC
  NVICICPR1 = (1 << (49 % 32));
  NVICISER1 = (1 << (49 % 32));

  // Enable receiver and transmitter
  UART2_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

  // FIFOs
  FIFO_Init(&RxFIFO);
  FIFO_Init(&TxFIFO);

  return TRUE;
}

/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
void UART_InChar(uint8_t * const dataPtr)
{
  FIFO_Get (&RxFIFO, dataPtr);
}

/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
void UART_OutChar(const uint8_t data)
{
  FIFO_Put (&TxFIFO, data);
}

/*! @brief Thread to receive data
 *
 *  @param data
 *  @note Assumes that UART_Init has been called.
 */
void UARTReceiveThread(void *data)
{
  for(;;)
  {
    OS_SemaphoreWait(RxSem, 0);
    FIFO_Put(&RxFIFO, rxData);
    UART2_C2 |= UART_C2_RIE_MASK; // Enable Receiver Full Interrupt
  }
}

/*! @brief Thread to transmit data
 *
 *  @param data
 *  @note Assumes that UART_Init has been called.
 */
void UARTTransmitThread(void *data)
{
  uint8_t txData;
  for(;;)
  {
    OS_SemaphoreWait(TxSem, 0);
    FIFO_Get(&TxFIFO, &txData);
    UART2_D = txData;
    UART2_C2 |= UART_C2_TIE_MASK; // Enable Transmitter Interrupt
  }
}

/*! @brief Interrupt service routine for the UART.
 *
 *  @note Assumes the transmit and receive FIFOs have been initialized.
 */
void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  // Receive
  if (UART2_C2 & UART_C2_RIE_MASK)
    // Clear RDRF flag by reading the status register
    if (UART2_S1 & UART_S1_RDRF_MASK)
    {
      UART2_C2 &= ~UART_C2_RIE_MASK; // Disable Receiver Full Interrupt
      rxData = UART2_D;
      OS_SemaphoreSignal(RxSem);
    }

  // Transmit
  if (UART2_C2 & UART_C2_TIE_MASK)
    // Clear TDRE flag by reading the status register
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      UART2_C2 &= ~UART_C2_TIE_MASK; // Disable Transmitter Interrupt
      OS_SemaphoreSignal(TxSem);
    }

  OS_ISRExit();
}
