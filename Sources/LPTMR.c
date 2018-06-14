/*! @file
 *
 *  @brief Routines for controlling Low-power Timer (LPTMR) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the Low-power Timer (LPTMR).
 *
 *  @author Leda Yang
 *  @date 2018-06-25
 */

#include "LPTMR.h"


/*! @brief Sets up the LPTMR before first use.
 *
 *  Enables the LPTMR.
 *  @param count The module clock count number.
 *  @return bool - TRUE if the PIT was successfully initialized.
 */
bool LPTMR_Init(const uint16_t count)
{
  LPTMRSem = OS_SemaphoreCreate(0);

  // Enable clock gate to LPTMR module
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

  // Disable the LPTMR while we set up
  // This also clears the CSR[TCF] bit which indicates a pending interrupt
  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;

  // Enable LPTMR interrupts
  LPTMR0_CSR |= LPTMR_CSR_TIE_MASK;
  // Reset the LPTMR free running counter whenever the 'counter' equals 'compare'
  LPTMR0_CSR &= ~LPTMR_CSR_TFC_MASK;
  // Set the LPTMR as a timer rather than a counter
  LPTMR0_CSR &= ~LPTMR_CSR_TMS_MASK;

  // Bypass the prescaler
  LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK;
  // Select the prescaler clock source
  LPTMR0_PSR = (LPTMR0_PSR & ~LPTMR_PSR_PCS(0x3)) | LPTMR_PSR_PCS(1);

  // Set compare value
  LPTMR0_CMR = LPTMR_CMR_COMPARE(count);

  // Initialize NVIC
  // see p. 91 of K70P256M150SF3RM.pdf
  // Vector 0x65=101, IRQ=85
  // NVIC non-IPR=2 IPR=21
  // Clear any pending interrupts on LPTMR
  NVICICPR2 = NVIC_ICPR_CLRPEND(1 << 21);
  // Enable interrupts from LPTMR module
  NVICISER2 = NVIC_ISER_SETENA(1 << 21);

  //Turn on LPTMR and start counting
  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
}


/*! @brief Interrupt service routine for the LPTMR.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) LPTMR_ISR(void)
{
  OS_ISREnter();

  // Clear interrupt flag
  LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;

  OS_SemaphoreSignal(LPTMRSem);

  OS_ISRExit();
}
