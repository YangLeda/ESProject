/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Leda Yang
 *  @date 2018-06-26
 */

#include "PIT.h"

static uint32_t PITModuleClk;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk)
{
  PIT0Sem = OS_SemaphoreCreate(0);

  PITModuleClk = moduleClk;

  // Enable clock register 6 for PIT
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  // Disable PIT
  PIT_MCR |= PIT_MCR_MDIS_MASK;

  // Initialize NVICs for PIT
  NVICICPR2 = (1 << (68 % 32));
  NVICISER2 = (1 << (68 % 32));

  // Enable PIT
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;

  // Timer interrupt enable
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;

  return TRUE;
}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart)
{
  uint32_t trigger = (PITModuleClk / (1e9 / period)) - 1;

  if (restart)
    PIT_Enable(FALSE);

  PIT_LDVAL0 = PIT_LDVAL_TSV(trigger);

  if (restart)
    PIT_Enable(TRUE);
}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  if (enable)
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  else
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();

  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
  OS_SemaphoreSignal(PIT0Sem);

  OS_ISRExit();
}
