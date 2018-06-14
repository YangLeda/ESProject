/*! @file
 *
 *  @brief Routines for controlling Low-power Timer (LPTMR) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the Low-power Timer (LPTMR).
 *
 *  @author Leda Yang
 *  @date 2018-06-25
 */

#ifndef LPTMR_H
#define LPTMR_H

// new types
#include "types.h"
#include "PE_Types.h"
#include "MK70F12.h"
#include "OS.h"

// Semaphore of LPTMR
OS_ECB* LPTMRSem;

/*! @brief Sets up the LPTMR before first use.
 *
 *  Enables the LPTMR.
 *  @param count The module clock count number.
 *  @return bool - TRUE if the PIT was successfully initialized.
 */
bool LPTMR_Init(const uint16_t count);


/*! @brief Interrupt service routine for the LPTMR.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) LPTMR_ISR(void);

#endif
