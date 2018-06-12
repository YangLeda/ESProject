/*! @file
 *
 *  @brief Routines to access the LEDs on the TWR-K70F120M.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author Leda Yang, Jiayi Li
 *  @date 2018-05-30
 */

// new types
#include "PE_Types.h"
#include "LEDs.h"
#include "MK70F12.h"

/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  // Enable clock for port A
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Set portA pins to be general purpose output
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR10 |= PORT_PCR_MUX(1);

  // Set port data direction registers as general-purpose output
  GPIOA_PDDR |= LED_ORANGE;
  GPIOA_PDDR |= LED_YELLOW;
  GPIOA_PDDR |= LED_GREEN;
  GPIOA_PDDR |= LED_BLUE;

  // LEDs off
  LEDs_Off(LED_ORANGE);
  LEDs_Off(LED_YELLOW);
  LEDs_Off(LED_GREEN);
  LEDs_Off(LED_BLUE);

  return TRUE;
}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  GPIOA_PCOR |= color;
}

/*! @brief Turns off an LED.
 *
 *  @param color THe color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  GPIOA_PSOR |= color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR |= color;
}
