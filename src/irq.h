/***************************************************************************//**
 * @file
 * @brief IRQ header file.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * Editor: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
 *
 *
 ******************************************************************************/

// Students: Remember, a header file (a .h file) defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.
#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_


#include <stdint.h>
// brief: IRQ Service Routine for LETIMER0
// parameters: none
// returns: none
void LETIMER0_IRQHandler(void);


// brief: IRQ Service Routine for I2C0
// parameters: none
// returns: none
void I2C0_IRQHandler(void);


// brief: returns the global variable
// letimerMillis for timestamp purposes
// parameters: none
// returns: uint32_t
uint32_t letimerMilliseconds(void);


// brief: PB0 button press IRQ event
// Services the button 0 interrupt
// and schedules the event which will
// call the gecko_external_signal() with
// button press event.
// parameters: none
// returns: none
void GPIO_EVEN_IRQHandler(void);


// brief: PB1 button press IRQ event
// Services the button 1 interrupt
// and schedules the event which will
// call the gecko_external_signal() with
// button press event.
// parameters: none
// returns: none
void GPIO_ODD_IRQHandler(void);





#endif /* SRC_IRQ_H_ */
