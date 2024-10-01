/***************************************************************************//**
 * @file
 * @brief Timer config header file provided to main().
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
#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_
#include "app.h"


#define PRSC                  (4)
#if (LOWEST_ENERGY_MODE == 3)
#define ACTUAL_CLK_FREQ       (1000 / PRSC)  // 250 Hz
#else
#define ACTUAL_CLK_FREQ       (32768 / PRSC)  // 8192 Hz
#endif
#define LETIMER_PERIOD_MS     (3000)          //3s
#define LETIMER_ON_TIME_MS    (175)           //175ms
#define ONESECOND_MICRO       (1000000)

#define VALUE_TO_LOAD         ((LETIMER_PERIOD_MS*ACTUAL_CLK_FREQ)/1000)
    //EM0-EM2                 // (3000            *8192)          /1000
    //EM3                     // (3000            *250)          /1000


// Might not need COMP1 for A3
#define VALUE_TO_LOAD_ON      ((LETIMER_ON_TIME_MS*ACTUAL_CLK_FREQ)/1000)
                              // (175             *8192)          /1000
                              // (175             *250)           / 1000
// Might not need COMP1 for A3



void initLETIMER0();


/*
 * brief: blocks (polls) atleast us_wait microseconds,
 *        using LETIMER0 tick counts as a reference,
 *        and which supports waits as long as those needed
 *        by the Load Power Mngmt and I2C steps
 *
 */
void timerWaitUs(uint32_t us_wait);



/*
 * brief: non blocking waits (interrupt driven)
 *      for wait states needed by Si7021 I2C routines
 */
void timerWaitUs_irq(uint32_t us_wait);


#endif /* SRC_TIMERS_H_ */
