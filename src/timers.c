/***************************************************************************//**
 * @file
 * @brief Timer selection and configuration file.
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
#include "em_letimer.h"
#include "timers.h"
#include "app.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define COMP1_IFC     (1)
/*
 * Initialization of LETIMER0 function.
 * bufTop- set to 0 as REP is not needed.
 * comp0Top- loading comp0 into cnt in underflow conditions
 * debugRun- set to 1 for debugging purposes
 * enable- don't enable here, but later in the code
 * out0Pol- not concerned with idle values
 * out1Pol- not concerned with idle values
 * repMode- repeat mode to RepeatFree
 * ufa0- no underflow action required
 * ufa1- no underflow action required
 *
 *  LETIMER_Init is called with the appropriate
 *  parameters
 *  UF interrupt is enabled and finally
 *  LETIMER is enabled.
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void initLETIMER0()
{
    LETIMER_Init_TypeDef letimer_values;
    letimer_values.bufTop = 0;
    letimer_values.comp0Top = 0; //change
    letimer_values.debugRun = 1; //change
    letimer_values.enable = 0; //change
    letimer_values.out0Pol = 0;
    letimer_values.out1Pol = 0;
    letimer_values.repMode = letimerRepeatFree;
    //letimer_values.topValue = VALUE_TO_LOAD;
    letimer_values.ufoa0 = letimerUFOANone;
    letimer_values.ufoa1 = letimerUFOANone;

    LETIMER_Init(LETIMER0, &letimer_values);

    LETIMER_CompareSet(LETIMER0, 0, VALUE_TO_LOAD); // COMP0 Top CompareSet
    //LETIMER_CompareSet(LETIMER0, 1, VALUE_TO_LOAD_ON); // COMP1 CompareSet
    //VALUE_TO_LOAD_ON NEEDS TO BE CHANGED
    //VALUE_TO_LOAD_ON NEEDS TO BE CHANGED 02/12
    //VALUE_TO_LOAD_ON NEEDS TO BE CHANGED
    //VALUE_TO_LOAD_ON NEEDS TO BE CHANGED

    LETIMER0->IEN = LETIMER_IEN_UF;

    LETIMER_Enable(LETIMER0, 1);

    //Debug Statements

   // uint32_t temp = LETIMER_CounterGet (LETIMER0);
   //  temp = LETIMER_CounterGet (LETIMER0);
   // temp = LETIMER_CounterGet (LETIMER0);


}


/*
 * Polling method for a time delay
 * using the LETIMER0 as a reference
 * Conversion of microseconds->milliseconds->ticks
 * Range conditions for invalid inputs
 * and appropriately waits in a while
 * loop until the desired ticks are reached.
 * Two cases are implemented in which in one,
 * the ticks are possible without reset of the
 * time, and the other, wrap_around functionality
 * is implemented.
 *
 * Parameters:
 *   wait time in microseconds
 *
 * Returns:
 *   None
 */
void timerWaitUs(uint32_t us_wait) //uint32_t
{
  // Add code to range check the input parameter to ensure that
  // the requested delay is NOT less than or longer than the routine
  // is capable of providing, do an exception for this.

    uint32_t ms_wait = us_wait / 1000;

    uint16_t tick_conversion = (ms_wait * ACTUAL_CLK_FREQ) / 1000; // 655 ticks for 80ms
    uint16_t wrap_around = 0;
    uint16_t wrap_amt = 0;

    if(tick_conversion > VALUE_TO_LOAD || tick_conversion < 1)
      {
        if(tick_conversion > VALUE_TO_LOAD)
          {
                LOG_ERROR("ticks exceeded max bounds. ADJUSTED to VALUE TO LOAD\r\n");
                tick_conversion = VALUE_TO_LOAD;
          }
        if(tick_conversion < 1)
          {
                LOG_ERROR("ticks exceeded min bounds. ADJUSTED TO 1\r\n");
                tick_conversion = 1;
          }
      }

    uint16_t starting_tick = LETIMER_CounterGet(LETIMER0);
    uint16_t difference_tick = starting_tick - tick_conversion; // both case 1 and 2 valid
    //                           40         - 30

    // Case 1
    if (difference_tick <= VALUE_TO_LOAD) // case 1
      while(LETIMER_CounterGet(LETIMER0) != difference_tick);


   // Case 2
    else
      {
        wrap_amt = 0xFFFF - difference_tick;
        wrap_around = VALUE_TO_LOAD - (wrap_amt + 1);
        while(LETIMER_CounterGet(LETIMER0) != wrap_around);
      }

}


/*
 * Interrupt driven non blocking waits
 * for the wait states needed for Si7021
 * I2C routines. Uses the CompareSet() for
 * COMP1 to set an appropriate tick and then
 * triggers a COMP1 interrupt after the timer ticks
 * have reached.
 *
 * Parameters:
 *   wait time in microseconds
 *
 * Returns:
 *   None
 */
void timerWaitUs_irq(uint32_t us_wait)
{
  // Add code to range check the input parameter to ensure that
  // the requested delay is NOT less than or longer than the routine
  // is capable of providing, do an exception for this.

    uint32_t ms_wait = us_wait / 1000;

    uint16_t tick_conversion = (ms_wait * ACTUAL_CLK_FREQ) / 1000; // 655 ticks for 80ms
    uint16_t wrap_around = 0;
    uint16_t wrap_amt = 0;

    if(tick_conversion > VALUE_TO_LOAD || tick_conversion < 1)
      {
        if(tick_conversion > VALUE_TO_LOAD)
          {
                LOG_ERROR("ticks exceeded max bounds. ADJUSTED to VALUE TO LOAD\r\n");
                tick_conversion = VALUE_TO_LOAD;
          }
        if(tick_conversion < 1)
          {
                LOG_ERROR("ticks exceeded min bounds. ADJUSTED to 1\r\n");
                tick_conversion = 1;
          }
      }

    uint16_t starting_tick = LETIMER_CounterGet(LETIMER0);
    uint16_t difference_tick = starting_tick - tick_conversion; // both case 1 and 2 valid
    // Case 1
    if (difference_tick <= VALUE_TO_LOAD)
      {
        LETIMER_CompareSet(LETIMER0, 1, difference_tick);
        LETIMER0->IFC ^= (1<<COMP1_IFC);
        LETIMER0->IEN |= LETIMER_IEN_COMP1;
      }

   // Case 2
    else
      {
        wrap_amt = 0xFFFF - difference_tick;
        wrap_around = VALUE_TO_LOAD - (wrap_amt);
        LETIMER_CompareSet(LETIMER0, 1, wrap_around);
        LETIMER0->IFC ^= (1<<COMP1_IFC);
        LETIMER0->IEN |= LETIMER_IEN_COMP1;
      }
}
