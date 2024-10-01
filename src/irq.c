/***************************************************************************//**
 * @file
 * @brief Interrupt handling implementation file.
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

#include "irq.h"
#include "i2c.h"
#include "ble.h"
#include "timers.h"
#include "em_letimer.h"
#include "gpio.h"
#include "em_core.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "scheduler.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "stdbool.h"
#include "lcd.h"

#define BUTTON0_PORT      gpioPortF
#define EXAMPLE_PORT      gpioPortF
#define EXAMPLE_PIN       (3)
#define BUTTON0_PIN       (6)
#define BUTTON1_PIN       (7)
#define GPIO_INT_CLEAR        (1 << BUTTON0_PIN)
#define GPIO_INT_CLEAR_B1      (1 << BUTTON1_PIN)
#define EXAMPLE_PIN_CLEAR      (1 << EXAMPLE_PIN)

uint32_t letimerMillis = 0;



/*
 *  IRQ Handler for LETIMER0.
 *  Depending on the flag, appropriate
 *  action is taken to set scheduler events
 *  .Upto A3.
 *
 *  A4 update: COMP1 interrupt added to
 *  handle delays in between I2C transfers
 *  COMP1 interrupt is disabled upon
 *  entering the Handler. Scheduler
 *  event is added when the timer
 *  is completed in the COMP1 interrupt.
 *  Upon three seconds completion to set
 *  a temperature read event, a global
 *  variable letimerMillis is declared
 *  for timestamp purposes.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void LETIMER0_IRQHandler(void)
{

    CORE_DECLARE_IRQ_STATE;
    uint32_t  flags;

    //First: determine source of IRQ
    flags = LETIMER_IntGetEnabled(LETIMER0);

    //Second: clear source of IRQ set in step 3
    LETIMER_IntClear(LETIMER0, flags);

    //Third: perform whatever processing is required

    CORE_ENTER_CRITICAL();

        if(flags & LETIMER_IF_COMP1)
        {
            LETIMER0->IEN &= ~(LETIMER_IEN_COMP1);
            schedulerEvtTimerDone();
        }

        if(flags & LETIMER_IF_UF)
        {
           letimerMillis += LETIMER_PERIOD_MS;
           schedulerSetReadTempEvent();
#if DEVICE_IS_BLE_SERVER
           schedulerSetHREvent();
#endif
        }
     CORE_EXIT_CRITICAL();


} //LETIMER0_IRQHandler()


/*
 *  IRQ Handler for I2C0 peripheral.
 *  Schedules an I2C Transfer Complete
 *  event. The program enters this handler
 *  upon completion of the I2CTransferInit
 *  and checks the status of the transfer
 *  to see if it's done or not. Once the
 *  transfer is completed, it appends an
 *  event in the scheduler, in order to
 *  move forward with the state machine
 *  implemented in app.c
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void I2C0_IRQHandler(void)
{
  I2C_TransferReturn_TypeDef  transferStatus;

  transferStatus = I2C_Transfer(I2C0);

  if (transferStatus == i2cTransferDone)
    {
      schedulerEvtI2CTransferDone();
    }

  if (transferStatus < 0)
    {
      LOG_ERROR("herere %d\r\n", transferStatus);
    }

} //I2C0_IRQHandler()


/*
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   uint32_t: returns the program time
 *   counter in 3000 milliseconds
 *   increments.
 */
uint32_t letimerMilliseconds(void)
{
    return (letimerMillis);
}


//External Button 0 IRQ Handler
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_IntClear(GPIO_INT_CLEAR);
  GPIO_IntClear(EXAMPLE_PIN_CLEAR);

  if (GPIO_PinInGet(gpioPortF, BUTTON0_PIN) == 0)
    {
      schedulerEvtButtonPress();
    }

  else
    {
      schedulerEvtButtonRelease();
    }

}

#if !DEVICE_IS_BLE_SERVER
void GPIO_ODD_IRQHandler(void)
{

  uint32_t flags = GPIO_IntGetEnabled();
  GPIO_IntClear(flags);



  //if (GPIO_PinInGet(gpioPortF, BUTTON1_PIN) == 0)
  if (flags & (1 << BUTTON1_PIN))
    {
      schedulerEvtButton1Press();
    }


  if (flags & (1 << EXAMPLE_PIN))
    {
      schedulerEvtCapTouched();
    }



}

#endif



