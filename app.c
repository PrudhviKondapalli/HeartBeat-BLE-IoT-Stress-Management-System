/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
 * Date:        02-25-2022
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 *
 *
 * Student edit: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
 *
 *
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "src/oscillators.h"
#include "src/timers.h"
#include "src/scheduler.h"
#include "src/ble.h"

// *************************************************
// Students: It is OK to modify this file.
//           Make edits appropriate for each
//           assignment.
// *************************************************

#include "sl_status.h"             // for sl_status_print()

#include "src/ble_device_type.h"
#include "src/gpio.h"
#include "src/lcd.h"
#include "src/oscillators.h"
#include "src/i2c.h"
#include "src/timers.h"


// Students: Here is an example of how to correctly include logging functions in
//           each .c file.
//           Apply this technique to your other .c files.
//           Do not #include "src/log.h" in any .h file! This logging scheme is
//           designed to be included at the top of each .c file that you want
//           to call one of the LOG_***() functions from.

// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"






// *************************************************
// Power Manager
// *************************************************

// See: https://docs.silabs.com/gecko-platform/latest/service/power_manager/overview
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

// -----------------------------------------------------------------------------
// defines for power manager callbacks
// -----------------------------------------------------------------------------
// Return values for app_is_ok_to_sleep():
//   Return false to keep sl_power_manager_sleep() from sleeping the MCU.
//   Return true to allow system to sleep when you expect/want an IRQ to wake
//   up the MCU from the call to sl_power_manager_sleep() in the main while (1)
//   loop.
//
// Students: We'll need to modify this for A2 onward so that compile time we
//           control what the lowest EM (energy mode) the MCU sleeps to. So
//           think "#if (expression)".
#if (LOWEST_ENERGY_MODE == 0)
#define APP_IS_OK_TO_SLEEP      (false)
#else
#define APP_IS_OK_TO_SLEEP      (true)
#endif



// Return values for app_sleep_on_isr_exit():
//   SL_POWER_MANAGER_IGNORE; // The module did not trigger an ISR and it doesn't want to contribute to the decision
//   SL_POWER_MANAGER_SLEEP;  // The module was the one that caused the system wakeup and the system SHOULD go back to sleep
//   SL_POWER_MANAGER_WAKEUP; // The module was the one that caused the system wakeup and the system MUST NOT go back to sleep
//
// Notes:
//       SL_POWER_MANAGER_IGNORE, we see calls to app_process_action() on each IRQ. This is the
//       expected "normal" behavior.
//
//       SL_POWER_MANAGER_SLEEP, the function app_process_action()
//       in the main while(1) loop will not be called! It would seem that sl_power_manager_sleep()
//       does not return in this case.
//
//       SL_POWER_MANAGER_WAKEUP, doesn't seem to allow ISRs to run. Main while loop is
//       running continuously, flooding the VCOM port with printf text with LETIMER0 IRQs
//       disabled somehow, LED0 is not flashing.

#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_IGNORE)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_SLEEP)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_WAKEUP)

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




// *************************************************
// Power Manager Callbacks
// The values returned by these 2 functions AND
// adding and removing power manage requirements is
// how we control when EM mode the MCU goes to when
// sl_power_manager_sleep() is called in the main
// while (1) loop.
// *************************************************

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

bool app_is_ok_to_sleep(void)
{
  return APP_IS_OK_TO_SLEEP;
} // app_is_ok_to_sleep()

sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  return APP_SLEEP_ON_ISR_EXIT;
} // app_sleep_on_isr_exit()

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Put your application 1-time initialization code here.
  // This is called once during start-up.
  // Don't call any Bluetooth API functions until after the boot event.

  // Student Edit: Add a call to gpioInit() here
    gpioInit(); //Initialize GPIO pins here
    cmuConfig(); //Configure oscillators + enable LETIMER0 with appropriate LETIMER setup
    initLETIMER0(); // Setup LETIMER0
#if !DEVICE_IS_BLE_SERVER
    initCAP1203();
#else
    init_max30105();
#endif

    gpiopPB11SetOn();

    //power requirement section
#if(LOWEST_ENERGY_MODE == 2 || LOWEST_ENERGY_MODE == 1)
      {
        sl_power_manager_add_em_requirement(LOWEST_ENERGY_MODE);
      }
#endif

    //NVIC settings upto A3
    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

#if !DEVICE_IS_BLE_SERVER
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
#endif

    //unit test
    //timerWaitUs_irq(10000);

} // app_init()




/*****************************************************************************
 * delayApprox(), private to this file.
 * A value of 3500000 is ~ 1 second. After assignment 1 you can delete or
 * comment out this function. Wait loops are a bad idea in general.
 * We'll discuss how to do this a better way in the next assignment.
 *****************************************************************************/
//static void delayApprox(int delay)
//{
//  volatile int i;
//
//  for (i = 0; i < delay; ) {
//      i=i+1;
//  }
//
//} // delayApprox()


// Test Function for Timer Wait IRQ Style

//void unitTest_TimerWait(uint32_t evt)
//{
//    uint32_t currentState;
//    static uint32_t nextState = 0;
//    currentState = nextState;
//
//    switch (currentState)
//    {
//      case 0: // turn1
//        {
//          if(evt == evtTimerDone)
//            {
//              gpioLed0SetOn();
//              timerWaitUs_irq(10000);
//              nextState = 1;
//            }
//          break;
//        }
//      case 1: //turn0
//        {
//          if(evt == evtTimerDone)
//            {
//              gpioLed0SetOff();
//              timerWaitUs_irq(10000);
//              nextState = 0;
//            }
//          break;
//        }
//        //
//    }
//}

/*
 * This function implements the I2C
 * state machine used in A4 in order
 * to perform a temperature measurement
 * of the Si7021 temperature sensor. The state
 * machine consists of five states, stateIdle,
 * waitForTimerToWrite, waitForI2CWriteComplete,
 * waitForTimerToRead, and waitForI2CReadComplete
 * The state machine is implemented in such
 * a way that the CPU will sleep to
 * EM1 while there are I2C transfers happening,
 * and will sleep to EM3 while there is a
 * time delay between the I2C transfers. The
 * interrupts utilized are COMP1, UF, and
 * I2CTransferComplete
 *
 * Parameters:
 * evt - event that's being passed from the
 * getNextEvent() function
 *
 * return:
 * none
 *
 */
void I2C_SM(uint32_t evt)
{
            State_t     currentState;
     static State_t     nextState = stateIdle;

     currentState = nextState;

     switch (currentState)
     {
       //Timer UF interrupt requests temp measurement
       //Turn power on and Setup Timer
       //Advance State
       case stateIdle:
         nextState = stateIdle;
         if (evt == evtReadTemperature)
           {
             power_on_si7021();
             timerWaitUs_irq(80000);
             nextState = waitForTimerToWrite;
           }
         break;
       //Timer Event Complete
       //Start I2C Write
       //Advance State
       case waitForTimerToWrite:
         nextState = waitForTimerToWrite;
         if (evt == evtTimerDone) //80ms
           {
             sl_power_manager_add_em_requirement(1); //EM1
             send_command_si7021();
             nextState = waitForI2CWriteComplete;
           }
         break;
       //I2C transfer complete event
       //Setup Timer (10.8 milly)
       //Advance State
       case waitForI2CWriteComplete:
         nextState = waitForI2CWriteComplete;
         if (evt == evtI2CTransferComplete)
           {
             NVIC_DisableIRQ(I2C0_IRQn);
             sl_power_manager_remove_em_requirement(1);
             timerWaitUs_irq(10800); //10.8ms
             nextState = waitForTimerToRead;
           }
         break;
       //Timer event complete
       //StartI2CRead
       //Advance State
       case waitForTimerToRead:
         nextState = waitForTimerToRead;
         if (evt == evtTimerDone)
           {
             sl_power_manager_add_em_requirement(1);
             read_temp_from_si7021();
             nextState = waitForI2CReadComplete;
           }
         break;
       //I2C transfer complete event
       // Turn Power Off
       //Advance State
       case waitForI2CReadComplete:
         nextState = waitForI2CReadComplete;
         if (evt == evtI2CTransferComplete)
           {
             NVIC_DisableIRQ(I2C0_IRQn);
             sl_power_manager_remove_em_requirement(1);
             power_off_si7021();
             read_data_convert();
             nextState = stateIdle;
           }
         break;

       default:
         break;
     } //switch


} //state_machine()

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  // Put your application code here for A1 to A4.
  // This is called repeatedly from the main while(1) loop
  // Notice: This function is not passed or has access to Bluetooth stack events.
  //         We will create/use a scheme that is far more energy efficient in
//  //         later assignments.
} // app_process_action()





/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *
 * The code here will process events from the Bluetooth stack. This is the only
 * opportunity we will get to act on an event.
 *
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{

    handle_ble_event(evt); // put this code in ble.c/.h


#if DEVICE_IS_BLE_SERVER
  // sequence through states driven by events
  discovery_state_machine_for_server(evt);
  temperature_state_machine(evt);    // put this code in scheduler.c/.h
  hr_state_machine(evt);


#else
  //Client state machine

  discovery_state_machine(evt); //client implementation
#endif

  //read max30105 heart rate sensor





} // sl_bt_on_event()

