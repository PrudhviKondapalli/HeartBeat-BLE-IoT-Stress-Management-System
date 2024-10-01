/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.
   
   Jan 24, 2023
   Dave Sluiter: Cleaned up gpioInit() to make it less confusing for students regarding
                 drive strength setting. 

 *
 * Student edit: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
 *
 
 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"
#include "ble_device_type.h"


// Student Edit: Define these, 0's are placeholder values.
//
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.
// If these links have gone bad, consult the reference manual and/or the datasheet for the MCU.
// Change to correct port and pins:
#define LED_port   gpioPortF
#define LED0_pin   (4)
#define LED1_pin   (5)
#define LED2_pin   (2)  //PA2
#define LED3_pin   (3)  //PA3
#define LED4_pin   (4)  //PF4
#define LED5_pin   (5)  //PF5
#define LED6_pin   (3)  //PF3
#define PB11_PIN    (11)
#define B_PORT    gpioPortB


#define SIPORT         gpioPortD
#define LCDPORT        gpioPortD

#define EXAMPLE_PORT   gpioPortF

#define RANDOM_PIN      (3)

#define SI_pin_enable   (15)
#define DISP_ENABLE     (15)
#define DISP_EXTCOMIN   (13)

#define SCL_pin         (10)
#define SDA_pin         (11)

#define PUSH_BUTTON0_PORT      gpioPortF
#define PUSH_BUTTON0_PIN       (6)
#define PUSH_BUTTON1_PIN       (7)






/*
 * Set GPIO drive strengths and modes of operation
 *  for the on-board LEDs (LED0 & LED1). Set to
 *  WeakAlternateWeak.
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioInit()
{
    // Student Edit:

    // Set the port's drive strength. In this MCU implementation, all GPIO cells
    // in a "Port" share the same drive strength setting. 


	GPIO_DriveStrengthSet(LED_port, gpioDriveStrengthWeakAlternateWeak);
	//GPIO_DriveStrengthSet(gpioPortA, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(gpioPortB, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(gpioPortD, gpioDriveStrengthStrongAlternateStrong);
	
	//DISP_EXTCOMIN GPIO setup for A6
	GPIO_DriveStrengthSet(LCDPORT, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LCDPORT, DISP_EXTCOMIN, gpioModePushPull, false);


	//////////
	GPIO_PinModeSet(B_PORT, PB11_PIN, gpioModePushPull, false); //PB11
	GPIO_PinModeSet(gpioPortF, 0, gpioModePushPull, true); //PF0
	GPIO_PinModeSet(gpioPortF, 1, gpioModePushPull, true); //PF1
	GPIO_PinModeSet(gpioPortF, 2, gpioModePushPull, true); //PF2
	GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, true); //PD14
	GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, true); //PD13


#if DEVICE_IS_BLE_SERVER
	//Initialize PB0 GPIO for A8
	//Set it to input with glitch input filtering enabled (to help debouncing problems)
	GPIO_PinModeSet(PUSH_BUTTON0_PORT, PUSH_BUTTON0_PIN, gpioModeInputPullFilter, true);
	//Make sure it's gpioModeInputPullFilter
	GPIO_IntClear(GPIO_IntGet());
	// True if interrupt is triggered for falling or rising edge.
	GPIO_ExtIntConfig(PUSH_BUTTON0_PORT, PUSH_BUTTON0_PIN, PUSH_BUTTON0_PIN, true, true, true);
#else
	//Initialize PB0 and PB1 for A9
	//Make sure it's gpioModeInputPullFilter
	GPIO_PinModeSet(PUSH_BUTTON0_PORT, PUSH_BUTTON0_PIN, gpioModeInputPullFilter, true);
	GPIO_PinModeSet(PUSH_BUTTON0_PORT, PUSH_BUTTON1_PIN, gpioModeInputPullFilter, true);

	GPIO_PinModeSet(EXAMPLE_PORT, RANDOM_PIN, gpioModeInput, true);

	GPIO_IntClear(GPIO_IntGet());
	//True if interrupt is triggered for falling or rising edge.
	GPIO_ExtIntConfig(PUSH_BUTTON0_PORT, PUSH_BUTTON0_PIN, PUSH_BUTTON0_PIN, true, true, true);
	GPIO_ExtIntConfig(PUSH_BUTTON0_PORT, PUSH_BUTTON1_PIN, PUSH_BUTTON1_PIN, true, true, true);
	GPIO_ExtIntConfig(EXAMPLE_PORT, RANDOM_PIN, RANDOM_PIN, true, true, true);

#endif


} // gpioInit()


/*
 * Sets LED0 on using the HAL
 * function GPIO_PinOutSet(port, pin)
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED_port, LED0_pin);
}

//GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, false); //PB11
//GPIO_PinModeSet(gpioPortF, 0, gpioModePushPull, false); //PF0
//GPIO_PinModeSet(gpioPortF, 1, gpioModePushPull, false); //PF1
//GPIO_PinModeSet(gpioPortF, 2, gpioModePushPull, false); //PF2
//GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, false); //PD14
//GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, false); //PD13
void gpiopPB11SetOn()
{
  GPIO_PinOutSet(B_PORT, PB11_PIN);
}

void gpiopPF0SetOn()
{
  GPIO_PinOutSet(gpioPortF, 0);
}

void gpiopPF1SetOn()
{
  GPIO_PinOutSet(gpioPortF, 1);
}

void gpiopPF2SetOn()
{
  GPIO_PinOutSet(gpioPortF, 2);
}

void gpiopPD14SetOn()
{
  GPIO_PinOutSet(gpioPortD, 14);
}

void gpiopPD13SetOn()
{
  GPIO_PinOutSet(gpioPortD, 13);
}


/*
 * Sets LED0 off using the HAL
 * function GPIO_PinOutClear(port, pin)
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED_port, LED0_pin);
}



void gpiopPB11SetOff()
{
  GPIO_PinOutClear(B_PORT, PB11_PIN);
}

void gpiopPF0SetOff()
{
  GPIO_PinOutClear(gpioPortF, 0);
}

void gpiopPF1SetOff()
{
  GPIO_PinOutClear(gpioPortF, 1);
}

void gpiopPF2SetOff()
{
  GPIO_PinOutClear(gpioPortF, 2);
}

void gpiopPD14SetOff()
{
  GPIO_PinOutClear(gpioPortD, 14);
}

void gpiopPD13SetOff()
{
  GPIO_PinOutClear(gpioPortD, 13);
}

/*
 * Sets LED1 on using the HAL
 * function GPIO_PinOutSet(port, pin)
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED_port, LED1_pin);
}

/*
 * Sets LED1 off using the HAL
 * function GPIO_PinOutClear(port, pin)
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED_port, LED1_pin);
}



/*
 *  Powers on the SI7021 module.
 *  Selects the appropriate pin selects
 *  + sets them up as gpioModePushPull for the
 *  ENABLE_SELECT pin and gpioModeWiredAnd for the
 *  SCL and the SDA pin. 80ms max delay required after
 *  this to ensure proper power on protocol.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void power_on_si7021(void)
{
  GPIO_PinModeSet(SIPORT, SI_pin_enable, gpioModePushPull, false);
  GPIO_PinOutSet(SIPORT, SI_pin_enable);
  //GPIO_PinModeSet(SIPORT, SCL_pin, gpioModeWiredAnd, true);
  //GPIO_PinModeSet(SIPORT, SDA_pin, gpioModeWiredAnd, true);
}




/*
 *  Teardown function of the SI 7021 which shuts off
 *  the I2C0 connection by clearing the ENABLE_SELECT pin
 *  SCL and SDA are both disabled by using the
 *  gpioModeDisabled macro in the HAL function
 *  GPIO_PinModeSet(port, pin, mode_sel, bool)
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void power_off_si7021(void)
{
  GPIO_PinModeSet(SIPORT, SCL_pin, gpioModeDisabled, true);
  GPIO_PinModeSet(SIPORT, SDA_pin, gpioModeDisabled, true);
  GPIO_PinOutClear(SIPORT, SI_pin_enable);
}


/*
 *  Sets/Clears the EXTCOMIN value depending
 *  on the value passed in bool. will be called
 *  from lcd.c function displayUpdate()
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void gpioSetDisplayExtcomin(bool value)
{
    if(value)
        GPIO_PinOutSet(LCDPORT, DISP_EXTCOMIN);
    else if (!value)
        GPIO_PinOutClear(LCDPORT, DISP_EXTCOMIN);
}


void power_on_cap1203(void)
{

}








