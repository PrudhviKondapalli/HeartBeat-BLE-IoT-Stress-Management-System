/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 *
 * Student edit: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
 *
 
 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_


#include <stdbool.h>

// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void power_on_si7021(void);
void power_off_si7021(void);
void gpioSetDisplayExtcomin(bool value);
void gpioPA2SetOn();
void gpioPA3SetOn();
void gpioPF4SetOn();
void gpioPF5SetOn();
void gpiopf3SetOn();
void gpioPA2SetOff();
void gpioPA3SetOff();
void gpioPF4SetOff();
void gpioPF5SetOff();
void gpiopf3SetOff();

void gpiopPB11SetOn();
void gpiopPF0SetOn();
void gpiopPF1SetOn();
void gpiopPF2SetOn();
void gpiopPD14SetOn();
void gpiopPD13SetOn();

void gpiopPB11SetOff();
void gpiopPF0SetOff();
void gpiopPF1SetOff();
void gpiopPF2SetOff();
void gpiopPD14SetOff();
void gpiopPD13SetOff();




void power_on_cap1203(void);




#endif /* SRC_GPIO_H_ */
