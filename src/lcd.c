/***********************************************************************
 * @file      lcd.c
 * @version   1.0
 * @brief     LCD implementation file. A complete re-write of the LCD support code
 *            based on Gecko SDK 3.1 and Simplicity Studio 5.1.
 *            Required components are:
 *               Memory LCD with USART SPI drive
 *               Monochrome Sharp memory LCD
 *               GLIB Graphics Library (glib.c)
 *               GLIB driver for Sharp Memory LCD (dmd_memlcd.c, dmd.h)
 *
 * @author    Dave Sluiter, David.Sluiter@colorado.edu
 * @date      March 15, 2021
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Starter code
 * @due        NA
 *
 * @resources  This code is based on the Silicon Labs example MEMLCD_baremetal
 *             as part of SSv5 and Gecko SDK 3.1.
 *
 * @copyright  All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 *
 * Students:
 * Use these steps to integrate the LCD module with your source code:
 *
 * 3 edits are required to lcd.c
 *
 * 1) Edit #1, Create functions gpioSensorEnSetOn() and,
 *    Edit #2, gpioSetDisplayExtcomin(bool value) in your gpio.c and gpio.h files, and include.
 *
 * 2) Edit #3, add a BT Stack soft timer which can provide a 1Hz update for the display EXTCOMIN pin
 *    through a call to displayUpdate().
 *
 *    Note that the Blue Gecko development board uses the same pin for both the sensor and display enable
 *    pins.  This means you cannot disable the temperature sensor for load power management if enabling the display.
 *    Your GPIO routines need to account for this.
 *
 * 3) Call displayInit() in your sl_bt_evt_system_boot_id event handler, before attempting to
 *    write the display. This needs to be called after event sl_bt_evt_system_boot_id because we
 *    set up a BT Stack soft timer and we aren't supposed to call any BT API calls prior to the
 *    boot event.
 */


/*
 * Student edit: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
*/


#include "stdarg.h" // for arguments

#include "string.h"
#include "sl_bt_api.h"

#include "ble_device_type.h"
#include "gpio.h"

#include "glib.h" // the low-level graphics driver/library
#include "dmd.h"  // the dot matrix display driver


#include "lcd.h"


// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "log.h"





/**
 * A global structure containing information about the data we want to
 * display on a given LCD display
 */
struct display_data {

  uint32_t                 dmdInitConfig; // DMD_InitConfig type is defined as void?

  // tracks the state of the extcomin pin for toggling purposes
	bool                     last_extcomin_state_high;

	// GLIB_Context required for use with GLIB_ functions
	GLIB_Context_t           glibContext;

};


/**
 * We only support a single global display data structure and a
 * single display with this design
 * Declared as static so the variable name is private to this file.
 */
static struct display_data     global_display_data;


// private function to return pointer to the display data
static struct display_data         *displayGetData() {
	return &global_display_data;
}



// ****************************************************************
// The following routines are the public functions
// ****************************************************************

/**
 * This functions takes
 *    a) An LCD row index in the range of 0 to DISPLAY_NUMBER_OF_ROWS-1
 *    b) 1 format string like we'd pass to printf()
 *    c) and a variable length list of arguments that match the number of
 *       % conversions in the format string.
 *    Example:
 *       displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d", temp);
 *
 *    The implementation always erases a row first before drawing the
 *    string passed in. This is done so that all pixels from the previously
 *    displayed text will be erased.
 *    To erase a row, pass in a format string of either "" or " ".
 *
 *    Row indexes >= DISPLAY_NUMBER_OF_ROWS will throw a LOG_ERROR() msg and
 *    return.
 *    Format strings that expand to more than DISPLAY_ROW_LEN characters will
 *    be truncated to DISPLAY_ROW_LEN characters.
 */

void displayPrintf(enum display_row row, const char *format, ...)
{
   va_list     va;        // Declare a variable argument list, see the
                          // implementation of sprintf() for an example
                          // of handling variable number of arguments passed to
                          // a function.

   EMSTATUS               status;
   struct display_data    *display = displayGetData();
   size_t                 strLen;
   char                   strToDisplay[DISPLAY_ROW_LEN+1]; // +1 for null terminator
   char                   strToErase[DISPLAY_ROW_LEN+1];   // +1 for null terminator

   // Range check the row number
   if (row >= DISPLAY_NUMBER_OF_ROWS) {
       LOG_ERROR("row parameter %d is greater than max row index %d", (int) row, (int) DISPLAY_NUMBER_OF_ROWS-1);
       return;
   }
   // Note: enum types are unsigned, so negative row values passed in become large
   //       positive values trapped by the the range check above.
   //if (row < 0) {
   //    LOG_ERROR("row parameter %d is negative", (int) row);
   //    return;
   //}

   // Convert the variable length / formatted input to a string
   // IMPORTANT: Don't use sprintf() as that can write beyond the end of the buffer
   //            allocated for strToDisplay!
   //            And we have to use the "v" versions as these are designed to
   //            accept the variadic (variable length) argument list.
   va_start(va, format);  // initialize the list with args after format
   strLen = vsnprintf(strToDisplay, DISPLAY_ROW_LEN+1, format, va);
   // strLen represents the number of characters in the string after substitution,
   // including the null terminator, not the number of characters copied to strToDisplay
   va_end(va);

   if (strLen == 0) {
       // If a null string was passed in, make it a space + null
       // this is how we can clear a whole line on the LCD display.
       // This is really a trap to keep GLIB_drawStringOnLine() from throwing an error
       // for a zero length string.
       strToDisplay[0] = ' '; // space
       strToDisplay[1] = 0;   // null
       strLen          = 2;
   } else {
     // Not null string, then check if it's too big & warn the user that their
     // string got truncated
     if ((strLen-1) > DISPLAY_ROW_LEN) {
         // For feedback to the user, we don't count the null terminator char, so
         // DISPLAY_ROW_LEN and not DISPLAY_ROW_LEN+1
         LOG_WARN("Your formatted string for row=%d was truncated to (%d) characters", row, DISPLAY_ROW_LEN);
         LOG_WARN("  The truncated string is: %s", strToDisplay);
     } // if
   } // else


   // We always erase the whole line first, then draw the new string. This way
   // we don't leave any pixels set from the previous characters.
   for (int i=0; i<DISPLAY_ROW_LEN; i++) {
       strToErase[i] = ' ';         // space
   }
   strToErase[DISPLAY_ROW_LEN] = 0; // null

   // Erase the row
   status = GLIB_drawStringOnLine(&display->glibContext,
                                   &strToErase[0],
                                   row,
                                   GLIB_ALIGN_CENTER,
                                   0,        // x offset
                                   0,        // y offset
                                   true);    // opaque
   if (status != GLIB_OK) {
       LOG_ERROR("Erase GLIB_drawStringOnLine() returned non-zero error code=0x%04x", (unsigned int) status);
   }


   // Draw the new string on the memory lcd display
   status = GLIB_drawStringOnLine(&display->glibContext,
                                  &strToDisplay[0],
                                  row,
                                  GLIB_ALIGN_CENTER,
                                  0,        // x offset
                                  0,        // y offset
                                  true);    // opaque
   if (status != GLIB_OK) {
       LOG_ERROR("Draw GLIB_drawStringOnLine() returned non-zero error code=0x%04x", (unsigned int) status);
   }


   // Update the data the LCD is displaying
   status = DMD_updateDisplay();
   if (status != DMD_OK) {
       LOG_ERROR("DMD_updateDisplay() returned non-zero error code=0x%04x", (unsigned int) status);
   }

} // displayPrintf()




/**
 * Initialize the LCD display.
 * This also starts a BT stack soft timer, don't call this until after the boot event.
 */
void displayInit()
{

    EMSTATUS    status;
    struct      display_data   *display = displayGetData();


    // Init our private data structure
    memset(display,0,sizeof(struct display_data));
    display->last_extcomin_state_high = false;


    // Edit #1
    // Students: If you created a function for A3, A4 and A5 that turns power on and
    //           off to the Si7021, call the "On" function here. If not create the function
    //           gpioSensorEnSetOn() to set SENSOR_ENABLE=1, see main board schematic,
    //           SENSOR_ENABLE=1 is tied to DISP_ENABLE. We need this on all the
    //           the time now for the LCD to function properly.
    //           Create that function in gpio.c/.h Then add that function call here.
    //
    //gpioSensorEnSetOn(); // we need SENSOR_ENABLE=1 which is tied to DISP_ENABLE
    //                     // for the LCD, on all the time now
    power_on_si7021();



    // Init the dot matrix display data structure
    display->dmdInitConfig = 0;
    //status = DMD_init(&display->dmdInitConfig);
    status = DMD_init(0);
    if (status != DMD_OK) {
        LOG_ERROR("DMD_init() returned non-zero error code=0x%04x", (unsigned int) status);
    }


    // Initialize the glib context
    status = GLIB_contextInit(&display->glibContext);
    if (status != GLIB_OK) {
        LOG_ERROR("GLIB_contextInit() returned non-zero error code=0x%04x", (unsigned int) status);
    }
    // Set the fore and background colors
    display->glibContext.backgroundColor = White;
    display->glibContext.foregroundColor = Black;


    // Fill lcd with background color i.e. clear the LCD display
    status = GLIB_clear(&display->glibContext);
    if (status != GLIB_OK) {
        LOG_ERROR("GLIB_clear() returned non-zero error code=0x%04x", (unsigned int) status);
    }


    // Use Narrow font
    status = GLIB_setFont(&display->glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);
    if (status != GLIB_OK) {
        LOG_ERROR("GLIB_setFont() returned non-zero error code=0x%04x", (unsigned int) status);
    }


    status = DMD_updateDisplay();
    if (status != DMD_OK) {
        LOG_ERROR("DMD_updateDisplay() returned non-zero error code=0x%04x", (unsigned int) status);
    }


	  // The BT stack implements timers that we can setup and then have the stack pass back
	  // events when the timer expires.
	  // This assignment has us using the Sharp LCD which needs to be serviced approx
	  // every 1 second, in order to toggle the input "EXTCOMIN" input to the LCD display.
	  // The documentation is a bit sketchy, but apparently charge can build up within
	  // the LCD and it needs to be bled off. So toggling the EXTCOMIN input is the method by
	  // which this takes place.
	  // We will get a sl_bt_evt_system_soft_timer_id event as a result of calling
	  // sl_bt_system_set_soft_timer() i.e. starting the timer.

    // Edit #3
    // Students: Figure out what parameters to pass in to sl_bt_system_set_soft_timer() to
    //           set up a 1 second repeating soft timer and uncomment the following lines

    //TWO SOFT TIMERS: LCD_SEC_TIMER is a 1 sec timer that updated the LCD display every second

	  sl_status_t          timer_response;
	  timer_response = sl_bt_system_set_soft_timer(LCD_SEC_TIMER,TIMER_HANDLE_ZERO,TIMER_REPEAT);
	  if (timer_response != SL_STATUS_OK) {
	      LOG_ERROR("sl_bt_system_set_soft_timer() INVALID/ERROR  %X\r\n", timer_response);
     }

	  // HALF_SEC_TIMER is a 0.5 sec timer for checking if an indication time of 100-500ms has passed.
	  timer_response = sl_bt_system_set_soft_timer(TWO_HUNDRED_MILLY,TIMER_HANDLE_ONE,TIMER_REPEAT);
	  if (timer_response != SL_STATUS_OK) {
	          LOG_ERROR("sl_bt_system_set_soft_timer() INVALID/ERROR  %X\r\n", timer_response);
	  }


} // displayInit()




/**
 * Call this function from your event handler in response to sl_bt_evt_system_soft_timer_id
 * events to prevent charge buildup within the Liquid Crystal Cells.
 * See details in https://www.silabs.com/documents/public/application-notes/AN0048.pdf
 */
void displayUpdate()
{
	struct display_data *display = displayGetData();

	// toggle the var that remembers the state of EXTCOMIN pin
	display->last_extcomin_state_high = !display->last_extcomin_state_high;

	// Edit #2
  // Students: Create the function gpioSetDisplayExtcomin() that will set
	//           the EXTCOMIN input to the LCD. Add that function to gpio.c./.h
	//           Then uncomment the following line.
	//
	gpioSetDisplayExtcomin(display->last_extcomin_state_high);
	
} // displayUpdate()




