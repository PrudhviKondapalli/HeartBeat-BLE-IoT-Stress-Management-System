/*
 * log.c
 *
 *  Created on: Dec 18, 2018
 *      Author: Dan Walkes
 *
 *      Editor:  Jan 5, 2021, Dave Sluiter
 *      Changed: Updates to loggerGetTimestamp(), systicks usage, note to
 *               students.
 *
 *      Editor: Mar 17, 2021, Dave Sluiter
 *      Change: Commented out logInit() and logFlush() as not needed in SSv5.
 *
 */



/*
 * Student edit: Add your name and email address here:
 * @student    Awesome Student, Awesome.Student@Colorado.edu
*/


#include <stdbool.h>

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "log.h"
#include "irq.h"


/**
 * @return a timestamp value for the logging functions, typically based on a
 * free running timer.
 * This value will be printed at the beginning of each log message.
 */
uint32_t loggerGetTimestamp()
{

     // Students: You will eventually develop this function called letimerMilliseconds()
     //           and not return 0. This will be the function to call for your graded
     //           assignments that require logging (a fancy printf).
     //           Put the letimerMilliseconds() function in your irq.c/.h files.

     return letimerMilliseconds();
	  // return (0);
	   
} // loggerGetTimestamp



/**
 * Print a string for the Silicon Labs API error codes defined in sl_status.h
 * Depends on Components:
 *     Utilities / Status Code / Status Code Strings (sl_status.c)
 *     Utilities / Status Code / Status Code Definitions
 */
void printSLErrorString(sl_status_t status) {

  char              buffer[128+1]; // 128 chars should be long enough,
                                   // if not the string will truncated
  int32_t           result;

  // Attempt to convert the error code value into a string
  result = sl_status_get_string_n(status, (char *) &buffer[0], 128); // leave room for null terminator

  // return value:
  //   The number of characters that would have been written if the buffer_length
  //   had been sufficiently large, not counting the terminating null character.
  //   If the status code is invalid, 0 or a negative number is returned. Notice
  //   that only when this returned value is strictly positive and less than
  //   buffer_length, the status string has been completely written in the buffer.
  if ((result > 0) && (result < 128)) {
      LOG_ERROR("Error code 0x%04x is %s", (unsigned int) status, &buffer[0] );
  } else {
      LOG_ERROR("Unable to convert error code 0x%04x into a string", (unsigned int) status);
  }

} // printSLErrorString()





