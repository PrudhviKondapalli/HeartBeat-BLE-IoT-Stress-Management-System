/*
 * log.h
 *
 *  Created on: Dec 18, 2018
 *      Author: Dan Walkes
 *
 *      Editor: Mar 17, 2021, Dave Sluiter
 *      Change: Commented out logInit() and logFlush() as not needed in SSv5.
 *
 *      August 6, 2021. Got messages that sl_app_log() is deprecated and we
 *      should switch to app_log().
 *
 * Editor: Feb 26, 2022, Dave Sluiter
 * Change: Added comment about use of .h files.
 *
 */

// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.

#ifndef SRC_LOG_H_
#define SRC_LOG_H_
#include "stdio.h"
#include <inttypes.h>

#include "app_log.h"   // for LOG_INFO() / printf() / app_log() output the VCOM port
#include "sl_status.h" // for sl_status_print()


#ifndef LOG_ERROR
#define LOG_ERROR(message,...) \
	LOG_DO(message,"Error", ##__VA_ARGS__)
#endif

#ifndef LOG_WARN
#define LOG_WARN(message,...) \
	LOG_DO(message,"Warn ", ##__VA_ARGS__)
#endif

#ifndef LOG_INFO
#define LOG_INFO(message,...) \
	LOG_DO(message,"Info ", ##__VA_ARGS__)
#endif



// File by file logging control
#if INCLUDE_LOG_DEBUG

#define LOG_DO(message,level, ...) \
  app_log( "%5"PRIu32":%s:%s: " message "\n", loggerGetTimestamp(), level, __func__, ##__VA_ARGS__ )
uint32_t loggerGetTimestamp (void);
void     printSLErrorString (sl_status_t status);

#else

/*
 * Remove all logging related code where logging is not enabled
 */
//#define LOG_DO(message,level, ...)
static inline void LOG_DO() {}

#endif // #else


#endif /* SRC_LOG_H_ */
