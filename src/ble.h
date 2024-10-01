/***************************************************************************//**
 * @file
 * @brief BLE header file
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

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sl_bgapi.h"
#include "sl_bt_api.h"


// This is the number of entries in the queue. Please leave
// this value set to 16.
#define QUEUE_DEPTH      (16)

#define UINT8_TO_BITSTREAM(p, n)    { *(p)++ = (uint8_t) (n); }

#define UINT32_TO_BITSTREAM(p, n)   { *(p)++ = (uint8_t) (n); *(p)++ = (uint8_t) ((n) >> 8); \
                                      *(p)++ = (uint8_t) ((n) >> 16); *(p)++ = (uint8_t) ((n) >> 24); }

//#define INT32_TO_FLOAT(m, e)       (((int32_t) (m) & 0x00FFFFFFU) | (uint32_t) ((int32_t) (e) << 24))
#define INT32_TO_FLOAT(m, e) ( (int32_t) (((uint32_t) m) & 0x00FFFFFFU) | (((uint32_t) e) << 24) )

#define MAX_BUFFER_LENGTH  (5)
#define MIN_BUFFER_LENGTH  (1)


void handle_ble_event(sl_bt_msg_t *evt);

#define BUTTON0_PRESS     (0x01)
#define BUTTON0_RELEASE   (0x00)
// BLE Data Structure, save all of our private BT data in here.
//Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {

  // values that are common to servers and clients
  bd_addr           myAddress;
  uint8_t           myAddressType;


  // values unique for server
  // The advertising set handle allocated from Bluetooth stack.
  uint8_t           advertisingSetHandle;
  uint8_t           connectionHandle;
  bool              connection_open; // true when in an open connection
  bool              ok_to_send_htm_indications; // true when client enabled indications
  bool              indication_in_flight;    // true when an indicaion is in-flight
  //Assignment 8 addition
  uint8_t              button_press;
  uint8_t              capacitive_touch_status;
  bool                 bonded;
  bool                 passkey_status;
  bool              ok_to_send_button_indications;
  bool              ok_to_send_hr_indications;

  // values unique for client, more to be added


  uint32_t           htm_servicingHandle; //change nomenclature
  uint16_t           htm_characteristicHandle;

  uint32_t           button_servicingHandle;
  uint16_t           button_characteristicHandle;

  uint32_t           touch_servicingHandle;
  uint16_t           touch_characteristicHandle;
  bool              ok_to_send_captouch_indications;

  uint32_t           hr_servicingHandle;
  uint16_t           hr_characteristicHandle;




} ble_data_struct_t;



ble_data_struct_t*  getBleDataPtr(void);



typedef struct {

  uint16_t       charHandle;                 // GATT DB handle from gatt_db.h
  uint32_t       bufLength;                  // Number of bytes written to field buffer[5]
  uint8_t        buffer[MAX_BUFFER_LENGTH];  // The actual data buffer for the indication,
                                             //   need 5-bytes for HTM and 1-byte for button_state.
                                             //   For testing, test lengths 1 through 5,
                                             //   a length of 0 shall be considered an
                                             //   error, as well as lengths > 5

} queue_struct_t;

// Function prototypes. The autograder (i.e. the testbench) only uses these
// functions to test your design. Please do not change these definitions or
// the autograder will fail.
void     reset_queue      (void);
bool     write_queue      (uint16_t  charHandle, uint32_t  bufLength, uint8_t *buffer);
bool     read_queue       (uint16_t *charHandle, uint32_t *bufLength, uint8_t *buffer);
void     get_queue_status (uint32_t *wptr, uint32_t *rptr, bool *full, bool *empty);
uint32_t get_queue_depth  (void);
void ADC_COUNT_PROCESSING(const uint8_t *adc_buffer);








#endif /* SRC_BLE_H_ */
