/***************************************************************************//**
 * @file
 * @brief BLE implementation file
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
 * @credit     David Sluiter, for debugging purposes
 *
 ******************************************************************************/
#include "ble.h"
#include "math.h"
#include "gpio.h"
#include "sl_bt_api.h"
#include "sl_bgapi.h"
#include "stdint.h"
#include "gatt_db.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "ble_device_type.h"
#include "lcd.h"
#include "em_gpio.h"
#include "scheduler.h"
#include "i2c.h"
#include "timers.h"


#define       BUTTON0GPIO      (6)
#define       DISPLAYONLY          (0)
#define       DISPLAYYESNO         (1)
#define       QUEUE_DEPTH_LAST    (15)


#define IO_CAPABILITY     (DISPLAYYESNO)
#define CONF_FLAG         (0x0F) //Numeric comparison purposes
ble_data_struct_t ble_data;

#if !DEVICE_IS_BLE_SERVER
static int32_t FLOAT_TO_INT32(const uint8_t *buffer_ptr);
#endif

uint8_t cs2_counter = 0;
uint8_t server_cs2_counter = 0;
bool long_touch_flag = 0;
uint8_t indication_or_no = 0;
uint8_t hr_pad = 0;
uint8_t em_pad = 0;
uint8_t spo2_pad = 0;
uint32_t red_reading_client = 0;
uint8_t red_read_arr_client = 0;
uint32_t IR_reading_client = 0;
uint32_t  green_reading_client = 0;
#if DEVICE_IS_BLE_SERVER
uint8_t enter_once_cap = 1;

#else
uint8_t enter_once_heart = 1;
#endif
/*
 * Returns the ble_data struct created
 * in this file (ble.c) so that methods
 * in other files like scheduler.c can
 * access and modify this structure,
 * without the use of extern variable.
 *
 * Parameters:
 * none-
 *
 * return:
 * ble_data_struct_t
 *
 */
ble_data_struct_t*  getBleDataPtr(void)
{
  return (&ble_data);
} // getBleDataPtr



#if DEVICE_IS_BLE_SERVER
// Declare memory for the queue/buffer, and our write and read pointers.
queue_struct_t   my_queue[QUEUE_DEPTH]; // the queue
uint32_t         wptr = 0;              // write pointer
uint32_t         rptr = 0;              // read pointer
bool       empty = 0;       //empty flag
bool       full = 0;        //full flag
uint32_t     depth_rtime = 0;     //length of the cb



// ---------------------------------------------------------------------
// Private function used only by this .c file.
// Compute the next ptr value. Given a valid ptr value, compute the next valid
// value of the ptr and return it.
// Isolation of functionality: This defines "how" a pointer advances.
// ---------------------------------------------------------------------
static uint32_t nextPtr(uint32_t ptr) {

  if(ptr == QUEUE_DEPTH_LAST) return (uint32_t) 0;
  return (ptr+1);

} // nextPtr()




// ---------------------------------------------------------------------
// Public function.
// This function resets the queue.
// ---------------------------------------------------------------------
void reset_queue (void) {
  wptr = 0; //reset the write + read pointer and set depth = 0
  rptr = 0;
  depth_rtime = 0;
  empty = 1;
  full = 0;
} // reset_queue()


// ---------------------------------------------------------------------
// Public function.
// This function writes an entry to the queue if the the queue is not full.
// Input parameter "charHandle" should be written to queue_struct_t element "charHandle".
// Input parameter "bufLength" should be written to queue_struct_t element "bufLength"
// The bytes pointed at by input parameter "buffer" should be written to queue_struct_t element "buffer"
// Returns bool false if successful or true if writing to a full fifo.
// i.e. false means no error, true means an error occurred.
// ---------------------------------------------------------------------
bool write_queue (uint16_t charHandle, uint32_t bufLength, uint8_t *buffer) {

  // Student edit:
  // Create this function
  // Decide how you want to handle the "full" condition.

  // Don't forget to range check bufLength.
  // Isolation of functionality:
  //     Create the logic for "when" a pointer advances.


  if (bufLength < 1 || bufLength > 5)
    {
      return 1;
    }


  if (depth_rtime < QUEUE_DEPTH) // The only condition where I can add to the queue
  {
    my_queue[wptr].charHandle = charHandle;
    my_queue[wptr].bufLength = bufLength;
    memcpy(&my_queue[wptr].buffer[0], &buffer[0], bufLength);
    depth_rtime++; //Keep track of the depth of the queue here instead of new logic being added.

    if (depth_rtime == QUEUE_DEPTH) // limit reached, suppress write pointer here
      full = 1;
    else if (depth_rtime-1 == 0) //Empty queue to one element in the queue.
    {
      wptr = nextPtr(wptr);
      empty = 0;
    }
    else            // There is already some elements in the queue and space is left as well.
      wptr = nextPtr(wptr);

  }

  else
    {
      LOG_ERROR("EXCEEDED BUFFER CAPACITY\r\n");
      return 1;
    }

  return 0;


} // write_queue()


// ---------------------------------------------------------------------
// Public function.
// This function reads an entry from the queue, and returns values to the
// caller. The values from the queue entry are returned by writing
// the values to variables declared by the caller, where the caller is passing
// in pointers to charHandle, bufLength and buffer. The caller's code will look like this:
//
//   uint16_t     charHandle;
//   uint32_t     bufLength;
//   uint8_t      buffer[5];
//
//   status = read_queue (&charHandle, &bufLength, &buffer[0]);
//
// *** If the code above doesn't make sense to you, you probably lack the
// necessary prerequisite knowledge to be successful in this course.
//
// Write the values of charHandle, bufLength, and buffer from my_queue[rptr] to
// the memory addresses pointed at by charHandle, bufLength and buffer, like this :
//      *charHandle = <something>;
//      *bufLength  = <something_else>;
//      *buffer     = <something_else_again>; // perhaps memcpy() would be useful?
//
// In this implementation, we do it this way because
// standard C does not provide a mechanism for a C function to return multiple
// values, as is common in perl or python.
// Returns bool false if successful or true if reading from an empty fifo.
// i.e. false means no error, true means an error occurred.
// ---------------------------------------------------------------------
bool read_queue (uint16_t *charHandle, uint32_t *bufLength, uint8_t *buffer) {

  if(depth_rtime > 0) //not empty check, there is something in the queue to be read
  {

    if (depth_rtime == QUEUE_DEPTH) // wptr is frozen at the moment
    {
      *charHandle = my_queue[rptr].charHandle;
      *bufLength = my_queue[rptr].bufLength;
      memcpy(&buffer[0], &my_queue[rptr].buffer[0], my_queue[rptr].bufLength);
      depth_rtime--; // update depth of the queue here
      rptr = nextPtr(rptr);
      wptr = nextPtr(wptr); // un-suppress the write pointer here
      full = 0; //not full anymore
      return 0;
    }

    //legit good case
    *charHandle = my_queue[rptr].charHandle;
    *bufLength = my_queue[rptr].bufLength;
    memcpy(&buffer[0], &my_queue[rptr].buffer[0], my_queue[rptr].bufLength);
    depth_rtime--;  // update depth of the queue here
    rptr = nextPtr(rptr);
    if(rptr == wptr) // If there's a last element that was read, set empty to 1.
      empty = 1;

  }

  else
    return 1;

  return 0;

} // read_queue()


// ---------------------------------------------------------------------
// Public function.
// This function returns the wptr, rptr, full and empty values, writing
// to memory using the pointer values passed in, same rationale as read_queue()
// The "_" characters are used to disambiguate the global variable names from
// the input parameter names, such that there is no room for the compiler to make a
// mistake in interpreting your intentions.
// ---------------------------------------------------------------------
void get_queue_status (uint32_t *_wptr, uint32_t *_rptr, bool *_full, bool *_empty) {
  *_wptr = wptr;
  *_rptr = rptr;
  *_full = full;
  *_empty = empty;
} // get_queue_status()


// ---------------------------------------------------------------------
// Public function.
// Function that computes the number of written entries currently in the queue. If there
// are 3 entries in the queue, it should return 3. If the queue is empty it should
// return 0. If the queue is full it should return either QUEUE_DEPTH if
// USE_ALL_ENTRIES==1 otherwise returns QUEUE_DEPTH-1.
// ---------------------------------------------------------------------
uint32_t get_queue_depth() {
  return depth_rtime; // No logic is needed here because the depth is being tracked
            // in write_queue() and read_queue()
} // get_queue_depth()


#endif


/*
 * Bluetooth Event responder
 * This is the BT stack event handling
 * code that responds to events such
 * as the
 * sl_bt_system_get_identity_address,
 * sl_bt_advertiser_create_set,
 * sl_bt_advertiser_set_timing,
 * sl_bt_advertiser_start,
 * sl_bt_advertiser_stop,
 * sl_bt_connection_set_parameters,
 * sl_bt_evt_connection_closed_id,
 * sl_bt_evt_connection_parameters_id.
 * These events don't dictate a state machine
 * but just an event handler. The application layer functions
 * are defined in sl_bt_api.h
 *
 *
 * Parameters:
 * sl_bt_msg_t *evt
 *
 * return:
 * none
 *
 */
void handle_ble_event(sl_bt_msg_t *evt)
{

  //sl_bt_gatt_server_write_attribute_value

#if DEVICE_IS_BLE_SERVER
  uint16_t con_interval_ms = 75;
  uint16_t min_interval = (con_interval_ms / 1.25);  // 75 ms / 1.25 = 60 (value)
  uint16_t max_interval = (con_interval_ms / 1.25); // 75 ms / 1.25 = 60 (value)
  uint16_t latency = (300 / con_interval_ms);  //300/75
  uint16_t timeout = 83; //0x11A8...(1+4)*(75*2) + 75 = 825ms
  //convert 825 ms to a value --> 825 ms/ 10ms = 82.5,  ~83
  uint16_t min_ce_length = 0;
  uint16_t max_ce_length = 0;
  uint8_t *return_touch_state = 0;

  //MASTER SETS
  //uint16_t master_interval = 0;
  //uint16_t master_latency = 0;
  //uint16_t master_timeout = 0;
  sl_status_t status = 0;
  sl_status_t indication_stat = 0;
  //Defined an instance of queue_struct_t to pass values for read queueing
  queue_struct_t node;
  uint16_t       temp_charHandle;
  uint32_t        temp_bufLength;

  switch(SL_BT_MSG_ID(evt->header)) {
    //Events common to both servers and Clients

    // This event indicates the device has started and the radio is ready.
    // Do not call any stack API commands before receiving this boot event
    //Including starting BT stack soft timers!

    case sl_bt_evt_system_boot_id:


      status = sl_bt_sm_delete_bondings();
      ble_data.connection_open = 0;
      ble_data.indication_in_flight = 0;
      ble_data.ok_to_send_button_indications = 0;
      ble_data.ok_to_send_htm_indications = 0;
      ble_data.ok_to_send_hr_indications = 0;
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      reset_queue();

      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_delete_bondings() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }


      status = sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                       0,
                                                       1,
                                                       &ble_data.button_press);

      status = sl_bt_gatt_server_write_attribute_value(gattdb_heart_state,
                                                             0,
                                                             1,
                                                             &ble_data.button_press);

      //might need a server write for heart rate???

      if (status != SL_STATUS_OK)
        {
            LOG_ERROR("sl_bt_gatt_server_write_attribute_value() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      status = sl_bt_sm_configure(CONF_FLAG, sm_io_capability_displayyesno);

      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_configure() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      //Get this checked
      //Get this checked
      //Get this checked
      displayInit();
      //Get this checked
      //Get this checked
      //Get this checked
      displayPrintf(DISPLAY_ROW_9, "%s", "Button Released");




      status = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_system_get_identity_address INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      displayPrintf(DISPLAY_ROW_NAME, "%s", "SERVER");
      displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ble_data.myAddress.addr[5],
                    ble_data.myAddress.addr[4],
                    ble_data.myAddress.addr[3],
                    ble_data.myAddress.addr[2],
                    ble_data.myAddress.addr[1],
                    ble_data.myAddress.addr[0]);

      displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", "Course Project");


      status = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_create_set INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      status = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle, 0x190, 0x190, 0, 0);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_set_timing INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      status = sl_bt_advertiser_start(ble_data.advertisingSetHandle,sl_bt_advertiser_general_discoverable,sl_bt_advertiser_connectable_scannable);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_start INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Advertising");

      break;

    case sl_bt_evt_connection_opened_id:

      send_block_max30105(MAX30105_MODE_REG,MAX30105_SHUTDOWN_CONFIG);
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      ble_data.connectionHandle = evt->data.evt_connection_opened.connection;
      //LOG_INFO("Inside of connection open event.\r\n");

      status = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_stop INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Connected");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s ", " ");
      ble_data.connection_open = 1; //May want to maintain a flag that tracks whether a connection is open or closed.
      status = sl_bt_connection_set_parameters(ble_data.connectionHandle,
                                               min_interval,
                                               max_interval,
                                               latency,
                                               timeout,
                                               min_ce_length,
                                               max_ce_length);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_connection_set_parameters INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      break;


      /*
       * Indicate that a GATT service in the remote GATT database was
       * discovered
       */
    case sl_bt_evt_gatt_service_id:
      ble_data.touch_servicingHandle = evt->data.evt_gatt_service.service;
      break;


      /*
       * Indicate that a GATT service characteristic in the remote GATT database was
       * discovered
       */
    case sl_bt_evt_gatt_characteristic_id:
      ble_data.touch_characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
      break;


    case sl_bt_evt_gatt_characteristic_value_id:
            if( (evt->data.evt_gatt_characteristic.characteristic == gattdb_captouch_state)
                && (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication))
              {
                status = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionHandle);
                if (status != SL_STATUS_OK)
                  {
                    LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() INVALID/ERROR. STATCODE -> %X/r/n", status);
                  }
                return_touch_state = evt->data.evt_gatt_characteristic_value.value.data;

                if (*return_touch_state == 1)
                  {
                    displayPrintf(DISPLAY_ROW_11, "%s", "Heart Rate");
                  }
                else if (*return_touch_state == 2)
                  {
                    server_cs2_counter++;
                    if (server_cs2_counter > 32)
                      {
                        displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-Saving");
                        server_cs2_counter = 0;
                        send_block_max30105(MAX30105_MODE_REG,MAX30105_SHUTDOWN_CONFIG);
                      }
                    else if(server_cs2_counter > 6)
                      {
                        displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-Active");
                        send_block_max30105(MAX30105_MODE_REG,MAX30105_REDIRGREEN_MODE);
                      }
                   }
                else if (*return_touch_state == 4)
                  {
                    displayPrintf(DISPLAY_ROW_11, "%s", "SpO2");
                  }
                else if (*return_touch_state == 3)
                  {
                    displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-Saving");
                  }
              }
      break;

    case sl_bt_evt_connection_closed_id:

      send_block_max30105(MAX30105_MODE_REG,MAX30105_SHUTDOWN_CONFIG);
      status =  sl_bt_sm_delete_bondings();
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      ble_data.connection_open = 0; //May want to maintain a flag that tracks whether a connection is open or closed.
      ble_data.indication_in_flight = 0; //yet to recieve a confirmation from client
      ble_data.ok_to_send_htm_indications = 0;
      ble_data.ok_to_send_hr_indications = 0;
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s ", " ");
      displayPrintf(DISPLAY_ROW_HEARTRATE, "%s ", " ");
      displayPrintf(DISPLAY_ROW_10, "%s", " ");
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Advertising");
      displayPrintf(DISPLAY_ROW_PASSKEY, "%s", " ");
      displayPrintf(DISPLAY_ROW_ACTION, "%s", " ");
      displayPrintf(DISPLAY_ROW_11, "%s", " ");
      gpioLed1SetOff();
      gpioLed0SetOff();


      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_delete_bondings() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      status = sl_bt_advertiser_start(ble_data.advertisingSetHandle,sl_bt_advertiser_general_discoverable,sl_bt_advertiser_connectable_scannable);
      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_evt_connection_closed_id INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      break;

    case sl_bt_evt_connection_parameters_id:
      break;



      //Events for the security responder
      //Events for the security responder
      //Events for the security responder
      //Events for the security responder

      //Indicates a user request to display that the new bonding request
    case sl_bt_evt_sm_confirm_bonding_id:

      //LOG_INFO("ENTERED sl_bt_evt_sm_confirm_bonding_id event.\r\n");
      status = sl_bt_sm_bonding_confirm(ble_data.connectionHandle, 1);
      //Accept the bonding request
      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_bonding_confirm() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      break;

      //Indicates a request for passkey display and confirmation by the user
    case sl_bt_evt_sm_confirm_passkey_id:

      //LOG_INFO("ENTERED sl_bt_evt_sm_confirm_passkey event.\r\n");

      displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey : %d", evt->data.evt_sm_confirm_passkey.passkey);//0-99999
      displayPrintf(DISPLAY_ROW_ACTION, "%s", "Confirm with PB0");
      ble_data.passkey_status = 1;

      break;

    case sl_bt_evt_system_external_signal_id:


      if ((ble_data.bonded) && (ble_data.connection_open))
        {

              status =  sl_bt_gatt_set_characteristic_notification(ble_data.connectionHandle,
                                                   gattdb_captouch_state,
                                                   sl_bt_gatt_indication);
              //enter_once_cap = 0;
              if(status != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_set_characteristic_notification() for CAPTOUCH INVALID/ERROR. STATCODE -> %X\r\n", status);
                }

        }

      //Button Press event
      if(evt->data.evt_system_external_signal.extsignals & evtButtonPress)
        {
          ble_data.button_press = BUTTON0_PRESS;
          displayPrintf(DISPLAY_ROW_9, "%s", "Button Pressed");

          if((!ble_data.bonded) && (ble_data.passkey_status) && (ble_data.connection_open))
            {
              displayPrintf(DISPLAY_ROW_PASSKEY, "%s ", " ");
              displayPrintf(DISPLAY_ROW_ACTION, "%s ", " ");
              //GOOD TO BOND
              status = sl_bt_sm_passkey_confirm(ble_data.connectionHandle, 1);

              if (status != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_sm_passkey_confirm() INVALID/ERROR. STATCODE -> %X\r\n", status);
                }
            }

          //PB0 presses and releases update our local GATT database, and send indications when enabled
          status = sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                           0,
                                                           sizeof(ble_data.button_press),
                                                           &ble_data.button_press);

          if (status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_write_attribute_value() INVALID/ERROR. STATCODE -> %X\r\n", status);}


          if(ble_data.bonded)

            {

          if ((ble_data.bonded) &&
              (ble_data.connection_open) &&
              (!ble_data.indication_in_flight) &&
              (get_queue_depth() == 0) &&
              (ble_data.ok_to_send_button_indications))
            {
              //Send indication
              status = sl_bt_gatt_server_send_indication(
                                            ble_data.connectionHandle,
                                             gattdb_button_state,
                                             sizeof(ble_data.button_press),
                                             &ble_data.button_press
                                             );

              if ( status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_send_indication ERROR with sc: %u. Indications might not be enabled for button_state\r\n", status);}
              // Set indication in flight to 1
              else{ ble_data.indication_in_flight = 1;}

            }

          else
            {
                // Enqueue/Write to the buffer only for button_state
              //LOG_INFO("BUTTON WRITE QUEUE evtButtonPressed\r\n");
              status = write_queue(gattdb_button_state, sizeof(ble_data.button_press), &ble_data.button_press);
              if (status)
                {
                  LOG_ERROR("write_queue() not SUCCESSFUL for BUTTON PRESS.\r\n");
                }

            }

           }

          else
            {
              //LOG_INFO("NOT BONDED. BUTTON PRESS. NO QUEUE WRITES. IGNORE THIS.\r\n");
            }



        }

      //Button release event
      if (evt->data.evt_system_external_signal.extsignals & evtButtonRelease)
        {
          ble_data.button_press = BUTTON0_RELEASE;
          displayPrintf(DISPLAY_ROW_9, "%s", "Button Released");

          //PB0 presses and releases update our local GATT database, and send indications when enabled
          status = sl_bt_gatt_server_write_attribute_value(gattdb_button_state,
                                                           0,
                                                           sizeof(ble_data.button_press),
                                                           &ble_data.button_press);
          if (status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_write_attribute_value() INVALID/ERROR. STATCODE -> %X\r\n", status);}

          //need to be bonded first before going in here
          if (ble_data.bonded)

            {


          if ((ble_data.bonded) &&
              (ble_data.connection_open) &&
              (!ble_data.indication_in_flight) &&
              (get_queue_depth() == 0) &&
              (ble_data.ok_to_send_button_indications))
            {
                        //Send indication for button press
              status = sl_bt_gatt_server_send_indication(
                                  ble_data.connectionHandle,
                                  gattdb_button_state,
                                  sizeof(ble_data.button_press),
                                  &ble_data.button_press
                                        );

              if ( status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_send_indication ERROR with sc: %u. Indications might not be enabled for button_state\r\n", status);}
              // Set indication in flight to 1 in successful indication case
              else{ ble_data.indication_in_flight = 1;}

            }

          else
            {
              // Enqueue/Write to the buffer only for button_state
              //LOG_INFO("BUTTON WRITE QUEUE evtButtonRelease\r\n");
              status = write_queue(gattdb_button_state, sizeof(ble_data.button_press), &ble_data.button_press);
              if (status)
                {
                  LOG_ERROR("write_queue() not SUCCESSFUL for BUTTON RELEASE\r\n");
                }
            }

            }

          else
            {
              //LOG_INFO("NOT BONDED. BUTTON RELEASE. NO QUEUE WRITES. IGNORE THIS.\r\n");
            }


        }

      break;
//Events for the security responder above
//Events for the security responder above
//Events for the security responder above
//Events for the security responder above
//Events for the security responder above

//A8 ADDITION
      //Bonding successful
    case sl_bt_evt_sm_bonded_id:

      //send_block_max30105(MAX30105_MODE_REG,MAX30105_REDIRGREEN_MODE);
      ble_data.bonded = 1;
      ble_data.passkey_status = 0;
      //LOG_INFO("BONDED CONFIRMED.\r\n");
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s ", "Bonded");

      break;

      //Bonding not successful
    case sl_bt_evt_sm_bonding_failed_id:
      //LOG_INFO("BOND FAILED.\r\n");
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      //LOG_INFO("BONDED FAILED.\r\n");
//      displayPrintf(DISPLAY_ROW_PASSKEY, "%s", " ");
//      displayPrintf(DISPLAY_ROW_ACTION, "%s", " ");
      break;
//A8 ADDITION





      //Credit: David Sluiter for helping with the server characteristic
      // for the temperature measurement as of A5.
    case sl_bt_evt_gatt_server_characteristic_status_id:
      //Handle for three characteristics
      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
        //this is the temperature measurement characteristic
        {

          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            // a client is writing to the CCCD
            {

                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
                      {
                        ble_data.ok_to_send_htm_indications = 0;
                        displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s ", " ");
                        gpioLed0SetOff();
                        //a client is turning off indications
                      }

                  else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
                      {
                        ble_data.ok_to_send_htm_indications = 1;
                        gpioLed0SetOn();
                        //a client is turning on indications
                      }
            }



          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
            {
              ble_data.indication_in_flight = 0; //Use reception of indication confirmation to clear the indication_in flight flag
            }

        }

      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_heart_state)
        //this is the temperature measurement characteristic
        {

          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            // a client is writing to the CCCD
            {

                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
                      {
                        ble_data.ok_to_send_hr_indications = 0;

                        //displayPrintf(DISPLAY_ROW_HEARTRATE, "%s", "here");
                        //gpioLed0SetOff();
                        //a client is turning off indications
                      }

                  else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
                      {
                        ble_data.ok_to_send_hr_indications = 1;
                        //gpioLed1SetOn();
                        //LOG_INFO("hehehehehehe\r\n");
                        //displayPrintf(DISPLAY_ROW_HEARTRATE, "%s", "e");
                        //gpioLed0SetOn();
                        //a client is turning on indications
                      }
            }



          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
            {
              ble_data.indication_in_flight = 0; //Use reception of indication confirmation to clear the indication_in flight flag
            }

        }


      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state)
        {
          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            {
              if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
                                    {
                                      ble_data.ok_to_send_button_indications = 0;
                                      gpioLed1SetOff();
                                      //a client is turning off indications
                                    }
              else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
                                    {
                                      ble_data.ok_to_send_button_indications = 1;
                                      gpioLed1SetOn();
                                      //a client is turning on indications
                                    }

            }
              if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
                          {
                            ble_data.indication_in_flight = 0; //Use reception of indication confirmation to clear the indication_in flight flag
                          }

        }
      break;

    case sl_bt_evt_gatt_server_indication_timeout_id:
      ble_data.indication_in_flight = 0; //indication timeout
      //displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s", " ");
      break;

    case sl_bt_evt_system_soft_timer_id:
      {

        //LCD DISPLAY TIMER
        if (evt->data.evt_system_soft_timer.handle == 0) // only update lcd, every 1 seconds
          {
            displayUpdate();
          }

        //INDICATIONS SOFT TIMER

        if (evt->data.evt_system_soft_timer.handle == 1)
          {
            //There are queued indications AND no indication in flight
            if((empty != 1) && (!ble_data.indication_in_flight)) // if empty is 1, queue is empty
                {

                status = read_queue (&node.charHandle, &node.bufLength, &node.buffer[0]);

                temp_charHandle = node.charHandle;
                temp_bufLength = node.bufLength;


                if (status)
                  {
                    //Read queue from the queue didn't result in success.
                    LOG_ERROR("read_queue() not SUCCESSFUL.\r\n");
                  }


                else
                  {
                      //Successful read case, now able to indicate.
                        //LOG_INFO("Sending indication in soft timer for handle %u\r\n", node.charHandle);
                        indication_stat = sl_bt_gatt_server_send_indication(
                              ble_data.connectionHandle,
                              temp_charHandle,
                              temp_bufLength,
                              &node.buffer[0]
                              );
                        if ( indication_stat != SL_STATUS_OK)
                          {
                              LOG_ERROR("server send indication soft timer ERROR with status: %u\r\n", indication_stat);
                          }
                        else {
                          ble_data.indication_in_flight = 1;
                        }

                  }
                }
          }
      }


      break;


    default:
      break;

  }

#else

  //sl_status_t status = 0;
      uint8_t phy_sel = 1;
      uint8_t passive_scan = 0;
      uint16_t scan_interval = (50/0.625); //Value = 80
      uint16_t scan_window = (25/0.625); //Value = 40;

      uint16_t con_interval_ms = 75;
      uint16_t min_interval = (con_interval_ms / 1.25);  // 75 ms / 1.25 = 60 (value)
      uint16_t max_interval = (con_interval_ms / 1.25); // 75 ms / 1.25 = 60 (value)
      uint16_t latency = (300 / con_interval_ms);  //300/75
      uint16_t timeout = 83; //0x11A8...(1+4)*(75*2) + 75 = 825ms
        //convert 825 ms to a value --> 825 ms/ 10ms = 82.5,  ~83
      uint16_t min_ce_length = 0;
      uint16_t max_ce_length = 4;

      uint8_t scanning_phy = 1;
      uint8_t discover_mode = 2;

      uint8_t initiating_phy = 1;

      //from ble_device_type.h
      uint8_t server_bt_addy[6] = SERVER_BT_ADDRESS;

      int32_t return_temp = 0;
      uint8_t *return_button_state = 0;
    //  uint8_t return_heart_state[9] = {0};
    //  uint8_t return_heart_state[9] = {0};
      sl_status_t status = 0;
      uint8_t touch_area_status = 0;

      uint8_t comp = 0;
     // bool  indications_requested = 0;




  switch(SL_BT_MSG_ID(evt->header)) {

    //Getting this event means the device has started
    // and the device is ready
    case sl_bt_evt_system_boot_id:


      status = sl_bt_sm_delete_bondings();
      ble_data.connection_open = 0;
      ble_data.indication_in_flight = 0;
      ble_data.ok_to_send_button_indications = 0;
      ble_data.ok_to_send_htm_indications = 0;
      ble_data.ok_to_send_hr_indications = 0;
      ble_data.ok_to_send_captouch_indications = 0;
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;

      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_delete_bondings() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      status = sl_bt_gatt_server_write_attribute_value(gattdb_captouch_state,
                                                       0,
                                                       sizeof(ble_data.capacitive_touch_status),
                                                       &ble_data.capacitive_touch_status);

      if (status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_write_attribute_value() INVALID/ERROR. STATCODE -> %X\r\n", status);}



      status = sl_bt_sm_configure(CONF_FLAG, sm_io_capability_displayyesno);

      if (status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_sm_configure() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      displayInit(); //Might need this in app_init().

      //Get the clients address and display it
      status = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_system_get_identity_address INVALID/ERROR. STATCODE -> %X\r\n", status);
        }
      displayPrintf(DISPLAY_ROW_NAME, "%s", "CLIENT");
      displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ble_data.myAddress.addr[5],
                    ble_data.myAddress.addr[4],
                    ble_data.myAddress.addr[3],
                    ble_data.myAddress.addr[2],
                    ble_data.myAddress.addr[1],
                    ble_data.myAddress.addr[0]);

      displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", "Course Project");

      //Select 1M phy with passive scanning
      status = sl_bt_scanner_set_mode(phy_sel, passive_scan);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_set_mode() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      //Timing parameters specify
      status = sl_bt_scanner_set_timing(phy_sel, scan_interval, scan_window);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_set_timing() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      /*
       * Default Bluetooth connection parameters
       */
      status = sl_bt_connection_set_default_parameters(min_interval,
                                                       max_interval,
                                                       latency,
                                                       timeout,
                                                       min_ce_length,
                                                       max_ce_length);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_connection_set_default_parameters() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }


      //Start scannning for devices
      status = sl_bt_scanner_start(scanning_phy, discover_mode);
      if(status != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_start() INVALID/ERROR. STATCODE -> %X\r\n", status);
          }

      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Discovering");

      break; //End of sl_bt_evt_system_boot_id

    case sl_bt_evt_scanner_scan_report_id:

      comp = memcmp((uint8_t*)&evt->data.evt_scanner_scan_report.address.addr, &server_bt_addy, sizeof(server_bt_addy));
      if (comp == 0) //match case
        {
          //LOGGING FOR DEBUGGING PURPOSES
//          LOG_INFO("Value of event bt addr = %0x %0x %0x %0x %0x %0x and the server bt addr = %0x %0x %0x %0x %0x %0x\r\n",
//                   evt->data.evt_scanner_scan_report.address.addr[0],
//                   evt->data.evt_scanner_scan_report.address.addr[1],
//                   evt->data.evt_scanner_scan_report.address.addr[2],
//                   evt->data.evt_scanner_scan_report.address.addr[3],
//                                      evt->data.evt_scanner_scan_report.address.addr[4],
//                                      evt->data.evt_scanner_scan_report.address.addr[5],
//                                      server_bt_addy[0],
//                                      server_bt_addy[1],
//                                      server_bt_addy[2],
//                                      server_bt_addy[3],
//                                      server_bt_addy[4],
//                                      server_bt_addy[5]);

             status = sl_bt_scanner_stop();
             if (status != SL_STATUS_OK)
               {
                  LOG_ERROR("sl_bt_scanner_stop() INVALID/ERROR. STATCODE -> %X/r/n", status);
               }
             ble_data.connection_open = 1;
             status = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                            0, //use enum
                                            initiating_phy, //1M phy enum
                                            &ble_data.connectionHandle);
             if (status != SL_STATUS_OK)
               {
                 LOG_ERROR("sl_bt_connection_open() INVALID/ERROR. STATCODE -> %X/r/n", status);
               }
        }
      break;

    case sl_bt_evt_connection_opened_id:
      //Connection has been established
      ble_data.connectionHandle = evt->data.evt_connection_opened.connection;
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Connected");
      displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X",
                          server_bt_addy[5],
                          server_bt_addy[4],
                          server_bt_addy[3],
                          server_bt_addy[2],
                          server_bt_addy[1],
                          server_bt_addy[0]);
      send_block_cap1203(MAIN_CTRL_REG,MAIN_CTRL_REG_DEEP_SLEEP);
      break;

      /*
       * Indicate that a GATT service in the remote GATT database was
       * discovered
       */
    case sl_bt_evt_gatt_service_id:
      ble_data.htm_servicingHandle = evt->data.evt_gatt_service.service;
      ble_data.hr_servicingHandle = evt->data.evt_gatt_service.service;
      ble_data.button_servicingHandle = evt->data.evt_gatt_service.service;
      break;

      /*
       * Indicate that a GATT service characteristic in the remote GATT database was
       * discovered
       */
    case sl_bt_evt_gatt_characteristic_id:
      ble_data.htm_characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
      ble_data.hr_characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
      ble_data.button_characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
      break;

      //Send confirmation when characteristics was received
    case sl_bt_evt_gatt_characteristic_value_id:
      //htm characteristic handle check
      if( (evt->data.evt_gatt_characteristic.characteristic == gattdb_temperature_measurement)
          && (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication))
        {
          status = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionHandle);
          if (status != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() INVALID/ERROR. STATCODE -> %X/r/n", status);
            }

          return_temp = FLOAT_TO_INT32(&evt->data.evt_gatt_characteristic_value.value.data[0]);
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s= %d", "Temperature", return_temp);
        }

      //button characteristic handle check
      if( (evt->data.evt_gatt_characteristic.characteristic == gattdb_button_state)
          && (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication))
        {
          status = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionHandle);
          if (status != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() INVALID/ERROR. STATCODE -> %X/r/n", status);
            }
          return_button_state = evt->data.evt_gatt_characteristic_value.value.data;

          if (*return_button_state == 1)
            {
              displayPrintf(DISPLAY_ROW_9, "%s", "Button Pressed");
            }
          else
            {
              displayPrintf(DISPLAY_ROW_9, "%s", "Button Released");
            }
        }

      if( (evt->data.evt_gatt_characteristic.characteristic == gattdb_heart_state)
          && (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication))
        {
          status = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionHandle);
          if (status != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() INVALID/ERROR. STATCODE -> %X/r/n", status);
            }

          ADC_COUNT_PROCESSING(&evt->data.evt_gatt_characteristic_value.value.data[0]);
         // *return_heart_state = (uint8_t *) (&evt->data.evt_gatt_characteristic_value.value.data[0]);
          LOG_INFO("hr pad is %u, em_pad is %u, spo2_pad is %u\r\n", hr_pad, em_pad, spo2_pad);

          if(hr_pad)
            {
              displayPrintf(DISPLAY_ROW_HEARTRATE, "%s= %d", "BPM", red_read_arr_client);
              if(red_read_arr_client <30)
                {

                }
              if(red_read_arr_client > 50)
                {
                  //gpiopPB11SetOn();
                }
              if(red_read_arr_client > 60)
                {
                  //gpiopPF0SetOn();
                }
              if(red_read_arr_client > 70)
                {
                  //gpiopPF1SetOn();
                }
              if(red_read_arr_client > 80)
                {
                  //gpiopPF2SetOn();
                }
              if(red_read_arr_client > 90)
                {
                  //gpiopPD14SetOn();
                }
              if(red_read_arr_client > 100)
                {
                  //gpiopPD13SetOn();
                }

              displayPrintf(DISPLAY_ROW_ACTION, "%s= %d", "IR ADC", IR_reading_client);
              //displayPrintf(DISPLAY_ROW_11, "%s", " ");
              displayPrintf(DISPLAY_ROW_10, "%s", " ");
            }
          else if(em_pad)
            {

              displayPrintf(DISPLAY_ROW_10, "%s", "EM0");
             displayPrintf(DISPLAY_ROW_HEARTRATE, "%s", " ");
             displayPrintf(DISPLAY_ROW_ACTION, "%s", " ");
            }
          else if(spo2_pad)
            {

              displayPrintf(DISPLAY_ROW_10, "%s= %d", "Green ADC", green_reading_client);
              displayPrintf(DISPLAY_ROW_HEARTRATE, "%s", " ");
              displayPrintf(DISPLAY_ROW_ACTION, "%s= %d", "IR ADC", IR_reading_client);
            }
        }

      break;

      //This event indicates that a event is closed.
    case sl_bt_evt_connection_closed_id:
      status =  sl_bt_sm_delete_bondings();
      ble_data.connection_open = 0;
      gpioLed0SetOff();
      gpioLed1SetOff();
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      displayPrintf(DISPLAY_ROW_PASSKEY, "%s", " ");//0-99999
      displayPrintf(DISPLAY_ROW_ACTION, "%s", " ");
      displayPrintf(DISPLAY_ROW_10, "%s", " ");
      send_block_cap1203(MAIN_CTRL_REG,MAIN_CTRL_REG_DEEP_SLEEP);
      status = sl_bt_scanner_start(scanning_phy, discover_mode);
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Discovering");
      displayPrintf(DISPLAY_ROW_BTADDR2, "%s", " ");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s", " ");
      displayPrintf(DISPLAY_ROW_HEARTRATE, "%s", " ");
      //displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", " ");
      displayPrintf(DISPLAY_NUMBER_OF_ROWS, "%s", " ");
      displayPrintf(DISPLAY_ROW_9, "%s", " ");

      break;


    case sl_bt_evt_system_soft_timer_id:
          //This should be it
        displayUpdate();

      break;


    case sl_bt_evt_gatt_procedure_completed_id:
      //After bonding, read characteristic will not yield error and enter here.
      if(evt->data.evt_gatt_procedure_completed.result == SL_STATUS_OK)
        {
          if(ble_data.ok_to_send_button_indications == 0)
            {
                return_button_state = evt->data.evt_gatt_characteristic_value.value.data;
                if (*return_button_state == 1)
                  {
                    displayPrintf(DISPLAY_ROW_9, "%s", "Button Pressed");
                  }
                else if (*return_button_state == 0)
                  {
                    displayPrintf(DISPLAY_ROW_9, "%s", "Button Released");
                  }
            }

          if(ble_data.ok_to_send_hr_indications == 0)
            {
              ADC_COUNT_PROCESSING(&evt->data.evt_gatt_characteristic_value.value.data[0]);
            }
        }



      //Read characteristic first timer yields an error.
      if(evt->data.evt_gatt_procedure_completed.result == SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION)
      {

          //indications_requested = 0;
          LOG_INFO("increase security triggered.\r\n");
         status =  sl_bt_sm_increase_security(ble_data.connectionHandle);//Increase security for push button

         if(status != SL_STATUS_OK)
           {
             LOG_ERROR("sl_bt_sm_increase_security() INVALID/ERROR. STATCODE -> %X\r\n", status);
           }

      }

      break;

    case sl_bt_evt_connection_parameters_id:
      //DEBUGS
      break;

    case sl_bt_evt_sm_bonded_id:
      ble_data.bonded = 1;
      ble_data.passkey_status = 0;
      displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s ", "Bonded");
      //Wake from deep sleep
       send_block_cap1203(MAIN_CTRL_REG,MAIN_CTRL_REG_DEF);
      break;

    case sl_bt_evt_sm_bonding_failed_id:
      ble_data.bonded = 0;
      ble_data.passkey_status = 0;
      send_block_cap1203(MAIN_CTRL_REG,MAIN_CTRL_REG_DEEP_SLEEP);
      break;

    case sl_bt_evt_sm_confirm_passkey_id:
      ble_data.passkey_status = 1;
      displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey : %d", evt->data.evt_sm_confirm_passkey.passkey);//0-99999
      displayPrintf(DISPLAY_ROW_ACTION, "%s", "Confirm with PB0");
      break;


    case sl_bt_evt_system_external_signal_id:


      if ((ble_data.bonded) && (ble_data.connection_open))
        {
      status =  sl_bt_gatt_set_characteristic_notification(ble_data.connectionHandle,
                                                         gattdb_heart_state,
                                                         sl_bt_gatt_indication);

      //enter_once_heart = 0;
      if(status != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_set_characteristic_notification() HEART INVALID/ERROR. STATCODE -> %X\r\n", status);
        }

        }

      //evtButtonPress is for PB0, evtButton1Press is for PB1
      if(evt->data.evt_system_external_signal.extsignals & evtCapTouch)
        {
          /*****************************************/
          send_command_cap1203(SENSOR_GEN_STATUS);
          touch_area_status = read_cap_1203();
          LOG_INFO("status reg is %X\n\r",touch_area_status);
          //touch_area_status &= (0x00010000);
          if(touch_area_status == 16)
            {
              displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-2sec");
              hr_pad = 0;
              em_pad = 1;
              spo2_pad = 0;
            }
          else
            {
              /*****************************************/
              send_command_cap1203(SENSOR_STAT_REG);
              touch_area_status = read_cap_1203();

              if(long_touch_flag)
                {
                  touch_area_status = 3;
                  ble_data.capacitive_touch_status = 3;
                  long_touch_flag = 0;
                  for(int i = 0; i<1000; i++);
                }
              else
                {
                  ble_data.capacitive_touch_status = touch_area_status;
                }
              clear_cap1203_interrupt();
              LOG_INFO("Touched Area is %X\r\n", touch_area_status);

              if(touch_area_status == 1)
                {
                    displayPrintf(DISPLAY_ROW_11, "%s", "Heart Rate Requested");
                    hr_pad = 1;
                    em_pad = 0;
                    spo2_pad = 0;
                }

              else if(touch_area_status == 2)
                {
                  cs2_counter++;
                  if(cs2_counter > 32)
                  {
                      displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-Saving");
                      cs2_counter = 0;
                      long_touch_flag = 1;
                      ble_data.capacitive_touch_status = 3;

                  }
                  else if (cs2_counter > 6)
                    {
                      displayPrintf(DISPLAY_ROW_11, "%s", "Energy Mode-Active");
                      hr_pad = 0;
                      em_pad = 1;
                      spo2_pad = 0;
                    }
                }
              else if(touch_area_status == 4)
                {
                  displayPrintf(DISPLAY_ROW_11, "%s", "SpO2 Requested");
                  hr_pad = 0;
                  em_pad = 0;
                  spo2_pad = 1;
                }
            }

          status = sl_bt_gatt_server_write_attribute_value(gattdb_captouch_state,
                                                           0,
                                                           sizeof(ble_data.capacitive_touch_status),
                                                           &ble_data.capacitive_touch_status);

          if (status != SL_STATUS_OK) {LOG_ERROR("sl_bt_gatt_server_write_attribute_value() INVALID/ERROR. STATCODE -> %X\r\n", status);}


          if ((ble_data.bonded) &&
              (ble_data.connection_open) &&
              (!ble_data.indication_in_flight) &&
              (ble_data.ok_to_send_captouch_indications))
            {
              status = sl_bt_gatt_server_send_indication(
                                            ble_data.connectionHandle,
                                             gattdb_captouch_state,
                                             sizeof(ble_data.capacitive_touch_status),
                                             &ble_data.capacitive_touch_status
                                             );

              if ( status != SL_STATUS_OK) {
                  LOG_ERROR("sl_bt_gatt_server_send_indication ERROR with sc: %u. Indications might not be enabled for button_state\r\n", status);
              }
              // Set indication in flight to 1
              else{
                  ble_data.indication_in_flight = 1;
              }
            }
        }


      if(evt->data.evt_system_external_signal.extsignals & evtButtonPress)
        {
          if ((ble_data.passkey_status) && (!ble_data.bonded) && (ble_data.connection_open))
            {

              displayPrintf(DISPLAY_ROW_PASSKEY, "%s ", " ");
              displayPrintf(DISPLAY_ROW_ACTION, "%s ", " ");
              //GOOD TO BOND
              status = sl_bt_sm_passkey_confirm(ble_data.connectionHandle, 1);

              if (status != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_sm_passkey_confirm() CL INVALID/ERROR. STATCODE -> %X\r\n", status);
                }
            }
        }


      if(evt->data.evt_system_external_signal.extsignals & evtButton1Press)
        {

          if ((!ble_data.passkey_status) && (ble_data.bonded) && (GPIO_PinInGet(gpioPortF, BUTTON0GPIO) == 0))
            {
              if (!indication_or_no) //toggling variable
                {
                     status =  sl_bt_gatt_set_characteristic_notification(ble_data.connectionHandle,
                                                                            gattdb_button_state,
                                                                            sl_bt_gatt_disable);
                     if(status != SL_STATUS_OK)
                       {
                         LOG_ERROR("sl_bt_gatt_set_characteristic_notification() INVALID/ERROR. STATCODE -> %X\r\n", status);
                       }
                      indication_or_no = 1;

                }

              else
                {
                     status =  sl_bt_gatt_set_characteristic_notification(ble_data.connectionHandle,
                                                                  gattdb_button_state,
                                                                  sl_bt_gatt_indication);

                     if(status != SL_STATUS_OK)
                       {
                         LOG_ERROR("sl_bt_gatt_set_characteristic_notification() INVALID/ERROR. STATCODE -> %X\r\n", status);
                       }
                    indication_or_no = 0;
                }

            }


          else {


          status =  sl_bt_gatt_read_characteristic_value(ble_data.connectionHandle, gattdb_button_state);
          LOG_INFO("After sl_bt_gatt_read_characteristic_value().\r\n");
         // indications_requested = 1;

          if (status != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_read_characteristic_value() INVALID/ERROR. STATCODE -> %X\r\n", status);
            }

          }
        }


      break;


    case sl_bt_evt_gatt_server_characteristic_status_id:
      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_captouch_state)
        //this is the temperature measurement characteristic
        {

          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
            // a client is writing to the CCCD
            {

                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)
                      {
                        ble_data.ok_to_send_captouch_indications = 0;
                        gpioLed0SetOff();
                        //a client is turning off indications
                      }

                  else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
                      {
                        ble_data.ok_to_send_captouch_indications = 1;
                        gpioLed0SetOn();
                        //a client is turning on indications
                      }
            }



          if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
            {
              ble_data.indication_in_flight = 0; //Use reception of indication confirmation to clear the indication_in flight flag
            }

        }
      break;




  }



#endif

}


// -----------------------------------------------
// Private function, original from Dan Walkes. I fixed a sign extension bug.
// We'll need this for Client A7 assignment to convert health thermometer
// indications back to an integer. Convert IEEE-11073 32-bit float to signed integer.
// -----------------------------------------------

#if !DEVICE_IS_BLE_SERVER

static int32_t FLOAT_TO_INT32(const uint8_t *buffer_ptr)
{
    uint8_t signByte = 0;
    int32_t mantissa;
    // input data format is:
    // [0] = flags byte, bit[0] = 0 -> Celsius; =1 -> Fahrenheit
    // [3][2][1] = mantissa (2's complement)
    // [4] = exponent (2's complement)
    // BT buffer_ptr[0] has the flags byte
    int8_t exponent = (int8_t)buffer_ptr[4];
    // sign extend the mantissa value if the mantissa is negative
    if (buffer_ptr[3] & 0x80) { // msb of [3] is the sign of the mantissa
        signByte = 0xFF;
    }
    mantissa = (int32_t) (buffer_ptr[1] << 0) |
                          (buffer_ptr[2] << 8) |
                          (buffer_ptr[3] << 16) |
                          (signByte << 24) ;
    // value = 10^exponent * mantissa, pow() returns a double type
    return (int32_t) (pow(10, exponent) * mantissa);
} // FLOAT_TO_INT32



void ADC_COUNT_PROCESSING(const uint8_t *adc_buffer)
{
      uint8_t red_byte3 = 0;
      uint8_t red_byte2 = 0;
      uint8_t red_byte1 = 0;

      uint8_t IR_byte3 = 0;
      uint8_t IR_byte2 = 0;
      uint8_t IR_byte1 = 0;

      uint8_t green_byte3 = 0;
      uint8_t green_byte2 = 0;
      uint8_t green_byte1 = 0;


      red_byte3 = adc_buffer[0];
      red_byte2 = adc_buffer[1];
      red_byte1 = adc_buffer[2];

      red_reading_client = (uint32_t) ((red_byte3 << 16) | (red_byte2 << 8) | red_byte1);
      red_reading_client &= 0x0003FFFF;

      LOG_INFO("Red Reading Raw : %u\r\n", red_reading_client);
      red_read_arr_client = red_reading_client/100;
      LOG_INFO("Red Reading : %u\r\n", red_read_arr_client);

      IR_byte3 = adc_buffer[3];
      IR_byte2 = adc_buffer[4];
      IR_byte1 = adc_buffer[5];
      IR_reading_client = (uint32_t) ((IR_byte3 << 16) | (IR_byte2 << 8) | IR_byte1);
      IR_reading_client &= 0x0003FFFF;
      LOG_INFO("IR Reading: %u\r\n", IR_reading_client);


      green_byte3 = adc_buffer[6];
      green_byte2 = adc_buffer[7];
      green_byte1 = adc_buffer[8];
      green_reading_client = (uint32_t) ((green_byte3 << 16) | (green_byte2 << 8) | green_byte1);
      green_reading_client &= 0x0003FFFF;
      LOG_INFO("Green Reading: %u\r\n", green_reading_client);


}

#endif

