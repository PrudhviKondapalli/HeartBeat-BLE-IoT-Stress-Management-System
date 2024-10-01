/***************************************************************************//**
 * @file
 * @brief Scheduler implementation file
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
 * @credit     David Sluiter for debugging help throughout Assignment 7
 *             regarding the discovery state machine
 * @credit2    Prudhvi Kondapalli for debugging help
 *
 *
 ******************************************************************************/
#include "em_core.h"
#include "scheduler.h"
#include "app.h"
#include "i2c.h"
#include "timers.h"
#include "gpio.h"
#include "sl_bt_api.h"
#include "ble.h"
#include "gatt_db.h"
#include "lcd.h"
#include "ble_device_type.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

//uint32_t myEvents = 0;
  int32_t return_temp;
  int32_t three_sec_count = 0;
  static uint8_t output_fifo[9] = {0};
  uint16_t hrate = 0;
  uint16_t spo2 = 0;
  uint8_t red_byte3 = 0;
  uint8_t red_byte2 = 0;
  uint8_t red_byte1 = 0;
  uint32_t red_reading = 0;
  uint8_t red_read_arr[1] = {0};
  uint8_t IR_byte3 = 0;
  uint8_t IR_byte2 = 0;
  uint8_t IR_byte1 = 0;
  uint32_t IR_reading = 0;
  uint8_t green_byte3 = 0;
  uint8_t green_byte2 = 0;
  uint8_t green_byte1 = 0;
  uint32_t  green_reading = 0;
  uint32_t delta = 0;
  uint32_t old_time = 0;
  uint32_t beatsPerMinute = 0;
  uint32_t beatsPer3sec = 0;
  uint8_t heart_beats = 0;
  uint8_t heart_blips = 0;
  uint8_t status = 0;

#if DEVICE_IS_BLE_SERVER
  static void attribute_write_and_send(void);
  static void attribute_write_and_send_hr(void);
#endif
/*
 *  Sets an event in the myEvents
 *  global variable for the read
 *  temperature event. CRITICAL SECTIONS
 *  implemented.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void schedulerSetReadTempEvent(void)
{
  CORE_DECLARE_IRQ_STATE;
  //set event
  CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
  //myEvents |= evtReadTemperature;
  sl_bt_external_signal(evtReadTemperature);
  CORE_EXIT_CRITICAL();
}


#if DEVICE_IS_BLE_SERVER
void schedulerSetHREvent(void)
{
  CORE_DECLARE_IRQ_STATE;
  //set event
  CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
  sl_bt_external_signal(evtReadHR);
  CORE_EXIT_CRITICAL();
}
#endif

/*
 *  Sets an event in the myEvents
 *  global variable for the timer
 *  done. CRITICAL SECTIONS
 *  implemented.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void schedulerEvtTimerDone(void)
{
  CORE_DECLARE_IRQ_STATE;
   //set event
   CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
   //myEvents |= evtTimerDone;
   sl_bt_external_signal(evtTimerDone);
   CORE_EXIT_CRITICAL();
}

/*
 *  Sets an event in the myEvents
 *  global variable to set an
 *  event for I2C transfer completes.
 *  CRITICAL SECTIONS implemented.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void schedulerEvtI2CTransferDone(void)
{
  CORE_DECLARE_IRQ_STATE;
   //set event
   CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
  // myEvents |= evtI2CTransferComplete;
   sl_bt_external_signal(evtI2CTransferComplete);
   CORE_EXIT_CRITICAL();
}



void schedulerEvtButtonPress(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
  sl_bt_external_signal(evtButtonPress);
  CORE_EXIT_CRITICAL();
}


void schedulerEvtButtonRelease(void)
{
   CORE_DECLARE_IRQ_STATE;
   CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
   sl_bt_external_signal(evtButtonRelease);
   CORE_EXIT_CRITICAL();
}


#if !DEVICE_IS_BLE_SERVER
void schedulerEvtButton1Press(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
    sl_bt_external_signal(evtButton1Press);
    CORE_EXIT_CRITICAL();
}

void schedulerEvtButton1Release(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
    sl_bt_external_signal(evtButton1Release);
    CORE_EXIT_CRITICAL();
}

void schedulerEvtCapTouched(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL(); //enter critical section, turn off interrupts NVIC
  sl_bt_external_signal(evtCapTouch);
  CORE_EXIT_CRITICAL();
}
#endif

int32_t ret_three_sec_count(void)
{
  return three_sec_count;
}

/*
 * This function implements the temperature
 * state machine used in A5 in order
 * to perform a temperature measurement
 * of the Si7021 temperature sensor. The state
 * machine consists of five states, stateIdle,
 * waitForTimerToWrite, waitForI2CWriteComplete,
 * waitForTimerToRead, and waitForI2CReadComplete
 * The state machine is implemented in such
 * a way that the CPU will sleep to
 * EM1 while there are I2C transfers happening,
 * and will sleep to EM2 while there is a
 * time delay between the I2C transfers. The
 * interrupts utilized are COMP1, UF, and
 * I2CTransferComplete. This function
 * takes the evt external signals in from
 * the bluetooth event responder and
 * handles these external signals appropriately.
 * It also calls the attribute_write_and_send()
 * function which writes the temperature
 * data into the local BT stack.
 *
 * Parameters:
 * evt - external event
 * that's being processed by the bluetooth
 * event responder
 *
 * return:
 * none
 *
 */

#if DEVICE_IS_BLE_SERVER
void temperature_state_machine(sl_bt_msg_t *evt) //event to the Bluetooth, external evvent
{
    //if statement here
    if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
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
         if (evt->data.evt_system_external_signal.extsignals & evtReadTemperature)
           {
             //power_on_si7021();
             three_sec_count++;
             timerWaitUs_irq(80000);
             nextState = waitForTimerToWrite;
           }
         break;
       //Timer Event Complete
       //Start I2C Write
       //Advance State
       case waitForTimerToWrite:
         nextState = waitForTimerToWrite;
         if (evt->data.evt_system_external_signal.extsignals & evtTimerDone) //80ms
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
         if (evt->data.evt_system_external_signal.extsignals & evtI2CTransferComplete)
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
         if (evt->data.evt_system_external_signal.extsignals & evtTimerDone)
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
         if (evt->data.evt_system_external_signal.extsignals & evtI2CTransferComplete)
           {
             NVIC_DisableIRQ(I2C0_IRQn);
             sl_power_manager_remove_em_requirement(1);
             //power_off_si7021();
             return_temp = read_data_convert();
            // LOG_INFO("Read temperature: %d celsius\r\n", return_temp);
             //Write our local GATT DB
             attribute_write_and_send();
             nextState = stateIdle;
           }
         break;

       default:
         break;
     } //switch


      }

} //state_machine()


void hr_state_machine(sl_bt_msg_t *evt)
{
    if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
      {

    State_heart_t            currentState;
    static State_heart_t     nextState = STATE_IDLE_HEART;

    currentState = nextState;

    switch(currentState)
    {
      case STATE_IDLE_HEART:
        nextState = STATE_IDLE_HEART;
        if (evt->data.evt_system_external_signal.extsignals & evtReadHR)
          {
            sl_power_manager_add_em_requirement(1); //EM1
            send_command_max30105(MAX30105_FIFO_DATA_REG);
            nextState = STATE_HR_MEASURE;
          }
        break;

      case STATE_HR_MEASURE:
        nextState = STATE_HR_MEASURE;
        if (evt->data.evt_system_external_signal.extsignals & evtI2CTransferComplete)
          {
            NVIC_DisableIRQ(I2C0_IRQn);
            sl_power_manager_remove_em_requirement(1);
            sl_power_manager_add_em_requirement(1);
            status = read_from_max30105_block(&output_fifo[0], sizeof(output_fifo));
            nextState = STATE_HR_CALCULATE;
          }
        break;

      case STATE_HR_CALCULATE:
        nextState = STATE_HR_CALCULATE;
        if (evt->data.evt_system_external_signal.extsignals & evtI2CTransferComplete)
          {
            NVIC_DisableIRQ(I2C0_IRQn);
            sl_power_manager_remove_em_requirement(1);
            red_byte3 = output_fifo[0];
            red_byte2 = output_fifo[1];
            red_byte1 = output_fifo[2];
            red_reading = (uint32_t) ((red_byte3 << 16) | (red_byte2 << 8) | red_byte1);
            red_reading &= 0x0003FFFF;
            LOG_INFO("Red Reading Raw : %u\r\n", red_reading);
            red_read_arr[0] = red_reading/100;
            LOG_INFO("Red Reading : %u\r\n", red_read_arr[0]);

            IR_byte3 = output_fifo[3];
            IR_byte2 = output_fifo[4];
            IR_byte1 = output_fifo[5];
            IR_reading = (uint32_t) ((IR_byte3 << 16) | (IR_byte2 << 8) | IR_byte1);
            IR_reading &= 0x0003FFFF;
            LOG_INFO("IR Reading: %u\r\n", IR_reading);


            green_byte3 = output_fifo[6];
            green_byte2 = output_fifo[7];
            green_byte1 = output_fifo[8];
            green_reading = (uint32_t) ((green_byte3 << 16) | (green_byte2 << 8) | green_byte1);
            green_reading &= 0x0003FFFF;
            LOG_INFO("Green Reading: %u\r\n", green_reading);

            if(red_reading > 500)
               {
                 heart_blips = heart_blips + 1;
               //  LOG_INFO("Blip count is %u\n\r",heart_blips);
                 if (heart_blips % 10 == 0)
                   {
                     heart_beats = heart_beats + 1;
                  //   LOG_INFO("Beat count is %u\n\r",heart_beats);
                  //   LOG_INFO("sec_counter is %u\n\r",ret_three_sec_count());
                     if(ret_three_sec_count() - old_time > 0)
                       {
                         old_time = ret_three_sec_count();
                         beatsPer3sec = heart_beats;
                         beatsPerMinute = 20*beatsPer3sec;
                         LOG_INFO("BPM is %u\n\r", beatsPerMinute);
                         heart_beats = 0;
                         beatsPer3sec = 0;
                         heart_blips = 0;
                       }
                   }
               }


            attribute_write_and_send_hr();
            nextState = STATE_IDLE_HEART;
          }
        break;

      default:
        break;

    }
      }
}


void discovery_state_machine_for_server(sl_bt_msg_t *evt) //event to the Bluetooth, external event
{
    ble_data_struct_t *bleDataPtr1 = getBleDataPtr();
    sl_status_t status = 0;
    const uint8_t touch_service_uuid[16] = {0xe5, 0xf5, 0x1f, 0x03, 0x62, 0x71, 0x17, 0x86, 0x61, 0x4e, 0x3c, 0x05, 0xc8, 0x6e, 0x25, 0x10};
                                            //10256ec8-053c-4e61-8617-716203 1ff5e5
    size_t touch_service_uuid_len = sizeof(touch_service_uuid);
    const uint8_t touch_chararacteristic_uuid[16] = {0x0c, 0x38, 0xc6, 0xbd, 0x15, 0xd6, 0xaf, 0x98, 0x5a, 0x46, 0xe3, 0xdd, 0x30, 0x1d, 0xfc, 0x62};
    size_t touch_characteristic_uuid_len = sizeof(touch_chararacteristic_uuid);
        // 62fc1d30-dde3-465a-98af-d615bdc6380c

    State_t     currentState;
    static State_t     nextState = STATE_IDLE;
    currentState = nextState;

    switch(currentState)
    {
      case STATE_IDLE:
        nextState = STATE_IDLE;
        if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id) //should be open id event
                  {
                    //LOG_INFO("STATE IDLE\n\r");
                    //finds temp service
                    status = sl_bt_gatt_discover_primary_services_by_uuid(
                        bleDataPtr1->connectionHandle,
                        touch_service_uuid_len,
                        &touch_service_uuid[0]);
                    if (status != SL_STATUS_OK)
                     {
                          LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x\n\r", (unsigned int)status);
                     }
                    else
                     {
                        nextState = STATE_DISCOVER_SERVICE_TOUCH;
                     }
                  }
                if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
                {
                  nextState = STATE_IDLE;
                }
                break;
        break;
      case STATE_DISCOVER_SERVICE_TOUCH:
        nextState = STATE_DISCOVER_SERVICE_TOUCH;
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id))
                  {
                    status = sl_bt_gatt_discover_characteristics_by_uuid(
                        bleDataPtr1->connectionHandle,
                        bleDataPtr1->touch_servicingHandle,
                        touch_characteristic_uuid_len,
                        &touch_chararacteristic_uuid[0]);
                    if (status != SL_STATUS_OK)
                     {
                          LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x\n\r", (unsigned int)status);
                     }
                    else
                     {
                       nextState = STATE_DISCOVER_CHARACTERISTIC_TOUCH;
                     }
                  }
                if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
                {
                  nextState = STATE_IDLE;
                }
        break;
      case STATE_DISCOVER_CHARACTERISTIC_TOUCH:
        nextState = STATE_DISCOVER_CHARACTERISTIC_TOUCH;
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id))
          {
            status = sl_bt_gatt_set_characteristic_notification(
                bleDataPtr1->connectionHandle,
                bleDataPtr1->touch_characteristicHandle,
                sl_bt_gatt_indication);
            if (status != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_set_characteristic_notification() returned != 0 status=0x%04x\n\r", (unsigned int)status);
              }
            else
              {
                nextState = STATE_DISCOVER_CHARACTERISTIC_VALUE_TOUCH;
                LOG_INFO("happened once set char notif. cap touch\r\n");
              }
          }
        if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
          {
            nextState = STATE_IDLE;
          }
        break;
      case STATE_DISCOVER_CHARACTERISTIC_VALUE_TOUCH:
        nextState = STATE_DISCOVER_CHARACTERISTIC_VALUE_TOUCH;
        if((SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id))
          {
            //displayPrintf(DISPLAY_ROW_11,"%s","Hand. Ind. Touch");
            nextState = STATE_PROCEDURE_COMPLETED_CLOSE_WAIT;
          }
        if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
          {
            nextState = STATE_IDLE;
          }
        break;
      case STATE_PROCEDURE_COMPLETED_CLOSE_WAIT:
        nextState = STATE_PROCEDURE_COMPLETED_CLOSE_WAIT;
        if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
          {
            nextState = STATE_IDLE;
          }
        break;

      default:
        break;

    }


}



static void attribute_write_and_send_hr(void)
{
  sl_status_t sc;
  ble_data_struct_t *bleDataPtr1 = getBleDataPtr();

  sc = sl_bt_gatt_server_write_attribute_value(
            gattdb_heart_state, //handle from gatt_db.h
            0,                              //offset
            sizeof(output_fifo),           //length
            &output_fifo[0]      // in IEEE-11073 format
        );

  if(sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_gatt_server_write_attribute_value failed with sc = %u\r\n", sc);
      }


  if (bleDataPtr1->connection_open && bleDataPtr1->ok_to_send_hr_indications && !(bleDataPtr1->indication_in_flight) && (bleDataPtr1->bonded))
      {

      LOG_INFO("HR INDICATION SEND\r\n");

        sc = sl_bt_gatt_server_send_indication(
            bleDataPtr1->connectionHandle,
            gattdb_heart_state,
            sizeof(output_fifo),
            &output_fifo[0]
            );
        displayPrintf(DISPLAY_ROW_HEARTRATE, "%s= %u","BPM", red_read_arr[0]);
        displayPrintf(DISPLAY_ROW_ACTION, "%s= %u", "IR Count", IR_reading);
        displayPrintf(DISPLAY_ROW_10, "%s= %u", "Green Count", green_reading);

         if ( sc != SL_STATUS_OK) {LOG_ERROR("server send indication ERROR with sc: %u\r\n", sc);}
         else{ bleDataPtr1->indication_in_flight = 1;}

      }

  else if (bleDataPtr1->ok_to_send_hr_indications && (bleDataPtr1->indication_in_flight) && (bleDataPtr1->connection_open) && (bleDataPtr1->bonded))
    {
      LOG_INFO("HR INDICATION QUEUED\r\n");
      //enqueue here
      //LOG_INFO("HTM TEMPERATURE WRITE QUEUE\r\n");
      sc = write_queue(gattdb_heart_state, sizeof(output_fifo), &output_fifo[0]);// gattdb_temperature_measurement OR gattdb_button_state
      if (sc)
        {
          LOG_ERROR("write_queue() HR not SUCCESSFUL for press.\r\n");
        }
      displayPrintf(DISPLAY_ROW_HEARTRATE, "%s ", " ");
    }

  else
    {
      LOG_INFO("HR INDICATION NOT QUEUED\r\n");
      displayPrintf(DISPLAY_ROW_HEARTRATE, "%s ", " ");
    }
}
/*
 * Writes the temperature data into the
 * local BT GATT database.
 * This is done by calling the
 * sl_bt_gatt_server_send_indication()
 * function. Appropriate logs are provided
 *
 * Parameters:
 * none-
 *
 * return:
 * none
 *
 */
static void attribute_write_and_send(void) //htm
{

  sl_status_t sc;
  ble_data_struct_t *bleDataPtr1 = getBleDataPtr();

  uint8_t     htm_temperature_buffer[5];
  uint8_t     *p = &htm_temperature_buffer[0];
  uint32_t    htm_temperature_fit;
  uint8_t     flags = 0x00;

  UINT8_TO_BITSTREAM(p, flags); // insert the flags byte
  htm_temperature_fit = INT32_TO_FLOAT(return_temp * 1000, -3);


  // insert the temperature measurment
  UINT32_TO_BITSTREAM(p, htm_temperature_fit);

  sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_temperature_measurement, //handle from gatt_db.h
          0,                              //offset
          5,                              //length
          &htm_temperature_buffer[0]      // in IEEE-11073 format
      );

  if(sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_gatt_server_write_attribute_value failed with sc = %u\r\n", sc);
    }

  //LOG_INFO("ok to send = %d and indication in flight = %d \r\n", bleDataPtr1->ok_to_send_htm_indications, bleDataPtr1->indication_in_flight);

  if (bleDataPtr1->connection_open && bleDataPtr1->ok_to_send_htm_indications && !(bleDataPtr1->indication_in_flight))
      {

      LOG_INFO("HTM INDICATION SEND\r\n");

        sc = sl_bt_gatt_server_send_indication(
            bleDataPtr1->connectionHandle,
            gattdb_temperature_measurement,
            5,
            &htm_temperature_buffer[0]
            );
        displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s= %u", "Temperature", return_temp);

         if ( sc != SL_STATUS_OK) {LOG_ERROR("server send indication ERROR with sc: %u\r\n", sc);}
         else{ bleDataPtr1->indication_in_flight = 1;}

      }

  //Indication in flight
  else if (bleDataPtr1->ok_to_send_htm_indications && (bleDataPtr1->indication_in_flight) && bleDataPtr1->connection_open)
    {
      LOG_INFO("HTM INDICATION QUEUED\r\n");
      //enqueue here
      //LOG_INFO("HTM TEMPERATURE WRITE QUEUE\r\n");
      sc = write_queue(gattdb_temperature_measurement, sizeof(htm_temperature_buffer),&htm_temperature_buffer[0]);// gattdb_temperature_measurement OR gattdb_button_state
      if (sc)
        {
          LOG_ERROR("write_queue() HTM not SUCCESSFUL for press.\r\n");
        }
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s= %u", "Temperature", return_temp);
    }

  else
    {
      LOG_INFO("HTM INDICATION NOT QUEUED\r\n");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s ", " ");
    }

}





#else

/*
 * This function implements the discovery state
 * machine which supports a Bluetooth BLE Client
 * implementation, which receives and displays
 * indications from the temperature state
 * machine and displays the indications +
 * HTM temperature measurements on a second
 * Blue Gecko. This function is only enabled
 * when setting DEVICE_IS_BLE_SERVER to 0,
 * which sets the client implementation. This is
 * a one time state machine that does a 1-time
 * discovery of the HTM service using its service
 * UUID, 1-time discovery of the Temperature
 * Measurement characteristic using the HTM
 * thermometer characteristic UUID, 1-time
 * enabling of the HTM temperature indications, and
 * disabling indications for when the HTM characteristic
 * is not required. This state machine also receives the
 * temperature indications from the Server (first Blue
 * Gecko) and displays it on the Client's LCD (second
 * Blue Gecko).
 *
 * Parameters:
 * evt - external event
 * that's being processed by the bluetooth
 * event responder
 *
 * return:
 * none
 *
 */
void discovery_state_machine(sl_bt_msg_t *evt) //event to the Bluetooth, external evvent
{
         ble_data_struct_t *bleDataPtr1 = getBleDataPtr();
         sl_status_t status = 0;
         const uint8_t services_uuid[2] =  {0x09, 0x18};      //0x1809, little endian format, HTM
         size_t services_uuid_len = sizeof(services_uuid);
         const uint8_t characteristic_uuid[2] = {0x1C, 0x2A};  //0x2A1C;
         size_t characteristic_uuid_len = sizeof(characteristic_uuid);

         const uint8_t button_service_uuid[16] = {0x89, 0x62, 0x13, 0x2D, 0x2A, 0x65, 0xEC, 0x87, 0x3E, 0x43, 0xC8, 0x38, 0x01, 0x00, 0x00, 0x00};
         size_t button_service_uuid_len = sizeof(button_service_uuid);
         const uint8_t button_chararacteristic_uuid[16] = {0x89, 0x62, 0x13, 0x2D, 0x2A, 0x65, 0xEC, 0x87, 0x3E, 0x43, 0xC8, 0x38, 0x02, 0x00, 0x00, 0x00};
         size_t button_characteristic_uuid_len = sizeof(button_chararacteristic_uuid);

         //bd79978a-184e-4ab1-9b57-a2b57b988f28
         const uint8_t heartrate_service_uuid[16] = {0x28, 0x8f, 0x98, 0x7b, 0xB5, 0xA2, 0x57, 0x9B, 0xB1, 0x4A, 0x4E, 0x18, 0x8A, 0x97, 0x79, 0xBD};
         size_t heartrate_service_uuid_len = sizeof(heartrate_service_uuid);
         //aa427df2-bd03-4116-9b39-b33abe2aa9c4
         const uint8_t heartrate_characterisitic_uuid[16] = {0xC4, 0xA9, 0x2A, 0xBE, 0x3A, 0xB3, 0x39, 0x9B, 0x16, 0x41, 0x03, 0xBD, 0xF2, 0x7D, 0x42, 0xAA};
         size_t heartrate_characterisitic_uuid_len = sizeof(heartrate_characterisitic_uuid);

         State_t     currentState;
         static State_t     nextState = stateClosed;
         currentState = nextState;
         switch (currentState)
         {
           //Check for when a connection has been opened.
           case stateClosed:
             nextState = stateClosed;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id)
               {
                 //If a connection has been opened, search for the primary services
                 //offered by the server. In this case, search for the HTM
                 //service using its service UUID
                 status = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr1->connectionHandle,
                                                                       services_uuid_len,
                                                                       &services_uuid[0]);
                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid HTM INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateClosed passed.\r\n");

                 //Upon succesion, change state.
                 nextState = stateDiscPriServicesHTM;
               }
             break;

           // If the searching for primary services has been a success,
           // search for the characteristic offered by this service.
           case stateDiscPriServicesHTM:
             nextState = stateDiscPriServicesHTM;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 //Through the servicing handle, search for a specific characteristic
                 // in this case the HTM thermometer characteristic UUID.
                 status = sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr1->connectionHandle,
                                                                      bleDataPtr1 -> htm_servicingHandle,
                                                                      characteristic_uuid_len,
                                                                      &characteristic_uuid[0]);
                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid HTM INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateDiscPriServicesHTM passed.\r\n");

                 //Upon succesion, change state.
                 nextState = stateDiscCharServicesHTM;
               }

             //Credit to Prof. David Sluiter for debugging
             if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id) nextState = stateClosed;
             break;

           case stateDiscCharServicesHTM:
             nextState = stateDiscCharServicesHTM;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 //After the service and characteristic of the service have been succesful,
                 //send notification indication.
                 status = sl_bt_gatt_set_characteristic_notification(bleDataPtr1->connectionHandle,
                                                                     gattdb_temperature_measurement,
                                                                     sl_bt_gatt_indication);
                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_set_characteristic_notification() HTM INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateDiscCharServicesHTM passed.\r\n");

                 //Upon succession, change state
                 nextState = stateDiscPriServicesButton;
               }
             break;


           case stateDiscPriServicesButton:
             nextState = stateDiscPriServicesButton;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 //Button service discovery
                 status = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr1->connectionHandle,
                                                                       button_service_uuid_len,
                                                                       &button_service_uuid[0]);

                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_primary_services by uuid BUTTON INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateDiscPriServicesButton passed.\r\n");
                 nextState = stateDiscCharServiesButton;
               }
             break;


           case stateDiscCharServiesButton:
             nextState = stateDiscCharServiesButton;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 status = sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr1->connectionHandle,
                                                                      bleDataPtr1 -> htm_servicingHandle,
                                                                      button_characteristic_uuid_len,
                                                                      &button_chararacteristic_uuid[0]);

                 if (status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid BUTTON INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateDiscCharServiesButton passed.\r\n");
                 nextState = stateButtonIndicationSet;
               }
             break;

           case stateButtonIndicationSet:
             nextState = stateButtonIndicationSet;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 status = sl_bt_gatt_set_characteristic_notification(bleDataPtr1->connectionHandle,
                                                                     gattdb_button_state,
                                                                     sl_bt_gatt_indication);
                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_set_characteristic_notification() BUTTON INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                // LOG_INFO("stateButtonIndicationSet passed.\r\n");
                 nextState = stateHeartRateServiceDiscovery;
               }

             break;
           case stateHeartRateServiceDiscovery:
             nextState = stateHeartRateServiceDiscovery;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 //Heart rate service discovery
                 status = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr1->connectionHandle,
                                                                       heartrate_service_uuid_len,
                                                                       &heartrate_service_uuid[0]);

                 if(status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_primary_services by uuid HEART INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                 //LOG_INFO("stateHeartRateServiceDiscovery passed.\r\n");
                 nextState = stateHeartRateCharacteristicDiscovery;
               }

             break;
           case stateHeartRateCharacteristicDiscovery:
             nextState = stateHeartRateCharacteristicDiscovery;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 status = sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr1->connectionHandle,
                                                                      bleDataPtr1 -> htm_servicingHandle,
                                                                      heartrate_characterisitic_uuid_len,
                                                                      &heartrate_characterisitic_uuid[0]);

                 if (status != SL_STATUS_OK)
                   {
                     LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid HEART INVALID/ERROR. STATCODE -> %X\r\n", status);
                   }

                // LOG_INFO("stateHeartRateCharacteristicDiscovery passed.\r\n");
                 nextState = stateHeartRateCharacteristicNotification;
               }


             break;
           case stateHeartRateCharacteristicNotification:
             nextState = stateHeartRateCharacteristicNotification;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {

//                       status = sl_bt_gatt_set_characteristic_notification(bleDataPtr1->connectionHandle,
//                                                                     gattdb_heart_state,
//                                                                     sl_bt_gatt_indication);
//                       LOG_INFO("happened once set char notif heart rate.\r\n");
//                         if(status != SL_STATUS_OK)
//                           {
//                             LOG_ERROR("sl_bt_gatt_set_characteristic_notification() HEART INVALID/ERROR. STATCODE -> %X\r\n", status);
//                           }


                 //LOG_INFO("stateHeartRateCharacteristicNotification passed\r\n");
                 nextState = stateDiscCharServicesIndication;
               }

             break;

             //If indications were sent properly, it will enter the if statement,
           case stateDiscCharServicesIndication:
             nextState = stateDiscCharServicesIndication;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
               {
                 //Client is now able to handle indications
               //  gpioLed1SetOn();
               //  gpioLed0SetOn();
                 //LOG_INFO("stateDiscCharServicesIndication passed\r\n");
                 displayPrintf(DISPLAY_ROW_CLIENTADDR, "%s", "Handling Indications");
                 nextState = stateIdleClient;
               }
             break;
            //Last event which checks for the connection closed id
             //and goes back to the first state
           case stateIdleClient:
             nextState = stateIdleClient;
             if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id)
               {
                 //Go back to fist state where there is not connection.
                 //LOG_INFO("stateIdleClient passed\r\n");
                 nextState = stateClosed;
               }
             break;

           default:
             break;
         }

}

#endif


/*
 * Scheduler routine to return 1 event to main() code and
 * clear that event. theEvent
 * is the local variable which extracts the event using bitwise
 * ops and returns it.
 *  Clears the event under CRITICAL SECTIONS
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   uint32_t theEvent - the next event to execute
 *   which will be passed to the main code
 */

//Commented out for A5
//uint32_t getNextEvent(void)
//{
//    CORE_DECLARE_IRQ_STATE;
//    uint32_t theEvent;
//    //Select 1 event to return to main() code, apply priorities etc.
//    // <fill in your code here>
//    if(myEvents & evtReadTemperature)
//        theEvent = evtReadTemperature; // 1 event to return to the caller
//    else if (myEvents & evtTimerDone)
//        theEvent = evtTimerDone;
//    else if (myEvents & evtI2CTransferComplete)
//        theEvent = evtI2CTransferComplete;
//
//     //the Event could for example be Underflow or COMP1, but in A3, we only concerne with UF
//
//    // enter critical section
//     CORE_ENTER_CRITICAL(); //enter crit section, turn off interrupts NVIC
//    // clear the event in your data structure, this has to be a read-modify-write
//     myEvents &= ~(theEvent);
//     //myEvent ^= theEvent;
//    //exit critical section
//     CORE_EXIT_CRITICAL();
//
//    return (theEvent);
//} // getNextEvent()
