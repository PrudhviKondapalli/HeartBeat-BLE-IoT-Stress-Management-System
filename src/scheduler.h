/***************************************************************************//**
 * @file
 * @brief Scheduler header file
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

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdint.h>
#include "sl_bt_api.h"
#include "ble_device_type.h"

typedef enum {
  evtReadTemperature = 1, // 0b0001 , corresponding to UF
  evtTimerDone = 2,       // 0b0010, corresponding to COMP1
  evtI2CTransferComplete = 4, // 0b0100
  evtButtonPress = 8, // 0b1000, corresponding to button  0 press INT
  evtButtonRelease = 16, // 0b10000, corresponding to button  0 release INT
  evtButton1Press = 32, // 0b100000
  evtButton1Release = 64, //0b1000000
#if !DEVICE_IS_BLE_SERVER
  evtCapTouch = 128 // 0b10000000
#else
  evtReadHR = 128 // 0b1000000
#endif
} some_type_t;


//typedef enum {
//  evtReadHR = 1;
//}some_type_t2;

//Commented out for A5
//uint32_t getNextEvent(void);
void schedulerSetReadTempEvent(void);
void schedulerEvtTimerDone(void);
void schedulerEvtI2CTransferDone(void);
void schedulerEvtButtonPress(void);
void schedulerEvtButtonRelease(void);
void schedulerEvtButton1Press(void);
void schedulerEvtButton1Release(void);
void schedulerEvtCapTouched(void);
void schedulerSetHREvent(void);

#if DEVICE_IS_BLE_SERVER
void temperature_state_machine(sl_bt_msg_t *evt);
void hr_state_machine(sl_bt_msg_t *evt);
void discovery_state_machine_for_server(sl_bt_msg_t *evt); //event to the Bluetooth, external event
int32_t ret_three_sec_count(void);


#else
void discovery_state_machine(sl_bt_msg_t *evt);
#endif

#endif /* SRC_SCHEDULER_H_ */
