/***************************************************************************//**
 * @file
 * @brief I2C header file.
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
#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdint.h>

#define SI7021_DEVICE_ADDR        (0x40)
#define CAP1203_DEVICE_ADDR       (0x28)
#define MAX30105_PART_ID_REG      (0xFF)
#define MAX30105_MODE_REG         (0x09)
#define MAX30105_PARTICLE_SENSE_REG (0X0A)
#define MAX30105_LED1_AMP_REG     (0x0C)
#define MAX30105_LED2_AMP_REG     (0x0D)
#define MAX30105_LED3_AMP_REG     (0x0E)
#define MAX30105_PULSE_AMP_REG    (0x10)
#define MAX30105_MULTI_LED_REG    (0x11)
#define MAX30105_MULTI_LED_REG1   (0x12)
#define MAX30105_LED1_RED         (0x01)
#define MAX30105_LED1_RED_IR      (0x21)
#define MAX30105_LED1_GREEN       (0x03)
#define MAX30105_LED_CONFIG       (0x02)

#define MAX30105_DEVICE_FREQ      (400000)
#define MAX30105_DEVICE_FREQ_SLOW      (100000)


#define MAX30105_RED_AMPL         (0x0A)
#define MAX30105_GREEN_AMPL       (0x00)

//MAX30105 FIFO PTRS
#define MAX30105_FIFO_WR_PTR      (0x04)
#define MAX30105_FIFO_OVF_PTR     (0x05)
#define MAX30105_FIFO_RD_PTR      (0x06)
#define MAX30105_FIFO_DATA_REG    (0x07)
#define FIFO_RESET                (0x00)

#define MAX30105_RST_CONFIG       (0x40)
#define MAX30105_PART_REV_REG     (0xFE)
#define MAX30105_FIFO_REG         (0x08)
#define MAX30105_FIFO_CONFIG      (0x50)
#define MAX30105_DEVICE_ADDR      (0x57)
#define MAX30105_RED_MODE_ONLY    (0X02)
#define MAX30105_REDIR_MODE_ONLY  (0X03)
#define MAX30105_REDIRGREEN_MODE  (0X07)
#define MAX30105_SHUTDOWN_CONFIG  (0x80)

#define CAP1203_PROD_ID     (0xFD)
#define MANUFACT_ID         (0xFE)
#define MEAS_TEMP_CMD       (0xF3)
#define SENSITIVITY_REG     (0x1F)
#define CAP1203_STBY_REG    (0x40)
#define CAP1203_PWR_REG_CONFIG    (0x01)
#define CAP1203_PWR_REG     (0x60)
#define CAP1203_PWR_BUT_REG (0x61)
#define CAP1203_PWR_BUT_CONFIG  (0x03)
#define SENSITIVITY_SET     (0x6F) // 2X sensitivity
#define CAP1203_STBY_CONFIG (0x02)  //Sets only section 2 as active in standby mode
#define MAIN_CTRL_REG       (0x00) //Main Control Register
#define MAIN_CTRL_REG_DEF   (0x00) //Main Control Register Default
#define MAIN_CTRL_REG_DEEP_SLEEP  (0x10)
#define SENSOR_STAT_REG     (0x03)
#define SENSOR_GEN_STATUS   (0x02)
#define MAX30105_PARTICLE_SENSE_CONFIG  (0x00)






void Init_I2C(void);
void Init_I2C_MAX30105(void);

void send_command_si7021(void);

void read_temp_from_si7021(void);

int32_t read_data_convert(void);

uint8_t read_cap_1203(void);

uint8_t readblock_cap_1203(void);

void send_command_cap1203(uint8_t cmd);
void send_block_cap1203(uint8_t reg1,uint8_t reg2);

void clear_cap1203_interrupt(void);

void initCAP1203(void);

/////////////////////////////////////////
void init_max30105(void);

uint8_t soft_reset_max30105(void);

void send_block_max30105(uint8_t cmd1,uint8_t cmd2);

void send_command_max30105(uint8_t cmd);

uint8_t read_from_max30105(void);
uint8_t read_from_max30105_block(uint8_t *received_data, uint8_t received_data_size);

#endif /* SRC_I2C_H_ */
