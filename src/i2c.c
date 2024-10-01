/***************************************************************************//**
 * @file
 * @brief I2C Implementation file
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
 * @credit    David Sluiter - for debugging code
 *
 *
 ******************************************************************************/
//hi

#include "i2c.h"
#include "sl_i2cspm.h"
#include "em_i2c.h"
#include "timers.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"





//Setup Measure Send Command
I2C_TransferSeq_TypeDef         transferSequence;
uint8_t                         cmd_data1;
uint8_t                         cmd_data2;
uint8_t                         cmd_data;
uint8_t                         read_data[2];
uint8_t                         cap_data[1];
uint8_t                         capblock_data[2];
uint8_t                         max30105_data[1];
uint8_t                         max30105_block_data[3];




uint8_t cap_int_clear[2] = {MAIN_CTRL_REG, MAIN_CTRL_REG_DEF};
//uint8_t max30105_block_data[2];



const float multiplier = 175.72;
const float divisor = 65536.0;
const float subtractor = 46.85;
uint8_t spo2_prod_rev_id = 0;


/*
 * Initialization of I2C0 function.
 * .port - port selection of I2C0
 * .sclPort- port C
 * .sclPin- 10 (datasheet)
 * .sdaPort- port C
 * .sdaPin- 11 (datasheet)
 * .portLocationScl- sch location 14
 * .portLocationSda - sch location 16
 * .i2cRefFreq- 0
 * .i2cMaxFreq- Standard 100Khz (90.566Khz)
 * .i2cClhr- Standard
 *
 * Initializes the I2C0 using the HAL given function
 * I2CSPM_Init(I2CSPM_Init_TypeDef conf)
 *
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void Init_I2C(void)
{
  //initialize the I2C hardware

   I2CSPM_Init_TypeDef I2C_Config;
   I2C_Config.port = I2C0;
   I2C_Config.sclPort = gpioPortC;
   I2C_Config.sclPin = 10;
   I2C_Config.sdaPort = gpioPortC;
   I2C_Config.sdaPin = 11;
   I2C_Config.portLocationScl = 14;
   I2C_Config.portLocationSda = 16;
   I2C_Config.i2cRefFreq = 0;
   I2C_Config.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
   I2C_Config.i2cClhr = i2cClockHLRStandard;

   I2CSPM_Init(&I2C_Config);

  // uint32_t i2c_bus_frequency = I2C_BusFreqGet(I2C0);
}

void Init_I2C_MAX30105(void)
{
  //initialize the I2C hardware

   I2CSPM_Init_TypeDef I2C_Config;
   I2C_Config.port = I2C0;
   I2C_Config.sclPort = gpioPortC;
   I2C_Config.sclPin = 10;
   I2C_Config.sdaPort = gpioPortC;
   I2C_Config.sdaPin = 11;
   I2C_Config.portLocationScl = 14;
   I2C_Config.portLocationSda = 16;
   I2C_Config.i2cRefFreq = 0;
   I2C_Config.i2cMaxFreq = MAX30105_DEVICE_FREQ_SLOW;
   I2C_Config.i2cClhr = i2cClockHLRStandard;

   I2CSPM_Init(&I2C_Config);

  // uint32_t i2c_bus_frequency = I2C_BusFreqGet(I2C0);
}

void initCAP1203(void)
{
    read_cap_1203();
    uint8_t result = 0;
    uint8_t result1 = 0;
    uint8_t result2 = 0;
    send_command_cap1203(CAP1203_PROD_ID);
    result = read_cap_1203();
    LOG_INFO("Product ID is %X\r\n", result);

    send_command_cap1203(MANUFACT_ID);
    result1 = read_cap_1203();
    LOG_INFO("Manufacturer ID is %X\r\n", result1);

    send_block_cap1203(SENSITIVITY_REG, SENSITIVITY_SET);
    result2 = readblock_cap_1203();
    LOG_INFO("Sensitivity Set is %X\r\n", result2);

    //Set standby channel register
    //set CS2 as enabled in standby
    //send_block_cap1203(CAP1203_STBY_REG, CAP1203_STBY_CONFIG);

    //Configure CS2 as power button
    send_block_cap1203(CAP1203_PWR_REG, CAP1203_PWR_REG_CONFIG);
    //set power button configuration register to 2.24sec
    send_block_cap1203(CAP1203_PWR_BUT_REG, CAP1203_PWR_BUT_CONFIG);

    //Since all are written, send to deep sleep
    send_block_cap1203(MAIN_CTRL_REG,MAIN_CTRL_REG_DEEP_SLEEP);

}


/*
 *  Sets up a transfer sequence that reads two bytes
 *  from the appropriate SI 7021 to get the needed
 *  temperature measurement. FLAG_READ is asserted
 *  such that this is a read operation on the I2C0
 *  protocol bus. LOG_ERROR provided if transmission is
 *  not complete. Temperature measurement is received
 *  in MSB and LSB. Simple conversion is performed +
 *  additional math from the datasheet resulting
 *  in a floating celsius temperature which is logged.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void read_temp_from_si7021(void)
{

    I2C_TransferReturn_TypeDef      transferStatus;

    Init_I2C(); //Initialization of I2C0 peripheral

    transferSequence.addr = SI7021_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_READ;
    transferSequence.buf[0].data = &read_data[0]; // read_data[2];
    transferSequence.buf[0].len = sizeof(read_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0 )
      {
        LOG_ERROR("I2C_Transfer() Read error = %d", transferStatus);
      }
}

uint8_t read_cap_1203(void)
{
  I2C_TransferReturn_TypeDef      transferStatus;

  Init_I2C(); //Initialization of I2C0 peripheral

  transferSequence.addr = CAP1203_DEVICE_ADDR << 1; // shift device address to the left
  transferSequence.flags = I2C_FLAG_READ;
  transferSequence.buf[0].data = &cap_data[0];
  transferSequence.buf[0].len = sizeof(cap_data);

  NVIC_EnableIRQ(I2C0_IRQn);

  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
    {
      LOG_ERROR("I2C_Transfer() Read error = %d", transferStatus);
    }

  for(int i = 0; i < 10000; i++);
//  LOG_INFO("cap_data[0] is %X\r\n", cap_data[0]);
//  LOG_INFO("cap_data[0] is %u\r\n", cap_data[0]);
//  LOG_INFO("cap_data[0] is %d\r\n", cap_data[0]);
//  LOG_INFO("cap_data[0] is %x\r\n", cap_data[0]);

  return cap_data[0];

}

uint8_t readblock_cap_1203(void)
{
  I2C_TransferReturn_TypeDef      transferStatus;

  Init_I2C(); //Initialization of I2C0 peripheral

  transferSequence.addr = CAP1203_DEVICE_ADDR << 1; // shift device address to the left
  transferSequence.flags = I2C_FLAG_READ;
  transferSequence.buf[0].data = &capblock_data[0];
  transferSequence.buf[0].len = sizeof(capblock_data);

  NVIC_EnableIRQ(I2C0_IRQn);

  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
    {
      LOG_ERROR("I2C_Transfer() Read error = %d", transferStatus);
    }

  LOG_INFO("Delay\r\n");


  return capblock_data[0];

}

/*
 *  Sets up a transfer sequence for the I2C0
 *  protocol. The appropriate command for the SI7021
 *  temperature read is sent(0xF3) usign the I2CSPM_Transfer(...)
 *  HAL provided function. FLAG WRITE is indicated
 *  to assure that this is a write command.
 *  LOG_ERROR provided if transmission is
 *  not complete.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void send_command_si7021(void)
{
    I2C_TransferReturn_TypeDef      transferStatus;

    Init_I2C(); //Initialization of I2C0 peripheral
    cmd_data = MEAS_TEMP_CMD;
    transferSequence.addr = SI7021_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &cmd_data;   //pointer to data to write
    transferSequence.buf[0].len = sizeof(cmd_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0)
      {
        LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
      }

}

void send_command_cap1203(uint8_t cmd)
{
    I2C_TransferReturn_TypeDef      transferStatus;

    Init_I2C(); //Initialization of I2C0 peripheral
    cmd_data = cmd;
    transferSequence.addr = CAP1203_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &cmd_data;   //pointer to data to write
    transferSequence.buf[0].len = sizeof(cmd_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0)
      {
        LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
      }


}

//for sensitivity set
void send_block_cap1203(uint8_t reg1,uint8_t reg2)
{
    I2C_TransferReturn_TypeDef      transferStatus;

    Init_I2C(); //Initialization of I2C0 peripheral
    //cmd_data1 = cmd1;
    //uint8_t config_data[2] = {SENSITIVITY_REG, SENSITIVITY_SET};
    uint8_t config_data[2] = {reg1, reg2};
    transferSequence.addr = CAP1203_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &config_data[0];   //pointer to data to write
    transferSequence.buf[0].len = sizeof(config_data);
    //transferSequence.buf[0].len = 2;

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0)
      {
        LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
      }
    for(int i = 0; i < 10000; i++);

}

/*
 *  Converts the read_data global variable
 *  passed within the read_temp_from_si7021()
 *  function and converts it to celsius.
 *  read_data[0] - msb
 *  read_data[1] - lsb
 *  Calculation conversion to celsius is done
 *  using bitwise operations and used the calculation
 *  equation from datasheet from the Si7021 AppNote.
 *  The temperature value is then logged.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
int32_t read_data_convert(void)
{

  int32_t temp_meas = 0;
  int32_t temp_meas_celsius = 0;

  temp_meas = ((read_data[0] << 8) | (read_data[1] >> 8));

  temp_meas_celsius = (int32_t) ((multiplier * temp_meas)/divisor) - subtractor;
  //LOG_INFO("Read temperature: %d celsius\r\n", temp_meas_celsius);

  return temp_meas_celsius;

}



void clear_cap1203_interrupt(void)
{
    I2C_TransferReturn_TypeDef      transferStatus;

      Init_I2C(); //Initialization of I2C0 peripheral
      //cmd_data1 = cmd1;
      transferSequence.addr = CAP1203_DEVICE_ADDR << 1; // shift device address to the left
      transferSequence.flags = I2C_FLAG_WRITE;
      transferSequence.buf[0].data = &cap_int_clear[0];   //pointer to data to write
      transferSequence.buf[0].len = sizeof(cap_int_clear);
      //transferSequence.buf[0].len = 2;

      NVIC_EnableIRQ(I2C0_IRQn);

      transferStatus = I2C_TransferInit(I2C0, &transferSequence);
      if (transferStatus < 0)
        {
          LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
        }
}

/*
 * Heart Rate and Spo2 sensor API's
 */

/*
 * MAX30105 Initialization
 */
void init_max30105(void)
{
  //Read Product ID first
  send_command_max30105(MAX30105_PART_ID_REG);
  uint8_t spo2_prod_id = read_from_max30105();
  LOG_INFO("Product id = 0x%X\n\r",spo2_prod_id );
  if(spo2_prod_id == 0x15)
    {
      //Read Product revision
      send_command_max30105(MAX30105_PART_REV_REG);
      spo2_prod_rev_id = read_from_max30105();

      //Soft reset all registers
      soft_reset_max30105();

      //Setting FIFO Configuration register
      //Setting Sampling Avg = 32
      //Enabling Roll over in FIFO
      //Almost full register set as 32
      send_block_max30105(MAX30105_FIFO_REG,MAX30105_FIFO_CONFIG);

      //Configuration
      //Sets to RED LED IR only -for Heart Beat
      send_block_max30105(MAX30105_MODE_REG,MAX30105_REDIRGREEN_MODE);

      //Particle Sensing Configuration
      //Set ADC Range as 2048 - 00
      //Set sample rate control as 50 - 00
      //Set LED PW as 69us - 00
      send_block_max30105(MAX30105_PARTICLE_SENSE_REG,MAX30105_PARTICLE_SENSE_CONFIG);

      //LED Pulse Amplitude Configuration
      //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
      //set LED  pulse amplitude all to 0.4mA
      send_block_max30105(MAX30105_LED1_AMP_REG,MAX30105_LED_CONFIG);
      send_block_max30105(MAX30105_LED2_AMP_REG,MAX30105_LED_CONFIG);
      send_block_max30105(MAX30105_LED3_AMP_REG,MAX30105_LED_CONFIG);
      //Setting Pulse Amplitude as 0.4mA
      send_block_max30105(MAX30105_PULSE_AMP_REG,MAX30105_LED_CONFIG);



      //Set Multi-LED as RED LED, MIGHT NOT NEED Multi-LED register at all
      send_block_max30105(MAX30105_MULTI_LED_REG,MAX30105_LED1_RED_IR);
      send_block_max30105(MAX30105_MULTI_LED_REG1,MAX30105_LED1_GREEN);


      //Clear the FIFO
      send_block_max30105(MAX30105_FIFO_WR_PTR, FIFO_RESET);
      send_block_max30105(MAX30105_FIFO_OVF_PTR, FIFO_RESET);
      send_block_max30105(MAX30105_FIFO_RD_PTR, FIFO_RESET);

      //Line 55 Example5_HeartRate
     // send_block_max30105(MAX30105_LED1_AMP_REG, MAX30105_RED_AMPL);
     // send_block_max30105(MAX30105_LED3_AMP_REG, MAX30105_GREEN_AMPL);

      //since all congifs are set, send to shutdown mode/deep sleep mode
      send_block_max30105(MAX30105_MODE_REG,MAX30105_SHUTDOWN_CONFIG);

    }
  else
    {
      LOG_ERROR("I2C link down for MAX30105, Check wiring");
    }
}

/*
 * MAX30105 - Soft Reset
 */
uint8_t soft_reset_max30105(void)
{
  send_block_max30105(MAX30105_MODE_REG,MAX30105_RST_CONFIG);
  for(int i = 0; i <99999;i++);
  uint8_t result = read_from_max30105();
  return result;
}


/*
 * MAX30105 - I2C BYTE READ
 */
uint8_t read_from_max30105(void)
{

    I2C_TransferReturn_TypeDef      transferStatus;

    //Init_I2C(); //Initialization of I2C0 peripheral
    Init_I2C_MAX30105();

    transferSequence.addr = MAX30105_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_READ;
    transferSequence.buf[0].data = &max30105_data[0];
    transferSequence.buf[0].len = sizeof(max30105_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0 )
      {
        LOG_ERROR("I2C_Transfer() Read error = %d", transferStatus);
      }

    //for(int i = 0; i < 5000; i++);
  //  LOG_INFO("cap_data[0] is %X\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %u\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %d\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %x\r\n", cap_data[0]);
    //This log is giving enough time for me the I2C transfer to complete
        //LOG_INFO("Delay\r\n");
      LOG_INFO("max30105_data is 0x%X\n\r",max30105_data[0]);

    return max30105_data[0];
}



uint8_t read_from_max30105_block(uint8_t *received_data, uint8_t received_data_size)
{

    I2C_TransferReturn_TypeDef      transferStatus;

    //Init_I2C(); //Initialization of I2C0 peripheral
    Init_I2C_MAX30105();

    transferSequence.addr = MAX30105_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_READ;
    transferSequence.buf[0].data = (uint8_t *) received_data;
    transferSequence.buf[0].len = received_data_size;

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0 )
      {
        LOG_ERROR("I2C_Transfer() Read error = %d", transferStatus);
      }

    //for(int i = 0; i < 5000; i++);
  //  LOG_INFO("cap_data[0] is %X\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %u\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %d\r\n", cap_data[0]);
  //  LOG_INFO("cap_data[0] is %x\r\n", cap_data[0]);
    //This log is giving enough time for me the I2C transfer to complete
        //LOG_INFO("Delay\r\n");
     // LOG_INFO("max30105_data is 0x%X\n\r",max30105_data[0]);

    return 0;
}

/*
 * MAX30105 - I2C BLOCK WRITE
 */
void send_block_max30105(uint8_t cmd1,uint8_t cmd2)
{
    I2C_TransferReturn_TypeDef      transferStatus;

    //Init_I2C(); //Initialization of I2C0 peripheral
    Init_I2C_MAX30105();

    //cmd_data1 = cmd1;
    max30105_block_data[0] = cmd1;
    max30105_block_data[1] = cmd2;
    transferSequence.addr = MAX30105_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &max30105_block_data[0];   //pointer to data to write
    transferSequence.buf[0].len = sizeof(max30105_block_data);
    //transferSequence.buf[0].len = 2;

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0)
      {
        LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
      }
    //for(int i = 0; i < 10000; i++);


}

/*
 * MAX30105 - I2C BYTE WRITE
 */
void send_command_max30105(uint8_t cmd)
{

    I2C_TransferReturn_TypeDef      transferStatus;

    //Init_I2C(); //Initialization of I2C0 peripheral

    Init_I2C_MAX30105();

    cmd_data = cmd;
    transferSequence.addr = MAX30105_DEVICE_ADDR << 1; // shift device address to the left
    transferSequence.flags = I2C_FLAG_WRITE;
    transferSequence.buf[0].data = &cmd_data;   //pointer to data to write
    transferSequence.buf[0].len = sizeof(cmd_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    transferStatus = I2C_TransferInit(I2C0, &transferSequence);
    if (transferStatus < 0)
      {
        LOG_ERROR("I2C_TransferInit() Write error = %d\r\n", transferStatus);
      }


}
