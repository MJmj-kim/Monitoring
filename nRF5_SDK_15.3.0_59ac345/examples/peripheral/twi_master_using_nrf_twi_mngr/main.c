/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_atomic.h"
#include "nrf_twi_mngr.h" // to use twi_sensor
//#include "lm75b.h"
#include "mma7660.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_twi.h" //to use twi_mngr

#include "nrf_twi_sensor.h" //add++   to read and write value over register of iis2dlpc sensor
#include "nrf_drv_twi.h" //add++      to use twi_mngr
#include "iis2dlpc_reg.h" // add++    to refer to reg addresses and function pointers
#include "nrf_delay.h"




#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    4 

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, 0);                          /** add++ < TWI transaction manager instance. */
NRF_TWI_SENSOR_DEF(m_nrf_twi_sensor, &m_nrf_twi_mngr, NRF_TWI_SENSOR_SEND_BUF_SIZE);    /** add++  < TWI sensor instance. */

//NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
APP_TIMER_DEF(m_timer);


//add++
static int16_t          data_raw_acceleration[3];                                 /**add++  < Load raw data from iis2dlpc sensor into this variable. */ 
static stmdev_ctx_t        dev_ctx;                                               /**add++ < Handle iis2dlpc sensor's read/write instance with this variable. */
static uint8_t               whoamI, rst;                                           /**add++ < Validate iis2dlpc sensor address and status to use. */ 
static float acceleration_mg[3];                                                    /**add++ < Convert raw data to float(usable) data into this variable by using iis2dlpc sdk. */
#define II_ADDR (0x33U >>1)                                                         /**add++ < IIS2DLPC Sensor Adress. */

static uint8_t tx_buffer[1000];                                              /**add++ < Maximum numbers of pending transactions. */


static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);




#define BUFFER_SIZE  11
static uint8_t m_buffer[BUFFER_SIZE];




void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



//add++

static void twi_config(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t ii_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_MID, // HIGHEST
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &ii_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_twi_sensor_init(&m_nrf_twi_sensor);
    APP_ERROR_CHECK(err_code);
}


void iis2dlpc_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;

   //nrf_delay_ms(1000);

  /* Check device ID */
  iis2dlpc_device_id_get(&dev_ctx, &whoamI);



//IIS2DLPC_ID
  if (whoamI != IIS2DLPC_ID)  
    while (1) {
     printf(" Error ");
    }


  /*Restore default configuration */
  iis2dlpc_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis2dlpc_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis2dlpc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*Set full scale */
  iis2dlpc_full_scale_set(&dev_ctx, IIS2DLPC_8g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth
   */
  iis2dlpc_filter_path_set(&dev_ctx, IIS2DLPC_LPF_ON_OUT);
  iis2dlpc_filter_bandwidth_set(&dev_ctx, IIS2DLPC_ODR_DIV_4);
  /*Configure power mode */
  //iis2dlpc_power_mode_set(&dev_ctx, IIS2DLPC_HIGH_PERFORMANCE);
  iis2dlpc_power_mode_set(&dev_ctx,
                          IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit);
  /*Set Output Data Rate */
  iis2dlpc_data_rate_set(&dev_ctx, IIS2DLPC_XL_ODR_25Hz);

  /*Read samples in polling mode (no int) */
  while (1) {
    uint8_t reg;
    /* Read output only if new value is available */
    iis2dlpc_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      iis2dlpc_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = iis2dlpc_from_fs8_lp1_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = iis2dlpc_from_fs8_lp1_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = iis2dlpc_from_fs8_lp1_to_mg(
                             data_raw_acceleration[2]);
      
      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      //tx_com(tx_buffer, strlen((char const *)tx_buffer));
     
      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

    }
  }
}


/**@brief Function for connection with the iis2dlpc_read_reg function and nrf52840 SDK.
 * @param[out] return Should be 0.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
 nrf_twi_sensor_reg_read(&m_nrf_twi_sensor, II_ADDR, reg, NULL, bufp, len);
 nrf_delay_us(1000);
 //NRF_LOG_FLUSH();
 return 0;
}




/**@brief Function for connection with the iis2dlpc_write_reg function and nrf52840 SDK.
 * @param[out] return Should be 0.
 */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
 nrf_twi_sensor_reg_write(&m_nrf_twi_sensor, II_ADDR, reg, bufp, len);  
 nrf_delay_us(1000);
 //NRF_LOG_FLUSH();
 return 0;
}





int main(void)
{
    ret_code_t err_code;

    log_init();   // => nrf_log check
    //bsp_board_init(BSP_INIT_LEDS); // =>if ~ led on

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).    

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_RAW_INFO("\r\nTWI master example started. \r\n");
    
    
    twi_config();
    iis2dlpc_read_data_polling();  
    
    

    // Initialize sensors.
    /*
    APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lm75b_init_transfers,
        LM75B_INIT_TRANSFER_COUNT, NULL)); */
    //APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, mma7660_init_transfers,
    //   MMA7660_INIT_TRANSFER_COUNT, NULL));

    while (true)
    {
        nrf_pwr_mgmt_run();
        //NRF_LOG_FLUSH();
    }
}


/** @} */
