/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*common*/
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h" 
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "string.h"

/*saadc Header*/
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

/*iisdlpc Header*/
#include "iis2dlpc_reg.h"
#include "nrf_twi.h" //to use twi_mngr
#include "nrf_twi_mngr.h"// to use twi_sensor
#include "nrf_twi_sensor.h" // to read and write value over register of iis2dlpc sensor
#include "nrf_drv_twi.h" //to use twi_mngr
#include "compiler_abstraction.h"
#include "app_timer.h"

/*fire sensor Header*/







/* LED pin */
#define LED NRF_GPIO_PIN_MAP(1, 15)  // LED pin


/* saadc define */
#define SAMPLES_IN_BUFFER 2
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;


/* iisdlpc define */
#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    4 

static int16_t               data_raw_acceleration[3];               /**< Load raw data from iis2dlpc sensor into this variable. */ 
static stmdev_ctx_t          dev_ctx;                                /**< Handle iis2dlpc sensor's read/write instance with this variable. */
static uint8_t               whoamI, rst;                            /**< Validate iis2dlpc sensor address and status to use. */ 
static float acceleration_mg[3];                                     /**< Convert raw data to float(usable) data into this variable by using iis2dlpc sdk. */
static uint8_t tx_buffer[1000];                                       /**< Maximum numbers of pending transactions. */
#define II_ADDR (0x33U >>1)                                          /**< IIS2DLPC Sensor Adress. */
                            


//Macro that simplifies defining a TWI transaction manager instance
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, 0);                          /**< TWI transaction manager instance. */
//Macro creating common twi sensor instance
NRF_TWI_SENSOR_DEF(m_nrf_twi_sensor, &m_nrf_twi_mngr, NRF_TWI_SENSOR_SEND_BUF_SIZE);    /**< TWI sensor instance. */

//NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
//APP_TIMER_DEF(m_timer);



/* saadc funtion */
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
  
}


void saadc_sampling_event_init(void) 
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 100);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);

    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}



void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
  float H,T;
  
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        int VT = p_event->data.done.p_buffer[0];
        int VH =p_event->data.done.p_buffer[1];

        float outdata = (float)VT * 3.6/1023;
        T = -66.875 + 218.75 * outdata/3.3;

        outdata = (float)VH * 3.6/1023;
        H = -12.5 + 125*outdata/3.3;    
       

       /* for debug Humidity*/
        printf("HUMI : %lf %% \n", H);
        printf("ADC event number: %d", p_event->data.done.p_buffer[0]);      
        printf("\n");

        m_adc_evt_counter++; 

      /* when humidity is over 50%  */
         if(H >= 50.0) 
          {
           printf(" Exceeded 50% humidity \n");
           nrf_gpio_pin_set(LED);
          }
          else
          { nrf_gpio_pin_clear(LED); }
    }
   
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config_TEMP =                         //variable modified_temperature
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);  //temperature pin. P0.03(A0)

    nrf_saadc_channel_config_t channel_config_HUMI =                     //variable modified_humidity
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);  //humidity pin. P0.04(A1)

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config_TEMP);   //modified
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_config_HUMI);    //modified
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}





/* iisdlpc function*/

/**@brief Function for TWI (with transaction manager and twi_sensor) initialization.
 */
static void twi_init(void)
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


/**@brief Function for connection with the iis2dlpc_read_reg function and nrf52840 SDK.
 * @param[out] return Should be 0.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
 nrf_twi_sensor_reg_read(&m_nrf_twi_sensor, II_ADDR, reg, NULL, bufp, len);
 nrf_delay_us(1000);
 NRF_LOG_FLUSH();
 return 0;
}

/**@brief Function for connection with the iis2dlpc_write_reg function and nrf52840 SDK.
 * @param[out] return Should be 0.
 */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
 nrf_twi_sensor_reg_write(&m_nrf_twi_sensor, II_ADDR, reg, bufp, len);  
 nrf_delay_us(1000);
 NRF_LOG_FLUSH();
 return 0;
}



void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



 void iis2dlpc_read_data_polling(void)
{ 
  
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;
  /*Initialize platform specific hardware */
  //platform_init();
  /* Check device ID */
  iis2dlpc_device_id_get(&dev_ctx, &whoamI);
  

  if (whoamI != IIS2DLPC_ID)
    while (1) {
     /* manage here device not found */// iis2dlpc_device_id_get(&dev_ctx, &whoamI);
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
      //printf("check\n");
      printf("%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }
  }
}






int main(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    /*ii2dlpc */ 
    log_init();
    printf("\r\nTWI master example started. \r\n");

    printf("\r\nTWI master example started. \r\n");
    NRF_LOG_FLUSH();

    twi_init();
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
   

    /*saadc */ 
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret_code_t ret_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret_code);
    
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();

    NRF_LOG_INFO("SAADC HAL simple example started.");

    iis2dlpc_read_data_polling(); //dlpc enable


    nrf_gpio_cfg_output(LED); // led output set



    while (1)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
        
    }
}


/** @} */
