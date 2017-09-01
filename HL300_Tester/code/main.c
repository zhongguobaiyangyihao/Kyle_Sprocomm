/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/** 
 * @brief BLE Location application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "app_trace.h"
#include "ble_location.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "W200_Scanner.h"
#include "ble_advdata.h"
#include "beaconSetting.h"
#include "DeviceAcc.h"
#include "ble_conn_params.h"
#include "device_manager.h"
#include "math.h"
#include "nrf_drv_gpiote.h"
#include "spi_master.h"
#include "app_pwm.h"
#include "nrf_delay.h"
#include "hl300_tester.h"
#include "nrf_adc.h"


APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

#define APP_TIMER_PRESCALER        0                                  /**< Value of the RTC1 PRESCALER register. */

#define APPL_LOG                   app_trace_log                      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SCAN_INTERVAL              0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x00A0                             /**< Determines scan window in units of 0.625 millisecond. */



#define DEVICE_NAME                "HL300_tester"                             /**< Name of device. Will be included in the advertising data. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                             /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define MIN_CONNECTION_INTERVAL    MSEC_TO_UNITS(500, UNIT_1_25_MS)   /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL    MSEC_TO_UNITS(1000, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY              0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */                     

#define APP_CFG_ADV_TIMEOUT             40                                 /**< Time for which the device must be advertising in Connectable undirected mode (in seconds). 0 disables timeout. */
#define APP_CFG_ADV_INTERVAL            MSEC_TO_UNITS(600, UNIT_0_625_MS) /**< The advertising interval for Connectable undirected advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20 * 1000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5 * 1000, APP_TIMER_PRESCALER)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3      
#define TX_POWER_LEVEL                  (0)                                               /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */


 
/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                             /**< Pointer to data. */
    uint16_t      data_len;                                           /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,                                                     /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                              /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                   /**< Fast advertising running. */
} ble_scan_mode_t;

static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
static ble_location_t               m_ble_location;                      /**< Structure used to identify the heart rate client module. */
static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */
const app_uart_comm_params_t comm_params_communicate_with_pc =
       {
           UART_PC_RX, //RX_PIN_NUMBER,
           UART_PC_TX, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud115200
       };
const app_uart_comm_params_t comm_params_communicate_with_hl300 =
       {
           UART_HL300_RX, //RX_PIN_NUMBER,
           UART_HL300_TX, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud115200
       };
/************************************************************************///scanner mode variable define
/************************************************************************///slave mode variable define   
#define                             POWER_RESETREAS                            0x0007000F
app_timer_id_t                      check_Time_timer_id;
Flag_t                              Flag;
uint8_t                             UART_RX_BUF[UART_RX_BUF_SIZE];
uint16_t                            UART_RX_BUF_CNT;
uint16_t                            UART_receive_over_judge_cnt = 0;
Test_instructions_t                 Test_instruction = Test_instruction_check_hl300_pcba;
uint8_t                             return_result_buff[200];
uint16_t                            return_result_len;
uint8_t                             config_sn[13];
uint16_t                            waiting_instruction_timeout_cnt = 0;
volatile int32_t                    adc_sample[ADC_SAMPLE_NUMBER];
uint8_t                             key_check_result = 0;
uint8_t                             open_sw_check_result = 0;
uint8_t                             close_sw_check_result = 0;
float                               motor_roll_a_voltage = 0;
float                               motor_roll_b_voltage = 0;
uint8_t                             enable_gps_check_result = 0;
uint8_t                             return_value_printf[100];
uint8_t                             return_value_cnt = 0;
ble_result_t                        ble_result = {false,-80};
float                               sample_voltage = 0;
float                               sample_max_voltage = 0;
float                               sample_min_voltage = 0;
uint8_t                             monitor_ble_addr[6] = {0};
extern uint16_t                     communicate_time_cnt;
extern const uint8_t                Timeout[];
extern const uint8_t                unlock_state_test[];
extern const uint8_t                lock_state_test[];
extern const uint8_t                enable_gps_calculate[];
extern const uint8_t                inquiry_gps_is_enable[];
extern const uint8_t                open_sw_test[];
extern const uint8_t                close_sw_test[];
/************************************************************************/
void scan_start(void);
void uart_init(app_uart_comm_params_t comm_params);
void communicate_with_pc_handler(void);
void check_open_close_sw_pin_init(void);
extern void communicate_with_hl300_handler(void);
/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. 
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event) 
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&UART_RX_BUF[UART_RX_BUF_CNT]));
            UART_RX_BUF_CNT++;
            if(UART_RX_BUF_CNT>=UART_RX_BUF_SIZE)
            {
              UART_RX_BUF_CNT = 0;
            }
            Flag.is_uart_receive_start = true;
            UART_receive_over_judge_cnt = 0;
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/************************************************************************
发送数据接口
************************************************************************/
void UART_Data_Send(uint8_t * p_data, uint16_t length)
{
    UART_RX_BUF_CNT = 0;
    memset(UART_RX_BUF,0,UART_RX_BUF_SIZE);
    for (uint32_t i = 0; i < length; i++)
    {
      app_uart_put(p_data[i]);
//        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
//    while(app_uart_put('\n') != NRF_SUCCESS);
}
/************************************************************************
发送数据接口
************************************************************************/
void nrf51822_Data_Send(uint8_t * p_data, uint16_t length)
{
  UART_Data_Send(p_data, length);
}
/************************************************************************
发送数据接口
************************************************************************/
void PC_Data_Send(uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
//    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}
/*******************************************************************************
把手环数据存入上传数据缓存
*******************************************************************************/
static bool is_and_check_success(uint8_t old_check, uint8_t *data, uint8_t length)
{
  uint8_t new_check;
  new_check = data[0];
  for (uint8_t i=1; i<length; i++)
  {
    new_check += data[i];
  }
  return (new_check == old_check);
}
/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                err_code;
    const ble_gap_evt_t     * p_gap_evt = &p_ble_evt->evt.gap_evt;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
          
          break;
        }
        case BLE_GAP_EVT_DISCONNECTED:
        {
          
          break;
        }
        case BLE_GAP_EVT_ADV_REPORT:                                  
        {
            #if 1
            if((p_gap_evt->params.adv_report.peer_addr.addr[5] == monitor_ble_addr[5])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[4] == monitor_ble_addr[4])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[3] == monitor_ble_addr[3])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[2] == monitor_ble_addr[2])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[1] == monitor_ble_addr[1])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[0] == monitor_ble_addr[0])
              )
            #else
            if(true)
            #endif
            {
              ble_result.is_received_match_signal = true;
              if(p_gap_evt->params.adv_report.rssi>ble_result.rssi)
                ble_result.rssi = p_gap_evt->params.adv_report.rssi;  
            }
            break;
        }
        
        case BLE_GAP_EVT_TIMEOUT:
           
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/************************************************************************
使能除能notification处理
************************************************************************/
void notification_enable_evt_handler(ble_beaconSetting_evt_t *p_evt)
{
  switch (p_evt->evt_type)
  {
    case BEACON_EVT_NOTIFICATION_ENABLED:
    {
      break;
    }

    case BEACON_EVT_NOTIFICATION_DISABLED:        
      break;

    default:
      // No implementation needed.
      break;
  }
}
/**@brief Function for handling beaconSetting events.
 *
 * @details This function will be called for all beaconSetting events which are passed to the
 *          application.
 *
 * @param[in]   p_beaconSetting  beaconSetting structure.
 * @param[in]   p_evt  Event received from the beaconSetting service.
 */
void on_beaconSetting_evt(ble_beaconSetting_t * p_beaconSetting, ble_beaconSetting_evt_t * p_evt)
{
  
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by ACCELEROMETER.
 */
void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_ACCELERATION:
         
          break;

        default:
            break;
    }
}


/**@brief Location Handler.
 */
static void location_evt_handler(ble_location_t * p_location, ble_location_evt_t * p_location_evt)
{
    uint32_t err_code;

    switch (p_location_evt->evt_type)
    {
        case BLE_LOCATION_EVT_DISCOVERY_COMPLETE:
            // Location service discovered. Enable notification of Location Measurement.
            err_code = ble_location_lm_notif_enable(p_location);
            APP_ERROR_CHECK(err_code);

            APPL_LOG("Location service discovered \r\n");
            break;

        case BLE_LOCATION_EVT_LM_NOTIFICATION:
        {
            APPL_LOG("[APPL]: Location Measurement received %d \r\n", p_location_evt->params.lm.hr_value);

            APPL_LOG("Location = %d\r\n", p_location_evt->params.lm.hr_value);
            break;
        }

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
  ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
  ble_location_on_ble_evt(&m_ble_location, p_ble_evt);
  on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
  pstorage_sys_event_handler(sys_evt);
  on_sys_evt(sys_evt);
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_scanner_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_250_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = false;
    ble_enable_params.gap_enable_params.role              = BLE_GAP_ROLE_CENTRAL;

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief location initialization.
 */
static void location_init(void)
{
    ble_location_init_t location_init_obj;

    location_init_obj.evt_handler = location_evt_handler;

    uint32_t err_code = ble_location_init(&m_ble_location, &location_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning.
 */
void scan_start(void)
{
    uint32_t              err_code;

    m_scan_param.active       = 1;            // Active scanning set主动扫描，该模式下接收扫描回应包.
//    m_scan_param.active       = 0;            // Active scanning not set.
    m_scan_param.selective    = 0;            // Selective scanning not set.
    m_scan_param.interval     = SCAN_INTERVAL;// Scan interval.
    m_scan_param.window       = SCAN_WINDOW;  // Scan window.
    m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
    m_scan_param.timeout      = 0x0000;       // No timeout.

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the UART.
 */
void uart_init(app_uart_comm_params_t comm_params)
{
    uint32_t err_code;
    APP_UART_FIFO_INIT(&comm_params,
                       256,
                       256,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_HIGH,
                       err_code);

    APP_ERROR_CHECK(err_code);

    app_trace_init();
}
/**
 * @brief ADC interrupt handler.
 */
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();
    Flag.is_single_sample_finish = true;
//    adc_sample = nrf_adc_result_get();

//    // trigger next ADC conversion
//    nrf_adc_start();
}
/**
 * @brief ADC initialization.
 */
void adc_config(nrf_adc_config_input_t config_input)
{
    const nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    nrf_adc_input_select(config_input);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);
}
/**
 * @brief ADC disable.
 */
void adc_disable(void)
{
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);
    nrf_adc_int_disable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_DisableIRQ(ADC_IRQn);
}
/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}
/** @brief error_reason_clear.
 */
void error_reason_clear(void)
{
  memset(return_value_printf,0,sizeof(return_value_printf));
  return_value_cnt = 0;
}
void error_reason_storage(uint8_t *buff,uint8_t len)
{
  char   *pA;
  for(uint8_t i=0;i<10;i++)
  {
    pA = strstr((char *)buff,"\r\n");
    if(pA != NULL)
      *pA = ';';
    else
      break;
  }
  memcpy(&return_value_printf[return_value_cnt],buff,len);
  return_value_cnt += len;
}
/** @brief adc sample schedule.
 */
static void ADC_Sample_schedule_handler(void)
{
  int32_t               adc_total = 0;
  float                 adc_max = 0;
  float                 adc_min = 0;
  float                 adc_average = 0;
  static adc_schedule_t adc_schedule = adc_schedule_config;
  static uint8_t        timeout_cnt  = 0;
  static uint8_t        sample_cnt   = 0;
  switch(adc_schedule)
  {
    case adc_schedule_config:
    {
      nrf_adc_start();
      adc_schedule = adc_schedule_waiting_result;
      break;
    }
    case adc_schedule_waiting_result:
    {
      timeout_cnt++;
      if((!Flag.is_single_sample_finish)&&(timeout_cnt<=5))
      {
        break;
      }
      if(Flag.is_single_sample_finish)
      {
        Flag.is_single_sample_finish = false;
        adc_sample[sample_cnt++] = nrf_adc_result_get();
        if(sample_cnt>=ADC_SAMPLE_NUMBER)
        {
          sample_cnt = 0;
          adc_max = adc_sample[0];
          adc_min = adc_sample[0];
          for(uint8_t i=0;i<ADC_SAMPLE_NUMBER;i++)
          {
            adc_total += adc_sample[i];
            if(adc_sample[i]>adc_max)
            {
              adc_max = adc_sample[i];
            }
            if(adc_sample[i]<adc_min)
            {
              adc_min = adc_sample[i];
            }
          }
          adc_average = adc_total/ADC_SAMPLE_NUMBER;
          sample_voltage = adc_average/1024*3*2*1200;
          sample_max_voltage = adc_max/1024*3*2*1200;
          sample_min_voltage = adc_min/1024*3*2*1200;
          Flag.is_sample_finish = true;
          Flag.is_adc_sample = false;
        }
      }
      timeout_cnt = 0;
      adc_schedule = adc_schedule_config;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
open开关检查进程
************************************************************************/
#if HL500_WAY
static void open_sw_check_schedule_handler()
{
  static open_close_sw_check_schedule_t open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(open_close_sw_check_schedule)
  {
    case open_close_sw_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        nrf_gpio_cfg_output(OPEN_SW_PIN);
        nrf_gpio_pin_set(OPEN_SW_PIN);
      }
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(OPEN_SW_PIN);
        nrf_gpio_pin_clear(OPEN_SW_PIN);
      }
      UART_Data_Send((uint8_t *)open_sw_test, 12);
      open_close_sw_check_schedule = open_close_sw_check_schedule_waiting_result;
      break;
    }
    case open_close_sw_check_schedule_waiting_result:
    {
      timeout_cnt++;
      if((!Flag.is_uart_receive_finish)&&(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
      {
        break;
      }
      if(Flag.is_uart_receive_finish)
      {
        Flag.is_uart_receive_finish = false;
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"H\r\n") == NULL)
          {
            timeout_cnt = 0;
            open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
            check_cnt = 0;
            Flag.is_open_sw_check_finish = true;
            Flag.is_open_sw_check = false;
            open_sw_check_result = 1;//失败
            check_open_close_sw_pin_init();
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"L\r\n") == NULL)
          {
            timeout_cnt = 0;
            open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
            check_cnt = 0;
            Flag.is_open_sw_check_finish = true;
            Flag.is_open_sw_check = false;
            open_sw_check_result = 1;//失败
            check_open_close_sw_pin_init();
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_open_sw_check_finish = true;
          Flag.is_open_sw_check = false;
          open_sw_check_result = 0;//成功
          check_open_close_sw_pin_init();
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_open_sw_check_finish = true;
        Flag.is_open_sw_check = false;
        open_sw_check_result = 1;//失败
        check_open_close_sw_pin_init();
        error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
      }
      timeout_cnt = 0;
      open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
#else
static void open_sw_check_schedule_handler()
{
  static open_close_sw_check_schedule_t open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(open_close_sw_check_schedule)
  {
    case open_close_sw_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        nrf_gpio_cfg_output(OPEN_SW_PIN);
        nrf_gpio_pin_set(OPEN_SW_PIN);
      }
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(OPEN_SW_PIN);
        nrf_gpio_pin_clear(OPEN_SW_PIN);
      }
      UART_Data_Send((uint8_t *)open_sw_test, 12);
      open_close_sw_check_schedule = open_close_sw_check_schedule_waiting_result;
      break;
    }
    case open_close_sw_check_schedule_waiting_result:
    {
      timeout_cnt++;
      Flag.is_uart_receive_finish = false;
      if(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
      {
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"H\r\n") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"L\r\n") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_open_sw_check_finish = true;
          Flag.is_open_sw_check = false;
          open_sw_check_result = 0;//成功
          check_open_close_sw_pin_init();
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_open_sw_check_finish = true;
        Flag.is_open_sw_check = false;
        open_sw_check_result = 1;//失败
        check_open_close_sw_pin_init();
        error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
      }
      timeout_cnt = 0;
      open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
#endif
/************************************************************************
close开关检查进程
************************************************************************/
#if HL500_WAY
static void close_sw_check_schedule_handler()
{
  static open_close_sw_check_schedule_t open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(open_close_sw_check_schedule)
  {
    case open_close_sw_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        nrf_gpio_cfg_output(CLOSE_SW_PIN);
        nrf_gpio_pin_set(CLOSE_SW_PIN);
      }
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(CLOSE_SW_PIN);
        nrf_gpio_pin_clear(CLOSE_SW_PIN);
      }
      UART_Data_Send((uint8_t *)close_sw_test, 13);
      open_close_sw_check_schedule = open_close_sw_check_schedule_waiting_result;
      break;
    }
    case open_close_sw_check_schedule_waiting_result:
    {
      timeout_cnt++;
      if((!Flag.is_uart_receive_finish)&&(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
      {
        break;
      }
      if(Flag.is_uart_receive_finish)
      {
        Flag.is_uart_receive_finish = false;
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"H\r\n") == NULL)
          {
            timeout_cnt = 0;
            open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
            check_cnt = 0;
            Flag.is_close_sw_check_finish = true;
            Flag.is_close_sw_check = false;
            close_sw_check_result = 1;//失败
            check_open_close_sw_pin_init();
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"L\r\n") == NULL)
          {
            timeout_cnt = 0;
            open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
            check_cnt = 0;
            Flag.is_close_sw_check_finish = true;
            Flag.is_close_sw_check = false;
            close_sw_check_result = 1;//失败
            check_open_close_sw_pin_init();
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_close_sw_check_finish = true;
          Flag.is_close_sw_check = false;
          close_sw_check_result = 0;//成功
          check_open_close_sw_pin_init();
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_close_sw_check_finish = true;
        Flag.is_close_sw_check = false;
        close_sw_check_result = 1;//失败
        check_open_close_sw_pin_init();
        error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
      }
      timeout_cnt = 0;
      open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
#else
static void close_sw_check_schedule_handler()
{
  static open_close_sw_check_schedule_t open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(open_close_sw_check_schedule)
  {
    case open_close_sw_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        nrf_gpio_cfg_output(CLOSE_SW_PIN);
        nrf_gpio_pin_set(CLOSE_SW_PIN);
      }
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(CLOSE_SW_PIN);
        nrf_gpio_pin_clear(CLOSE_SW_PIN);
      }
      UART_Data_Send((uint8_t *)close_sw_test, 13);
      open_close_sw_check_schedule = open_close_sw_check_schedule_waiting_result;
      break;
    }
    case open_close_sw_check_schedule_waiting_result:
    {
      timeout_cnt++;
      Flag.is_uart_receive_finish = false;
      if(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
      {
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"H\r\n") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"L\r\n") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_close_sw_check_finish = true;
          Flag.is_close_sw_check = false;
          close_sw_check_result = 0;//成功
          check_open_close_sw_pin_init();
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_close_sw_check_finish = true;
        Flag.is_close_sw_check = false;
        close_sw_check_result = 1;//失败
        check_open_close_sw_pin_init();
        error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
      }
      timeout_cnt = 0;
      open_close_sw_check_schedule = open_close_sw_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
#endif
/************************************************************************
电机电压检查进程
************************************************************************/
void motor_voltage_check_schedule_handler(void)
{
  static motor_voltage_check_schedule_t motor_voltage_check_schedule = motor_voltage_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(motor_voltage_check_schedule)
  {
    case motor_voltage_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        adc_config(MOTOR_ROLL_A);
        Flag.is_adc_sample = true;
      }
      if(check_cnt == 1)
      {
        adc_config(MOTOR_ROLL_B);
        Flag.is_adc_sample = true;
      }
      motor_voltage_check_schedule = motor_voltage_check_schedule_waiting_result;
      break;
    }
    case motor_voltage_check_schedule_waiting_result:
    {
      timeout_cnt++;
      if((!Flag.is_sample_finish)&&(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
      {
        break;
      }
      if(Flag.is_sample_finish)
      {
        Flag.is_sample_finish = false;
        adc_disable();
        if(check_cnt == 0)
        {
          motor_roll_a_voltage = sample_voltage;
        }
        else if(check_cnt == 1)
        {
          motor_roll_b_voltage = sample_voltage;
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_motor_voltage_check_finish = true;
          Flag.is_motor_voltage_check = false;
        }
      }
      timeout_cnt = 0;
      motor_voltage_check_schedule = motor_voltage_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
通过led指示程序运行情况
************************************************************************/
void led_indicate_program_run(void)
{
  static uint8_t delay_cnt = 0;
  delay_cnt++;
  if(delay_cnt>=(100/CHECK_TIME_TIMEOUT_INTERVAL))
  {
    delay_cnt = 0;
    nrf_gpio_pin_toggle(PROGRAM_RUN_INDICATE);
  }
}
/************************************************************************
断开与非法设备的连接
************************************************************************/
static void check_Time_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  /****************************///UART接收完成检测
  if(Flag.is_uart_receive_start)
  {
    UART_receive_over_judge_cnt++;
    if(UART_receive_over_judge_cnt>=25)
    {
      UART_receive_over_judge_cnt = 0;
      Flag.is_uart_receive_start = false;
      Flag.is_uart_receive_finish = true;
    }
  }
  /*****************************///UART接收等待超时计时
  if(Flag.is_communicate_start_timeout_calculate)
  {
    communicate_time_cnt++;
  }
  /*****************************///与PC通讯进程
  if(Flag.is_communicate_with_pc)
  {
    communicate_with_pc_handler();
    waiting_instruction_timeout_cnt++;
    if(waiting_instruction_timeout_cnt==(5000/CHECK_TIME_TIMEOUT_INTERVAL))
    {
      UART_Data_Send((uint8_t *)Timeout, strlen((char *)Timeout));
    }
    if(waiting_instruction_timeout_cnt>(5000/CHECK_TIME_TIMEOUT_INTERVAL))
    {
      waiting_instruction_timeout_cnt = 0;
      NVIC_SystemReset();
    }
  }
  /*****************************///与hl300通讯进程
  if(Flag.is_communicate_with_hl300)
  {
    communicate_with_hl300_handler();
  }
  /*****************************///ADC采样进程
  if(Flag.is_adc_sample)
  {
    ADC_Sample_schedule_handler();
  }
  /*****************************///open开关检查进程
  if(Flag.is_open_sw_check)
  {
    open_sw_check_schedule_handler();
  }
  /*****************************///close开关检查进程
  if(Flag.is_close_sw_check)
  {
    close_sw_check_schedule_handler();
  }
  /*****************************///电机电压检查进程
  if(Flag.is_motor_voltage_check)
  {
    motor_voltage_check_schedule_handler();
  }
  led_indicate_program_run();
}
/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_scanner_init(void)
{
  uint32_t err_code = NRF_SUCCESS;
  APP_TIMER_INIT(APP_TIMER_PRESCALER, 1,1, NULL);
  /*****************************************************************************************/
  err_code |= app_timer_create(&check_Time_timer_id,
                               APP_TIMER_MODE_REPEATED,
                               check_Time_timeout_handler);
  err_code |= app_timer_start(check_Time_timer_id, APP_TIMER_TICKS(CHECK_TIME_TIMEOUT_INTERVAL, APP_TIMER_PRESCALER), NULL);
  /*****************************************************************************************/
  APP_ERROR_CHECK(err_code);
}
/************************************************************************
标志初始化函数
************************************************************************/
void Main_Flags_scanner_init(void)
{
  memset(&Flag,0,sizeof(Flag_t));
}
/************************************************************************
与PC通讯协议解析
************************************************************************/
void communicate_with_pc_handler(void)
{
  if(!Flag.is_uart_receive_finish)
    return;
  Flag.is_uart_receive_finish = false;
  if(UART_RX_BUF[0] != FUNCTION_TEST_INSTRUCTIONS_SET)
  {
    UART_RX_BUF_CNT = 0;
    return;
  }
  if(UART_RX_BUF_CNT<3)
  {
    UART_RX_BUF_CNT = 0;
    return;
  }
  if(UART_RX_BUF[UART_RX_BUF_CNT-1] != FUNCTION_TEST_INSTRUCTION_END_BYTE)
  {
    UART_RX_BUF_CNT = 0;
    return;
  }
  Test_instruction = (Test_instructions_t)UART_RX_BUF[1];
  waiting_instruction_timeout_cnt = 0;
  switch(Test_instruction)
  {
    case Test_instruction_inquiry_sn:
    case Test_instruction_inquiry_acc_id:
    case Test_instruction_inquiry_acc_data:
    case Test_instruction_rw_flash_test:
    case Test_instruction_inquiry_software_version:
    case Test_instruction_led_turnon_test:
    case Test_instruction_led_turnoff_test:
    case Test_instruction_buzzer_test:
    case Test_instruction_monitor_ble:
    case Test_instruction_motor_right:
    case Test_instruction_motor_left:
    case Test_instruction_enable_gps_calculate:
    case Test_instruction_inquiry_gps_position:
    case Test_instruction_inquiry_sim_imsi:
    case Test_instruction_open_sw:
    case Test_instruction_close_sw:
    case Test_instruction_charge_current:
    case Test_instruction_VBUS_voltage:
    case Test_instruction_VBAT_voltage:
    case Test_instruction_VBAT_TEMP_voltage:
    case Test_instruction_enter_lpm:
    {
      Flag.is_uart_sw_to_hl300 = true;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
通讯切换
************************************************************************/
void uart_communicate_flag_clear(void)
{
  Flag.is_communicate_with_pc = false;
  Flag.is_communicate_with_hl300 = false;
}
void uart_communicate_pin_set_input(void)
{
  nrf_gpio_cfg_input(UART_PC_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_PC_RX, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(UART_HL300_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_HL300_RX, NRF_GPIO_PIN_NOPULL);
}
void sw_to_communicating_with_pc(void)
{
  uart_communicate_flag_clear();
  Flag.is_communicate_with_pc = true;
  uart_communicate_pin_set_input();
}
void sw_to_communicating_with_hl300(void)
{
  uart_communicate_flag_clear();
  Flag.is_communicate_with_hl300 = true;
  uart_communicate_pin_set_input();
}
/**********************************************
检测OPEN CLOSE开关管脚初始化
**********************************************/
void check_open_close_sw_pin_init(void)
{
  nrf_gpio_cfg_input(OPEN_SW_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CLOSE_SW_PIN, NRF_GPIO_PIN_NOPULL);
}
/**********************************************
检测LED管脚初始化
**********************************************/
void check_led_pin_init(void)
{
  nrf_gpio_cfg_input(CHECK_LED1_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_LED2_PIN, NRF_GPIO_PIN_NOPULL);
}
/************************************************************************
主函数
************************************************************************/
int main(void) 
{
    Main_Flags_scanner_init();
    nrf_gpio_cfg_output(PROGRAM_RUN_INDICATE);
    check_led_pin_init();
    check_open_close_sw_pin_init();
    timers_scanner_init();
    ble_stack_scanner_init();
    db_discovery_init();
    location_init();
    /****************************************/
    Flag.is_uart_sw_to_hl300 = true;
    for (;;)
    {
      power_manage();
      /****************************///切换到与PC通讯
      if(Flag.is_uart_sw_to_pc)
      {
        Flag.is_uart_sw_to_pc = false;
        sw_to_communicating_with_pc();
        uart_init(comm_params_communicate_with_pc);
        if(Flag.is_return_result_to_pc)
        {
          Flag.is_return_result_to_pc = false;
          UART_Data_Send(return_result_buff, return_result_len);
        }
      }
      /****************************///切换到与nrf51822通讯
      if(Flag.is_uart_sw_to_hl300)
      {
        Flag.is_uart_sw_to_hl300 = false;
        sw_to_communicating_with_hl300();
        uart_init(comm_params_communicate_with_hl300);
      }
    }
}
  
