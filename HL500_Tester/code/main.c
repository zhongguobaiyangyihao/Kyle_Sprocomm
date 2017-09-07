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
#include "ofo_tester.h"
#include "nrf_adc.h"


APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

#define APP_TIMER_PRESCALER        0                                  /**< Value of the RTC1 PRESCALER register. */

#define APPL_LOG                   app_trace_log                      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SCAN_INTERVAL              0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x00A0                             /**< Determines scan window in units of 0.625 millisecond. */



#define DEVICE_NAME                "ofo_tester"                             /**< Name of device. Will be included in the advertising data. */

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

#define UART_PC_TX                      9
#define UART_PC_RX                      11
#define UART_NRF51822_TX                24
#define UART_NRF51822_RX                25
#define UART_SIM868_TX                  22
#define UART_SIM868_RX                  23


 
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
           11,//RX_PIN_NUMBER,
           9, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud115200
       };
const app_uart_comm_params_t comm_params_communicate_with_nrf51822 =
       {
           25, //RX_PIN_NUMBER,
           24, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud9600
       };
const app_uart_comm_params_t comm_params_monitor_nrf51822 =
       {
           22, //RX_PIN_NUMBER,
           23, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud9600
       };
const app_uart_comm_params_t comm_params_communicate_with_sim868 =
       {
           23, //RX_PIN_NUMBER,
           22, //TX_PIN_NUMBER,
           RTS_PIN_NUMBER,
           CTS_PIN_NUMBER,
           APP_UART_FLOW_CONTROL_DISABLED,
           false,
           UART_BAUDRATE_BAUDRATE_Baud9600
       };
/************************************************************************///scanner mode variable define
/************************************************************************///slave mode variable define   
#define                             POWER_RESETREAS                            0x0007000F
app_timer_id_t                      check_Time_timer_id;
Flag_t                              Flag;
uint8_t                             UART_RX_BUF[UART_RX_BUF_SIZE];
uint16_t                            UART_RX_BUF_CNT;
uint16_t                            UART_receive_over_judge_cnt = 0;
Test_instructions_t                 Test_instruction = Test_instruction_check_ofo_pcba;
uint8_t                             return_result_buff[200];
uint16_t                            return_result_len;
uint8_t                             config_sn[13];
uint16_t                            waiting_instruction_timeout_cnt = 0;
volatile int32_t                    adc_sample[ADC_SAMPLE_NUMBER];
uint8_t                             key_check_result = 0;
uint8_t                             unlock_check_result = 0;
uint8_t                             lock_check_result = 0;
uint8_t                             enable_gps_check_result = 0;
uint8_t                             return_value_printf[100];
uint8_t                             return_value_cnt = 0;
ble_result_t                        ble_result = {false,-80};
float                               sample_voltage = 0;
uint8_t                             monitor_ble_addr[6] = {0};
extern uint16_t                     communicate_time_cnt;
extern const uint8_t                Timeout[];
extern const uint8_t                check_key[];
extern const uint8_t                unlock_state_test[];
extern const uint8_t                lock_state_test[];
extern const uint8_t                enable_gps_calculate[];
extern const uint8_t                inquiry_gps_is_enable[];
/************************************************************************/
void scan_start(void);
void uart_init(app_uart_comm_params_t comm_params);
void communicate_with_pc_handler(void);
void check_key_pin_init(void);
void check_unlock_lock_pin_init(void);
extern void communicate_with_nrf51822_handler(void);
extern void communicate_with_sim868_handler(void);
extern void communicate_with_monitor_nrf51822_handler(void);
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
�������ݽӿ�
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
�������ݽӿ�
************************************************************************/
void nrf51822_Data_Send(uint8_t * p_data, uint16_t length)
{
  UART_Data_Send(p_data, length);
}
/************************************************************************
�������ݽӿ�
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
���ֻ����ݴ����ϴ����ݻ���
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
            if((p_gap_evt->params.adv_report.peer_addr.addr[0] == monitor_ble_addr[0])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[1] == monitor_ble_addr[1])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[2] == monitor_ble_addr[2])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[3] == monitor_ble_addr[3])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[4] == monitor_ble_addr[4])&&
               (p_gap_evt->params.adv_report.peer_addr.addr[5] == monitor_ble_addr[5])
              )
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
ʹ�ܳ���notification����
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

    m_scan_param.active       = 1;            // Active scanning set����ɨ�裬��ģʽ�½���ɨ���Ӧ��.
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
          for(uint8_t i=0;i<ADC_SAMPLE_NUMBER;i++)
          {
            adc_total += adc_sample[i];
          }
          adc_average = adc_total/ADC_SAMPLE_NUMBER;
          sample_voltage = adc_average/1024*3*2*1200;
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
key������
************************************************************************/
static void key_check_schedule_handler(void)
{
  static key_check_schedule_t key_check_schedule = key_check_schedule_press;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(key_check_schedule)
  {
    case key_check_schedule_press:
    {
      check_key_pin_init();
      UART_RX_BUF_CNT = 0;
      memset(UART_RX_BUF,0,UART_RX_BUF_SIZE);
      nrf_gpio_cfg_output(check_key[check_cnt]);
      nrf_gpio_pin_clear(check_key[check_cnt]);
      key_check_schedule = key_check_schedule_waiting_result;
      break;
    }
    case key_check_schedule_waiting_result:
    {
      timeout_cnt++;
      Flag.is_uart_receive_finish = false;
      if(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
      {
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"Key1") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"Key2") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 2)
        {
          if(strstr((char *)UART_RX_BUF,"Key3") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 3)
        {
          if(strstr((char *)UART_RX_BUF,"Key4") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 4)
        {
          if(strstr((char *)UART_RX_BUF,"Keyc") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=KEY_TOTAL_NUMBER)
        {
          check_cnt = 0;
          Flag.is_key_check_finish = true;
          Flag.is_key_check = false;
          key_check_result = 0;//�ɹ�
          check_key_pin_init();
          Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
          UART_receive_over_judge_cnt = 0;
          Flag.is_uart_receive_start = false;
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_key_check_finish = true;
        Flag.is_key_check = false;
        key_check_result = 1;//ʧ��
        error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
        check_key_pin_init();
        Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
        UART_receive_over_judge_cnt = 0;
        Flag.is_uart_receive_start = false;
      }
      timeout_cnt = 0;
      key_check_schedule = key_check_schedule_press;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
����������
************************************************************************/
static void unlock_check_schedule_handler(void)
{
  static unlock_lock_check_schedule_t unlock_lock_check_schedule = unlock_lock_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(unlock_lock_check_schedule)
  {
    case unlock_lock_check_schedule_transmit:
    {
      UART_Data_Send((uint8_t *)unlock_state_test, 3);
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(CHECK_UNLOCK_PIN);
        nrf_gpio_pin_clear(CHECK_UNLOCK_PIN);
      }
      unlock_lock_check_schedule = unlock_lock_check_schedule_waiting_result;
      break;
    }
    case unlock_lock_check_schedule_waiting_result:
    {
      timeout_cnt++;
      Flag.is_uart_receive_finish = false;
      if(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
      {
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"0") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"1") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_unlock_check_finish = true;
          Flag.is_unlock_check = false;
          unlock_check_result = 0;//�ɹ�
          check_unlock_lock_pin_init();
          Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
          UART_receive_over_judge_cnt = 0;
          Flag.is_uart_receive_start = false;
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_unlock_check_finish = true;
        Flag.is_unlock_check = false;
        unlock_check_result = 1;//ʧ��
        error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        check_unlock_lock_pin_init();
        Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
        UART_receive_over_judge_cnt = 0;
        Flag.is_uart_receive_start = false;
      }
      timeout_cnt = 0;
      unlock_lock_check_schedule = unlock_lock_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
����������
************************************************************************/
static void lock_check_schedule_handler(void)
{
  static unlock_lock_check_schedule_t unlock_lock_check_schedule = unlock_lock_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(unlock_lock_check_schedule)
  {
    case unlock_lock_check_schedule_transmit:
    {
      UART_Data_Send((uint8_t *)lock_state_test, 3);
      if(check_cnt == 1)
      {
        nrf_gpio_cfg_output(CHECK_LOCK_PIN);
        nrf_gpio_pin_clear(CHECK_LOCK_PIN);
      }
      unlock_lock_check_schedule = unlock_lock_check_schedule_waiting_result;
      break;
    }
    case unlock_lock_check_schedule_waiting_result:
    {
      timeout_cnt++;
      Flag.is_uart_receive_finish = false;
      if(timeout_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
      {
        if(check_cnt == 0)
        {
          if(strstr((char *)UART_RX_BUF,"0") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"1") == NULL)
          {
            break;
          }
          error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        }
        check_cnt++;
        if(check_cnt>=2)
        {
          check_cnt = 0;
          Flag.is_lock_check_finish = true;
          Flag.is_lock_check = false;
          lock_check_result = 0;//�ɹ�
          check_unlock_lock_pin_init();
          Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
          UART_receive_over_judge_cnt = 0;
          Flag.is_uart_receive_start = false;
        }
      }
      else
      {
        check_cnt = 0;
        Flag.is_lock_check_finish = true;
        Flag.is_lock_check = false;
        lock_check_result = 1;//ʧ��
        error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
        check_unlock_lock_pin_init();
        Flag.is_uart_receive_finish = false;//��ֹ������������ɸ���
        UART_receive_over_judge_cnt = 0;
        Flag.is_uart_receive_start = false;
      }
      timeout_cnt = 0;
      unlock_lock_check_schedule = unlock_lock_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
ʹ��GPS������
************************************************************************/
static void enable_gps_schedule_handler()
{
  static enable_gps_check_schedule_t enable_gps_check_schedule = enable_gps_check_schedule_transmit;
  static uint16_t                    timeout_cnt  = 0;
  static uint8_t                     check_cnt    = 0;
  switch(enable_gps_check_schedule)
  {
    case enable_gps_check_schedule_transmit:
    {
      if(check_cnt == 0)
      {
        UART_Data_Send((uint8_t *)inquiry_gps_is_enable, 13);
      }
      else if(check_cnt == 1)
      {
        UART_Data_Send((uint8_t *)enable_gps_calculate, 14);
      }
      else if(check_cnt == 2)
      {
        UART_Data_Send((uint8_t *)inquiry_gps_is_enable, 13);
      }
      enable_gps_check_schedule = enable_gps_check_schedule_waiting_result;
      break;
    }
    case enable_gps_check_schedule_waiting_result:
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
          if(strstr((char *)UART_RX_BUF,"+CGNSPWR: 1"))
          {
            check_cnt = 0;
            Flag.is_enable_gps_check_finish = true;
            Flag.is_enable_gps_check = false;
            enable_gps_check_result = 0;//�ɹ�
            timeout_cnt = 0;
            enable_gps_check_schedule = enable_gps_check_schedule_transmit;
            break;
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
          }
        }
        else if(check_cnt == 1)
        {
          if(strstr((char *)UART_RX_BUF,"OK"))
          {
            check_cnt = 0;
            Flag.is_enable_gps_check_finish = true;
            Flag.is_enable_gps_check = false;
            enable_gps_check_result = 0;//�ɹ�
            timeout_cnt = 0;
            enable_gps_check_schedule = enable_gps_check_schedule_transmit;
            break;
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
          }
        }  
        else if(check_cnt == 2)
        {
          if(strstr((char *)UART_RX_BUF,"+CGNSPWR: 1"))
          {
            check_cnt = 0;
            Flag.is_enable_gps_check_finish = true;
            Flag.is_enable_gps_check = false;
            enable_gps_check_result = 0;//�ɹ�
            timeout_cnt = 0;
            enable_gps_check_schedule = enable_gps_check_schedule_transmit;
            break;
          }
          else
          {
            check_cnt = 0;
            Flag.is_enable_gps_check_finish = true;
            Flag.is_enable_gps_check = false;
            enable_gps_check_result = 1;//ʧ��
            timeout_cnt = 0;
            enable_gps_check_schedule = enable_gps_check_schedule_transmit;
            
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            break;
          }
        }
        check_cnt++;
      }
      else
      {
        check_cnt = 0;
        Flag.is_enable_gps_check_finish = true;
        Flag.is_enable_gps_check = false;
        enable_gps_check_result = 1;//ʧ��
        error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
      }
      timeout_cnt = 0;
      enable_gps_check_schedule = enable_gps_check_schedule_transmit;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
ͨ��ledָʾ�����������
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
�Ͽ���Ƿ��豸������
************************************************************************/
static void check_Time_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  /****************************///UART������ɼ��
  if(Flag.is_uart_receive_start)
  {
    UART_receive_over_judge_cnt++;
    if(UART_receive_over_judge_cnt>=5)
    {
      UART_receive_over_judge_cnt = 0;
      Flag.is_uart_receive_start = false;
      Flag.is_uart_receive_finish = true;
    }
  }
  /*****************************///UART���յȴ���ʱ��ʱ
  if(Flag.is_communicate_start_timeout_calculate)
  {
    communicate_time_cnt++;
  }
  /*****************************///��PCͨѶ����
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
  /*****************************///��nrf51822ͨѶ����
  if(Flag.is_communicate_with_nrf51822)
  {
    communicate_with_nrf51822_handler();
  }
  /*****************************///��sim868ͨѶ����
  if(Flag.is_communicate_with_sim868)
  {
    communicate_with_sim868_handler();
  }
  /*****************************///��������ǰ�����nrf51822���͸�ͨѶ����
  if(Flag.is_monitor_nrf51822_output)
  {
    communicate_with_monitor_nrf51822_handler();
  }
  /*****************************///ADC��������
  if(Flag.is_adc_sample)
  {
    ADC_Sample_schedule_handler();
  }
  /*****************************///Key������
  if(Flag.is_key_check)
  {
    key_check_schedule_handler();
  }
  /*****************************///����������
  if(Flag.is_unlock_check)
  {
    unlock_check_schedule_handler();
  }
  /*****************************///����������
  if(Flag.is_lock_check)
  {
    lock_check_schedule_handler();
  }
  /*****************************///ʹ��GPS������
  if(Flag.is_enable_gps_check)
  {
    enable_gps_schedule_handler();
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
��־��ʼ������
************************************************************************/
void Main_Flags_scanner_init(void)
{
  memset(&Flag,0,sizeof(Flag_t));
}
/************************************************************************
��PCͨѶЭ�����
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
    case Test_instruction_config_sn:
    {
      Flag.is_uart_sw_to_nrf51822 = true;
      memcpy(config_sn,&UART_RX_BUF[2],sizeof(config_sn));
      break;
    }
    case Test_instruction_inquiry_sn:
    case Test_instruction_motor_before_pwron_test:
    case Test_instruction_SIM_VDD_test:
    case Test_instruction_GSM_4V_test:
    case Test_instruction_VBAT_test:
    case Test_instruction_motor_after_pwron_test:
    case Test_instruction_inquiry_lis3dh_id:
    case Test_instruction_inquiry_lis3dh_data:
    case Test_instruction_rw_flash_test:
    case Test_instruction_inquiry_software_version:
    case Test_instruction_key_test:
    case Test_instruction_led_turnon_test:
    case Test_instruction_led_turnoff_test:
    case Test_instruction_buzzer_test:
    case Test_instruction_enter_lpm:
    case Test_instruction_unlock_state_test:
    case Test_instruction_lock_state_test:
    case Test_instruction_monitor_ble:
    {
      Flag.is_uart_sw_to_nrf51822 = true;
      break;
    }
    case Test_instruction_inquiry_gsm_signal_strength:
    case Test_instruction_enable_gps_calculate:
    case Test_instruction_inquiry_gps_position:
    case Test_instruction_inquiry_sim_imsi:
    case Test_instruction_pwroff_sim868:
    case Test_instruction_inquiry_sim868_imei:
    {
      Flag.is_uart_sw_to_sim868 = true;
      break;
    }
    default:
      break;
  }
}
/************************************************************************
ͨѶ�л�
************************************************************************/
void uart_communicate_flag_clear(void)
{
  Flag.is_communicate_with_pc = false;
  Flag.is_communicate_with_nrf51822 = false;
  Flag.is_communicate_with_sim868 = false;
  Flag.is_monitor_nrf51822_output = false;
}
void uart_communicate_pin_set_input(void)
{
  #if 0
  nrf_gpio_cfg_input(UART_PC_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_PC_RX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_NRF51822_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_NRF51822_RX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_SIM868_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_SIM868_RX, NRF_GPIO_PIN_PULLUP);
  #else
  nrf_gpio_cfg_input(UART_PC_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_PC_RX, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(UART_NRF51822_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_NRF51822_RX, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(UART_SIM868_TX, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(UART_SIM868_RX, NRF_GPIO_PIN_NOPULL);
  #endif
}
void sw_to_communicating_with_pc(void)
{
  uart_communicate_flag_clear();
  Flag.is_communicate_with_pc = true;
  uart_communicate_pin_set_input();
}
void sw_to_communicating_with_nrf51822(void)
{
  uart_communicate_flag_clear();
  Flag.is_communicate_with_nrf51822 = true;
  uart_communicate_pin_set_input();
}
void sw_to_communicating_with_sim868(void)
{
  uart_communicate_flag_clear();
  Flag.is_communicate_with_sim868 = true;
  uart_communicate_pin_set_input();
}
void sw_to_communicating_with_monitor_nrf51822(void)
{
  uart_communicate_flag_clear();
  Flag.is_monitor_nrf51822_output = true;
  uart_communicate_pin_set_input();
}
/**********************************************
���KEY�ܽų�ʼ��
**********************************************/
void check_key_pin_init(void)
{
  nrf_gpio_cfg_input(CHECK_KEY1_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_KEY2_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_KEY3_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_KEY4_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_KEY5_PIN, NRF_GPIO_PIN_NOPULL);
}
/**********************************************
���LED�ܽų�ʼ��
**********************************************/
void check_led_pin_init(void)
{
  nrf_gpio_cfg_input(CHECK_LED1_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_LED2_PIN, NRF_GPIO_PIN_NOPULL);
}
/**********************************************
���BUZZER�ܽų�ʼ��
**********************************************/
void check_buzzer_pin_init(void)
{
  nrf_gpio_cfg_input(CHECK_BUZZER_PIN, NRF_GPIO_PIN_NOPULL);
}
/**********************************************
���BUZZER�ܽų�ʼ��
**********************************************/
void check_unlock_lock_pin_init(void)
{
  nrf_gpio_cfg_input(CHECK_UNLOCK_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(CHECK_LOCK_PIN, NRF_GPIO_PIN_NOPULL);
}
/************************************************************************
������
************************************************************************/
int main(void) 
{
    Main_Flags_scanner_init();
    nrf_gpio_cfg_output(PROGRAM_RUN_INDICATE);
    check_key_pin_init();
    check_led_pin_init();
    check_buzzer_pin_init();
    check_unlock_lock_pin_init();
    timers_scanner_init();
    ble_stack_scanner_init();
    db_discovery_init();
    location_init();
    /****************************************/
    Flag.is_uart_sw_to_nrf51822 = true;
    for (;;)
    {
      power_manage();
      /****************************///�л�����PCͨѶ
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
      /****************************///�л�����nrf51822ͨѶ
      if(Flag.is_uart_sw_to_nrf51822)
      {
        Flag.is_uart_sw_to_nrf51822 = false;
        sw_to_communicating_with_nrf51822();
        uart_init(comm_params_communicate_with_nrf51822);
      }
      /****************************///�л�����sim868ͨѶ
      if(Flag.is_uart_sw_to_sim868)
      {
        Flag.is_uart_sw_to_sim868 = false;
        sw_to_communicating_with_sim868();
        uart_init(comm_params_communicate_with_sim868);
      }
      /****************************///�л�����������ǰ�����nrf51822���͸�sim868������
      if(Flag.is_uart_sw_to_monitor_nrf51822_output)
      {
        Flag.is_uart_sw_to_monitor_nrf51822_output = false;
        sw_to_communicating_with_monitor_nrf51822();
        uart_init(comm_params_monitor_nrf51822);
      }
    }
}
  
