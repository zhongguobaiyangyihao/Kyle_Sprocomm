/********************************************************
********************************************************/

#ifndef BEACON_SETTING_H
#define BEACON_SETTING_H


#include "types.h"
#include <stdint.h>
#include "ble.h"
/************************************************************************************************///slave mode
#define MAX_SEID_NUMBER                 10
#define BLE_UUID_BEACON_SETTING_SERVICE  0x18F0 
#define BLE_UUID_BEACON_SETTING_CHAR    0x2AF1
#define MAX_SIMULATE_BEACON             10
#define MAX_ADV_BEACON_ONE_CYCLE        10
/************************************************************************************************///scanner mode
#define                             UART_TX_BUF_SIZE                  50      
#define                             UART_RX_BUF_SIZE                  100

#define ADV_MANUFACTURE_DATA_LENGTH                         5
#define SCAN_RESPONSE_MANUFACTURE_DATA_LENGTH               20
#define FACTORY_ID_LENGTH               8
#define WATCH_ID_LENGTH                 9
#define TIME_LENGTH                     6
#define WATCH_MOVE_DELAY_TIME           5
//#define RSSI_IN_ONE_METER               (-71)//-61dbm

#define MAX_RECEIVE_LOCATIONDATA_BUFF_SIZE         10
#define MAX_UPLOAD_LOCATIONDATA_BUFF_SIZE          10
#define BASE_PAR_N                                 2.0
#define MAX_FILTER_SPACE_SIZE                      50

#define WATCH_NORECEIVE_CLEAR_TIME_WINDOW          5
#define CHECK_TIME_TIMEOUT_INTERVAL                20
/************************************************************************************************/
typedef struct 
{
  uint8_t  reserved0                     : 1;
  uint8_t  is_enable_gps_check_finish    : 1;
  uint8_t  is_enable_gps_check           : 1;
  uint8_t  is_lock_check_finish          : 1;
  uint8_t  is_lock_check                 : 1;
  uint8_t  is_unlock_check_finish        : 1;
  uint8_t  is_unlock_check               : 1;
  uint8_t  is_key_check_finish           : 1;
  
  uint8_t  is_key_check                  : 1;
  uint8_t  is_sample_finish              : 1;//全部采样
  uint8_t  is_single_sample_finish       : 1;//单次采样
  uint8_t  is_adc_sample                 : 1;
  uint8_t  is_return_result_to_pc        : 1;
  uint8_t  is_communicate_start_timeout_calculate : 1;
  uint8_t  is_monitor_nrf51822_output    : 1;
  uint8_t  is_uart_sw_to_monitor_nrf51822_output : 1;
  
  uint8_t  is_communicate_with_sim868    : 1;
  uint8_t  is_uart_sw_to_sim868          : 1;
  uint8_t  is_communicate_with_nrf51822  : 1;
  uint8_t  is_uart_sw_to_nrf51822        : 1;
  uint8_t  is_uart_sw_to_pc              : 1;
  uint8_t  is_communicate_with_pc        : 1;
  uint8_t  is_uart_receive_finish        : 1;
  uint8_t  is_uart_receive_start         : 1;
}Flag_t;


typedef struct
{
    uint8_t     year;                                         
    uint8_t     month; 
    uint8_t     date;                                         
    uint8_t     hour;    
    uint8_t     minute;                                         
    uint8_t     second;  
}BJ_Time_t;
typedef BJ_Time_t ble_bs_time;

/************************************************************************************************///scanner mode
typedef struct
{
  uint8_t address[WATCH_ID_LENGTH];
}watchid_t;
//typedef struct
//{
//  uint8_t data[SCAN_RESPONSE_MANUFACTURE_DATA_LENGTH];//receive buff:data[0]->rssi，data[1]->空着，data[2]->电池电量，data[3]->扩展一，data[4]->扩展二;
//                                  //upload buff:data[0]->距离整数米，data[1]->距离小数分米，data[2]->电池电量，data[3]->扩展一，data[4]->扩展二;
//}watchdata_t;
typedef struct
{
//  uint8_t  data[2];//receive buff:data[0]->rssi，data[1]->空着
//                   //upload buff:data[0]->距离整数米，data[1]->距离小数分米
  int8_t rssi;
  float  value;
}distance_t;
typedef struct
{
  uint8_t      length;
  uint8_t      data[ADV_MANUFACTURE_DATA_LENGTH];//电池电量,扩展一，扩展二
}adv_manufacture_t;

typedef struct
{
  uint8_t      length;
  uint8_t      data[SCAN_RESPONSE_MANUFACTURE_DATA_LENGTH];//扩展
}scan_response_manufacture_t;

typedef struct
{
  watchid_t                          watchid;
  long                               time_stamp;
  uint8_t                            ab_access;//是否开门通过
}location_data_t;

typedef struct
{
  watchid_t                          watchid;              //用于区分手环
  uint8_t                            storaged_history_number;               //已经存储进来的历史数据的条数
  uint8_t                            watch_noreceive_cnt;                   //监测未接收到该手环的时长，长于5s就清位
}watch_filter_space_t;

typedef struct
{
  uint8_t  data[8];
  long     time_stamp;
}SEID_t;
/************************************************************************************************/
/************************************************************************************************///slave mode
/**@brief Beacon Service event type. */
typedef enum
{
    BLE_BS_EVT_ADV_INTVAL_OLD = 0x01,
    BLE_BS_EVT_BT_ADDR_CHANGE_INTVAL = 0x02,
    BLE_BS_EVT_LOW_POWER_CONFG = 0x03,
    BLE_BS_EVT_DATA_INFO = 0x04,
    BLE_BS_EVT_LED_CONTRL = 0x05,
    BLE_BS_EVT_BEEP_CONTROL = 0x06,
    BLE_BS_EVT_MAJ_MIN_ID = 0x07,
    BLE_BS_EVT_SET_TIME = 0x08,
    BLE_BS_EVT_SET_TX_POWER = 0x09,
    BLE_BS_EVT_SOFTWARE_VER = 0x0a,
    BLE_BS_EVT_DEVICE_ID = 0x0b,//跟BLE_BS_EVT_SETDEVID 0x12功能重复
    BLE_BS_EVT_BAT_AND_MOTION = 0x0c,
    BLE_BS_EVT_BT_ADDR = 0x0d,
    BLE_BS_EVT_CLEAR_MOTION = 0x0e,
    BLE_BS_EVT_GSENSOR_CONTRL = 0x0f,
    BLE_BS_EVT_ADVLED_CONTROL = 0x10,
    BLE_BS_EVT_SETUUID = 0x11,
    BLE_BS_EVT_SETDEVID = 0x12,
    BLE_BS_EVT_ADV_INTVAL = 0x13,
    BLE_BS_EVT_SET_RSSI_1m = 0x14,
    BLE_BS_EVT_EXC_BCON_IBCON = 0x15,
    BLE_BS_EVT_CLEAR_EEPROM = 0x16,//临时测试用
    BLE_BS_EVT_UART_RESET = 0x17,
    BLE_BS_BC_AUTH = 0x18,
    BLE_BS_BC_OTA = 0x19,
    BLE_BS_CONNECTION_INDICATION = 0x1A,
    BLE_BS_MINOR_CHANGE = 0x1B,
    BLE_BS_WIFI_ONOFF = 0x1C,
    BLE_BS_WIFI_STATE = 0x1D,
    BLE_BS_WIFI_START = 0x1E,
    BLE_BS_WIFI_ADD_REFERENCE_TIME = 0x1F,
    BLE_BS_EVT_BRIGHTNESS = 0x20,
    BLE_BS_EVT_SET_BRIGHTNESS = 0x21,
    BLE_BS_EVT_ACTIVATION = 0x22,
    BLE_BS_EVT_TIME_PARAMETER = 0x23,

    BLE_BS_EVT_SIMULATE_BEACON = 0x25,
    BLE_BS_EVT_TIMESTAMP_TO_SIMULATEBEACON = 0x26,
    BLE_BS_EVT_ENCRYPTION_PASSWORD = 0x27,
    BEACON_EVT_NOTIFICATION_ENABLED = 0x28,
    BEACON_EVT_NOTIFICATION_DISABLED = 0x29,
    BEACON_EVT_ENTER_SCANNER_MODE_SWITCH = 0x2A,

    BEACON_EVT_BEACONLIST_ACK = 0x30,  
    BEACON_EVT_WIFI_SSID = 0x31,               
    BEACON_EVT_WIFI_PASSWORD = 0x32,
    BEACON_EVT_M2M_SERVER_ADDR = 0x33,               
    BEACON_EVT_RTLS_SERVER_ADDR = 0x34,

    BLE_BS_DATA_WITHOUT_RW = 0x7F,
    BLE_BS_EVT_READ = 0x80,
    BLE_BS_EVT_FORWARD = 0xD2,
    BLE_BS_EVT_BACKWARD = 0xD3,
    BLE_BS_EVT_ST_RETURN = 0xD4
} ble_beaconsetting_evt_type_t;

typedef struct
{
  uint16_t adv_intval;      //unit: 10ms.
  uint16_t data_update_int;  //unit: ms.
}ble_bs_adv_intval;

typedef struct
{
  uint16_t majoy_id;        //majoy ID.
  uint16_t minor_id;        //minor ID.
}ble_bs_majoy_minor;

typedef struct
{
  uint8_t Tx_Power;        //unit: (-1)*db.
}ble_bs_set_txpower;

typedef struct
{
  uint8_t led_enable;          //unit: bool.
  uint8_t led_freq_time;      //n senconds blink for a time.
}ble_bs_led_control;

typedef struct
{
  uint8_t btaddr[6];          //bluetooth address of the local device.
}ble_bs_btaddr;

typedef struct
{
  uint8_t motionScale;        //motion scale: 0~2 2 is the highest sensitivity.
  uint8_t motionFlag;        //0: no motion, 1: have motion.
}ble_bs_motion;

typedef struct
{
  uint8_t onoff;
  uint8_t resv;
}ble_bs_gsensor;

typedef struct
{
  uint8_t onoff;
  uint8_t freq;
}ble_bs_advled;

typedef struct
{
  uint8_t uuidEnFlag;
  uint8_t resvd;
  uint8_t uuid[16];
}ble_bs_uuid;

typedef struct
{
  uint8_t  deviceID[8];
}ble_bs_devid;

typedef struct
{
  uint8_t rssi1m;
  uint8_t resvd;
}ble_bs_set1mrssi;

typedef struct
{
  uint8_t actFlag;        //activation flag, 0x55 means activited.
}ble_bs_activation;

typedef enum
{
   iBeacon = 1,
   Android_Beacon  = 2
}ble_beaconsetting_beacon_type_t;
typedef struct//定义虚拟beacon的UUID和major，minor类型
{
  ble_beaconsetting_beacon_type_t  beacon_type;   //ble_beaconsetting_beacon_type_t
  uint8_t beacon_id[20];
}ble_bs_simulate_beacon;

typedef uint8_t ble_bs_timestamp_to_simulatebeacon;//定义时间戳用来存储需要获取的虚拟beacon库中的序号

typedef struct
{
  uint8_t  password[8];
}encryption_password_t; 
typedef encryption_password_t ble_bs_encryption_password;

typedef uint8_t               ble_bs_connection_indication;

typedef uint8_t               ble_bs_minor_change;

typedef struct
{
  uint8_t  onoff;
  uint8_t  frequency;
}wifi_onoff_t;
typedef wifi_onoff_t          ble_bs_wifi_onoff;

typedef uint8_t               ble_bs_wifi_config_result;

typedef uint8_t               ble_bs_wifi_add_reference_time;

typedef struct
{
  uint16_t majoy_ver;          //majoy version number.
  uint16_t minor_ver;          //minor version number.
  uint8_t device_type;        //device type:0x01(fixed for this kinds of device)
}ble_bs_software_version;

typedef struct
{
  uint8_t battery;          //battery left
  uint8_t motion;            //have motion or not, can be cleared by the host.
}ble_bs_bat_motion;

typedef struct
{
  uint8_t ota;
}ble_bs_ota;

/**@brief Beacon Service event. */
typedef struct
{
  ble_beaconsetting_evt_type_t           evt_type;                        /**< Type of event. */
  union
  {
    ble_bs_adv_intval                    m_adv_intval;
    ble_bs_led_control                   m_led_control;
    ble_bs_majoy_minor                   m_majoy_minor;
    ble_bs_set_txpower                   m_set_txpower;
    ble_bs_software_version              m_software_version;
    ble_bs_bat_motion                    m_bat_motion;
    ble_bs_btaddr                        m_btaddr;
    ble_bs_motion                        m_motion;
    ble_bs_gsensor                       m_gsensor;
    ble_bs_advled                        m_advled;
    ble_bs_uuid                          m_uuid;
    ble_bs_devid                         m_devid;
    ble_bs_set1mrssi                     m_set1mrssi;
    ble_bs_ota m_ota;
    ble_bs_activation                    m_activation;
    /******************************************/
    ble_bs_time                          beacon_time;
    ble_bs_encryption_password           encryption_password;
    ble_bs_connection_indication         connection_indication;
    ble_bs_minor_change                  minor_change;
    ble_bs_wifi_onoff                    wifi_onoff;
    uint8_t data[50];
  } params;
  uint8_t length;                                    
} ble_beaconSetting_evt_t;

typedef struct ble_beaconSetting_s ble_beaconSetting_t;

typedef void (*ble_beaconSetting_evt_handler_t) (ble_beaconSetting_t * p_beaconSetting, ble_beaconSetting_evt_t * p_evt);

typedef struct
{
    ble_beaconSetting_evt_handler_t evt_handler;                 
} ble_beaconSetting_init_t;

typedef struct ble_beaconSetting_s
{
    ble_beaconSetting_evt_handler_t     evt_handler;               
    uint16_t                            service_handle;            
    ble_gatts_char_handles_t            beaconSetting_handles;     
    uint16_t                            conn_handle;           
    /*************************************/
    ble_bs_adv_intval                    m_adv_intval;
    ble_bs_led_control                  m_led_control;
    ble_bs_majoy_minor                  m_majoy_minor;
    ble_bs_set_txpower                  m_set_txpower;
    ble_bs_software_version             m_software_version;
    ble_bs_bat_motion                   m_bat_motion;
    ble_bs_btaddr                       m_btaddr;
    ble_bs_motion                       m_motion;
    ble_bs_gsensor                      m_gsensor;
    ble_bs_advled                       m_advled;
    ble_bs_uuid                         m_uuid;
    ble_bs_devid                        m_devid;
    ble_bs_set1mrssi                    m_set1mrssi;
    ble_bs_ota                          m_ota;
    ble_bs_activation                   m_activation;
    /*************************************/
    ble_bs_simulate_beacon              simulate_beacon[MAX_SIMULATE_BEACON];                
    ble_bs_timestamp_to_simulatebeacon  timestamp_to_simulatebeacon[MAX_ADV_BEACON_ONE_CYCLE]; 
    /*************************************/
    ble_bs_encryption_password          encryption_password; 
    ble_bs_connection_indication        connection_indication;
    ble_bs_minor_change                 minor_change;
    ble_bs_wifi_onoff                   wifi_onoff;
} ble_beaconSetting_t;

uint32_t ble_beaconSetting_init(ble_beaconSetting_t * p_beaconSetting, const ble_beaconSetting_init_t * p_beaconSetting_init);

void ble_beaconSetting_on_ble_evt(ble_beaconSetting_t * p_beaconSetting, ble_evt_t * p_ble_evt);

uint32_t send_beaconSetting_resp_data(ble_beaconSetting_t * p_beaconSetting, uint16_t  length, uint8_t * p_value);


#endif


