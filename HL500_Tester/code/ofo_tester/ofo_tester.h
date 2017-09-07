#ifndef OFO_TESTER_H
#define OFO_TESTER_H
#include "nrf_adc.h"

#define FUNCTION_TEST_INSTRUCTIONS_SET  0x54   //功能测试指令集
#define FUNCTION_TEST_INSTRUCTION_END_BYTE 0x00
#define ADC_SAMPLE_NUMBER               5
#define MOTOR_ADC_TEST_PIN              NRF_ADC_CONFIG_INPUT_5//P0.04
#define SIM_VDD_ADC_TEST_PIN            NRF_ADC_CONFIG_INPUT_6
#define GSM_4V_ADC_TEST_PIN             NRF_ADC_CONFIG_INPUT_7

#define PROGRAM_RUN_INDICATE            21
#define CHECK_KEY1_PIN                  7
#define CHECK_KEY2_PIN                  12
#define CHECK_KEY3_PIN                  13
#define CHECK_KEY4_PIN                  14
#define CHECK_KEY5_PIN                  15
#define KEY_TOTAL_NUMBER                5
#define CHECK_LED1_PIN                  18
#define CHECK_LED2_PIN                  20
#define LED_TOTAL_NUMBER                2
#define CHECK_BUZZER_PIN                30
#define CHECK_UNLOCK_PIN                8
#define CHECK_LOCK_PIN                  10
typedef enum
{
  communicate_schedule_transmit = 0,
  communicate_schedule_waiting_result,
}communicate_schedule_t;
typedef enum
{
  adc_schedule_config = 0,
  adc_schedule_waiting_result,
}adc_schedule_t;
typedef enum
{
  key_check_schedule_press = 0,
  key_check_schedule_waiting_result,
}key_check_schedule_t;
typedef enum
{
  unlock_lock_check_schedule_transmit = 0,
  unlock_lock_check_schedule_waiting_result,
}unlock_lock_check_schedule_t;
typedef enum
{
  enable_gps_check_schedule_transmit = 0,
  enable_gps_check_schedule_waiting_result,
}enable_gps_check_schedule_t;
typedef enum
{
  Test_instruction_check_ofo_pcba = 0,
  Test_instruction_config_sn = 1,
  Test_instruction_motor_before_pwron_test= 2,
  Test_instruction_SIM_VDD_test           = 3,
  Test_instruction_GSM_4V_test            = 4,
  Test_instruction_enable_gps_calculate   = 5,
  Test_instruction_pwroff_sim868          = 6,
  Test_instruction_monitor_ble            = 7,
  Test_instruction_pwron_GSM_module       = 8,//HL600使用
  Test_instruction_inquiry_sim868_imei    = 9,
  Test_instruction_motor_after_pwron_test = 0x30,
  Test_instruction_inquiry_lis3dh_id   = 0x31,
  Test_instruction_inquiry_lis3dh_data = 0x32,
  Test_instruction_inquiry_gsm_signal_strength = 0x33,
  Test_instruction_inquiry_gps_position        = 0x34,
  Test_instruction_inquiry_sim_imsi            = 0x3F,
  
  Test_instruction_rw_flash_test               = 0x35,
  Test_instruction_inquiry_software_version    = 0x37,
  Test_instruction_key_test                    = 0x39,
  Test_instruction_led_turnon_test             = 0x3A,
  Test_instruction_led_turnoff_test            = 0x3B,
  Test_instruction_buzzer_test                 = 0x3C,
  Test_instruction_unlock_state_test           = 0x3D,
  Test_instruction_lock_state_test             = 0x3E,
  Test_instruction_VBAT_test                   = 0x40,
  Test_instruction_inquiry_sn                  = 0x41,
  Test_instruction_enter_lpm                   = 0x43,
  Test_instruction_inquiry_ble_mac             = 0x44
}Test_instructions_t;
typedef struct
{
  bool   is_received_match_signal;
  int8_t rssi;
}ble_result_t;
#endif
