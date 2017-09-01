#ifndef OFO_TESTER_H
#define OFO_TESTER_H
#include "nrf_adc.h"

#define FUNCTION_TEST_INSTRUCTIONS_SET  0x54   //功能测试指令集
#define FUNCTION_TEST_INSTRUCTION_END_BYTE 0x00
#define ADC_SAMPLE_NUMBER               10

#define MOTOR_ROLL_A                    NRF_ADC_CONFIG_INPUT_6//P0.05//A高B低正转，A低B高反转
#define MOTOR_ROLL_B                    NRF_ADC_CONFIG_INPUT_7//P0.06
#define BUZZER_ADC_TEST_PIN             NRF_ADC_CONFIG_INPUT_5//P0.04
#define UART_PC_TX                      9
#define UART_PC_RX                      11
#define UART_HL300_TX                   24
#define UART_HL300_RX                   25
#define PROGRAM_RUN_INDICATE            21
#define CHECK_LED1_PIN                  18
#define CHECK_LED2_PIN                  19
#define OPEN_SW_PIN                     14
#define CLOSE_SW_PIN                    15
#define HL500_WAY                       0
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
  open_close_sw_check_schedule_transmit = 0,
  open_close_sw_check_schedule_waiting_result,
}open_close_sw_check_schedule_t;
typedef enum
{
  motor_voltage_check_schedule_transmit = 0,
  motor_voltage_check_schedule_waiting_result,
}motor_voltage_check_schedule_t;
typedef enum
{
  Test_instruction_check_hl300_pcba       = 0,
  Test_instruction_config_sn              = 1,
  Test_instruction_motor_before_pwron_test= 2,
  Test_instruction_SIM_VDD_test           = 3,
  Test_instruction_GSM_4V_test            = 4,
  Test_instruction_enable_gps_calculate   = 5,
  Test_instruction_pwroff_sim868          = 6,
  Test_instruction_monitor_ble            = 7,
  Test_instruction_motor_right            = 8,
  Test_instruction_motor_left             = 9,
  Test_instruction_open_sw                = 10,
  Test_instruction_close_sw               = 11,
  Test_instruction_charge_current         = 12,
  Test_instruction_VBUS_voltage           = 13,
  Test_instruction_VBAT_voltage           = 14,
  Test_instruction_VBAT_TEMP_voltage      = 15,
  Test_instruction_motor_after_pwron_test = 0x30,
  Test_instruction_inquiry_acc_id         = 0x31,
  Test_instruction_inquiry_acc_data       = 0x32,
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
  
  Test_instruction_inquiry_ble_mac             = 0xFF
}Test_instructions_t;
typedef struct
{
  bool   is_received_match_signal;
  int8_t rssi;
}ble_result_t;
#endif
