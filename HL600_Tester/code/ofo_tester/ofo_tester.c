#include "stdint.h"
#include "ofo_tester.h"
#include "string.h"
#include "beaconSetting.h"
#include "stdbool.h"
#include "nrf_adc.h"
#include "stdio.h"
#include "stdlib.h"
#include "nrf_gpio.h"

extern void UART_Data_Send(uint8_t * p_data, uint16_t length);
extern void adc_config(nrf_adc_config_input_t config_input);
extern void adc_disable(void);
extern uint8_t stringToHex(uint8_t *strIn, uint8_t strLen, uint8_t *hexOut);
extern void scan_start(void);
/***************************************************************************
相关命令
***************************************************************************/
/*设备连接*/
  const uint8_t device_connection[] = "#0123456789012345";
/*1. 电机测试*/
  const uint8_t motor_test[] = {0x54,(uint8_t)Test_instruction_motor_after_pwron_test,0x00};
  
/*2. 加速度计LIS3DH测试*/
  /*读lis3dh ID*/
  const uint8_t read_lis3dh_id[] = {0x54,(uint8_t)Test_instruction_inquiry_lis3dh_id,0x00};
  /*lis3dh数据测试*/
  const uint8_t lis3dh_data_test[] = {0x54,(uint8_t)Test_instruction_inquiry_lis3dh_data,0x00};
  
/*3. GSM模块测试*/
  /*GSM模块上电*/
  const uint8_t pwron_gsm_module[] = {0x54,0x33,0x00};
  const uint8_t pwroff_gsm_module[] = {0x54,0x34,0x00};
  /*获取GSM信号强度*/
  const uint8_t inquiry_gsm_signal_strength[] = "AT+CSQ\r\n";
  /*获取GPS位置数据*/
  const uint8_t inquiry_gps_is_enable[] = "AT+CGNSPWR?\r\n";
  const uint8_t enable_gps_calculate[] = "AT+CGNSPWR=1\r\n";
  const uint8_t disable_gps_calculate[] = "AT+CGNSPWR=0\r\n";
  const uint8_t inquiry_gps_position[] = {0x54,0x38,0x00};
  /*获取SIM卡IMSI信息*/
  const uint8_t inquiry_sim_imsi[] = "AT+CIMI\r\n";
  const uint8_t inquiry_sim_imei[] = "AT+CGSN\r\n";
  
/*4. 读写片上Flash测试*/
  const uint8_t rw_flash_test[] = {0x54,(uint8_t)Test_instruction_rw_flash_test,0x00};
  
/*5. 读软件版本号测试*/
  const uint8_t inquiry_software_version[] = {0x54,(uint8_t)Test_instruction_inquiry_software_version,0x00};
  
/*6. 按键测试*/  
  const uint8_t key_test[] = {0x54,(uint8_t)Test_instruction_key_test,0x00};
  
/*7. LED测试*/
  /*板子上4个LED灯都亮*/
  const uint8_t led_turnon_test[] = {0x54,(uint8_t)Test_instruction_led_turnon_test,0x00};
  /*板子上4个LED灯都灭*/
  const uint8_t led_turnoff_test[] = {0x54,(uint8_t)Test_instruction_led_turnon_test,0x00};
  
/*8. Buzzer测试*/
  const uint8_t buzzer_test[] = {0x54,(uint8_t)Test_instruction_buzzer_test,0x00};
  
/*9. 开锁状态测试*/
  const uint8_t unlock_state_test[] = {0x54,(uint8_t)Test_instruction_unlock_state_test,0x00};
  
/*10. 闭锁状态测试*/
  const uint8_t lock_state_test[] = {0x54,(uint8_t)Test_instruction_lock_state_test,0x00};
  
/*11. 电源电压测试*/
  const uint8_t power_voltage_test[] = {0x54,(uint8_t)Test_instruction_VBAT_test,0x00};
  
/*12. 读取锁序列号测试*/
  const uint8_t inquiry_sn[] = {0x54,(uint8_t)Test_instruction_inquiry_sn,0x00};
  
/*13. 进入低功耗测试电流*/
  const uint8_t enter_low_power_state[] = {0x54,(uint8_t)Test_instruction_enter_lpm,0x00};
/*14. 获取蓝牙MAC地址*/
  const uint8_t inquiry_ble_mac[] = {0x54,(uint8_t)Test_instruction_inquiry_ble_mac,0x00};
/*返回结果*/
  const uint8_t OK[] = "OK\r\n";
  const uint8_t Error[] = "Error\r\n";
  const uint8_t Timeout[] = "Timeout\r\n";
  const uint8_t Ready[] = "Ready\r\n";
//  const uint8_t ofo_pcba_present[] = "ofoV1.100.9";
  const uint8_t ofo_pcba_present[] = "Enter Factory Mode.";
  const uint8_t GPS_Error[] = "GPS_Error\r\n";
  const uint8_t check_key[KEY_TOTAL_NUMBER] = {CHECK_KEY1_PIN,CHECK_KEY2_PIN,CHECK_KEY3_PIN, \
                                               CHECK_KEY4_PIN,CHECK_KEY5_PIN
                                              };
  const uint8_t check_led[LED_TOTAL_NUMBER] = {CHECK_LED1_PIN,CHECK_LED2_PIN};
  
uint16_t                                   communicate_time_cnt = 0;
extern Test_instructions_t                 Test_instruction;
extern Flag_t                              Flag;
extern uint8_t                             return_result_buff[];
extern uint16_t                            return_result_len;
extern uint8_t                             UART_RX_BUF[];
extern uint16_t                            UART_RX_BUF_CNT;
extern uint8_t                             config_sn[];
extern float                               sample_voltage;
extern uint8_t                             key_check_result;
extern uint8_t                             unlock_check_result;
extern uint8_t                             lock_check_result;
extern uint8_t                             enable_gps_check_result;
extern uint8_t                             monitor_ble_addr[];
extern ble_result_t                        ble_result;
extern uint8_t                             return_value_printf[];
extern uint8_t                             return_value_cnt;
  
extern void error_reason_clear(void);
extern void error_reason_storage(uint8_t *buff,uint8_t len);
extern uint8_t stringToHex(uint8_t *strIn, uint8_t strLen, uint8_t *hexOut);
/******************************************************************
错误原因上传组包
******************************************************************/
void error_reason_upload_packetage(void)
{
  memcpy(return_result_buff,"Error:",strlen("Error:"));
  return_result_len = strlen("Error:");
  memcpy(&return_result_buff[return_result_len],return_value_printf,return_value_cnt);
  return_result_len += return_value_cnt;
  return_result_buff[return_result_len++] = 0x0D;
  return_result_buff[return_result_len++] = 0x0A;
}
/************************************************
与NRF51822通讯协议解析
************************************************/
void communicate_with_nrf51822_handler(void)
{
  static communicate_schedule_t communicate_schedule = communicate_schedule_transmit;
  uint8_t                       config_params_buff[20];
  static bool                   is_config_sent = false;
  static uint8_t                delay_cnt      = 0;
  char * pA;
  char * pB;
  switch(Test_instruction)
  {
    case Test_instruction_check_ofo_pcba:
    {
      #if 0
      if(Flag.is_uart_receive_finish)
      {
        Flag.is_uart_receive_finish = false;
        if(!strncmp((const char *)UART_RX_BUF,(const char *)ofo_pcba_present,strlen((char *)ofo_pcba_present)))
        {
          Test_instruction = Test_instruction_inquiry_ble_mac;
        }
        UART_RX_BUF_CNT = 0;
      }
      #else
      if(Flag.is_uart_receive_finish)
      {
        Flag.is_uart_receive_finish = false;
        pA = strstr((char *)UART_RX_BUF,"BLE MAC:");
        if((pA != NULL)&&(*(pA+26) == '\r'))
        {
          stringToHex((uint8_t *)(pA+8), 2, &monitor_ble_addr[5]);
          stringToHex((uint8_t *)(pA+11), 2, &monitor_ble_addr[4]);
          stringToHex((uint8_t *)(pA+14), 2, &monitor_ble_addr[3]);
          stringToHex((uint8_t *)(pA+17), 2, &monitor_ble_addr[2]);
          stringToHex((uint8_t *)(pA+20), 2, &monitor_ble_addr[1]);
          stringToHex((uint8_t *)(pA+23), 2, &monitor_ble_addr[0]);
          scan_start();
          memcpy(return_result_buff,Ready,strlen((char *)Ready));
          return_result_len = strlen((char *)Ready);
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
        }
        UART_RX_BUF_CNT = 0;
      }
      #endif
      break;
    }
    case Test_instruction_inquiry_ble_mac:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)inquiry_ble_mac, sizeof(inquiry_ble_mac));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"BLE MAC:");
            if((pA != NULL)&&(*(pA+26) == '\r'))
            {
              stringToHex((uint8_t *)(pA+8), 2, &monitor_ble_addr[5]);
              stringToHex((uint8_t *)(pA+11), 2, &monitor_ble_addr[4]);
              stringToHex((uint8_t *)(pA+14), 2, &monitor_ble_addr[3]);
              stringToHex((uint8_t *)(pA+17), 2, &monitor_ble_addr[2]);
              stringToHex((uint8_t *)(pA+20), 2, &monitor_ble_addr[1]);
              stringToHex((uint8_t *)(pA+23), 2, &monitor_ble_addr[0]);
              scan_start();
              memcpy(return_result_buff,Ready,strlen((char *)Ready));
              return_result_len = strlen((char *)Ready);
              Flag.is_return_result_to_pc = true;
              Flag.is_uart_sw_to_pc = true;
              communicate_schedule = communicate_schedule_transmit;
            }
          }
          else
          {
            Test_instruction = Test_instruction_check_ofo_pcba;
            communicate_schedule = communicate_schedule_transmit;
          }
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_config_sn:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          config_params_buff[0] = 0x54;config_params_buff[1] = 0x36;
          memcpy(&config_params_buff[2],config_sn,13);
          config_params_buff[15] = 0x55;config_params_buff[16] = 0x55;
          UART_Data_Send(config_params_buff, 17);
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          memcpy(return_result_buff,OK,strlen((char *)OK));
          return_result_len = strlen((char *)OK);
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_sn:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_sn, sizeof(inquiry_sn));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"SN: ");
            if(pA != NULL)
            {
              pA = pA+4;
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"SN : ",strlen("SN : "));
                return_result_len = strlen("SN : ");
                memcpy(&return_result_buff[return_result_len],pA,(pB-pA+1));
                return_result_len += (pB-pA+1);
              }
              else
              {
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_motor_before_pwron_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          adc_config(MOTOR_ADC_TEST_PIN);
          Flag.is_adc_sample = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"Motor_Before : ",strlen("Motor_Before : "));
            return_result_len = strlen("Motor_Before : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_voltage);
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_VMCU_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          adc_config(VMCU_ADC_TEST_PIN);
          Flag.is_adc_sample = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"VMCU : ",strlen("VMCU : "));
            return_result_len = strlen("VMCU : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_voltage);
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_VGSM_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          adc_config(VGSM_ADC_TEST_PIN);
          Flag.is_adc_sample = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"VGSM : ",strlen("VGSM : "));
            return_result_len = strlen("VGSM : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_voltage);
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_VGPS_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          adc_config(VGPS_ADC_TEST_PIN);
          Flag.is_adc_sample = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"VGPS : ",strlen("VGPS : "));
            return_result_len = strlen("VGPS : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_voltage);
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }      
    case Test_instruction_monitor_ble:
    {
      if(ble_result.is_received_match_signal)
      {
        memcpy(return_result_buff,"OK,",strlen("OK,"));
        return_result_len = strlen("OK,");
        return_result_len += sprintf((char *)&return_result_buff[return_result_len],"Rssi:%d,",ble_result.rssi);
        return_result_len += sprintf((char *)&return_result_buff[return_result_len],"Mac:");
        for(uint8_t i=6;i>0;i--)
        {
          return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%02X",monitor_ble_addr[i-1]);
        }
        return_result_len += sprintf((char *)&return_result_buff[return_result_len],"\r\n");
      }
      else
      {
        memcpy(return_result_buff,Error,strlen((char *)Error));
        return_result_len = strlen((char *)Error);
      }
      Flag.is_return_result_to_pc = true;
      Flag.is_uart_sw_to_pc = true;
      break;
    }
    case Test_instruction_pwron_GSM_module:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)pwron_gsm_module, sizeof(pwron_gsm_module));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            if(strstr((char *)UART_RX_BUF,"GSM enable"))
            {
              memcpy(return_result_buff,"OK\r\n",strlen("OK\r\n"));
              return_result_len = strlen("OK\r\n");
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_pwroff_GSM_module:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)pwroff_gsm_module, sizeof(pwroff_gsm_module));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            if(strstr((char *)UART_RX_BUF,"GSM diable"))
            {
              memcpy(return_result_buff,"OK\r\n",strlen("OK\r\n"));
              return_result_len = strlen("OK\r\n");
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }      
    case Test_instruction_motor_after_pwron_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          if(!is_config_sent)
          {
            UART_Data_Send((uint8_t *)motor_test, sizeof(motor_test));
            is_config_sent = true;
          }
          else
          {
            delay_cnt++;
            if(delay_cnt>=(400/CHECK_TIME_TIMEOUT_INTERVAL))
            {
              delay_cnt = 0;
              is_config_sent = false;
              adc_config(MOTOR_ADC_TEST_PIN);
              Flag.is_adc_sample = true;
              Flag.is_communicate_start_timeout_calculate = true;
              communicate_time_cnt = 0;
              communicate_schedule = communicate_schedule_waiting_result;
            }
          }
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"Motor_After : ",strlen("Motor_After : "));
            return_result_len = strlen("Motor_After : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_voltage);
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_lis3dh_id:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)read_lis3dh_id, sizeof(read_lis3dh_id));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(5000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,"CMD_3D_CHIPID : OK\r\n",strlen("CMD_3D_CHIPID : OK\r\n"));
              return_result_len = strlen("CMD_3D_CHIPID : OK\r\n");
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_lis3dh_data:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)lis3dh_data_test, sizeof(lis3dh_data_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"3D: ");
            if(pA != NULL)
            {
              pA = pA+4;
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"3D : ",strlen("3D : "));
                return_result_len = strlen("3D : ");
                memcpy(&return_result_buff[return_result_len],pA,(pB-pA+1));
                return_result_len += (pB-pA+1);
              }
              else
              {
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_gps_position:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_gps_position, sizeof(inquiry_gps_position));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          nrf_gpio_cfg_input(UART_CC2640_TX, NRF_GPIO_PIN_NOPULL);
          nrf_gpio_input_disconnect(UART_CC2640_TX);
          if(communicate_time_cnt<=(70000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(Flag.is_uart_receive_finish)
            {
              Flag.is_uart_receive_finish = false;
              pA = strstr((char *)UART_RX_BUF,"GPS Finish!\r\n\r\n");
              if(pA != NULL)
              {
                pA = pA+strlen("GPS Finish!\r\n\r\n");
                pB = strchr(pA, '\n');
                if(pB != NULL)
                {
                  memcpy(return_result_buff,"CGNSINF : ",strlen("CGNSINF : "));
                  return_result_len = strlen("CGNSINF : ");
                  memcpy(&return_result_buff[return_result_len],pA,(pB-pA+1));
                  return_result_len += (pB-pA+1);
                }
                else
                {
                  error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                  break;
                }
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                UART_RX_BUF_CNT = 0;
                memset(UART_RX_BUF,0,UART_RX_BUF_SIZE);
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_rw_flash_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)rw_flash_test, sizeof(rw_flash_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"CMD_Flash_RW : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+1));
                return_result_len = (pB-pA+1);
              }
              else
              {
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_software_version:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_software_version, sizeof(inquiry_software_version));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"Software Ver:");
            if(pA != NULL)
            {
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"Version : ",strlen("Version : "));
                return_result_len = strlen("Version : ");
                memcpy(&return_result_buff[return_result_len],pA+strlen("Software Ver:"),(pB-(pA+strlen("Software Ver:"))+1));
                return_result_len += (pB-(pA+strlen("Software Ver:"))+1);
              }
              else
              {
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_key_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          Flag.is_key_check = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_key_check_finish)&&(communicate_time_cnt<=(10000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_key_check_finish)
          {
            Flag.is_key_check_finish = false;
            if(key_check_result == 0)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_upload_packetage();
            }
            key_check_result = 0;
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_led_turnon_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)led_turnon_test, sizeof(led_turnon_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            if((!nrf_gpio_pin_read(CHECK_LED1_PIN))&&
               (!nrf_gpio_pin_read(CHECK_LED2_PIN))
              )
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
              Flag.is_return_result_to_pc = true;
              Flag.is_uart_sw_to_pc = true;
              communicate_schedule = communicate_schedule_transmit;
              break;
            }
          }
          else
          {
            memcpy(return_result_buff,Error,strlen((char *)Error));
            return_result_len = strlen((char *)Error);
            Flag.is_return_result_to_pc = true;
            Flag.is_uart_sw_to_pc = true;
            communicate_schedule = communicate_schedule_transmit;
          }
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_led_turnoff_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)led_turnoff_test, sizeof(led_turnoff_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            if((nrf_gpio_pin_read(CHECK_LED1_PIN))&&
               (nrf_gpio_pin_read(CHECK_LED2_PIN))
              )
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
              Flag.is_return_result_to_pc = true;
              Flag.is_uart_sw_to_pc = true;
              communicate_schedule = communicate_schedule_transmit;
              break;
            }
          }
          else
          {
            memcpy(return_result_buff,Error,strlen((char *)Error));
            return_result_len = strlen((char *)Error);
            Flag.is_return_result_to_pc = true;
            Flag.is_uart_sw_to_pc = true;
            communicate_schedule = communicate_schedule_transmit;
          }
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_buzzer_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          if(!nrf_gpio_pin_read(CHECK_BUZZER_PIN))
          {
            nrf_gpio_cfg_input(CHECK_BUZZER_PIN, NRF_GPIO_PIN_PULLUP);
            if(!nrf_gpio_pin_read(CHECK_BUZZER_PIN))
            {
              delay_cnt ++;
              if(delay_cnt>=COMMUNITCATE_TIMEOUT_TIME)
              {
                delay_cnt = 0;
                memcpy(return_result_buff,"Error:along buzz.\r\n",strlen("Error:along buzz.\r\n"));
                return_result_len = strlen("Error:along buzz.\r\n");
                Flag.is_return_result_to_pc = true;
                Flag.is_uart_sw_to_pc = true;
              }
              break;
            }
          }
          delay_cnt = 0;
          UART_Data_Send((uint8_t *)buzzer_test, sizeof(buzzer_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
             if(!nrf_gpio_pin_read(CHECK_BUZZER_PIN))//OK
             {
               memcpy(return_result_buff,OK,strlen((char *)OK));
               return_result_len = strlen((char *)OK);
               Flag.is_return_result_to_pc = true;
               Flag.is_uart_sw_to_pc = true;
               communicate_schedule = communicate_schedule_transmit;
               break;
             }
          }
          else
          {
            memcpy(return_result_buff,Error,strlen((char *)Error));
            return_result_len = strlen((char *)Error);
            Flag.is_return_result_to_pc = true;
            Flag.is_uart_sw_to_pc = true;
            communicate_schedule = communicate_schedule_transmit;
          }
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_unlock_state_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          Flag.is_unlock_check = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_unlock_check_finish)&&(communicate_time_cnt<=(6000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_unlock_check_finish)
          {
            Flag.is_unlock_check_finish = false;
            if(unlock_check_result == 0)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_upload_packetage();
            }
            unlock_check_result = 0;
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_lock_state_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          Flag.is_lock_check = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_lock_check_finish)&&(communicate_time_cnt<=(6000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_lock_check_finish)
          {
            Flag.is_lock_check_finish = false;
            if(lock_check_result == 0)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_upload_packetage();
            }
            lock_check_result = 0;
          }
          else
          {
            memcpy(return_result_buff,Timeout,strlen((char *)Timeout));
            return_result_len = strlen((char *)Timeout);
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_VBAT_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)power_voltage_test, sizeof(power_voltage_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"Voltage: ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');//这个指令的返回值在值和单位mV之间有空格例如：3596 mV\r\n
              if(pB != NULL)
              {
                memcpy(return_result_buff,"VBAT : ",strlen("VBAT : "));
                return_result_len = strlen("VBAT : ");
                memcpy(&return_result_buff[return_result_len],pA+strlen("Voltage: "),(pB-(pA+strlen("Voltage: "))-3));
                return_result_len += (pB-(pA+strlen("Voltage: "))-3);
                return_result_buff[return_result_len++] = 'm';
                return_result_buff[return_result_len++] = 'V';
                return_result_buff[return_result_len++] = 0x0D;
                return_result_buff[return_result_len++] = 0x0A;
              }
              else
              {
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_enter_lpm:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)enter_low_power_state, sizeof(enter_low_power_state));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          memcpy(return_result_buff,OK,strlen((char *)OK));
          return_result_len = strlen((char *)OK);
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    default:
    {
      Flag.is_uart_sw_to_pc = true;
      break;
    }
  }
}
/************************************************
与NRF51822通讯协议解析
************************************************/
void communicate_with_sim868_handler(void)
{
  char * pA;
  char * pB;
  static uint8_t execute_cnt = 0;
  static communicate_schedule_t communicate_schedule = communicate_schedule_transmit;
  switch(Test_instruction)
  {
    case Test_instruction_inquiry_GSM_imei:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_sim_imei, strlen((char *)inquiry_sim_imei));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"CGSN: ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"IMEI : ",strlen("IMEI : "));
                return_result_len = strlen("IMEI : ");
                memcpy(&return_result_buff[return_result_len],pA+strlen("CIMI: "),(pB-(pA+strlen("CIMI: "))+1));
                return_result_len += (pB-(pA+strlen("CGSN: "))+1);
              }
              else
              {
                break;
              }
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
              execute_cnt++;
              if(execute_cnt<=3)
              {
                communicate_schedule = communicate_schedule_transmit;
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          execute_cnt = 0;
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_gsm_signal_strength:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_gsm_signal_strength, strlen((char *)inquiry_gsm_signal_strength));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"+CSQ: ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"CSQ : ",strlen("CSQ : "));
                return_result_len = strlen("CSQ : ");
                memcpy(&return_result_buff[return_result_len],pA+strlen("+CSQ: "),(pB-(pA+strlen("+CSQ: "))+1));
                return_result_len += (pB-(pA+strlen("+CSQ: "))+1);
              }
              else
              {
                break;
              }
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
              execute_cnt++;
              if(execute_cnt<=3)
              {
                communicate_schedule = communicate_schedule_transmit;
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          execute_cnt = 0;
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_sim_imsi:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_sim_imsi, strlen((char *)inquiry_sim_imsi));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=COMMUNITCATE_TIMEOUT_TIME)
          {
            pA = strstr((char *)UART_RX_BUF,"CIMI: ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\n');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"IMSI : ",strlen("IMSI : "));
                return_result_len = strlen("IMSI : ");
                memcpy(&return_result_buff[return_result_len],pA+strlen("CIMI: "),(pB-(pA+strlen("CIMI: "))+1));
                return_result_len += (pB-(pA+strlen("CIMI: "))+1);
              }
              else
              {
                break;
              }
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
              execute_cnt++;
              if(execute_cnt<=3)
              {
                communicate_schedule = communicate_schedule_transmit;
                break;
              }
            }
            else
            {
              break;
            }
          }
          else
          {
            #if CESHI
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            #else
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            #endif
            error_reason_upload_packetage();
          }
          execute_cnt = 0;
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
        default:
        {
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    default:
    {
      Flag.is_uart_sw_to_pc = true;
      break;
    }
  }
}
/************************************************
监控nrf51822发送给sim868的数据解析
************************************************/
void communicate_with_monitor_nrf51822_handler(void)
{
  char * pA;
  char * pB;
  uint8_t ofo_mac[12] = {0};
  if(Flag.is_uart_receive_finish)
  {
    Flag.is_uart_receive_finish = false;
    if(strstr((char *)UART_RX_BUF,"ID"))
    {
      pA = strchr((char *)UART_RX_BUF, 'D');
      pB = pA+6;
      memcpy(ofo_mac,pB,sizeof(ofo_mac));
      stringToHex(ofo_mac, sizeof(ofo_mac), monitor_ble_addr);
      scan_start();
      Flag.is_uart_sw_to_nrf51822 = true;
    }
    memset(UART_RX_BUF,0,UART_RX_BUF_SIZE);
    UART_RX_BUF_CNT = 0;
  }
}
/*
nrf51822发给sim868的数据：
AT+EBIKENETCFG="URL","http://gsm.ofo.so/twx"
AT+SET="\"ID\":\"DAD9A03C5574\""
AT+SET="\"MCUVer\":\"ofoV1.100.9\""
AT+SET="\"tm\":\"126\""
AT+SET="\"Vot\":\"1995\""
AT+SET="\"3D\":\"0,0,0\""
AT+SET="\"ACTION\":\"2\""
AT+CGNSPWR=1
*/

