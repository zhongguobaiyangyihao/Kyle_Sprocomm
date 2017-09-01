#include "stdint.h"
#include "hl300_tester.h"
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
/*1. 被测板在位检测*/
  const uint8_t hl300_is_present[] = "AT+HRDY\r\n";
/*2. 读取锁序列号测试*/
  const uint8_t inquiry_sn[] = "AT+HRDSN\r\n";  
/*3. 加速度计LIS3DH测试*/
  /*读ACC ID*/
  const uint8_t read_acc_id[] = "AT+HACCID\r\n";
  /*ACC DATA测试*/
  const uint8_t acc_data_test[] = "AT+HACCDATA\r\n";
/*4. GPS测试*/
  const uint8_t enable_gps_calculate[] = "AT+HGPSON\r\n";
  /*获取GPS位置数据*/
  const uint8_t inquiry_gps_position[] = "AT+HGPSINFO\r\n";
/*5. 获取SIM卡IMSI信息*/
  const uint8_t inquiry_sim_imsi[] = "AT+HIMSI\r\n";
/*6. 读写片上Flash测试*/
  const uint8_t rw_flash_test[] = "AT+HFLASH\r\n";
/*7. 读软件版本号测试*/
  const uint8_t inquiry_software_version[] = "AT+HVER\r\n";
/*8. LED测试*/
  /*板子上2个LED灯都亮*/
  const uint8_t led_turnon_test[] = "AT+HLEDON\r\n";
  /*板子上4个LED灯都灭*/
  const uint8_t led_turnoff_test[] = "AT+HLEDOFF\r\n";
/*9. 电机正转测试*/
  const uint8_t motor_right_test[] = "AT+HMOTOR\r\n";
/*10. 电机反转测试*/
  const uint8_t motor_left_test[] = "AT+HMOTOL\r\n";
/*11. OPEN开关测试*/
  const uint8_t open_sw_test[] = "AT+HOPENSW\r\n";
/*12. CLOSE开关测试*/
  const uint8_t close_sw_test[] = "AT+HCLOSESW\r\n";
/*13. 获取蓝牙MAC地址*/
  const uint8_t inquiry_ble_mac[] = "AT+HBTMAC\r\n";
/*14. 获取充电电流*/
  const uint8_t inquiry_charge_current[] = "AT+HCHARGE\r\n";
/*15. VBUS电压测试*/
  const uint8_t vbus_voltage_test[] = "AT+HVBUS\r\n";
/*16. VBAT电压测试*/
  const uint8_t vbat_voltage_test[] = "AT+HVBAT\r\n";
/*17. VBAT_TEMP电压测试*/
  const uint8_t vbat_temp_voltage_test[] = "AT+HVTEMP\r\n";
/*18. BUZ电压测试*/
  const uint8_t buz_voltage_test[] = "AT+HBUZ\r\n";
/*19. 使能睡眠模式*/
  const uint8_t enable_sleep_mode[] = "AT+ESLEEP\r\n";
/*20. 测试完成*/
  const uint8_t test_finish[] = "AT+HTESTFINISH\r\n";
/*返回结果*/
  const uint8_t OK[] = "OK\r\n";
  const uint8_t Error[] = "Error\r\n";
  const uint8_t Timeout[] = "Timeout\r\n";
  const uint8_t Ready[] = "Ready\r\n";
//  const uint8_t ofo_pcba_present[] = "ofoV1.100.9";
  const uint8_t ofo_pcba_present[] = "ofoV";
  const uint8_t GPS_Error[] = "GPS_Error\r\n";
uint16_t                                   communicate_time_cnt = 0;
extern Test_instructions_t                 Test_instruction;
extern Flag_t                              Flag;
extern uint8_t                             return_result_buff[];
extern uint16_t                            return_result_len;
extern uint8_t                             UART_RX_BUF[];
extern uint16_t                            UART_RX_BUF_CNT;
extern uint8_t                             config_sn[];
extern float                               sample_voltage;
extern float                               sample_max_voltage;
extern float                               sample_min_voltage;
extern float                               motor_roll_a_voltage;
extern float                               motor_roll_b_voltage;
extern uint8_t                             key_check_result;
extern uint8_t                             open_sw_check_result;
extern uint8_t                             close_sw_check_result;
extern uint8_t                             enable_gps_check_result;
extern uint8_t                             monitor_ble_addr[];
extern ble_result_t                        ble_result;
extern uint8_t                             return_value_printf[];
extern uint8_t                             return_value_cnt;
  
extern void error_reason_clear(void);
extern void error_reason_storage(uint8_t *buff,uint8_t len);
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
与HL300通讯协议解析
************************************************/
void communicate_with_hl300_handler(void)
{
  static communicate_schedule_t communicate_schedule = communicate_schedule_transmit;
  static bool                   is_config_sent = false;
  static uint8_t                delay_cnt      = 0;
  char * pA;
  char * pB;
  switch(Test_instruction)
  {
    case Test_instruction_check_hl300_pcba:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)hl300_is_present, strlen((char *)hl300_is_present));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(1000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            if(strstr((char *)UART_RX_BUF,"READY"))
            {
              Test_instruction = Test_instruction_inquiry_ble_mac;
            }
          }
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(1000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(strstr((char *)UART_RX_BUF,"READY"))
            {
              Test_instruction = Test_instruction_inquiry_ble_mac;
              communicate_schedule = communicate_schedule_transmit;
            }
          }
          else
          {
            communicate_schedule = communicate_schedule_transmit;
          }
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_ble_mac:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)inquiry_ble_mac, strlen((char *)inquiry_ble_mac));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            if(strstr((char *)UART_RX_BUF,"BTMAC : "))
            {
              memcpy(monitor_ble_addr,&UART_RX_BUF[8],6);
              scan_start();
              memcpy(return_result_buff,Ready,strlen((char *)Ready));
              return_result_len = strlen((char *)Ready);
              Flag.is_return_result_to_pc = true;
              Flag.is_uart_sw_to_pc = true;
            }
          }
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"BTMAC : ");
            if((pA != NULL)&&(*(pA+14) == '\r'))
            {
              memcpy(monitor_ble_addr,(pA+8),6);
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
            Test_instruction = Test_instruction_check_hl300_pcba;
            communicate_schedule = communicate_schedule_transmit;
          }
          #endif
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
          UART_Data_Send((uint8_t *)inquiry_sn, strlen((char *)inquiry_sn));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"SN : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"SN : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          error_reason_clear();
          UART_Data_Send((uint8_t *)test_finish, strlen((char *)test_finish));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"OK");
            if(pA != NULL)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"OK");
            if(pA != NULL)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              break;
            }
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
        return_result_len += sprintf((char *)&return_result_buff[return_result_len],"Rssi:%d\r\n",ble_result.rssi);
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
    case Test_instruction_motor_right:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          if(!is_config_sent)
          {
            UART_Data_Send((uint8_t *)motor_right_test, strlen((char *)motor_right_test));
            is_config_sent = true;
          }
          else
          {
            delay_cnt++;
            if(delay_cnt>=(400/CHECK_TIME_TIMEOUT_INTERVAL))
            {
              delay_cnt = 0;
              is_config_sent = false;
              Flag.is_motor_voltage_check = true;
              Flag.is_communicate_start_timeout_calculate = true;
              communicate_time_cnt = 0;
              communicate_schedule = communicate_schedule_waiting_result;
            }
          }
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_motor_voltage_check_finish)&&(communicate_time_cnt<=(4000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_motor_voltage_check_finish)
          {
            Flag.is_motor_voltage_check_finish = false;
            memcpy(return_result_buff,"MOTORA : ",strlen("MOTORA : "));
            return_result_len = strlen("MOTORA : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV,",(uint32_t)motor_roll_a_voltage);
            memcpy(&return_result_buff[return_result_len],"MOTORB : ",strlen("MOTORB : "));
            return_result_len += strlen("MOTORB : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)motor_roll_b_voltage);
          }
          else
          {
            memcpy(return_result_buff,Error,strlen((char *)Error));
            return_result_len = strlen((char *)Error);
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_motor_left:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          if(!is_config_sent)
          {
            UART_Data_Send((uint8_t *)motor_left_test, strlen((char *)motor_left_test));
            is_config_sent = true;
          }
          else
          {
            delay_cnt++;
            if(delay_cnt>=(400/CHECK_TIME_TIMEOUT_INTERVAL))
            {
              delay_cnt = 0;
              is_config_sent = false;
              Flag.is_motor_voltage_check = true;
              Flag.is_communicate_start_timeout_calculate = true;
              communicate_time_cnt = 0;
              communicate_schedule = communicate_schedule_waiting_result;
            }
          }
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_motor_voltage_check_finish)&&(communicate_time_cnt<=(4000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_motor_voltage_check_finish)
          {
            Flag.is_motor_voltage_check_finish = false;
            memcpy(return_result_buff,"MOTORA : ",strlen("MOTORA : "));
            return_result_len = strlen("MOTORA : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV,",(uint32_t)motor_roll_a_voltage);
            memcpy(&return_result_buff[return_result_len],"MOTORB : ",strlen("MOTORB : "));
            return_result_len += strlen("MOTORB : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)motor_roll_b_voltage);
          }
          else
          {
            memcpy(return_result_buff,Error,strlen((char *)Error));
            return_result_len = strlen((char *)Error);
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_open_sw:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          Flag.is_open_sw_check = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_open_sw_check_finish)&&(communicate_time_cnt<=(4000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_open_sw_check_finish)
          {
            Flag.is_open_sw_check_finish = false;
            if(open_sw_check_result == 0)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          open_sw_check_result = 0;
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_close_sw:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          Flag.is_close_sw_check = true;
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if((!Flag.is_close_sw_check_finish)&&(communicate_time_cnt<=(4000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_close_sw_check_finish)
          {
            Flag.is_close_sw_check_finish = false;
            if(close_sw_check_result == 0)
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          close_sw_check_result = 0;
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_charge_current:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)inquiry_charge_current, strlen((char *)inquiry_charge_current));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"CHARGE : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"CHARGE : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_VBUS_voltage:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)vbus_voltage_test, strlen((char *)vbus_voltage_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"VBUS : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"VBUS : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_VBAT_voltage:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)vbat_voltage_test, strlen((char *)vbat_voltage_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"VBAT : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"VBAT : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_VBAT_TEMP_voltage:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)vbat_temp_voltage_test, strlen((char *)vbat_temp_voltage_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"VBAT_TEMP : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"VBAT_TEMP : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_acc_id:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)read_acc_id, strlen((char *)read_acc_id));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,"ACC_ID : OK\r\n",strlen("ACC_ID : OK\r\n"));
              return_result_len = strlen("ACC_ID : OK\r\n");
            }
            else
            {
              memcpy(return_result_buff,"ACC_ID : Error\r\n",strlen("ACC_ID : Error\r\n"));
              return_result_len = strlen("ACC_ID : Error\r\n");
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,"ACC_ID : OK\r\n",strlen("ACC_ID : OK\r\n"));
              return_result_len = strlen("ACC_ID : OK\r\n");
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              memcpy(return_result_buff,"ACC_ID : Error\r\n",strlen("ACC_ID : Error\r\n"));
              return_result_len = strlen("ACC_ID : Error\r\n");
            }
            else
            {
              break;
            }
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
          break;
        }
      }
      break;
    }
    case Test_instruction_inquiry_acc_data:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)acc_data_test, strlen((char *)acc_data_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"3D : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"3D : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          UART_Data_Send((uint8_t *)rw_flash_test, strlen((char *)rw_flash_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,"CMD_Flash_RW : OK\r\n",strlen("CMD_Flash_RW : OK\r\n"));
              return_result_len = strlen("CMD_Flash_RW : OK\r\n");
            }
            else
            {
              memcpy(return_result_buff,"CMD_Flash_RW : Error\r\n",strlen("CMD_Flash_RW : Error\r\n"));
              return_result_len = strlen("CMD_Flash_RW : Error\r\n");
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,"CMD_Flash_RW : OK\r\n",strlen("CMD_Flash_RW : OK\r\n"));
              return_result_len = strlen("CMD_Flash_RW : OK\r\n");
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              memcpy(return_result_buff,"CMD_Flash_RW : Error\r\n",strlen("CMD_Flash_RW : Error\r\n"));
              return_result_len = strlen("CMD_Flash_RW : Error\r\n");
            }
            else
            {
              break;
            }
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          UART_Data_Send((uint8_t *)inquiry_software_version, strlen((char *)inquiry_software_version));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"VER : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"Version : ",strlen("Version : "));
                return_result_len = strlen("Version : ");
                memcpy(&return_result_buff[return_result_len],pA+6,(pB-(pA+6)+2));
                return_result_len += (pB-(pA+6)+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"VER : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,"Version : ",strlen("Version : "));
                return_result_len = strlen("Version : ");
                memcpy(&return_result_buff[return_result_len],pA+6,(pB-(pA+6)+2));
                return_result_len += (pB-(pA+6)+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          error_reason_clear();
          UART_Data_Send((uint8_t *)led_turnon_test, strlen((char *)led_turnon_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
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
      }
      break;
    }
    case Test_instruction_led_turnoff_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          UART_Data_Send((uint8_t *)led_turnoff_test, strlen((char *)led_turnoff_test));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
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
      }
      break;
    }
    case Test_instruction_buzzer_test:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          if(!is_config_sent)
          {
            UART_Data_Send((uint8_t *)buz_voltage_test, strlen((char *)buz_voltage_test));
            is_config_sent = true;
          }
          else
          {
            delay_cnt++;
            if(delay_cnt>=(400/CHECK_TIME_TIMEOUT_INTERVAL))
            {
              delay_cnt = 0;
              is_config_sent = false;
              adc_config(BUZZER_ADC_TEST_PIN);
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
          if((!Flag.is_sample_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_sample_finish)
          {
            Flag.is_sample_finish = false;
            memcpy(return_result_buff,"BUZ : ",strlen("BUZ : "));
            return_result_len = strlen("BUZ : ");
            return_result_len += sprintf((char *)&return_result_buff[return_result_len],"%dmV\r\n",(uint32_t)sample_max_voltage);
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          adc_disable();
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          break;
        }
      }
      break;
    }
    case Test_instruction_enable_gps_calculate:
    {
      switch(communicate_schedule)
      {
        case communicate_schedule_transmit:
        {
          error_reason_clear();
          UART_Data_Send((uint8_t *)enable_gps_calculate, strlen((char *)enable_gps_calculate));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            if(strstr((char *)UART_RX_BUF,"OK"))
            {
              memcpy(return_result_buff,OK,strlen((char *)OK));
              return_result_len = strlen((char *)OK);
            }
            else if(strstr((char *)UART_RX_BUF,"ERROR"))
            {
              memcpy(return_result_buff,Error,strlen((char *)Error));
              return_result_len = strlen((char *)Error);
            }
            else
            {
              break;
            }
          }
          else
          {
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          UART_Data_Send((uint8_t *)inquiry_gps_position, strlen((char *)inquiry_gps_position));
          Flag.is_communicate_start_timeout_calculate = true;
          communicate_time_cnt = 0;
          communicate_schedule = communicate_schedule_waiting_result;
          break;
        }
        case communicate_schedule_waiting_result:
        {
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"GPSINFO : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"GPSINFO : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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
          #if HL500_WAY
          if((!Flag.is_uart_receive_finish)&&(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL)))
          {
            break;
          }
          if(Flag.is_uart_receive_finish)
          {
            Flag.is_uart_receive_finish = false;
            pA = strstr((char *)UART_RX_BUF,"IMSI : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
              }
              else
              {
                error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
                error_reason_upload_packetage();
              }
            }
            else
            {
              error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
              error_reason_upload_packetage();
            }
          }
          else
          {
            error_reason_storage((uint8_t *)"Timeout",strlen("Timeout"));
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #else
          Flag.is_uart_receive_finish = false;
          if(communicate_time_cnt<=(2000/CHECK_TIME_TIMEOUT_INTERVAL))
          {
            pA = strstr((char *)UART_RX_BUF,"IMSI : ");
            if(pA != NULL)
            {
              pB = strchr(pA, '\r');
              if(pB != NULL)
              {
                memcpy(return_result_buff,pA,(pB-pA+2));
                return_result_len = (pB-pA+2);
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
            error_reason_storage(UART_RX_BUF,UART_RX_BUF_CNT);
            error_reason_upload_packetage();
          }
          Flag.is_return_result_to_pc = true;
          Flag.is_uart_sw_to_pc = true;
          communicate_schedule = communicate_schedule_transmit;
          #endif
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

