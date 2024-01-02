#include "waterFlow.h"
#include "shellUsart.h"
#include "adc.h"
#include "revDataInfo.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include "stdio.h"
#define BUFFER_SIZE 2000 // 20ms
uint16_t adc_buffer[BUFFER_SIZE];

uint16_t calc_buff[1000];
static uint32_t rev_Cmd = 0;

static enum waterflow_Status proStatus = WATERFLOW_UNKNOW;

static float waterFlowHz = 0;
static float waterFlowPeriod = 0;

static uint32_t calcTime = 0;
static uint32_t simple_cur_max = 4096;
static uint32_t simple_cur_min = 0;
uint16_t modbus_crc(uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint16_t i, j;

  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001; // CRC-16多项式
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}
void Rs485SendData(uint8_t *buff, int32_t len)
{
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart2, (uint8_t *)&buff, len, 0xFF);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}
void componeSendData()
{
  if (rev_Cmd != 1)
    return;
  uint8_t buff[100] = {0};
  int sendLen = 0;
  buff[sendLen++] = 0x03; // address
  buff[sendLen++] = 0x01; // cmd
  uint32_t temp = waterFlowHz * 1000;
  memcpy(buff + sendLen, &temp, 4);
  sendLen += 4;
  temp = waterFlowPeriod * 1000;
  memcpy(buff + sendLen, &temp, 4);
  sendLen += 4;
  memcpy(buff + sendLen, &simple_cur_max, 4);
  sendLen += 4;
  memcpy(buff + sendLen, &simple_cur_min, 4);
  sendLen += 4;
  uint16_t crc = modbus_crc(buff, sendLen);
  memcpy(buff + sendLen, &crc, 2);
  sendLen += 2;
  Rs485SendData(buff, sendLen);
}

void Rs485RevData(uint8_t *buff, int32_t len)
{
  if (buff[0] == 0x03 && buff[1] == 0x01)
    rev_Cmd = 1;
}

void initWaterFlowComponet(void)
{
  // init led
  setLedStatus(LED_NARMAL);
  // init shell
  initShellusart();
  // shell.read = shellRead;
  shell.write = shellWriteDebug;
  shellInit(&shell);

  // init adc
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFER_SIZE);
  // init timer
  // HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Init(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  //init dma 
  
  // init rs485
  createFifoRev(&huart2, Rs485RevData, UART_REV_DMA);
}

static uint32_t led_difftime = 0;

void setLedStatus(enum_LED_STATUS status)
{

  switch (status)
  {
  case LED_NARMAL:
    led_difftime = 1000;
    break;
  case LED_485ERROR:
    led_difftime = 500;
    break;
  case LED_WATERWARNING:
    led_difftime = 100;
    break;
  case LED_WATERERROR:
    led_difftime = 0;
    HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_SET);
    return;
    break;
  }
}
void LoopLedStatus()
{
  static uint32_t led_proTime = 0;
  if (led_proTime++ > led_difftime && led_difftime != 0)
  {
    led_proTime = 0;
    HAL_GPIO_TogglePin(LED_STAT_GPIO_Port, LED_STAT_Pin);
  }
}

void checkWaterFlow(uint16_t curLeval)
{
  static int32_t waterFlowHigh = 0;
  static int32_t waterFlowLow = 0;
  static int32_t waterFlowMax = 0;
  static int32_t waterFlowMin = 0;
  static int32_t waterFlowCount = 0;
  // uint16_t curLeval = adc_buffer[0];
  // uint16_t current = adc_buffer[1];

  enum waterflow_Status curstatus = WATERFLOW_UNKNOW;
  if (curLeval > MAX_ADC_LEVEL)
  {
    curstatus = WATERFLOW_HIGH;
    waterFlowHigh++;
    if (simple_cur_max > curLeval)
    {
      simple_cur_max = curLeval;
    }
  }
  else if (curLeval < MIN_ADC_LEVEL)
  {
    waterFlowLow++;
    curstatus = WATERFLOW_LOW;
    if (simple_cur_min < curLeval)
      simple_cur_min = curLeval;
  }
  if (waterFlowCount ++ - calcTime > 500*1000)
  {
    waterFlowHz = 0.0;
    waterFlowPeriod = 0.0;
  }
  if (proStatus == WATERFLOW_LOW && curstatus == WATERFLOW_HIGH)
  {
    // raise
    calcTime = 0;
    waterFlowCount = 0;
    waterFlowHz = 1000.0 / ((waterFlowHigh + waterFlowLow) / 100.0);
    waterFlowPeriod = (float)(waterFlowHigh) / (waterFlowHigh + waterFlowLow);

    waterFlowHigh = 0;
    waterFlowLow = 0;
    simple_cur_max = 4096;
    // simple_cur_min = 0;
    //shellPrint(&shell, "hz = %.2f,period=%.2f,max = %d,min = %d\n", waterFlowHz, waterFlowPeriod, simple_cur_max, simple_cur_min);
    if (waterFlowHz > MAX_HZ || waterFlowHz < MIN_HZ)
    {
      waterFlowPeriod = 0;
      waterFlowHz = 0;
    }
  }
  if (curstatus != WATERFLOW_UNKNOW)
    proStatus = curstatus;
}
void hal_tim_timer3_callback()
{
  // 10us
  //checkWaterFlow();
}
static volatile int s_cov_flag = 0;
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // shellPrint(&shell, "halt\n");
//  for(int i= 0 ; i < 1000 ; i++ )
//    checkWaterFlow(adc_buffer[i]);
	memcpy(calc_buff,adc_buffer + 0 ,1000);
	s_cov_flag = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // shellPrint(&shell, "full\n");
//  for(int i= 1000 ; i < 2000 ; i++ )
//    checkWaterFlow(adc_buffer[i]);
	memcpy(calc_buff,adc_buffer + 1000 ,1000);
	s_cov_flag = 1;
}
uint32_t debug_log = 0;
uint32_t debug_period = 0;
void LoopWaterFlowComponet(void)
{
  static uint32_t period = 0;
  HAL_Delay(1);
  LoopLedStatus(); // 1ms
  if (debug_log == 1)
  {
    if (period++ >= debug_period)
    {
      period = 0;
      shellPrint(&shell, "hz = %.2f,period=%.2f,max = %d,min = %d\n", waterFlowHz, waterFlowPeriod, simple_cur_max, simple_cur_min);
    }
  }
	if(s_cov_flag == 1)
	{
		s_cov_flag = 0;
		for(int i= 0 ; i < 1000 ; i++ )
			checkWaterFlow(calc_buff[i]);
	}

  componeSendData();
}
void setDebugLog(int on, int timeout)
{
  debug_log = on;
  debug_period = timeout;
}

SHELL_EXPORT_CMD(debuglog, setDebugLog, set DebugLog command line [0:1][ms]);
