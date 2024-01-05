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
static char rev_Cmd = 0;

static enum waterflow_Status proStatus = WATERFLOW_UNKNOW;

static uint8_t rs485_addr = 0;
static float waterFlowHz = 0;
static float waterFlowPeriod = 0;
static uint32_t waterFlowCurrentAdc = 0;
static uint32_t calcTime = 0;
static float simple_max_ave = 0;
static float simple_min_ave = 0;
static uint32_t simple_cur_max = 0;
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
  HAL_StatusTypeDef rpy = HAL_UART_Transmit(&huart2, buff, len,0xff);
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
	
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{ 
}
void componeSendData()
{
  if (rev_Cmd == 0)
    return;
  rev_Cmd = 0;
  uint8_t buff[64] = {0};
  int sendLen = 0;

  buff[sendLen++] = rs485_addr; // address
  buff[sendLen++] = 0x03;       // cmd
  buff[sendLen++] = 10;         // byte count
  uint16_t temp = (uint16_t)(waterFlowHz * 100);
  memcpy(buff + sendLen, &temp, 2);
  sendLen += 2;

  temp = (uint16_t)(waterFlowPeriod * 100);
  memcpy(buff + sendLen, &temp, 2);
  sendLen += 2;

  temp = (uint16_t)simple_cur_max;
  memcpy(buff + sendLen, &temp, 2);
  sendLen += 2;

  temp = (uint16_t)simple_cur_min;
  memcpy(buff + sendLen, &temp, 2);
  sendLen += 2;

  temp = (uint16_t)(waterFlowCurrentAdc * 10);
  memcpy(buff + sendLen, &temp, 2);
  sendLen += 2;

  uint16_t crc = modbus_crc(buff, sendLen);
  memcpy(buff + sendLen, &crc, 2);
  sendLen += 2;

  Rs485SendData(buff, sendLen);
}

void Rs485RevData(uint8_t *buff, int32_t len)
{
  if ((buff[0] == rs485_addr) && (buff[1] == 0x03))
  {
    rev_Cmd = 1;
  }
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
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Init(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  //  init dma

  // init rs485
  createFifoRev(&huart2, Rs485RevData, UART_REV_DMA);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
  // init 485address
  rs485_addr |= HAL_GPIO_ReadPin(ADDR0_GPIO_Port, ADDR0_Pin) ? (1u << 0) : 0;
  rs485_addr |= HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin) ? (1u << 1) : 0;
  rs485_addr |= HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin) ? (1u << 2) : 0;
  rs485_addr |= HAL_GPIO_ReadPin(ADDR3_GPIO_Port, ADDR3_Pin) ? (1u << 3) : 0;
  rs485_addr++;
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

  enum waterflow_Status curstatus = WATERFLOW_UNKNOW;
  if (curLeval > MAX_ADC_LEVEL)
  {
    curstatus = WATERFLOW_HIGH;
    waterFlowHigh++;
    simple_cur_max += curLeval;
  }
  else if (curLeval < MIN_ADC_LEVEL)
  {
    waterFlowLow++;
    curstatus = WATERFLOW_LOW;
    simple_cur_min += curLeval;
  }

  if (proStatus == WATERFLOW_LOW && curstatus == WATERFLOW_HIGH)
  {
    // raise
    calcTime = 0;
    waterFlowCount = 0;
    waterFlowHz = 1000.0 / ((waterFlowHigh + waterFlowLow) / 100.0);
    waterFlowPeriod = (float)(waterFlowHigh) / (waterFlowHigh + waterFlowLow);

    simple_max_ave = (float)simple_cur_max / waterFlowHigh * 3300 / 4096 / 0.6;
    simple_min_ave = (float)simple_cur_min / waterFlowLow * 3300 / 4096 / 0.6;

    waterFlowHigh = 0;
    waterFlowLow = 0;
    simple_cur_max = 0;
    simple_cur_min = 0;

    if (waterFlowHz > MAX_HZ || waterFlowHz < MIN_HZ)
    {
      waterFlowPeriod = 0;
      waterFlowHz = 0;
      simple_max_ave = 0;
      simple_min_ave = 0;
    }
  }
  if (waterFlowCount++ - calcTime > 500 * 10)
  {
    waterFlowHz = 0.0;
    waterFlowPeriod = 0.0;
    simple_max_ave = 0;
    simple_min_ave = 0;
  }
  if (curstatus != WATERFLOW_UNKNOW)
    proStatus = curstatus;
}
void hal_tim_timer3_callback()
{
  // 10ms
  HAL_ADC_Start_IT(&hadc2);
}
static volatile int s_cov_flag = 0;
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    memcpy(calc_buff, adc_buffer + 0, sizeof(uint16_t) * 1000);
    s_cov_flag = 1;
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    memcpy(calc_buff, adc_buffer + 1000, sizeof(uint16_t) * 1000);
    s_cov_flag = 1;
  }
  if (hadc == &hadc2)
  {
    waterFlowCurrentAdc = (uint32_t)HAL_ADC_GetValue(&hadc2) * 3300 * 10 / 4096 / 42;
  }
}
uint32_t debug_log = 0;
uint32_t debug_period = 0;
void LoopWaterFlowComponet(void)
{
  static uint32_t period = 0;
  static uint32_t protime = 0;
  HAL_Delay(1);
  LoopLedStatus(); // 1ms
  if (debug_log == 1)
  {
    if (HAL_GetTick() - period >= debug_period)
    {
      period = HAL_GetTick();
      shellPrint(&shell, "hz = %.2f,period=%.2f,max = %.2f,min = %.2f,cur = %d\n", waterFlowHz, waterFlowPeriod, simple_max_ave, simple_min_ave, waterFlowCurrentAdc);
    }
  }
  if (s_cov_flag == 1)
  {
    s_cov_flag = 0;
    for (int i = 0; i < 1000; i++)
    {
      checkWaterFlow(calc_buff[i]);
    }
  }
  if(HAL_GetTick() - protime >= 1000)
  {
    protime = HAL_GetTick();
    rev_Cmd = 1;
  }
  componeSendData();
}
void setDebugLog(int on, int timeout)
{
  debug_log = on;
  debug_period = timeout;
}

SHELL_EXPORT_CMD(debuglog, setDebugLog, set DebugLog command line [0:1][ms]);
