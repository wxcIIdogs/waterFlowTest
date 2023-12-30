#include "waterFlow.h"
#include "shellUsart.h"
#include "adc.h"
#include "revDataInfo.h"
#include "usart.h"
#include "tim.h"
#define BUFFER_SIZE 100 // 10ms
uint16_t adc_buffer[BUFFER_SIZE];

void Rs485SendData(uint8_t *buff, int32_t len)
{
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart2, (uint8_t *)&buff, len, 0xFF);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}

void Rs485RevData(uint8_t *buff, int32_t len)
{
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

void hal_tim_timer3_callback()
{
  LoopLedStatus(); // 1ms
}
void LoopWaterFlowComponet(void)
{

  HAL_Delay(10);
  // shellPrint(&shell, "adc0 = %d,adc1 = %d\n", adc_buffer[0], adc_buffer[1]);
}
