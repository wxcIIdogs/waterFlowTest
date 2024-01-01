#ifndef __WATERFLOW__H
#define __WATERFLOW__H

#include "main.h"
typedef enum
{
  LED_NARMAL = 0,   // 1000ms
  LED_485ERROR,     // 500ms
  LED_WATERWARNING, // 100ms
  LED_WATERERROR,   // value = 1
} enum_LED_STATUS;

enum waterflow_Status
{
  WATERFLOW_UNKNOW = 0,
  WATERFLOW_HIGH,
  WATERFLOW_LOW,
  WATERFLOW_TIGGLE,

};
extern void initWaterFlowComponet(void);
extern void LoopWaterFlowComponet(void);
extern void setLedStatus(enum_LED_STATUS status);
extern void hal_tim_timer3_callback(void);

#define MAX_ADC_LEVEL 1900
#define MIN_ADC_LEVEL 100
#define MAX_HZ   200
#define MIN_HZ   1
#define SENSOR_CURRENT_MAX  15
#endif
