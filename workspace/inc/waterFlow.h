#ifndef __WATERFLOW__H
#define __WATERFLOW__H

#include "main.h"
typedef enum
{
  LED_NARMAL = 0,   //1000ms
  LED_485ERROR,     //500ms
  LED_WATERWARNING, //100ms
  LED_WATERERROR,   // value = 1
}enum_LED_STATUS;
extern void initWaterFlowComponet(void);
extern void LoopWaterFlowComponet(void);
extern void setLedStatus(enum_LED_STATUS status);
extern void hal_tim_timer3_callback(void);



#endif
