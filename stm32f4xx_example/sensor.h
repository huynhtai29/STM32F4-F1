/*
    sensor
    nguyen huynh tai
    date: 16/8/2019
*/
#ifndef sensor
#define sensor

#endif //sensor
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_it.h"
    void sensorInit(void);
    void EXTI_Handler(void);
    void TIM_Handler(void);
    int16_t GET_Sensor();
#ifdef __cplusplus
}
#endif
