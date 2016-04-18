#ifndef __STM32F4_SONAR_H
#define __STM32F4_SONAR_H
#include "main.h"
/*Definition for sonar tigger Pins */
#define SONAR_TRIG_PIN			GPIO_PIN_4
#define SONAR_TRIG_PORT			GPIOB
#define SONAR_CLK_ENABLE    __HAL_RCC_GPIOB_CLK_ENABLE()

/* Definition for TIMsonar Pins */
#define ICx_PORT            GPIOB
#define ICx_PIN             GPIO_PIN_3
#define ICx_PORT_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_AF_TIMsonar                   GPIO_AF1_TIM2

/* Definition for TIMsonar clock resources */
#define TIMsonar                           TIM2
#define TIMsonar_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()
/* Definition for TIMsonar's NVIC */
#define TIMsonar_IRQn                      TIM2_IRQn
#define TIMsonar_IRQHandler                TIM2_IRQHandler

#define SONAR_FILTER 3
#define SONAR_STEP_LIMIT 200
extern TIM_HandleTypeDef  TimHandleSonar;
extern int h_average;
extern int sonar_height[SONAR_FILTER], h_i;
void sonar_init();
void sonar_trig();
void BSP_sonar_MspInit(void);
#endif
