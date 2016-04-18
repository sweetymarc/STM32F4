#ifndef __PWM_H
#define __PWM_H
/* Definition for TIM_PWM clock resources */
#define TIM_PWM                           TIM4
#define TIM_PWM_CLK_ENABLE                __HAL_RCC_TIM4_CLK_ENABLE
#define TIM_PWM_GPIO_CLK_ENABLE						__HAL_RCC_GPIOD_CLK_ENABLE
#define TIM_PWM_IRQn                      TIM4_IRQn
#define TIM_PWM_IRQHandler								TIM4_IRQHandler
/* Definition for USARTx Pins */
#define TIM_PWM_GPIO_PORT							 GPIOD
#define GPIO_PIN_CHANNEL1              GPIO_PIN_12
#define GPIO_PIN_CHANNEL2              GPIO_PIN_13
#define GPIO_PIN_CHANNEL3              GPIO_PIN_14
#define GPIO_PIN_CHANNEL4              GPIO_PIN_15

extern TIM_HandleTypeDef TimHandle_PWM;
extern uint32_t rc_ch;
void TIM_PWM_init(void);
void rc_ch_CaptureCallback(TIM_HandleTypeDef *htim);
#endif
