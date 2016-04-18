/**
  ******************************************************************************
  * @file    Templates/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "stdbool.h"

//#define DEBUG_CAMERA
#define PRINTF_USE_USART2
#include "stm32f4_printf.h"
#include "stm32f4_nrf.h"
#include "stm32f4_sonar.h"
#include "stm32f4_uSD.h"
#include "vision_control.h"
#include "pwm.h"
#include "stm32f4_esp8266.h"
#include "websocket.h"

#define MAVLINK_BUFFER_SIZE 512
extern uint8_t mavlink_buffer[MAVLINK_BUFFER_SIZE];
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern UART_HandleTypeDef UartHandle, Uart1Handle, Uart2Handle;
extern bool Uart3Ready, Uart1Ready;
extern void uart1Init(), uart3Init();
extern TIM_HandleTypeDef  TimHandleSonar;
extern IWDG_HandleTypeDef IwdgHandle;

#define ESP8266
extern bool esp8266_exist, connected, handshaking;
#define YUV1
#define I2Cx I2C1

#define INPUT_PWM
#define UART1
#define UART3
volatile void soft_delay(volatile uint32_t n);
void postion_calc(void);
extern bool sending;
extern SPI_HandleTypeDef SpiHandle;
extern void HAL_NRF_MspInit(void);
void check_nrf(void);

#define SEND_SWITCH_PIN GPIO_PIN_2
#define SEND_SWITCH_PORT GPIOB
#define SEND_SWITCH_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
/*
#define EXTIx_PIN		GPIO_PIN_0
#define EXTIx_PORT  GPIOB
#define EXTIx_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
*/

#define DISABLE_IRQ HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn); HAL_NVIC_DisableIRQ(DCMI_IRQn); HAL_NVIC_DisableIRQ(TIMsonar_IRQn); 
#define ENABLE_IRQ  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);  HAL_NVIC_EnableIRQ(DCMI_IRQn);  HAL_NVIC_EnableIRQ(TIMsonar_IRQn); 
#define PRINTF(X) DISABLE_IRQ printf(X); ENABLE_IRQ

#define CAMERA_RESET_PORT GPIOE
#define CAMERA_RESET_PIN GPIO_PIN_3
#define CAMERA_RESET_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()

void HAL_CAMERA_MspInit(void);
extern void HAL_LED_MspInit(void);
extern void HAL_Button_MspInit(void);
#define LED_PORT GPIOB
#define LED1_PIN GPIO_PIN_9
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
/*
#define BUTTON_PORT GPIOI
#define BUTTON_PIN GPIO_PIN_11
#define BUTTON_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
*/

extern void Error_Handler(void);

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
