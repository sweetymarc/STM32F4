#ifndef __STM32F429_SZW_H
#define __STM32F429_SZW_H

#include "main.h"

#include "stdbool.h"
#define CAMERA_RESET_PORT GPIOC
#define CAMERA_RESET_PIN GPIO_PIN_8

#define LCD_BACKLIGHT_PORT GPIOI
#define LCD_BACKLIGHT_PIN GPIO_PIN_2
#define LCD_RESET_PORT GPIOC
#define LCD_RESET_PIN GPIO_PIN_7

void HAL_CAMERA_MspInit(void);
extern void HAL_LED_MspInit(void);
extern void HAL_Button_MspInit(void);
#define LED_PORT GPIOF
#define LED1_PIN GPIO_PIN_7
#define LED2_PIN GPIO_PIN_8
#define LED3_PIN GPIO_PIN_9
#define BUTTON_PORT GPIOI
#define BUTTON_PIN GPIO_PIN_11
#define BUTTON_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

bool button_pressed(void);
bool button_pressing(void);
bool button_releasing(void);

void test_led(uint32_t t);

#define SRAM_BANK_ADDR                 ((uint32_t)0x68000000)
#define SRAM_TIMEOUT     ((uint32_t)0xFFFF) 
void sram512K_init(void);
bool test_sram512K(void);
#endif
