#include "stm32f4xx_hal.h"

#define DCMI_DR_ADDRESS       0x50050028
#define FSMC_LCD_ADDRESS      0x68000000


void OV7725_Init(void);
void OV7725_Reset(void);
