/*--------------------------------------------------------------------------
// 文件名：Driver.c
// 描述：  MCU底层驱动  
// 设计者：EU电子
// 深圳EU电子出品-版权所有-翻版必究
// EU-热爱嵌入式开发
// http://euse.taobao.com
//-------------------------------------------------------------------------*/
/* 头文件包含 INCLUDES */
#include "main.h"
#include "stdio.h"
#include "stm32f429_szw.h"
/* type change */
typedef unsigned char       u8;
typedef unsigned short      u16;
#define  True  1
#define  False 0


/* INCLUDES */
//MCU
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_sram.h"
#include "stm32f4xx_ll_fmc.h"
//Derive
#include "stm32f429_lcd.h"

/*-------------------------------------------------------------------------------------------------------
*  函数声明												 
-------------------------------------------------------------------------------------------------------*/
//lcd
void BSP_LcdBacklight(bool sta);
void BSP_LcdReset(bool sta);
void BSP_LcdCS(bool sta);
void BSP_LcdRS(bool sta);
void BSP_LcdRD(bool sta);
void BSP_LcdWR(bool sta);
void BSP_LcdSendData(u16 Temp);

extern void HAL_LCD_MspInit(void);

void TransferComplete(DMA_HandleTypeDef *DmaHandle);
void TransferError(DMA_HandleTypeDef *DmaHandle);

DMA_HandleTypeDef     hDmaLCD;
bool LCD_DMA_busy = 0;
/*-------------------------------------------------------------------------------------------------------
*  执行代码													 
-------------------------------------------------------------------------------------------------------*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++  MCU ++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/********************************************************************************************************
*  Function: BSP_MCU_Init						                                                           
*  Object: MCU初始化               
*  输入： 无
*  输出： 无								                         	                                     
*  备注： MCU启动的一些配置                                      
********************************************************************************************************/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------- LCD DRIVER -----------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/********************************************************************************************************
*  Function: BSP_LCD_ON				                                                           
*  Object: FMC init
*  Input: none
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void BSP_LCD_Init(void)
{
	SRAM_HandleTypeDef gram;
	FMC_NORSRAM_TimingTypeDef Timing_read, Timing_write;
	
  gram.Instance = FMC_NORSRAM_DEVICE;
  gram.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  
   /* SRAM device configuration */  
  Timing_read.AddressSetupTime      = 0x1;
  Timing_read.AddressHoldTime       = 0;
  Timing_read.DataSetupTime         = 0xf;
  Timing_read.BusTurnAroundDuration = 0;
  Timing_read.CLKDivision           = 1;
  Timing_read.DataLatency           = 2;
  Timing_read.AccessMode            = FMC_ACCESS_MODE_A;
/*
  Timing_write.AddressSetupTime      = 0x2;
  Timing_write.AddressHoldTime       = 0;
  Timing_write.DataSetupTime         = 0x5;
  Timing_write.BusTurnAroundDuration = 0;
  Timing_write.CLKDivision           = 1;
  Timing_write.DataLatency           = 2;
  Timing_write.AccessMode            = FMC_ACCESS_MODE_A;
  */
  Timing_write.AddressSetupTime      = 0x2;
  Timing_write.AddressHoldTime       = 1;
  Timing_write.DataSetupTime         = 0x5;
  Timing_write.BusTurnAroundDuration = 0;
  Timing_write.CLKDivision           = 2;
  Timing_write.DataLatency           = 2;
  Timing_write.AccessMode            = FMC_ACCESS_MODE_A;

  gram.Init.NSBank             = FMC_NORSRAM_BANK4;
  gram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  gram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
  gram.Init.MemoryDataWidth    = SRAM_MEMORY_WIDTH;
  gram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
  gram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  gram.Init.WrapMode           = FMC_WRAP_MODE_DISABLE;
  gram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  gram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  gram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  gram.Init.ExtendedMode       = FMC_EXTENDED_MODE_ENABLE;
  gram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  gram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
	//gram.Init.ContinuousClock 		= 
  
  /* SRAM controller initialization */
  //SRAM_MspInit();
	HAL_SRAM_DeInit(&gram);
	HAL_LCD_MspInit();
  if(HAL_SRAM_Init(&gram, &Timing_read, &Timing_write) != HAL_OK)
  {
	}

  /*## -1- Enable DMA2 clock #################################################*/
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-2- Select the DMA functional Parameters ###############################*/
  hDmaLCD.Init.Channel = DMA_CHANNEL_0;                     /* DMA_CHANNEL_0                    */                     
  hDmaLCD.Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M transfer mode                */           
  hDmaLCD.Init.PeriphInc = DMA_PINC_ENABLE;               /* Peripheral increment mode Enable */                 
  hDmaLCD.Init.MemInc = DMA_MINC_DISABLE;                  /* Memory increment mode Enable     */                   
  hDmaLCD.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* Peripheral data alignment : Word */    
  hDmaLCD.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* memory data alignment : Word     */     
  hDmaLCD.Init.Mode = DMA_NORMAL;                         /* Normal DMA mode                  */  
  hDmaLCD.Init.Priority = DMA_PRIORITY_LOW;              /* priority level : high            */  
  hDmaLCD.Init.FIFOMode = DMA_FIFOMODE_DISABLE;           /* FIFO mode disabled               */        
  hDmaLCD.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;  
  hDmaLCD.Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */  
  hDmaLCD.Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */

  /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream0 #*/
  hDmaLCD.Instance = DMA2_Stream0;

  /*##-4- Select Callbacks functions called after Transfer complete and Transfer error */
  hDmaLCD.XferCpltCallback  = TransferComplete;
  hDmaLCD.XferErrorCallback = TransferError;
	
	HAL_DMA_DeInit(&hDmaLCD);
  /*##-5- Initialize the DMA stream ##########################################*/
  if(HAL_DMA_Init(&hDmaLCD) != HAL_OK)
  {
		printf("dma init error\r\n");
  }else{
		printf("dma init ok\r\n");
	}

  /*##-6- Configure NVIC for DMA transfer complete/error interrupts ##########*/
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt 
  *         is generated
  * @retval None
  */
void TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
	LCD_DMA_busy = 0;
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt 
  *         is generated during DMA transfer
  * @retval None
  */
void TransferError(DMA_HandleTypeDef *DmaHandle)
{
  /* Turn LED2 on: Transfer Error */
}

/********************************************************************************************************
*  Function: BSP_LCD_WriteReg				                                                           
*  Object: 通过FMC接口写REG
*  Input: Index
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void BSP_LCD_WriteReg(u16 Index)
{
		LCD->LCD_REG = Index*0x100 + Index/0x100;
}

/********************************************************************************************************
*  Function: BSP_LCD_WriteData				                                                           
*  Object: 通过FMC接口写Data
*  Input: Data
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void BSP_LCD_WriteData(u16 Data)
{
		LCD->LCD_RAM = Data*0x100 + Data/0x100;
}

/********************************************************************************************************
*  Function: BSP_LcdBacklight				                                                           
*  Object: lcd backlight control
*  Input: sta
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void BSP_LcdBacklight(bool sta)
{
	if(sta){
		HAL_GPIO_WritePin(LCD_BACKLIGHT_PORT, LCD_BACKLIGHT_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(LCD_BACKLIGHT_PORT, LCD_BACKLIGHT_PIN, GPIO_PIN_RESET);
	}
}

/********************************************************************************************************
*  Function: BSP_LcdReset				                                                           
*  Object: lcd reset control
*  Input: sta
*  Output: none                                  
*  brief: none
********************************************************************************************************/
void BSP_LcdReset(bool sta)
{	
	if(!sta)
		HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN, GPIO_PIN_SET);
}
