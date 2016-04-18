/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   HAL MSP module.
  * 
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by MicroXplorer and eventually modified 
    by the user

  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */

void HAL_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_Init_Structure;

  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
/* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
  VISION_BEEP_CLK_ENABLE();
  GPIO_Init_Structure.Pin       = VISION_BEEP_PIN;
  GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(VISION_BEEP_PORT, &GPIO_Init_Structure);
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */
}

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */

void HAL_LED_MspInit(){
  GPIO_InitTypeDef GPIO_Init_Structure;
  LED_GPIO_CLK_ENABLE();	
	/* reset pin */
  GPIO_Init_Structure.Pin       = LED1_PIN;
  GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_Init_Structure);
}
/*
void HAL_Button_MspInit(){	
  GPIO_InitTypeDef GPIO_Init_Structure;
  __GPIOI_CLK_ENABLE();	
  GPIO_Init_Structure.Pin       = BUTTON_PIN; 
  GPIO_Init_Structure.Mode      = GPIO_MODE_INPUT;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_Init_Structure);
}*/

void HAL_CAMERA_MspInit(void){	
  GPIO_InitTypeDef GPIO_Init_Structure;
  CAMERA_RESET_GPIO_CLK_ENABLE();	
	/* reset pin */
  GPIO_Init_Structure.Pin       = CAMERA_RESET_PIN; 
  GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(CAMERA_RESET_PORT, &GPIO_Init_Structure);
	
	/* clock pin */
	__GPIOA_CLK_ENABLE();
  GPIO_Init_Structure.Pin       = GPIO_PIN_8; 
  GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_FAST;;
  GPIO_Init_Structure.Alternate = GPIO_AF0_MCO;  
  HAL_GPIO_Init(GPIOA, &GPIO_Init_Structure);	
}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef *p)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
    __DCMI_CLK_ENABLE();
  
  /* Enable DMA2 clock */
  __DMA2_CLK_ENABLE(); 
  
  /* Enable GPIO clocks */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();

    /**DCMI GPIO Configuration    
    PE4     ------> DCMI_D4
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE0     ------> DCMI_D2
    PE1     ------> DCMI_D3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_0 
                          |GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __DCMI_CLK_DISABLE();
  
    /**DCMI GPIO Configuration    
    PE4     ------> DCMI_D4
    PE5     ------> DCMI_D6
    PE6     ------> DCMI_D7
    PA4     ------> DCMI_HSYNC
    PA6     ------> DCMI_PIXCK
    PC6     ------> DCMI_D0
    PC7     ------> DCMI_D1
    PB6     ------> DCMI_D5
    PB7     ------> DCMI_VSYNC
    PE0     ------> DCMI_D2
    PE1     ------> DCMI_D3 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_0 
                          |GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }

}
void HAL_I2C_MspInit(I2C_HandleTypeDef *hI2C_camera)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* Enable I2C1 clock */
  __HAL_RCC_I2C1_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_8;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;    
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
	__I2C1_FORCE_RESET();
	__I2C1_RELEASE_RESET();
  /*##-3- Configure the NVIC for I2C #########################################*/   
  /* NVIC for I2C1 */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
  }
}
/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
	if( htim == &TimHandleSonar){
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		TIMsonar_CLK_ENABLE();
			
		/* Enable GPIO channels Clock */
		ICx_PORT_CLK_ENABLE();
		
		/* Configure  (TIMx_Channel) in Alternate function, push-pull and 100MHz speed */
		GPIO_InitStruct.Pin = ICx_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF_TIMsonar;
		HAL_GPIO_Init(ICx_PORT, &GPIO_InitStruct);

		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIMsonar_IRQn, 0, 1);
		
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIMsonar_IRQn);  
	}else if( htim == &TimHandle_PWM ){
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIM_PWM Peripheral clock enable */
		TIM_PWM_CLK_ENABLE();
		/* Enable GPIO Channels Clock */
		TIM_PWM_GPIO_CLK_ENABLE();
		/*##-2- Configure I/Os #####################################################*/
		/*
		*/
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		
		GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1;
		HAL_GPIO_Init(TIM_PWM_GPIO_PORT, &GPIO_InitStruct);
		
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM_PWM_IRQn, 4, 1);
		
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM_PWM_IRQn);  
	}
}

void BSP_sonar_MspInit(){

  GPIO_InitTypeDef  GPIO_InitStruct;
  SONAR_CLK_ENABLE;
  
  GPIO_InitStruct.Pin       = SONAR_TRIG_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  
  HAL_GPIO_Init(SONAR_TRIG_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
}

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */
void _HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }

}

void _HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  SPIx_SCK_GPIO_CLK_ENABLE();
  SPIx_MISO_GPIO_CLK_ENABLE();
  SPIx_MOSI_GPIO_CLK_ENABLE();
  /* Enable SPI3 clock */
  SPIx_CLK_ENABLE(); 
  /* Enable DMA1 clock */
  DMAx_CLK_ENABLE();   
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = SPIx_SCK_AF;
  
  HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
    
  /* SPI MISO GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MISO_PIN;
  GPIO_InitStruct.Alternate = SPIx_MISO_AF;
  
  HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
  GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    
  HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = SPIx_TX_DMA_STREAM;
  
  hdma_tx.Init.Channel             = SPIx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  HAL_DMA_Init(&hdma_tx);   
  
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmatx, hdma_tx);
/*    
  hdma_rx.Instance                 = SPIx_RX_DMA_STREAM;
  
  hdma_rx.Init.Channel             = SPIx_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4; 

  HAL_DMA_Init(&hdma_rx);
    
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);*/
    
  /*##-4- Configure the NVIC for DMA #########################################*/ 
  /* NVIC configuration for DMA transfer complete interrupt (SPI3_TX) */
  HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 5, 5);
  HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
    
  /* NVIC configuration for DMA transfer complete interrupt (SPI3_RX) */
  //HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 5, 4);   
  //HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  SPIx_FORCE_RESET();
  SPIx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure SPI SCK as alternate function  */
  HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
  /* Configure SPI MISO as alternate function  */
  HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
  /* Configure SPI MOSI as alternate function  */
  HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
   
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Stream associate to reception process */
  HAL_DMA_DeInit(&hdma_rx);
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(SPIx_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(SPIx_DMA_RX_IRQn);
}

void HAL_NRF_MspInit(){
  GPIO_InitTypeDef  GPIO_InitStruct;

  NRF_CE_GPIO_CLK_ENABLE();
  NRF_CSN_GPIO_CLK_ENABLE();
	SEND_SWITCH_CLK_ENABLE();
  //nrf ce
  GPIO_InitStruct.Pin       = NRF_CE_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(NRF_CE_PORT, &GPIO_InitStruct);
	//nrf csn
  GPIO_InitStruct.Pin       = NRF_CSN_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(NRF_CSN_PORT, &GPIO_InitStruct);
	//nrf send_switch
  GPIO_InitStruct.Pin       = SEND_SWITCH_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  HAL_GPIO_Init(SEND_SWITCH_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
}

static DMA_HandleTypeDef hdma_tx, hdma_1tx;
static DMA_HandleTypeDef hdma_rx, hdma_3rx;
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  if( huart == &Uart3Handle ){
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		__HAL_RCC_GPIOD_CLK_ENABLE();
		/* Enable USART2 clock */
		__HAL_RCC_USART3_CLK_ENABLE();
		/* Enable DMA1 clock */
		__HAL_RCC_DMA1_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = GPIO_PIN_8;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
			
		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
			
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
			
		/* Configure the DMA handler for Transmission process */
		/*hdma_3tx.Instance                 = DMA1_Stream3;
		
		hdma_3tx.Init.Channel             = DMA_CHANNEL_4;
		hdma_3tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_3tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_3tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_3tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_3tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_3tx.Init.Mode                = DMA_NORMAL;
		hdma_3tx.Init.Priority            = DMA_PRIORITY_LOW;
		hdma_3tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_3tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_3tx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_3tx.Init.PeriphBurst         = DMA_PBURST_INC4;
		
		HAL_DMA_Init(&hdma_3tx);   
		
		__HAL_LINKDMA(huart, hdmatx, hdma_3tx);*/
	
		/* Configure the DMA handler for Transmission process */
		hdma_3rx.Instance                 = DMA1_Stream1;
		
		hdma_3rx.Init.Channel             = DMA_CHANNEL_4;
		hdma_3rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		hdma_3rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_3rx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_3rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_3rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_3rx.Init.Mode                = DMA_CIRCULAR;
		hdma_3rx.Init.Priority            = DMA_PRIORITY_HIGH;
		hdma_3rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
		hdma_3rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_3rx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_3rx.Init.PeriphBurst         = DMA_PBURST_INC4;

		HAL_DMA_Init(&hdma_3rx);
			
		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(huart, hdmarx, hdma_3rx);

		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		//HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 3);
		//HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
		//HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 5, 2);   
		//HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
		
		/* NVIC configuration for USART TC interrupt */
		HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
	}else if( huart == &UartHandle ){
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		DMAx_CLK_ENABLE();
		/* Enable GPIO TX/RX clock */
		USARTx_TX_GPIO_CLK_ENABLE();
		USARTx_RX_GPIO_CLK_ENABLE();
		/* Enable USART2 clock */
		USARTx_CLK_ENABLE(); 
		/* Enable DMA1 clock */
	
		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = USARTx_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = USARTx_TX_AF;
		
		HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
			
		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = USARTx_RX_PIN;
		GPIO_InitStruct.Alternate = USARTx_RX_AF;
			
		HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
			
		/*##-3- Configure the DMA streams ##########################################*/
		/* Configure the DMA handler for Transmission process */
		hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
		
		hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
		hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode                = DMA_NORMAL;
		hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
		
		HAL_DMA_Init(&hdma_tx);   
		
		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(huart, hdmatx, hdma_tx);
			
		/* Configure the DMA handler for Transmission process */
		hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;
		
		hdma_rx.Init.Channel             = USARTx_RX_DMA_CHANNEL;
		hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode                = DMA_CIRCULAR;
		hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
		hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
		hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

		HAL_DMA_Init(&hdma_rx);
			
		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(huart, hdmarx, hdma_rx);
			
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 5, 3);
		HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
			
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
		//HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 5, 2);   
		//HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
		
		/* NVIC configuration for USART TC interrupt */
		HAL_NVIC_SetPriority(USARTx_IRQn, 6, 1);
		HAL_NVIC_EnableIRQ(USARTx_IRQn);
	}else if( huart == &Uart1Handle ){
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/* Enable USART2 clock */
		__HAL_RCC_USART1_CLK_ENABLE();
		/* Enable DMA1 clock */
		__HAL_RCC_DMA2_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = GPIO_PIN_9;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
			
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			
		/* Configure the DMA handler for Transmission process */
		hdma_1tx.Instance                 = DMA2_Stream7;
		
		hdma_1tx.Init.Channel             = DMA_CHANNEL_4;
		hdma_1tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_1tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_1tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_1tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_1tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_1tx.Init.Mode                = DMA_NORMAL;
		hdma_1tx.Init.Priority            = DMA_PRIORITY_LOW;
		hdma_1tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_1tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_1tx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_1tx.Init.PeriphBurst         = DMA_PBURST_INC4;
		
		HAL_DMA_Init(&hdma_1tx);   
		
		__HAL_LINKDMA(huart, hdmatx, hdma_1tx);

		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 3);
		HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
		//HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 5, 2);   
		//HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);

		/* NVIC configuration for USART TC interrupt */
		HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if( huart == &Uart3Handle ){
  /*##-1- Reset peripherals ##################################################*/
		__HAL_RCC_USART3_FORCE_RESET();
		__HAL_RCC_USART3_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks ################################*/
		/* Configure UART Tx as alternate function */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
		/* Configure UART Rx as alternate function */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9);
		
		/*##-3- Disable the DMA Streams ############################################*/
		/* De-Initialize the DMA Stream associate to transmission process */
		//HAL_DMA_DeInit(&hdma_3tx);
		HAL_DMA_DeInit(&hdma_3rx);
		/*##-3- Disable the NVIC for UART ##########################################*/
		
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(USART3_IRQn);
	}else if( huart == &UartHandle ){
		USARTx_FORCE_RESET();
		USARTx_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks ################################*/
		/* Configure UART Tx as alternate function */
		HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
		/* Configure UART Rx as alternate function */
		HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
		
		/*##-3- Disable the DMA Streams ############################################*/
		/* De-Initialize the DMA Stream associate to transmission process */
		HAL_DMA_DeInit(&hdma_tx); 
		/* De-Initialize the DMA Stream associate to reception process */
		HAL_DMA_DeInit(&hdma_rx);
		/*##-3- Disable the NVIC for UART ##########################################*/
		
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
		
		HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
		
		HAL_NVIC_DisableIRQ(USARTx_IRQn);
	}else if( huart == &Uart1Handle ){
  /*##-1- Reset peripherals ##################################################*/
		__HAL_RCC_USART1_FORCE_RESET();
		__HAL_RCC_USART1_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks ################################*/
		/* Configure UART Tx as alternate function */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
		/* Configure UART Rx as alternate function */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
		
		/*##-3- Disable the DMA Streams ############################################*/
		/* De-Initialize the DMA Stream associate to transmission process */
		//HAL_DMA_DeInit(&hdma_3tx);
		HAL_DMA_DeInit(&hdma_1tx);
		/*##-3- Disable the NVIC for UART ##########################################*/
		
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
		HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
