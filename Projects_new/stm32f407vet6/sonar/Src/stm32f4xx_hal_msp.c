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
{  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
/* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
}

void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */
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
	}
	#ifdef INPUT_PWM
	else if( htim == &TimHandle_PWM ){
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
		
		GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2;
		HAL_GPIO_Init(TIM_PWM_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL3;
		HAL_GPIO_Init(TIM_PWM_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL4;
		HAL_GPIO_Init(TIM_PWM_GPIO_PORT, &GPIO_InitStruct);
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM_PWM_IRQn, 4, 1);
		
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM_PWM_IRQn);  
	}
	#endif
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


static DMA_HandleTypeDef hdma_tx, hdma_1tx;
static DMA_HandleTypeDef hdma_rx, hdma_3rx;
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
	if( huart == &UartHandle ){
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
 
	if( huart == &UartHandle ){
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
	}
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
