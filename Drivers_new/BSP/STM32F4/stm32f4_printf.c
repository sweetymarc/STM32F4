#include "main.h"
#include "stm32f4_printf.h"


extern bool esp8266_exist;
//variables uart printf
uint8_t txBuffer[2][UART_TX_BUFFER_SIZE], rxBuffer[16], bufAlt=0;
uint32_t txPos=0;
volatile bool UartReady = true, UartReceived = false;
UART_HandleTypeDef UartHandle;

/* Private functions ---------------------------------------------------------*/
int fputc(int ch, FILE *F){
	#ifdef ESP8266
	if( esp8266_exist ){
		return 0;
	}
	#endif
	txBuffer[bufAlt][txPos++] = ch;
	if( txPos >= 128 ){
		if( UartReady ){
			HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)(txBuffer[bufAlt]), txPos);
			UartReady = false;
			bufAlt = 1 - bufAlt;
			txPos = 0;
		}else{
			txPos %= UART_TX_BUFFER_SIZE;
		}
	}
	return ch;
}
int fgetc(FILE *f){
	return 0x1;
}
void uartInit(){
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  
  UartHandle.Init.BaudRate     = UART_BANDRATE;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    while(1);
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
extern UART_HandleTypeDef Uart1Handle;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set transmission flag: transfer complete */
  if( huart == &UartHandle ){
		UartReady = true;
	}
	#ifdef UART3
	else if( huart == &Uart3Handle ){
		Uart3Ready = true;
	}
	#endif
	#ifdef UART1
	else if( huart == &Uart1Handle ){
		Uart1Ready = true;
	}
	#endif
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Set transmission flag: transfer complete */
  //UartReady = true;
  if( huart == &UartHandle ){
		UartReceived = true;
	}
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
}
/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
