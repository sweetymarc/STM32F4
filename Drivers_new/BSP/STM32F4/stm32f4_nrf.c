#include "main.h"
#include "hal_nrf.h"

//variables SPI and NRF
SPI_HandleTypeDef SpiHandle;
 uint8_t *nrf_tx_data, *nrf_tx_data_end;
 uint8_t status, fifo_nrf_irq_flags;
 uint32_t nrf_count = 0;
bool SPI_busy = false, nrf_send_finished = true;

void SPI_Config(){
	  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  
  SpiHandle.Init.Mode = SPI_MODE_MASTER;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
   // Error_Handler();
		while(1);
  }
}
void nrf_init(){
	uint8_t *addr = JPG_buf[0];// avoid CCM
	addr[0] = 0x39; addr[1] = 0x39; addr[2] = 0x39; addr[3] = 0x39; addr[4] = 0x39;
	HAL_NRF_MspInit();
	while( !hal_nrf_tx_fifo_empty() ){
		hal_nrf_flush_tx();
	}
	while( !hal_nrf_rx_fifo_empty() ){
		hal_nrf_flush_rx();
	}
	while( hal_nrf_get_irq_flags() ){
		hal_nrf_get_clear_irq_flags();// 0111 000
		soft_delay(100);
	}
	hal_nrf_close_pipe(HAL_NRF_PIPE1);

	hal_nrf_set_rf_channel(0x60);
	hal_nrf_set_address(HAL_NRF_TX, addr);
	HAL_Delay(100);
	hal_nrf_set_address(HAL_NRF_PIPE0, addr);
	HAL_Delay(100);
	hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);
	soft_delay(100);
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
	HAL_Delay(100);
}
void nrf_send_IT(uint8_t *_data, uint32_t _size){
	//lanuch nrf sending, nonblocking, system tick
	if( !_data || !_size || SPI_busy ){
		return;
	}
	nrf_tx_data = _data;
	nrf_tx_data_end = _data + _size;
	
	while( !hal_nrf_tx_fifo_empty() ){
		hal_nrf_flush_tx();
	}
	hal_nrf_write_tx_payload( nrf_tx_data, 32U );
	nrf_tx_data += 32;
	nrf_send_finished = false;
}

uint8_t nrf_irq_flags, nrf_irq_check;
void check_nrf(){
	if(SPI_busy){
		return;
	}
	nrf_count++;
	nrf_irq_flags = hal_nrf_get_irq_flags();// never clear here, since maybe it's workingg, any writing shoud be done when strandby
	#ifdef DEBUG_NRF
	PRINTF("NRF INT\r\n")
	#endif
	/*if( 0x40 & nrf_irq_flags ){ //receive
		while( !hal_nrf_rx_fifo_empty() ){
			hal_nrf_flush_rx();
		}
	}
	do{
		nrf_irq_check = ();
		#ifdef DEBUG_NRF
		PRINTF("try to clear NRF IRQ\r\n")
		#endif
	}while(nrf_irq_check);*/
	if( 0x20 & nrf_irq_flags ){ // trans ok
		HAL_GPIO_TogglePin(LED_PORT, LED1_PIN);
		hal_nrf_get_clear_irq_flags();
		while( !hal_nrf_tx_fifo_empty() ){
			#ifdef DEBUG_NRF
			PRINTF("fifo not empty\r\n")
			#endif
			hal_nrf_flush_tx();
		}
		#ifdef DEBUG_NRF
		PRINTF("32bytes send\r\n")
		#endif
		if( nrf_tx_data >= nrf_tx_data_end ){ //a frame complete
			nrf_send_finished = true;
			return;
		}else{
			hal_nrf_write_tx_payload(nrf_tx_data, 32U);
			nrf_tx_data += 32;
		}
	}else	if( 0x10 & nrf_irq_flags ){ // max retry
		HAL_GPIO_TogglePin(LED_PORT, LED1_PIN);
		hal_nrf_get_clear_irq_flags();
		#ifdef DEBUG_NRF
		PRINTF("max retry\r\n")
		#endif
		hal_nrf_reuse_tx();
		CE_PULSE();
		//HAL_GPIO_TogglePin(LED_PORT, LED3_PIN);					
	}
}
/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle.
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SPI_busy = false;
  CSN_HIGH();
	CE_PULSE();
	#ifdef DEBUG_NRF
	PRINTF("SPI DMA finished")
	#endif
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
}
