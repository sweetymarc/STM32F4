/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2513 $
 */

/** @file
 * @brief Implementation of #hal_nrf_rw.
 *
 * @details #hal_nrf_rw is an SPI function which is hardware dependent. This file
 * contains an implementation for nRF24LE1.
 */
#include <stdint.h>
#include "hal_nrf.h"
#include "main.h"

extern void Error_Handler(void);
volatile bool radio_busy;
volatile bool trans_success, rf_data_ready;
/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
/** Macro that set radio's CSN line LOW.
 *
 */
//#define CSN_LOW() do {RFCSN = 0U; } while(false)
void CSN_LOW(){
	HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET);
}
/** Macro that set radio's CSN line HIGH.
 *
 */
//#define CSN_HIGH() do {RFCSN = 1U; } while(false)
void CSN_HIGH(){
	HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
}
	
/** Macro that set radio's CE line LOW.
 *
 */
//#define CE_LOW() do {RFCE = 0U;} while(false)
void CE_LOW(){
	HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
}
/** Macro that set radio's CE line HIGH.
 *
 */
//#define CE_HIGH() do {RFCE = 1U;} while(false)
void CE_HIGH(){
	HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET);
}
volatile uint32_t i_delay;
void CE_PULSE(){
	CE_HIGH();
	i_delay = 300;
	while(i_delay--);
	CE_LOW();
}

uint8_t hal_nrf_rw(uint8_t value)
{
  uint8_t ret;
	HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)&value, (uint8_t *)&ret, 1, 5000);
  return ret;             // return SPI read value
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t irq_flags;
  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();

  switch(irq_flags)
  {
    // Transmission success
    case (1 << (uint8_t)HAL_NRF_TX_DS):
			DISABLE_IRQ
			printf("tx_ok ");
			ENABLE_IRQ
      radio_busy = false;
			trans_success = true;
      // Data has been sent
      break;
    // Transmission failed (maximum re-transmits)
    case (1 << (uint8_t)HAL_NRF_MAX_RT):
			DISABLE_IRQ
			printf("tx_fail ");
			ENABLE_IRQ
      // When a MAX_RT interrupt occurs the TX payload will not be removed from the TX FIFO.
      // If the packet is to be discarded this must be done manually by flushing the TX FIFO.
      // Alternatively, CE_PULSE() can be called re-starting transmission of the payload.
      // (Will only be possible after the radio irq flags are cleared)
      //hal_nrf_flush_tx();
			trans_success = false;
      radio_busy = false;
      break;
		case  (1 << (uint8_t)HAL_NRF_RX_DR):		
			DISABLE_IRQ
			printf("rx_ok ");
			ENABLE_IRQ
			rf_data_ready = true;
			break;
    default:
      break;
  }
	while( hal_nrf_get_irq_flags()  ){
		PRINTF("clear IRQ failed")
		hal_nrf_get_clear_irq_flags();// 0111 000
		HAL_Delay(1);
	}
}  */
/** Macro specifyng the radio SPI busy flag.
 *
 */
//#define HAL_NRF_HW_SPI_BUSY (!(SPIRSTAT & 0x02U))


