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
 * @brief Macros and hardware includes for nRF24LE1
 * @ingroup hal_nrf24le1
 *
 * @{
 * @name Hardware dependencies
 * @{
 *
 */

#ifndef HAL_NRF_LE1_H__
#define HAL_NRF_LE1_H__
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"

extern volatile bool trans_success, rf_data_ready;
/* SPI handler declaration */
//extern uint8_t aTxBuffer[];
/* Buffer used for reception */
//extern uint8_t aRxBuffer[];
extern volatile bool radio_busy, trans_success, rf_data_ready;


/** Macro that set radio's CSN line LOW.
 *
 */
//#define CSN_LOW() do {RFCSN = 0U; } while(false)
void CSN_LOW(void);
/** Macro that set radio's CSN line HIGH.
 *
 */
//#define CSN_HIGH() do {RFCSN = 1U; } while(false)
void CSN_HIGH(void);
	
/** Macro that set radio's CE line LOW.
 *
 */
//#define CE_LOW() do {RFCE = 0U;} while(false)
void CE_LOW(void);
/** Macro that set radio's CE line HIGH.
 *
 */
//#define CE_HIGH() do {RFCE = 1U;} while(false)
void CE_HIGH(void);

/**
 * Pulses the CE to nRF24L01p for at least 10 us
 */
void CE_PULSE(void);
	
#endif // HAL_NRF_LE1_H__

/** @} */
/** @} */
