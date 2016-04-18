#ifndef __STM32F4_NRF_H
#define __STM32F4_NRF_H
#include "main.h"
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK 
    */
/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_10
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_2
#define SPIx_MISO_GPIO_PORT              GPIOC
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_3
#define SPIx_MOSI_GPIO_PORT              GPIOC
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_TX_DMA_STREAM               DMA1_Stream4
#define SPIx_RX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_RX_DMA_STREAM               DMA1_Stream3

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Stream4_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Stream3_IRQn
#define SPIx_DMA_TX_IRQHandler           DMA1_Stream4_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Stream3_IRQHandler

#define NRF_CSN_PIN  GPIO_PIN_8
#define NRF_CSN_PORT GPIOE
#define NRF_CSN_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

#define NRF_CE_PIN  GPIO_PIN_9
#define NRF_CE_PORT GPIOE
#define NRF_CE_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

extern bool SPI_busy, nrf_send_finished;
void SPI_Config(void);
void EXTILine0_Config(void);
void nrf_init(void);
void nrf_send_IT(uint8_t *data, uint32_t size);
void check_nrf(void);

#endif
