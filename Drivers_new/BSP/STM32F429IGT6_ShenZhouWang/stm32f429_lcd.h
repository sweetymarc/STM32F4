#ifndef __Driver__
#define __Driver__
#include "main.h"

/*--------------------------------------------------------------------------
// �ļ�����Driver.h
// ������  ϵͳ����ͷ�ļ�
// ����ߣ�EU����
// �������ڣ�2013��11��10��
// ����EU���ӳ�Ʒ-��Ȩ����-����ؾ�
// EU-�Ȱ�Ƕ��ʽ����
// http://euse.taobao.com
//-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------
*  ����ӿ�	ΪӦ�ò��ṩ�Ľӿ�
-------------------------------------------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported constants --------------------------------------------------------*/
#define SRAM_BANK_ADDR                 ((uint32_t)0x68000000)

/* #define SRAM_MEMORY_WIDTH            FMC_NORSRAM_MEM_BUS_WIDTH_8  */
#define SRAM_MEMORY_WIDTH               FMC_NORSRAM_MEM_BUS_WIDTH_16
/* #define SRAM_MEMORY_WIDTH            FMC_NORSRAM_MEM_BUS_WIDTH_32 */

/* #define SRAM_CONTINUOUS_CLOCK    FMC_CONTINUOUS_CLOCK_SYNC_ONLY */
#define SRAM_CONTINUOUS_CLOCK    FMC_CONTINUOUS_CLOCK_SYNC_ASYNC

#define SRAM_TIMEOUT     ((uint32_t)0xFFFF) 

//mcu
extern void BSP_LCD_Init(void);
//lcd
extern void BSP_LcdBacklight(bool sta);
extern void BSP_LcdReset(bool sta);
//fsmc
extern void BSP_LCD_WriteReg(uint16_t Index);
extern void BSP_LCD_WriteData(uint16_t Data);

extern DMA_HandleTypeDef hDmaLCD;
extern bool LCD_DMA_busy;

//LCD��ַ�ṹ��
typedef struct
{
	uint16_t LCD_REG;
	uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)

#endif



