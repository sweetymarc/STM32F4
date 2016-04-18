#include "main.h"

extern DCMI_HandleTypeDef hdcmi;
//extern I2C_HandleTypeDef hI2C_camera;

uint8_t BSP_CAMERA_Init(void);
void BSP_Reg_Set(uint8_t reg, uint8_t data);
uint8_t BSP_Reg_Get(uint8_t reg);
extern I2C_HandleTypeDef hI2C_camera; 

void BSP_CAMERA_DMA_IRQHandler(void);
void BSP_CAMERA_IRQHandler(void);
void BSP_CAMERA_Reset(uint8_t sta);
void BSP_Camera_Start(void);
void color2gray(void);

#ifndef GRAY_BUFFER_N
#define GRAY_BUFFER_N 1
#endif

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define IMAGE_SIZE IMAGE_WIDTH*IMAGE_HEIGHT
#define BUFFER_ROWS 15
#define FRAME_DIVIDE 16
#define CAMERA_DMA_SIZE IMAGE_WIDTH*BUFFER_ROWS*2*2/4
#define JPG_BUF_SIZE 8192
extern bool frame_int, dma_int, frame_received;
extern uint32_t line_num;
extern uint16_t lines_buffer[2][BUFFER_ROWS][IMAGE_WIDTH];
extern uint8_t gray[GRAY_BUFFER_N][IMAGE_HEIGHT][IMAGE_WIDTH];
extern int gray_buffer_n;
extern uint8_t pos_buffer;
extern uint32_t block_alt;
extern uint32_t block_num;
extern uint32_t frame_count;
// to avoid allocate in CCM, so put them here
extern uint32_t bin[IMAGE_SIZE/32];
extern uint8_t *bin_DMA;
extern uint8_t JPG_buf[][JPG_BUF_SIZE];
//extern uint32_t image_data[];
//extern void OV_Reg_Set(uint8_t reg, uint8_t data);
//extern uint8_t OV_Reg_Get(uint8_t reg);
/*
void     CAMERA_IO_Init(void);
void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  CAMERA_IO_Read(uint8_t addr, uint8_t reg);
*/
