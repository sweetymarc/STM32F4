#include "main.h"
#include "stm32f429_camera.h"

DCMI_HandleTypeDef hdcmi; 
DMA_HandleTypeDef hdma;
extern void DCMI_MspInit(void);
extern void OV_I2C_Init(void);
void cameraCXferCallback(DMA_HandleTypeDef *h);
void cameraHXferCallback(DMA_HandleTypeDef *h);
void cameraXferErrorCallback(DMA_HandleTypeDef *h);
void XferM1CpltCallback(DMA_HandleTypeDef *h);

bool frame_int = 0, line_int = 0, half_received = 0, dma_int = 0, frame_dma_over = 0, frame_over = 0, frame_received = 0;
uint16_t lines_buffer[2][BUFFER_ROWS][IMAGE_WIDTH];
uint32_t block_alt = 0;
uint16_t *pBuffer;
uint8_t gray[GRAY_BUFFER_N][IMAGE_HEIGHT][IMAGE_WIDTH];
int gray_buffer_n = 0;
uint8_t pos_buffer = 1; // point to that not buffering
uint32_t frame_count=0, block_num = IMAGE_HEIGHT/BUFFER_ROWS - 1; // point to that can be using, not buffering
// to avoid allocate in CCM, so put them here
uint8_t JPG_buf[2][JPG_BUF_SIZE];
uint32_t bin[IMAGE_SIZE/32];
uint8_t *bin_DMA=JPG_buf[0];
uint8_t buf[280];
uint8_t txBuffer8266[128];

uint32_t line_base;
uint8_t *p_rgb;
static int i;
uint8_t *pPixel8, r, g, b, pixelGray;
uint8_t RGB565[2], rgb565_inverse[2];
uint16_t *pRGB565;

uint8_t *p_gray;
uint16_t *p_color_in;

DCMI_HandleTypeDef hdcmi;
I2C_HandleTypeDef hI2C_camera;

uint8_t BSP_CAMERA_Init()
{
	HAL_CAMERA_MspInit(); //REST PIN AND MCO
	#ifdef STM32F407xx
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_5);// 2 is min
	#elif defined STM32F429xx
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_HSE, RCC_MCODIV_5);// 2 is min
	#else
	#error MCOConfig_not_defined
	#endif
	
  /*** Enable peripherals and GPIO clocks ***/
  /* Enable DCMI clock */
  __DCMI_CLK_ENABLE();

  /* Enable DMA2 clock */
  __DMA2_CLK_ENABLE(); 
  /*** Configures the DCMI to interface with the camera module ***/
  /* DCMI configuration */
  hdcmi.Init.CaptureRate      = DCMI_CR_ALL_FRAME;  
  hdcmi.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.VSPolarity       = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
  hdcmi.Instance              = DCMI;  
	hdcmi.DMA_Handle						= &hdma;

  /* DCMI Initialization */
  HAL_DCMI_MspInit(&hdcmi);  
  HAL_DCMI_Init(&hdcmi);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_LINE);  // before the first frame start, lint int make no sense
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_VSYNC);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
	
  /*** Configure the DMA ***/
  /* Set the parameters to be configured */
  hdma.Init.Channel             = DMA_CHANNEL_1;
  hdma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma.Init.MemInc              = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma.Init.Mode                = DMA_CIRCULAR;
  hdma.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma.Init.MemBurst            = DMA_MBURST_SINGLE;
  hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
	hdma.XferCpltCallback 				= cameraCXferCallback;
	hdma.XferHalfCpltCallback			= cameraHXferCallback;
	hdma.XferErrorCallback 				= cameraXferErrorCallback;
	hdma.XferM1CpltCallback				= XferM1CpltCallback;

  hdma.Instance = DMA2_Stream1;

  /* Configure the DMA stream */
  HAL_DMA_Init(&hdma);
	__HAL_DMA_DISABLE_IT(&hdma, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma, DMA_IT_TC);
  /* Associate the initialized DMA handle to the DCMI handle */
  __HAL_LINKDMA(&hdcmi, DMA_Handle, hdma);
	
  /*** Configure the NVIC for DCMI and DMA ***/
  /* NVIC configuration for DCMI transfer complete interrupt */
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
  
  /* NVIC configuration for DMA2D transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  /* Init the I2C */
  /*##-1- Configure the I2C peripheral ######################################*/
  hI2C_camera.Instance             = I2Cx;  
  hI2C_camera.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hI2C_camera.Init.ClockSpeed      = 10000;
  hI2C_camera.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hI2C_camera.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hI2C_camera.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hI2C_camera.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  hI2C_camera.Init.OwnAddress1     = 0xC0;

  if(HAL_I2C_Init(&hI2C_camera) != HAL_OK)
  {
		printf("i2c init error\r\n");
  }else{
		printf("i2c init ok\r\n");
	}
  return 1;
}
void BSP_Camera_Start(){
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)lines_buffer, CAMERA_DMA_SIZE); 
}
#define OV_ADDRESS  0x42 // for OV7670
void BSP_Reg_Set(uint8_t reg, uint8_t value){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data[] = {reg, value};
  status = HAL_I2C_Master_Transmit(&hI2C_camera, OV_ADDRESS, data, 2, 0xff); 
  //Check the communication status 
  if(status != HAL_OK)
  {
		printf("ov reg set error: %x\r\n", status);
  }
}
uint8_t BSP_Reg_Get(uint8_t reg){
// Prepare the sensor to read the Camera ID 
	HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  // HAL_I2C_Mem_Write(&hI2C_camera, OV_ADDRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, &t, 1, 0xffff); 
  // Get the camera ID 
  status = HAL_I2C_Master_Transmit(&hI2C_camera, OV_ADDRESS, &reg, 1, 0xff); 
  //status = HAL_I2C_Mem_Read(&hI2C_camera, OV_ADDRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0xffff);
  // Check the communication status 
  if(status != HAL_OK)
  {
		printf("i2c read 1 error: %x\r\n", status);
  }
	status = HAL_I2C_Master_Receive(&hI2C_camera, OV_ADDRESS, &Value, 1, 0xff); 
  // Check the communication status 
  if(status != HAL_OK)
  {
		printf("i2c read 2 error: %x\r\n", status);
  }
  return Value;  
}
void BSP_CAMERA_Reset(uint8_t sta){
	if(sta){
		HAL_GPIO_WritePin(CAMERA_RESET_PORT, CAMERA_RESET_PIN, GPIO_PIN_SET);
	}else{		
		HAL_GPIO_WritePin(CAMERA_RESET_PORT, CAMERA_RESET_PIN, GPIO_PIN_RESET);
	}
}

uint32_t lastFrameTime;
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi){
	//BSP_CAMERA_Reset(0);
	//block_num++;
	//frame_over = true;
	frame_count ++;
	frame_int = true;
	v_out.period = HAL_GetTick() - lastFrameTime;
	lastFrameTime = HAL_GetTick();
	pos_buffer = 1 - pos_buffer;
	block_alt++;
	block_alt %= FRAME_DIVIDE;
	if( (IMAGE_HEIGHT/BUFFER_ROWS - 1) == block_num  && CAMERA_DMA_SIZE == __HAL_DMA_GET_COUNTER(&hdma) ){  //if and only if when block_num = 15	
		frame_received = true;
		#ifdef DEBUG_CAMERA
		DISABLE_IRQ
		printf("frame: %d\r\n", frame_count);
		ENABLE_IRQ
		#endif
		gray_buffer_n++;
		gray_buffer_n %= GRAY_BUFFER_N;
		//trig the soft interrupt, so that position_calc() will be called
		__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_1);
	}else{
		//DMA ERROR restart it
		#ifdef DEBUG_CAMERA
		PRINTF("frame lost\r\n");
		#endif
		__HAL_DMA_DISABLE(&hdma);
		__HAL_DMA_SET_COUNTER(&hdma, CAMERA_DMA_SIZE);
		block_num = IMAGE_HEIGHT/BUFFER_ROWS - 1;  //reset
		__HAL_DMA_ENABLE(&hdma);
	}
	pos_buffer = 1; //reset
	if( 3 == frame_count ){ // the first two frames are abondoned
		__HAL_DMA_ENABLE_IT(&hdma, DMA_IT_HT);
		__HAL_DMA_ENABLE_IT(&hdma, DMA_IT_TC);
	}
  __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);
}
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_LINE);
}
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_VSYNC);
	//printf("vsync\r\n");
}
/**
  * @brief  Handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void blockReceived(void){
	dma_int = 1;
	block_num ++;
	block_num %= IMAGE_HEIGHT / BUFFER_ROWS;
	pos_buffer = 1-pos_buffer;
	color2gray();
	 // for display
	#ifdef YUV
	p_color_in = &(lines_buffer[pos_buffer][0][0]);
	for(i=0; i<BUFFER_ROWS*IMAGE_WIDTH; i++){
		*(p_color_in++) &= 0x00F8;
	}
	#endif
	#ifdef LCD_DISPLAY
	if( block_alt == ((block_num) % (uint32_t)FRAME_DIVIDE)){// whether this block should be displayed
		LCD_DisAPhoto_DMA( (block_num) * BUFFER_ROWS, 0, BUFFER_ROWS, IMAGE_WIDTH, lines_buffer[pos_buffer] );
	}
	#endif
	#ifdef DEBUG_CAMERA 
	DISABLE_IRQ
	printf("%d", block_num);
	ENABLE_IRQ
	#endif
}
void cameraHXferCallback(DMA_HandleTypeDef *h)
{
	#ifdef DEBUG_CAMERA
	DISABLE_IRQ
	printf("HT");
	ENABLE_IRQ
	#endif
	blockReceived();
}
void cameraCXferCallback(DMA_HandleTypeDef *h)
{
	#ifdef DEBUG_CAMERA
	DISABLE_IRQ
	printf("CT");
	ENABLE_IRQ
	#endif
	blockReceived();
}

void cameraXferErrorCallback(DMA_HandleTypeDef *h){
	#ifdef DEBUG_CAMERA
	printf("XE");
	#endif
}
void XferM1CpltCallback(DMA_HandleTypeDef *h){
	#ifdef DEBUG_CAMERA
	printf("M1");
	#endif
}
void BSP_CAMERA_DMA_IRQHandler(void){
	// the HALxxx will not call back if TC interrupt, I don't know why, so I do it myself. but the HC interrupt works well, the cameraHXferCallback() will be called
	if(__HAL_DMA_GET_FLAG(hdcmi.DMA_Handle, __HAL_DMA_GET_TC_FLAG_INDEX(hdcmi.DMA_Handle)) != RESET){
		#ifdef DEBUG_CAMERA
		DISABLE_IRQ
		printf("TC");
		ENABLE_IRQ
		#endif
		if( 15 == block_num ){ // at the begining of each frame, there is a TC interrupt, I don't know where it come from, just ignore it. the first DMA INT should be HT
			return;
		}
		blockReceived();
	}else if( __HAL_DMA_GET_FLAG(hdcmi.DMA_Handle, __HAL_DMA_GET_DME_FLAG_INDEX(hdcmi.DMA_Handle)) != RESET ){
		#ifdef DEBUG_CAMERA
		printf("mde\r\n");
		#endif
	}else if( __HAL_DMA_GET_FLAG(hdcmi.DMA_Handle, __HAL_DMA_GET_FE_FLAG_INDEX(hdcmi.DMA_Handle)) != RESET ){
		#ifdef DEBUG_CAMERA
		printf("fe\r\n");
		#endif
	}
}
void color2gray(){
	p_gray = &(gray[gray_buffer_n][block_num*BUFFER_ROWS][0]);
	p_color_in = &(lines_buffer[pos_buffer][0][0]);
	for(i=0; i<BUFFER_ROWS*IMAGE_WIDTH; i++){
		#ifdef RGB565
		*p_gray = *p_color_in & 0x00f8;
		#elif defined YUV1
		*p_gray = *p_color_in;
		#else 
		#error no format specified
		#endif
		p_gray ++;
		p_color_in ++;
	}
	/*
	for(i=; i<(block_num+1)*BUFFER_ROWS; i++){
		for(j=0; j<IMAGE_WIDTH; j++){
			//pPixel8 = (uint8_t *)&lines_buffer[pos_buffer][i][j];
			gray[pos_gray_buffering][i][j] = lines_buffer[pos_buffer][i][j] & 0xf8;
			//pRGB565 = (uint16_t *)RGB565;
			//gray[1-pos_gray_calc][ line_base + i][j] = (r*38 + g*75 + b*15) >> 7;
		}
	} */
	/*
		pPixel8 = (uint8_t *)p_color_in;
		RGB565[0] = *(pPixel8 + 1);
		RGB565[1] = *pPixel8;
		pRGB565 = (uint16_t *)RGB565;
		b = *pRGB565 & (0x001F) << 3;
		g = (*pRGB565 & (0x003F<<5)) >> 3;
		r = (*pRGB565 & (0x001F<<11)) >> 8;
		*p_gray = (r*38 + g*75 + b*15) >> 7;
		*/ //only red
}
