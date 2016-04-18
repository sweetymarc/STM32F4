/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stm32f429_camera.h"
#include "OV7725.h"
#include "stdlib.h"
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

bool Uart3Ready = true, Uart1Ready = true;

//variables  JPEG encode
extern uint8_t *image;
uint8_t *JPG_enc_buf, pos_JPG=1;
uint32_t pt_buf, JPG_length[2];
int  width = 320;//图像的宽度
int  height = 240;//图像的高度
void exit(int i){ while(1);}
extern void jpeg_encode(void);
// for esp8266
uint8_t payload8266[512];
bool esp8266_exist=false, handshaking = false;
int time_last_send = 0;
int payload_length, context_length;
uint8_t *context;

/* IWDG and TIM handlers declaration */
IWDG_HandleTypeDef IwdgHandle;
TIM_HandleTypeDef  TimInputCaptureHandle;
RCC_ClkInitTypeDef RCC_ClockFreq;
void IWDG_init();

static void SystemClock_Config(void);
void Error_Handler(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	int i_encode;
	
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();

	uartInit();
	if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer8266, RXBUFFERSIZE_8266) != HAL_OK)
  {
    while(1);
  }
	HAL_Delay(1000);
	if( esp8266_init() ){
		esp8266_exist = true;
	}
	if( !esp8266_exist ){
		printf("stm32f407VET6 uart init ok %d\r\n", HAL_GetTick());
	}
	//rxBuffer[15] = '\0';  
	uart1Init(); uart3Init();
	if( HAL_UART_Receive_DMA(&Uart3Handle, (uint8_t*)mavlink_buffer, MAVLINK_BUFFER_SIZE) != HAL_OK ){
		Error_Handler();
	}
	OV7725_Init();
	HAL_LED_MspInit(); // IIC sda pin is reused, iic can never be used again
	TIM_PWM_init();
	uSD_init();
	vision_control_init();
	
	sonar_init();
	SPI_Config();
	nrf_init();
	
	if( !esp8266_exist ){
		printf("start capture\r\n");
	}
	if ( esp8266_exist ){
		
		esp8266_init();
		if(  !esp8266_cmd("AT+CWMODE=2\r\n", "no change\r\n", 1000) && !esp8266_cmd("AT+CWMODE=2\r\n", "\r\nOK\r\n", 100) ){
			while(1);
		}
		if( !esp8266_cmd("AT+CIPMUX=1\r\n", "\r\nOK\r\n", 1000) ){
			while(1);
		}
		if( !esp8266_cmd("AT+CIPSERVER=1,8080\r\n", "\r\nOK\r\n", 2000) ){
			while(1);
		}
	}
	BSP_Camera_Start();
	#ifdef WATCH_DOG
	IWDG_init();
	#endif
	image = &gray[0][0][0];
	while(1){
		/*
		if( GPIO_PIN_SET == HAL_GPIO_ReadPin(SEND_SWITCH_PORT, SEND_SWITCH_PIN) ){
			image_send = true;
		}else{
			image_send = false;
		}*/
		if( nrf_send_finished && vision.gray_send ){//
			nrf_send_IT(JPG_buf[1-pos_JPG], JPG_length[1-pos_JPG] );
			//lanuch JPEG encoding, blocking, the main cycle will stay here until the encoding finished
			JPG_enc_buf = JPG_buf[pos_JPG];
			jpeg_encode();
			for(i_encode = 0; i_encode < 128; i_encode++){
				JPG_enc_buf[pt_buf + i_encode] = 0;
			}
			JPG_length[pos_JPG] = pt_buf + 128;
			pos_JPG ++;  //always point to the buffer that be used by JPEG_encoder 
			pos_JPG %= 2;
		}
		if( nrf_send_finished && vision.bin_send ){
			// the first 4 bytes are replaced to 0xAAAAAAAA, as sync code
			memcpy(bin_DMA+1, bin+1, IMAGE_SIZE/8-4); // bin_DMA and bin are uint32_t, size is in byte
			nrf_send_IT((uint8_t *)bin_DMA, 9600 );
			}// problem remained : spi_busy not reset after working about 10min
		if( esp8266_exist ){
			switch( esp8266_parse(payload8266, &payload_length) ){ //maybe more than one event in the messages, but parse only one each time
				case NO_EVENT :
					break;
				case LINK :
					break;
				case UNLINK :
					websocketConnected = 0;
					break;
				case PAYLOAD :
					if( 0 == strncmp((const char *)payload8266, "GET /chat", 9) ){ //handshake
						web_soket_handshake( (char *)payload8266 );
						handshaking = true;// returned before the handshake data reached remote
					}else if( 0x81 == *payload8266 ){ //websocket data
						rm_mask(payload8266, payload_length, &context, &context_length);
						if( 0 == strncmp( (char *)context, "set ", 4 ) ){
							context[context_length] = '\0';
							set_parameter( (char *)context + 4);
						}else if( 0 == strncmp( (char *)context, "xxxxxx", 3 ) ){	
						}
					}
					break;
				case SEND_OK :
					if( handshaking ){
						websocketConnected = true; 
						handshaking = false;
					}
					break;
				case RST :
					break;
				default :
					break;
			}
			if( websocketConnected && HAL_GetTick() - time_last_send > 50 && 0 == update_length() && esp8266_sending==false ){
				time_last_send = HAL_GetTick();
				websocket_send( sprintf( (char *)txBuffer8266+2, "%d %.3f %.3f %.3f %.3f %d %d %.3f %.3f %.3f %.3f %.3f %.3f %d %d %d %d %d %d", v_out.count, v_out.x_est, v_out.y_est, v_out.vx_est, v_out.vy_est, 
																																										rc_ch, v_out.height, v_out.ux, v_out.uy,
																																										v_out.roll, v_out.pitch, v_out.x_measure, v_out.y_measure, 
																																										v_out.rawHeight, v_out.period, v_out.searchCost, v_out.wholeCost,
																																										v_out.diameterRow, v_out.diameterCol) );
			}
		}else{
			if( frame_int ){
				frame_int = 0;
				DISABLE_IRQ
				printf( "%d h: %d a: %.3f %.3f m: %.3f %.3f s: %.3f %.3f e: %.3f %.3f ve: %.3f %.3f u: %.3f %.3f rc_ch: %d\r\n" , 
					v_out.count, v_out.height, v_out.roll, v_out.pitch, v_out.x_measure, v_out.y_measure, v_out.x_sim, v_out.y_sim, v_out.x_est, v_out.y_est, v_out.vx_est, v_out.vy_est, v_out.ux, v_out.uy, rc_ch);
				ENABLE_IRQ
			}
			context_length = update_length();
			if( context_length ){
				copy_buffer((uint8_t *)txBuffer8266,-2);  //"x=123\r\n"
				if( 0 == strncmp(txBuffer8266, "\r\n", 2) ){
					copy_buffer((uint8_t *)txBuffer8266, context_length-2);
					txBuffer8266[context_length-2] = '\0';
					set_parameter(txBuffer8266);
					move_pos(context_length);
				}
			}
		}//if esp8266_exist
		#ifdef WATCH_DOG
		HAL_IWDG_Refresh(&IwdgHandle);
		#endif
	}
}

volatile void soft_delay(volatile uint32_t n){ // 200MHz, n = 300 , about 10us
	while(n--);
}

void IWDG_init(){
  /*##-1- Check if the system has resumed from IWDG reset ####################*/
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  { 
    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
  /*##-3- Configure the IWDG peripheral ######################################*/
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq / 32
     Counter Reload Value = 250ms / IWDG counter clock period
                          = 0.25s / (32/LsiFreq)
                          = LsiFreq / (32 * 4)
                          = LsiFreq / 128 */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
  IwdgHandle.Init.Reload    = 32000 / 32;  //1s : 32; 0.5s : 64;  0.25:128
  
  if(HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-4- Start the IWDG #####################################################*/ 
  if(HAL_IWDG_Start(&IwdgHandle) != HAL_OK)
  {
    Error_Handler();
  }
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
	
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 380;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
