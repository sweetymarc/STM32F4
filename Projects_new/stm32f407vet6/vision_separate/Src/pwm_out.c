#include "main.h"
#include "v1.0/common/mavlink.h"

#define APB2_DIVIDER 4 
// TIM PWM
uint16_t	roll=1500, pitch=1500, throttle=1500, yaw=1500;
uint64_t  CLK_CNT; //1125000;
uint64_t  CH1_roll;      /* Capture Compare 1 Value  */
uint64_t  CH2_pitch;        /* Capture Compare 2 Value  */
uint64_t  CH3_throttle;      /* Capture Compare 3 Value  */
uint64_t  CH4_yaw;        /* Capture Compare 4 Value  */
int roll_c = 1416, pitch_c = 1570;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle_PWM;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig_PWM;
/* Counter Prescaler value */
uint32_t uwPrescalerValue_PWM = 0;

void TIM_PWM_init(){
	CLK_CNT = (SystemCoreClock/APB2_DIVIDER)/50;
  TimHandle_PWM.Instance = TIM_PWM;
  
  TimHandle_PWM.Init.Prescaler     = ((SystemCoreClock /APB2_DIVIDER) / CLK_CNT) - 1;
  TimHandle_PWM.Init.Period        = CLK_CNT/50-1;
  TimHandle_PWM.Init.ClockDivision = 0;
  TimHandle_PWM.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&TimHandle_PWM) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
  sConfig_PWM.OCMode     = TIM_OCMODE_PWM1;
  sConfig_PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig_PWM.OCFastMode = TIM_OCFAST_DISABLE;
	
	/* Set the pulse value for channel 1 */
	CH1_roll = CLK_CNT*roll/1000000;       /* Capture Compare 1 Value  */
	CH2_pitch = CLK_CNT*pitch/1000000;         /* Capture Compare 2 Value  */
	CH3_throttle = CLK_CNT*throttle/1000000;         /* Capture Compare 3 Value  */
	CH4_yaw = CLK_CNT*yaw/1000000;         /* Capture Compare 4 Value  */
  sConfig_PWM.Pulse = CH1_roll ;  
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle_PWM, &sConfig_PWM, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Set the pulse value for channel 2 */
  sConfig_PWM.Pulse = CH2_pitch;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle_PWM, &sConfig_PWM, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Set the pulse value for channel 3 */
  sConfig_PWM.Pulse = CH3_throttle;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle_PWM, &sConfig_PWM, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Set the pulse value for channel 4 */
  sConfig_PWM.Pulse = CH4_yaw;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle_PWM, &sConfig_PWM, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&TimHandle_PWM, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&TimHandle_PWM, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&TimHandle_PWM, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 4 */
  if(HAL_TIM_PWM_Start(&TimHandle_PWM, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

void update_pwm(double x, double y){
	roll	= roll_c + x*57.3f*10.0f;
	pitch = pitch_c + y*57.3f*10.0f;
	__HAL_TIM_SET_COMPARE(&TimHandle_PWM, TIM_CHANNEL_1, CLK_CNT*roll/1000000);
	__HAL_TIM_SET_COMPARE(&TimHandle_PWM, TIM_CHANNEL_2, CLK_CNT*pitch/1000000);
}

static mavlink_message_t tx_msg;
extern uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

void mavlink_rc(double x, double y, bool give_up){
	if( give_up ){
		roll = 0;
		pitch = 0;
	}else{
		roll	= roll_c + x*57.3f*10.0f;
		pitch = pitch_c + y*57.3f*10.0f;
	}
	// Pack the message
	mavlink_msg_rc_channels_override_pack(0xff, 0xbe, &tx_msg, 0x01, 0x01, roll, pitch, 0, 0, 0, 0, 0, 0);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &tx_msg);
	if( Uart1Ready ){ //
		Uart1Ready = false;
		HAL_UART_Transmit_IT(&Uart1Handle, buf, len);
	}/*else{
		HAL_UART_DeInit(&Uart1Handle);
		uart1Init();
		Uart1Ready = true;
	}*/
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  
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
}
