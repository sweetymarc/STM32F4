#include "main.h"
#include "v1.0/common/mavlink.h"

#define APB2_DIVIDER 4 
// TIM PWM
uint64_t  CLK_CNT; //1125000;

static uint32_t               uwIC2Value1 = 0;
static uint32_t               uwIC2Value2 = 0;
static uint32_t               uwDiffCapture = 0;
static uint32_t							uwPrescalerValue = 0;
static uint16_t               uhCaptureIndex = 0;
uint32_t rc_ch;
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle_PWM;
/* Timer Output Compare Configuration Structure declaration */
TIM_IC_InitTypeDef sICConfig;
/* Counter Prescaler value */
uint32_t uwPrescalerValue_PWM = 0;

void TIM_PWM_init(){
	CLK_CNT = (SystemCoreClock/APB2_DIVIDER)/50;
  TimHandle_PWM.Instance = TIM_PWM;
  
  TimHandle_PWM.Init.Prescaler     = ((SystemCoreClock /APB2_DIVIDER) / CLK_CNT) - 1;
  TimHandle_PWM.Init.Period        = 0xffff;
  TimHandle_PWM.Init.ClockDivision = 0;
  TimHandle_PWM.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle_PWM) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimHandle_PWM, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&TimHandle_PWM, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

void rc_ch_CaptureCallback(TIM_HandleTypeDef *htim) // sonar echo pin rising or falling
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if(uhCaptureIndex == 0 && GPIO_PIN_SET == HAL_GPIO_ReadPin(TIM_PWM_GPIO_PORT, GPIO_PIN_CHANNEL1 ))
    {
      /* Get the 1st Input Capture value */
      uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1 && GPIO_PIN_RESET == HAL_GPIO_ReadPin(TIM_PWM_GPIO_PORT, GPIO_PIN_CHANNEL1 ))
    {
      /* Get the 2nd Input Capture value */
      uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 
      
      /* Capture computation */
      if (uwIC2Value2 > uwIC2Value1)
      {
        uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
      }
      else  /* (uwIC2Value2 <= uwIC2Value1) */
      {
        uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
      }
			rc_ch = (uint64_t)uwDiffCapture*1000000/CLK_CNT;
      uhCaptureIndex = 0;
    }
  }
}