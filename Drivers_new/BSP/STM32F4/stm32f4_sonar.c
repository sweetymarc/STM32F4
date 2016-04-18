#include "main.h"
#include "stm32f4_sonar.h"

//variable measure distance
TIM_HandleTypeDef      TimHandleSonar;
static TIM_IC_InitTypeDef     sICConfig;
static uint32_t               uwIC2Value1 = 0;
static uint32_t               uwIC2Value2 = 0;
static uint32_t               uwDiffCapture = 0;
static uint32_t							uwPrescalerValue = 0;
static uint16_t               uhCaptureIndex = 0;
int sonar_height[SONAR_FILTER]; // mm mm
int h_i=SONAR_FILTER-1, h_average;

void sonar_init(){
	BSP_sonar_MspInit();
  uwPrescalerValue = (uint32_t) ((SystemCoreClock/4) / 100000) - 1;  //5m 14ms 1400
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* Set TIMx instance */
  TimHandleSonar.Instance = TIMsonar;

  /* Initialize TIMx peripheral as follows:
       + Period = 0xFFFF
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandleSonar.Init.Period            = 0xFFFF;
  TimHandleSonar.Init.Prescaler         = uwPrescalerValue;
  TimHandleSonar.Init.ClockDivision     = 0;
  TimHandleSonar.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandleSonar.Init.RepetitionCounter = 0;

  if(HAL_TIM_IC_Init(&TimHandleSonar) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimHandleSonar, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    while(1);
  }
  
  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&TimHandleSonar, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    while(1);
  }
}

void sonar_trig(){
	HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_SET);
	soft_delay(300);
	HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
}
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim: TIM handle
  * @retval None
  */
uint32_t i_height;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // sonar echo pin rising or falling
{
	if( htim == &TimHandleSonar){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(uhCaptureIndex == 0 && GPIO_PIN_SET == HAL_GPIO_ReadPin(ICx_PORT, ICx_PIN ))
			{
				/* Get the 1st Input Capture value */
				uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				uhCaptureIndex = 1;
			}
			else if(uhCaptureIndex == 1 && GPIO_PIN_RESET == HAL_GPIO_ReadPin(ICx_PORT, ICx_PIN ))
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
				
				/* Capture computation */
				if (uwIC2Value2 > uwIC2Value1)
				{
					uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
				}
				else  /* (uwIC2Value2 <= uwIC2Value1) */
				{
					uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
				}
				h_i ++;
				h_i %= SONAR_FILTER;
				sonar_height[h_i] = (uwDiffCapture*17)/10;
				if( HAL_GetTick() > 1000 && (abs(sonar_height[h_i] - h_average) > SONAR_STEP_LIMIT )){
					sonar_height[h_i] = h_average;
				}
				h_average = 0;
				for(i_height = 0; i_height < SONAR_FILTER; i_height++){
					h_average += sonar_height[i_height];
				}
				h_average /= SONAR_FILTER;
				/* Frequency computation: for this example TIMx (TIM1) is clocked by
					 2xAPB2Clk */      
				//uwFrequency = (2*HAL_RCC_GetPCLK2Freq()) / uwDiffCapture;
				uhCaptureIndex = 0;
			}
		}
	}
#ifdef INPUT_PWM
	else if( htim == &TimHandle_PWM ){
		rc_ch_CaptureCallback(htim);		
	}
#endif
}