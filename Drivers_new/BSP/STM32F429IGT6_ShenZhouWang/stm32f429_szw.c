#include "main.h"
#include "stm32f429_szw.h"

bool button_state_old=0, button_state=0; // true: pressed ;  false: released
bool button_pressed(void){	
	button_state = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) );
	return button_state;
}
bool button_pressing(void){
	button_state_old = button_state;
	button_state = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) );
	if( button_state ^ button_state_old ){
		if( button_state ){
			return true;
		}else{
			return false;
		}
	}
	return false;
}
bool button_releasing(void){	
	button_state_old = button_state;
	button_state = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) );
	if( button_state ^ button_state_old ){
		if( button_state ){
			return false;
		}else{
			return true;
		}
	}
	return false;
}

void test_led(uint32_t t){
  /* Configure LED1, LED2, LED3 and LED4 */
	HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
	HAL_Delay(t);	
	HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET);
  /* Configure the System clock to 180 MHz */
}
/*
void sram512K_init(){
	SRAM_HandleTypeDef hsram;
	FMC_NORSRAM_TimingTypeDef SRAM_Timing;
  hsram.Instance  = FMC_NORSRAM_DEVICE;
  hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;
  
  SRAM_Timing.AddressSetupTime       = 0;
  SRAM_Timing.AddressHoldTime        = 0;
  SRAM_Timing.DataSetupTime          = 4;
  SRAM_Timing.BusTurnAroundDuration  = 1;
  SRAM_Timing.CLKDivision            = 0;
  SRAM_Timing.DataLatency            = 0;
  SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;
  
  hsram.Init.NSBank             = FMC_NORSRAM_BANK3;
  hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram.Init.MemoryType         = FMC_MEMORY_TYPE_PSRAM;
  hsram.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram.Init.WrapMode           = FMC_WRAP_MODE_DISABLE;
  hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
  hsram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
  hsram.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
	HAL_SRAM_DeInit(&hsram);
  // Initialize the SRAM controller 
  if(HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
  {
    //Initialization Error
  }
}

bool test_sram512K(){
  uint32_t uwIndex = 0;
  
  // Write data to the SRAM memory 
  for (uwIndex = 0; uwIndex < 0x10000; uwIndex++)
  {
    *(__IO uint16_t*) (SRAM_BANK_ADDR + 2*uwIndex) = 0xAAAA;
  }
  
  // Read back data from the SRAM memory 
  for (uwIndex = 0; uwIndex < 0x10000; uwIndex++)
  {
    if( 0xAAAA != *(__IO uint16_t*) (SRAM_BANK_ADDR + 2*uwIndex) ){
			return false;
		}
  }
	return true;
}
*/
