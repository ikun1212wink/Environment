#include "led.h"


led_t led = {

	.allLight = &Led_All_Light,
	.allShine = &Led_All_Shine,

};

void Led_All_Light(void)
{
	HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
}

void Led_All_Shine(uint16_t time)
{
	static uint16_t shine_cnt = 0;
	
	shine_cnt++;
	
	if(shine_cnt == time){
		
		HAL_GPIO_TogglePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin);
		shine_cnt = 0;
	}
}







