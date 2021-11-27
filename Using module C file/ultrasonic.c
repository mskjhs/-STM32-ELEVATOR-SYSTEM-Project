#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TRIG_PORT GPIOC
#define TRIG_PIN  GPIO_PIN_5

void ultrasonic_processing();
extern volatile int TIM11_10ms_ultrasonic_counter;

uint32_t distance=0;  // 거리
uint8_t ic_cpt_flag=0;   // rising edge/falling edge를 detect하는 flag
// rising edge/falling edge INT가 발생되면 이곳으로 집입
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t is_first_captured=0;

	if ( is_first_captured == 0 )   // risring edge detect !!!
	{
		__HAL_TIM_SET_COUNTER(htim,0);
		is_first_captured=1;   // risring edge detect flag set
	}
	else if (is_first_captured == 1)  // falling edge INT detect
	{
		is_first_captured=0;
		distance = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		ic_cpt_flag=1;
	}
}

void ultrasonic_processing()
{
	if (TIM11_10ms_ultrasonic_counter >= 100)   // timer 1sec reached
	{
		TIM11_10ms_ultrasonic_counter=0;
		make_trigger();
		if (ic_cpt_flag==1)
		{
			ic_cpt_flag=0;
			distance = distance * 0.034 / 2;  // 1usdp 0.034cm가 이동 하는데 /2는 왕복 값이 오기 떄문데 편도만 필요
			printf("distance : %d\n", distance);
		}
	}
}
void make_trigger(void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}
