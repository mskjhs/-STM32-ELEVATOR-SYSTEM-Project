#include "led_onoff.h"
#include "button.h"

extern volatile int Tms_counter;
extern void delay_us(unsigned long us);

void button1_ledall_on_off()
{
	static int button1=0;

	if (getButtonState(BUTTON1_GPIO_Port, BUTTON1_Pin, &prevButton1State))
	{
		button1++;
		button1 %=2;  // 1 0 1 0 1 0
		if (button1)  // if (button1 >= 1)
		{
			 led_all_on();
printf("led_all_on()\n");
		}
		else
		{
			 led_all_off();
printf("led_all_off()\n");
		}
	}
}

void led_on_up()     // 0 ->01 -> 012 -> 0123 --- 01234567
{
	for (int i=0; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOB, 0x01 << i, GPIO_PIN_SET);
		HAL_Delay(200);
	}
}

void led_on_down()
{
	for (int i=0; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOB, 0x80 >> i, GPIO_PIN_SET);
		HAL_Delay(200);
	}
}

void led_blink_up()
{
	for (int i=0; i < 8; i++)
	{
		led_all_off();
		HAL_GPIO_WritePin(GPIOB, 0x01 << i, GPIO_PIN_SET);
		HAL_Delay(200);
	}
}


void led_blink_down()
{
	for (int i=0; i < 8; i++)
	{
		led_all_off();
		HAL_GPIO_WritePin(GPIOB, 0x80 >> i, GPIO_PIN_SET);
		HAL_Delay(200);
	}
}

void led_all_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
}

void led_all_off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
}
//3.
void demoboard_led2_toggle_button()
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}

//2.
void demoboard_led2_button_on_off()
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}
// 1.
void demoboard_led2_onoff_hal_delay()
{
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  delay_us(65530);   // 65ms 65535
//	   HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	   HAL_Delay(1000);
	  delay_us(65530);   // 65ms
}

void demoboard_led2_onoff_systick()
{
	if (Tms_counter >= 1000)  //   if (Tms_counter >= 500)
	{
		Tms_counter=0;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}
