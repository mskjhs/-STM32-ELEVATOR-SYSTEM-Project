#include "button.h"

uint8_t prevButton1State = RELEASED;
uint8_t prevButton2State = RELEASED;
uint8_t prevButton3State = RELEASED;

uint8_t getButtonState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t *prevState)
{
	uint8_t curState;

	curState = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	if ( (curState == PRESSED) && (*prevState == RELEASED))
	{
		*prevState = curState;
		HAL_Delay(200);
		return 0;
	}
	else if ( (curState == RELEASED) && (*prevState == PRESSED))
	{
		*prevState = curState;
		return 1;
	}
	return 0;
}
