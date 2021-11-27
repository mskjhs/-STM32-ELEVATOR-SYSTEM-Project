#include "main.h"

#define BUTTON1_Pin GPIO_PIN_0
#define BUTTON1_GPIO_Port GPIOC
#define BUTTON2_Pin GPIO_PIN_1
#define BUTTON2_GPIO_Port GPIOC
#define BUTTON3_Pin GPIO_PIN_2
#define BUTTON3_GPIO_Port GPIOC

#define RELEASED 1
#define PRESSED  0   // S/W 눌렸을때  ACTIVE LOW

extern uint8_t prevButton1State;  // extern의 의미는 현재 변수 prevButton1State는 다른 화일에 정의 되어 있다고 compiler에 알려주기 위한것이다.
extern uint8_t prevButton2State;
extern uint8_t prevButton3State;

uint8_t getButtonState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t *prevState);
