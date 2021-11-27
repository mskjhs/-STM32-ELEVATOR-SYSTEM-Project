
#include "stepmotor.h"
#include "button.h"
//extern volatile uint8_t state_flag;
//
//extern volatile int TIM11_10ms_counter1;

extern void lcd_command(uint8_t command);
uint8_t runstate = IDLE;	// uint8_t = unsigned char
uint8_t prev_runstate = FORWARD;
uint8_t floor = FIRSTFLOOR;
uint8_t destination = FIRSTFLOOR;
//uint8_t state_lcd = 0;
// 4096 / 8(0.7도) = 512 sequence  : 360
void stepmotor_half_driver(int directrion)
{
	static int step = 0;

	switch (step)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
	}

	if (directrion == FORWARD)  // for (int =0; i < 8; i++)
	{
		step++;
		step %= 8;
	}
	else if (directrion == BACKWARD)  // BACKWARD  // for (int =7; i >=0; i--)
	{
		step--;
		if (step < 0) step=7;
	}
}

// 모터가 1바퀴(360도) 도는데 512개의 sequence가 걸림.
// 1sequence당 0.70312도
// 0.70312 x 512 = 360도
// 이걸 이용해서 sequence를 계산
void stepmotor_main()
{
	while(1)
	{
		switch(runstate)
		{
		case IDLE:
			if (getButtonState(BUTTON1_GPIO_Port, BUTTON1_Pin, &prevButton1State))
			{
				runstate=FORWARD;
			}
			break;
		case FORWARD:
			stepmotor_half_drive(FORWARD);
			delay_us(1126);
			if (getButtonState(BUTTON1_GPIO_Port, BUTTON1_Pin, &prevButton1State))
			{
				runstate=BACKWARD;
			}
			break;
		case BACKWARD:
			stepmotor_half_drive(BACKWARD);
			delay_us(1126);
			if (getButtonState(BUTTON1_GPIO_Port, BUTTON1_Pin, &prevButton1State))
			{
				runstate=FORWARD;
			}
			break;
		}
	}
}

void elevator_button()
{
	if (getButtonState(BUTTON1_GPIO_Port, BUTTON1_Pin, &prevButton1State))
	{
		destination = FIRSTFLOOR;
		if ((floor == SECONDFLOOR) || (floor == THIRDFLOOR))
			{
		    	move_cursor(1,0);
			    lcd_string("               ");
				move_cursor(1,0);
				lcd_string("====BACKWARD====");
			}

	}
	if (getButtonState(BUTTON2_GPIO_Port, BUTTON2_Pin, &prevButton2State))
	{
		destination = SECONDFLOOR;
		if (floor == FIRSTFLOOR)
			{
			    move_cursor(1,0);
			    lcd_string("                 ");
				move_cursor(1,0);
				lcd_string("====FORWARD====");
			}
		else if (floor == THIRDFLOOR)
			{
			    move_cursor(1,0);
			    lcd_string("                 ");
			    move_cursor(1,0);
				lcd_string("====BACKWARD====");
			}
	}
	if (getButtonState(BUTTON3_GPIO_Port, BUTTON3_Pin, &prevButton3State))
	{
		destination = THIRDFLOOR;
		if ((floor == FIRSTFLOOR) || (floor == SECONDFLOOR))
		{
		    move_cursor(1,0);
			lcd_string("                 ");
			move_cursor(1,0);
			lcd_string("====FORWARD====");
		}
	}
}



void elevator_processing()
{

	elevator_button();

	switch (destination)
	{
		case FIRSTFLOOR:
		{

			if ((floor == SECONDFLOOR) || (floor == THIRDFLOOR))
			{
				stepmotor_half_driver(BACKWARD);
				delay_us(1126);
			}
		}
		break;
		case SECONDFLOOR:
		{

			if (floor == FIRSTFLOOR)
			{
				stepmotor_half_driver(FORWARD);
				delay_us(1126);
			}
			else if (floor == THIRDFLOOR)
			{

				stepmotor_half_driver(BACKWARD);
				delay_us(1126);
			}
		}
		break;
		case THIRDFLOOR:
		{

			if ((floor == FIRSTFLOOR) || (floor == SECONDFLOOR))
			{


				stepmotor_half_driver(FORWARD);
				delay_us(1126);
			}
		}
		break;
	}
}





#if 0
void elevator_processing()
{
	if(state_flag == 1)
	{
		upelv();
		state_flag =0;
	}
	if(state_flag == 1)
	{
		downelv();
		state_flag =0;
	}
	elevator_button();

	switch (destination)
	{
		case FIRSTFLOOR:
		{

			if ((floor == SECONDFLOOR) || (floor == THIRDFLOOR))
			{
				if (TIM11_10ms_counter1 >= 11) // 1500ms
					{

					TIM11_10ms_counter1 =0;
				     stepmotor_half_driver(BACKWARD);
					}
			}
		}
		break;
		case SECONDFLOOR:
		{

			if (floor == FIRSTFLOOR)
			{
				if (TIM11_10ms_counter1 >= 11) // 1500ms
					{
					TIM11_10ms_counter1 =0;
					stepmotor_half_driver(FORWARD);
					}
			}
			else if (floor == THIRDFLOOR)
			{
				if (TIM11_10ms_counter1 >= 11) // 1500ms
					{

					TIM11_10ms_counter1 =0;
				     stepmotor_half_driver(BACKWARD);
					}
			}
		}
		break;
		case THIRDFLOOR:
		{

			if ((floor == FIRSTFLOOR) || (floor == SECONDFLOOR))
			{

				if (TIM11_10ms_counter1 >= 11) // 1500ms
					{

					TIM11_10ms_counter1 =0;
				     stepmotor_half_driver(FORWARD);
					}
			}
		}
		break;
	}
}
#endif

#if 0   // DEMO Version
#define SETPPERREV 4096   // 모터 1회전 하는데 필요한 step수
// 4096 / 8(0.7도) = 512 sequence  : 360
void stepmotor_half_drive(int step)
{
	switch (step)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
	}
}

// 모터가 1바퀴(360도) 도는데 512개의 sequence가 걸림.
// 1sequence당 0.70312도
// 0.70312 x 512 = 360도
// 이걸 이용해서 sequence를 계산
void setpmotor_main()
{
	while(1)
	{
		for (int i=0; i < 512; i++)  // 시계 방향
		{
			for (int j=0; j < 8; j++)
			{
				stepmotor_half_drive(j);
				delay_us(1126);
			}
		}

		for (int i=0; i < 512; i++)   // 시계 반대 방향
		{
			for (int j=7; j >= 0; j--)
			{
				stepmotor_half_drive(j);
				delay_us(1126);
			}
		}
	}
}
#endif









