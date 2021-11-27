#include "main.h"
#include "stepmotor.h"
//#include "stepmotor.h"
#define  COMMAND_MAX  50
#define  COMMAND_LENGTH 30
extern uint8_t floor;
extern uint8_t destination;
// Bluetooth
volatile uint8_t bt_buffer[COMMAND_LENGTH];   //  BT로 부터 수신된 char를 저장하는 공간 \n을만날때 까지 저장
volatile int bt_write_index=0;
volatile uint8_t bt_rx_cmd_flag=0;   // 완전한 문장을 만났다(\n)는 indicator 예)led0on\n

// PC
volatile uint8_t command_buffer[COMMAND_MAX][COMMAND_LENGTH];  // 3.
volatile int command_index=0;
volatile int read_index=0;   // while에서 가져 가는것
volatile int write_index=0;  // UART save
uint8_t command_count=0;  // command_buffer에 명령이 몇개가 들어 있는지를 알려 주는 indicator
// pc로부터 온 명령어를 저장하는 2차원 array(circular queue)
volatile uint8_t rx_data;   // 1. PC로 부터 1byte의 INT가 들어오면 저장 하는 변수
volatile uint8_t rxbt_data; // 1. BT로 부터 1byte의 INT가 들어오면 저장 하는 변수
uint8_t rx_cmd_flag;   // 완전한 문장을 만났다(\n)는 indicator 예)led0on\n

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern void SetRTCTime(char *date_time);

void buletooth_command_processing();
void pc_command_processing();

volatile uint8_t state_flag = 0;
// UART로 부터 1byte가 수신되면 H/W가 call을 해 준다.
// UART1 / UART2번으로 부터 1 byte가 수신(stop bit) 하면 rx interrupt가 발생
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)   // PC
	{
		// 30byte가 넘는 command에 대한 예외 처리
		if (write_index < COMMAND_LENGTH)
		{
			if (rx_data == '\n' || rx_data == '\r')
			{
				rx_cmd_flag=1;   // 완전한 command를 만났다는 indicator flag를 set
				write_index=0;  // 다음 command를 저장하기 위해서 변수를 0으로
				command_index++;  // 다음 저장할 command index를 준비
				command_index %= COMMAND_MAX;
				command_count++;   // 현재 들어온 command 갯수
			}
			else
			{
				command_buffer[command_index][write_index++] = rx_data;
			}
		}
		else  // 30byte가 넘는 command에 대한 예외 처리
		{
			printf("buffer overflow !!!\n");
		}

		// 주의: 반드시 HAL_UART_Receive_IT를 call 해주야 다음 INT가 발생
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

		// led1on\n
		// 1. command_buffer[command_index][write_index] = rx_data;
		// 2. write_index++
	}
	if (huart == &huart1)   // bluetooth
	{
		// 30byte가 넘는 command에 대한 예외 처리
		if (bt_write_index < COMMAND_LENGTH)
		{
			if (rxbt_data == '\n' || rxbt_data == '\r')
			{
				bt_buffer[bt_write_index] = 0;  // '\0'
				bt_write_index=0;  // 다음 save할 index를준비
				bt_rx_cmd_flag=1;
			}
			else
			{
				bt_buffer[bt_write_index++] = rxbt_data;
			}
		}
		else
		{
			bt_write_index=0;  // 다음 save할 index를준비
			printf("BT buffer overflow !!!\n");
		}
		// 주의: 반드시 HAL_UART_Receive_IT를 call 해주야 다음 INT가 발생
		HAL_UART_Receive_IT(&huart1, &rxbt_data, 1);
	}
}

// uart2
void pc_command_processing()
{
	if (command_count)   // command_buffer에 message가 있으면
	{
		command_count--;
		rx_cmd_flag=0;
		printf("pc: %s\n", command_buffer[read_index]);  // &command_buffer[read_index][0]
		if ( !strncmp( (const char *)command_buffer[read_index], "led2on", 6))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
		if ( !strncmp( (const char *)command_buffer[read_index], "led2off", 7))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		if ( !strncmp( (const char *)command_buffer[read_index], "setrtc", 6))
		{
			SetRTCTime(command_buffer[read_index]);
		}
		memset(command_buffer[read_index], 0, COMMAND_LENGTH);
		read_index++;
		read_index %= COMMAND_MAX;   // 다음 읽을 pointer를 계산
	}
}

// uart1
void buletooth_command_processing()
{
	if (bt_rx_cmd_flag)   // BT로 부터 완전한 command가 들어 왔으면
	{
		bt_rx_cmd_flag=0;
		printf("%s\n", bt_buffer);
		if ( !strncmp( (const char *)bt_buffer, "led2on", 6))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
		if ( !strncmp( (const char *)bt_buffer, "led2off", 7))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		if ( !strncmp( (const char *)bt_buffer, "first", 5))
		{
			move_cursor(0,0);
			lcd_string("====ELEVATOR====");
			destination = FIRSTFLOOR;
			if ((floor == SECONDFLOOR) || (floor == THIRDFLOOR))
						{
					    	move_cursor(1,0);
						    lcd_string("               ");
							move_cursor(1,0);
							lcd_string("====BACKWARD====");
						}
		}
		if ( !strncmp( (const char *)bt_buffer, "second", 6))
		{
			move_cursor(0,0);
			lcd_string("====ELEVATOR====");
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
		}
		if ( !strncmp( (const char *)bt_buffer, "third", 5))
		{
			move_cursor(0,0);
			lcd_string("====ELEVATOR====");
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



void uart_main()
{

}







