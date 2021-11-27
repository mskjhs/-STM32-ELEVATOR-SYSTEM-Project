#include "main.h"

#define FIRSTFLOOR		1
#define	SECONDFLOOR		2
#define THIRDFLOOR		3
#define IDLE		    1
#define	FORWARD		    2
#define BACKWARD		3

#define SETPPERREV 4096    // 모터 1회전 하는데 필요한 step수
void stepmotor_half_driver(int step);
void stepmotor_main();


