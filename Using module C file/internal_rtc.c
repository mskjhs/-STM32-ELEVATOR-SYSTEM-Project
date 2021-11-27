#include "main.h"

extern RTC_HandleTypeDef hrtc;
//extern volatile uint8_t i_RH, d_RH, i_Tmp, d_Tmp;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
// 예: 21
// high nibble low nibble
// 0010        0001
//   10
// 20 + 1
// 21
uint8_t B2D(unsigned char byte)
{
	unsigned char high, low;

	low = byte & 0x0f;   // low nibble을 취한다.
	high = ( (byte >> 4) & 0x0f) * 10;   // high nibble을 취함다.

	return high + low;   // 예) 21
}

uint8_t D2B(unsigned char byte)
{
	return ( ((byte / 10) << 4) + (byte % 10) );
}
//
//          1
//012345678901234567
//setrtc211018151000
void SetRTCTime(char *date_time)
{
	char yy[4], mm[4], dd[4];
	char hh[4], min[4], ss[4];

	strncpy(yy,date_time+6,2);
	strncpy(mm,date_time+8,2);
	strncpy(dd,date_time+10,2);

	strncpy(hh,date_time+12,2);
	strncpy(min,date_time+14,2);
	strncpy(ss,date_time+16,2);

	sDate.Year = D2B(atoi(yy));
	sDate.Month = D2B(atoi(mm));
	sDate.Date = D2B(atoi(dd));

	sTime.Hours = D2B(atoi(hh));
	sTime.Minutes = D2B(atoi(min));
	sTime.Seconds = D2B(atoi(ss));

	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
}

char GetRTCTime(void)
{
	static RTC_TimeTypeDef oldTime;
	char lcd_buff[40];

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	// B2D --> BDC to Decimal
	if (oldTime.Seconds != sTime.Seconds)
	{
		// YYYY-MM-DD HH:mm:SS
		sprintf(lcd_buff, "%04d-%02d-%02d %02d:%02d:%02d",
				B2D(sDate.Year)+2000, B2D(sDate.Month), B2D(sDate.Date),
				B2D(sTime.Hours), B2D(sTime.Minutes), B2D(sTime.Seconds));

//		sprintf(lcd_buff, "DATE:%04d-%02d-%02d",
//				B2D(sDate.Year)+2000, B2D(sDate.Month), B2D(sDate.Date));
//		        move_cursor1(0,0);
		        lcd_string1(lcd_buff);
	    sprintf(lcd_buff, "TIME : %02d:%02d:%02d  ",
		        B2D(sTime.Hours), B2D(sTime.Minutes), B2D(sTime.Seconds));
		        move_cursor1(0,0);
		        lcd_string1(lcd_buff);
//
//		sprintf(lcd_buff, "%02d:%02d:%02d |T:%02d R:%02d ",
//			    B2D(sTime.Hours), B2D(sTime.Minutes), B2D(sTime.Seconds),i_RH, i_Tmp);
//				move_cursor1(1,0);
//				lcd_string1(lcd_buff);

	}
	oldTime.Seconds = sTime.Seconds;
}



