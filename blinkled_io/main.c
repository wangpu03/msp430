#include <msp430.h> 
#include "msp430fr5969.h"
#include "system.h"
#include "led.h"


/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	Wdt_Off();//关闭看门狗
    Clock_Init();//系统时钟初始化
    Led_Init();//led灯控制IO初始化

    LED_L;//点亮led灯
    _BIS_SR(LPM4_bits);//SCG1+SCG0+OSCOFF+CPUOFF
	return 0;
}
