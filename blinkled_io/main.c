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
	Wdt_Off();//�رտ��Ź�
    Clock_Init();//ϵͳʱ�ӳ�ʼ��
    Led_Init();//led�ƿ���IO��ʼ��

    LED_L;//����led��
    _BIS_SR(LPM4_bits);//SCG1+SCG0+OSCOFF+CPUOFF
	return 0;
}
