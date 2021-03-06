#include <msp430.h> 


/**
 * main.c
 * 仿真WISP闪灯模式，利用中断技巧
 */
void main(void)
{
//	WDTCTL = WDTPW | WDTHOLD;	                // stop watchdog timer
//    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
    WDT_init();

    PJDIR |= BIT6;                          // Set PJ.6 to output direction，LED2
    PJOUT &= ~BIT0;

//    __enable_interrupt();                   //打开总中断
//    __bis_SR_register(LPM0_bits+GIE);       //进入低功耗模式,其中启动中断

    while(1){
        __bis_SR_register(LPM3_bits+GIE);       //进入低功耗模式,其中启动中断
        activeMode();
    }

}

#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void){
    __bic_SR_register_on_exit(LPM3_bits);
}
