// 该程序为点亮LED，按下按键LED熄灭，松开后点亮。操作都在中断中完成

#include "msp430fr5969.h"

/**
 * main.c
 */
void main(void){

    WDTCTL = WDTPW+WDTHOLD;                     // Stop WDT
    PM5CTL0 &= ~LOCKLPM5;                       // Disable the GPIO power-on default high-impedance mode

    P1DIR |= BIT0 + BIT3;

    P1DIR &= ~BIT1;
    P1REN |= BIT1;

    P1OUT |= BIT1+BIT0 + BIT3;

    P1IES |= BIT1;
    P1IE |= BIT1;
    P1IFG = 0x00;

    _BIS_SR(LPM0_bits+GIE);
}

// 如果有按键中断发生，延时之后，相当于排除抖动，再读取其中的值，若为高，则误判为中断；若为低，则确实发生该按键中断
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){
    char pushkey;
    pushkey = P1IFG & BIT1;                 //定义哪个按键中断
    _delay_cycles(10000);                   //短暂延时去抖

    if((P1IN & pushkey) == pushkey){
        P1IFG = 0x00;                       //中断标志清零
        return;
    }

    if(P1IFG & BIT1){                       //判断按键是否按下
        P1OUT ^= BIT0 + BIT3;
        while((P1IN & pushkey) != pushkey);  //判断按键是否松开
        P1OUT ^= BIT0 + BIT3;
    }

    P1IFG = 0x00;
    return;
}
