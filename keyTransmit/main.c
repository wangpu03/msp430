// �ó���Ϊ����LED�����°���LEDϨ���ɿ�����������������ж������

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

// ����а����жϷ�������ʱ֮���൱���ų��������ٶ�ȡ���е�ֵ����Ϊ�ߣ�������Ϊ�жϣ���Ϊ�ͣ���ȷʵ�����ð����ж�
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void){
    char pushkey;
    pushkey = P1IFG & BIT1;                 //�����ĸ������ж�
    _delay_cycles(10000);                   //������ʱȥ��

    if((P1IN & pushkey) == pushkey){
        P1IFG = 0x00;                       //�жϱ�־����
        return;
    }

    if(P1IFG & BIT1){                       //�жϰ����Ƿ���
        P1OUT ^= BIT0 + BIT3;
        while((P1IN & pushkey) != pushkey);  //�жϰ����Ƿ��ɿ�
        P1OUT ^= BIT0 + BIT3;
    }

    P1IFG = 0x00;
    return;
}