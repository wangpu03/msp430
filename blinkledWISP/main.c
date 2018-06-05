#include <msp430.h> 


/**
 * main.c
 * ����WISP����ģʽ�������жϼ���
 */
void main(void)
{
//	WDTCTL = WDTPW | WDTHOLD;	                // stop watchdog timer
//    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
    WDT_init();

    PJDIR |= BIT6;                          // Set PJ.6 to output direction��LED2
    PJOUT &= ~BIT0;

//    __enable_interrupt();                   //�����ж�
//    __bis_SR_register(LPM0_bits+GIE);       //����͹���ģʽ,���������ж�

    while(1){
        __bis_SR_register(LPM3_bits+GIE);       //����͹���ģʽ,���������ж�
        activeMode();
    }

}

#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void){
    __bic_SR_register_on_exit(LPM3_bits);
}
