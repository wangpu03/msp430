/*
 * led.c
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#include "msp430fr5969.h"
#include "led.h"

//LED�ƿ��Ƴ�ʼ��
void led_init(void){
    P1SEL0 &= ~BIT0;        //����P1.0 IO��Ϊ��ͨI/Oģʽ
    P1SEL0 &= ~BIT0;

    P1DIR |= BIT0;          //����P1.0 IO�ڷ���Ϊ���
    P1OUT |= BIT0;          //��ʼ����P1.0 IOΪ�ߵ�ƽ
}


