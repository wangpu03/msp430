/*
 * led.h
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#ifndef LED_H_
#define LED_H_

#include <msp430.h>

//IO�ڶ���
#define LED_H  P1OUT |= BIT0        //Ϩ��led��
#define LED_L  P1OUT &= ~BIT0       //����led��
#define LED_HL P1OUT ^= BIT0        //��תled��״̬

//����������LED�ƿ���IO��ʼ��
void led_init(void);

#endif /* LED_H_ */
