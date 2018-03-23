/*
 * led.h
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#ifndef LED_H_
#define LED_H_

#include <msp430.h>

//IO口定义
#define LED_H  P1OUT |= BIT0        //熄灭led灯
#define LED_L  P1OUT &= ~BIT0       //点亮led灯
#define LED_HL P1OUT ^= BIT0        //翻转led灯状态

//函数声明，LED灯控制IO初始化
void led_init(void);

#endif /* LED_H_ */
