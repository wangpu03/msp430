/*
 * led.c
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#include "msp430fr5969.h"
#include "led.h"

//LED灯控制初始化
void led_init(void){
    P1SEL0 &= ~BIT0;        //设置P1.0 IO口为普通I/O模式
    P1SEL0 &= ~BIT0;

    P1DIR |= BIT0;          //设置P1.0 IO口方向为输出
    P1OUT |= BIT0;          //初始设置P1.0 IO为高电平
}


