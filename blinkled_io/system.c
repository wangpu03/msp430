/*
 * system.c
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#include "msp430fr5969.h"
#include "system.h"
#include <intrinsics.h>

//系统时钟初始化
//MCLK = 16MHz
//SMCLK=8MHz
//ACLK=32.678KHz

void clk_init(void){
    unsigned int time = 10000;//外部时钟在这个时间内如果还是不能起振则选择内部时钟
    BCSCTL1 &= 0x00;   //开启XT2振荡器，LFXT1工作在低频晶体模式，ACLK的分频系数0

    do{
        IFG1 &= ~OFIFG;//清OFIFG标记
        __delay_cycles(100);//在外部晶振还没有起振时，时钟来源于内部大概1MHz的DCO
    }
    while((IFG1 & OFIFG) && time--);//查询时钟切换成功
    if(time == 0){              //外部时钟有问题
        BCSCTL1 |= XT2OFF;      //关闭XT2振荡器
    }else{//切换成功
        BCSCTL2 &= 0x00;
        BCSCTL2 |= SELM1+SELS+DIVS0;//MCLK与SMCLK的时钟源为XT2，SMCLK的分频系数2
    }
}

//关闭狗初始化
void Wdt_Off(void){
   WDTCTL = WDTPW + WDTHOLD;//关闭看门狗
}

//打开看门狗
void Wdt_On(void){
    WDTCTL = WDT_ARST_1000;//看门狗模式，定时1000ms
}

