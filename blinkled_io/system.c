/*
 * system.c
 *
 *  Created on: Mar 22, 2018
 *      Author: maxiaojun
 */

#include "msp430fr5969.h"
#include "system.h"
#include <intrinsics.h>

//ϵͳʱ�ӳ�ʼ��
//MCLK = 16MHz
//SMCLK=8MHz
//ACLK=32.678KHz

void clk_init(void){
    unsigned int time = 10000;//�ⲿʱ�������ʱ����������ǲ���������ѡ���ڲ�ʱ��
    BCSCTL1 &= 0x00;   //����XT2������LFXT1�����ڵ�Ƶ����ģʽ��ACLK�ķ�Ƶϵ��0

    do{
        IFG1 &= ~OFIFG;//��OFIFG���
        __delay_cycles(100);//���ⲿ����û������ʱ��ʱ����Դ���ڲ����1MHz��DCO
    }
    while((IFG1 & OFIFG) && time--);//��ѯʱ���л��ɹ�
    if(time == 0){              //�ⲿʱ��������
        BCSCTL1 |= XT2OFF;      //�ر�XT2����
    }else{//�л��ɹ�
        BCSCTL2 &= 0x00;
        BCSCTL2 |= SELM1+SELS+DIVS0;//MCLK��SMCLK��ʱ��ԴΪXT2��SMCLK�ķ�Ƶϵ��2
    }
}

//�رչ���ʼ��
void Wdt_Off(void){
   WDTCTL = WDTPW + WDTHOLD;//�رտ��Ź�
}

//�򿪿��Ź�
void Wdt_On(void){
    WDTCTL = WDT_ARST_1000;//���Ź�ģʽ����ʱ1000ms
}
