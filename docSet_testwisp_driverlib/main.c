/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include "driverlib.h"

#define COMPARE_VALUE 65000  //65535

void main (void){
    WDT_A_hold(WDT_A_BASE);

    //GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);

    //GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();


    //Low frequency option 8MHz
    CS_setDCOFreq(CS_DCORSEL_0 , CS_DCOFSEL_6 );

    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Start timer in continuous mode sourced by SMCLK
    Timer_A_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_16;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.startTimer = false;  //�������õ�ԭ�򣺵ȵ����ԼĴ��������ú��Ժ�ſ�ʼ����
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

    //Initiaze compare mode
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    Timer_A_initCompareModeParam initComParam = {0};
    initComParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    initComParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    initComParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    initComParam.compareValue = COMPARE_VALUE;
    Timer_A_initCompareMode(TIMER_A1_BASE, &initComParam);

    //��ʼ����
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

    __bis_SR_register(LPM3_bits+GIE);

    __no_operation();
}

//��Ϊ�Ƚ�ģʽ������TAxCCTL0���Ƚ�ֵҲ����TAxCCR0�У�����ֻ�������һ���ж�����TIMER1_A0_VECTOR������ֻ��һ���ж�ֵ
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void){

    //toggle P4.0
    GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN0);
}

//���ڼ�ʱ��������������ж���Ҫ��TIMER1_A1_VECTOR�д����������ܴ���CCR1-CCR2�Լ�TAxCTL�е��жϣ���7���ж�ֵ
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void){

    switch(__even_in_range(TA1IV, TA1IV_TAIFG)){
        case  TA1IV_NONE: break;                          //No interrupt
        case  TA1IV_TACCR1: break;
        case  TA1IV_TACCR2: break;
        case  TA1IV_3: break;                          //CCR3 not used
        case  TA1IV_4: break;                          //CCR4 not used
        case  TA1IV_5: break;                          //CCR5 not used
        case  TA1IV_6: break;                          //CCR6 not used
        case  TA1IV_TAIFG:
            //toggle P4.0
            GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN0);
            break;
        default: break;
    }

}
