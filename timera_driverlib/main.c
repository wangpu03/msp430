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
#define COMPARE_VALUE 50000  //该值决定了下次比较的周期
//******************************************************************************
//!
//!  启动一个计时器，设置为捕获模式，设定一个值，当计数器到达该值时产生中断，周期为COMPARE_VALUE，在中断中重新设定下一次的比较值
//   其为当前值+COMPARE_VALUE。所以设定周期为COMPARE_VALUE
//!
//******************************************************************************
void main (void) {
    //stop WDT
    WDT_A_hold(WDT_A_BASE);

    //set PJ.6 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN6);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    //start timer in continuous mode sourced by SMCLK
    Timer_A_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_4;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

    //initiaze compare mode
    //Clears the capture-compare interrupt flag. 后者为选择哪个寄存器的参数，将其作为接下来设为compare模式的寄存器，所以需要先清除该寄存器的中断标志
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    Timer_A_initCompareModeParam initCompParam = {0};
    initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;                  //选择所需要使用的比较寄存器
    initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;     //捕获比较中断使能
    initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;                   //指定输出的模式
    initCompParam.compareValue = COMPARE_VALUE;                                         //在比较模式中，设置所比较的值
    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

    //enter LPM0, enable interrupts
    __bis_SR_register(LPM0_bits + GIE);

    // for debugger
    __no_operation();

}

//this is the timer1_a3 interrupt vector service routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEM_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt
#elif defined (__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR(void){
    //读取TA1_CCR0寄存器内的值（即上次用于比较的值），用于设置下次的比较值，所以时间间隔为COMPARE_VALUE，但是该值最大值为65536,16bit长度
    uint16_t compVal = Timer_A_getCaptureCompareCount(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0) + COMPARE_VALUE;

    //toggle PJ.6
    GPIO_toggleOutputOnPin(GPIO_PORT_PJ, GPIO_PIN6);

    //add offset to CCRO
    Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0,compVal);
}

