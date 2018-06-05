/*
 --COPYRIGHT--,BSD
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
 * --/COPYRIGHT--
 */
#include "driverlib.h"

void main (void){

    WDT_A_hold(WDT_A_BASE);

    //set P1.0 as an output pin
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    //set P1.0 as putput low
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //set P4.2 as ternary module function output
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_TERNARY_MODULE_FUNCTION);

    PMM_unlockLPM5();

    //If ref generator busy, WAIT
    while(Ref_A_isRefGenBusy(REF_A_BASE)); //默认为1.2V

    //Turn on Reference Voltage
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    //Initialize the ADC12B Module

     /* Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Not use internal channel
     */

    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    //enable the ADC12_B_BASE
    ADC12_B_enable(ADC12_B_BASE);


     /* Base address of ADC12B Module
     * For memory buffers 0-7 sample/hold for 64 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Disable Multiple Sampling
     */

    //设置你选择的采样与转变时间中的采样t_sample时间
    ADC12_B_setupSamplingTimer(
            ADC12_B_BASE,
            ADC12_B_CYCLEHOLD_16_CYCLES,
            ADC12_B_CYCLEHOLD_4_CYCLES,
            ADC12_B_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer

     /* Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A1 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */

    ADC12_B_configureMemoryParam conMemParam = {0};
    conMemParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    conMemParam.inputSourceSelect = ADC12_B_INPUT_A7;
    conMemParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    conMemParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    conMemParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    conMemParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &conMemParam);

    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG0);

    ADC12_B_enableInterrupt(ADC12_B_BASE, ADC12_B_IE0, 0, 0);

    __delay_cycles(75);

    while(1){
        __delay_cycles(5000);                    // Delay between conversions
        //Enable/Start sampling and conversion

         /* Base address of ADC12B Module
         * Start the conversion into memory buffer 0
         * Use the single-channel, single-conversion mode
         */

        ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_MEMORY_0, ADC12_B_SINGLECHANNEL);

        __bis_SR_register(LPM0_bits+GIE);

        __no_operation();
    }
}


#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void){
    switch(__even_in_range(ADC12IV, 22)){
        case  0: break;                         // Vector  0:  No interrupt
        case  2: break;                         // Vector  2:  ADC12BMEMx Overflow
        case  4: break;                         // Vector  4:  Conversion time overflow
        case  6: break;                         // Vector  6:  ADC12BHI
        case  8: break;                         // Vector  8:  ADC12BLO
        case 10: break;                         // Vector 10:  ADC12BIN
        case 12:                                // Vector 12:  ADC12BMEM0 Interrupt
          if (ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0) >= 0x6B4) {
                //Set PJ.6 LED on
              GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
          }
          else {
            //Set PJ.6 LED off
              GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
          }
          __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
          break;                                // Clear CPUOFF bit from 0(SR)
        case 14:
            __no_operation();
            if (ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1) >= 0x6B4) {
                  //Set PJ.6 LED on
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            else {
              //Set PJ.6 LED off
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
            break;                         // Vector 14:  ADC12BMEM1
        case 16:
            __no_operation();
            if (ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_2) >= 0x6B4) {
                  //Set PJ.6 LED on
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            else {
              //Set PJ.6 LED off
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
            __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
            break;                         // Vector 16:  ADC12BMEM2
        case 18: break;                         // Vector 18:  ADC12BMEM3
        case 20: break;                         // Vector 20:  ADC12BMEM4
        case 22: break;                         // Vector 22:  ADC12BMEM5
        case 24: break;                         // Vector 24:  ADC12BMEM6
        case 26: break;                         // Vector 26:  ADC12BMEM7
        case 28: break;                         // Vector 28:  ADC12BMEM8
        case 30: break;                         // Vector 30:  ADC12BMEM9
        case 32: break;                         // Vector 32:  ADC12BMEM10
        case 34: break;                         // Vector 34:  ADC12BMEM11
        case 36: break;                         // Vector 36:  ADC12BMEM12
        case 38: break;                         // Vector 38:  ADC12BMEM13
        case 40: break;                         // Vector 40:  ADC12BMEM14
        case 42: break;                         // Vector 42:  ADC12BMEM15
        case 44: break;                         // Vector 44:  ADC12BMEM16
        case 46: break;                         // Vector 46:  ADC12BMEM17
        case 48: break;                         // Vector 48:  ADC12BMEM18
        case 50: break;                         // Vector 50:  ADC12BMEM19
        case 52: break;                         // Vector 52:  ADC12BMEM20
        case 54: break;                         // Vector 54:  ADC12BMEM21
        case 56: break;                         // Vector 56:  ADC12BMEM22
        case 58: break;                         // Vector 58:  ADC12BMEM23
        case 60: break;                         // Vector 60:  ADC12BMEM24
        case 62: break;                         // Vector 62:  ADC12BMEM25
        case 64: break;                         // Vector 64:  ADC12BMEM26
        case 66: break;                         // Vector 66:  ADC12BMEM27
        case 68: break;                         // Vector 68:  ADC12BMEM28
        case 70: break;                         // Vector 70:  ADC12BMEM29
        case 72: break;                         // Vector 72:  ADC12BMEM30
        case 74: break;                         // Vector 74:  ADC12BMEM31
        case 76: break;                         // Vector 76:  ADC12BRDY
        default: break;
    }
}
