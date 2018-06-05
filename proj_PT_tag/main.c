/*
 * mymain.c
 *
 *  Created on: Apr 2, 2018
 *      Author: wp
 *      only conduct the single sample, no modulate at the transmit
 */

#include <msp430.h>
#include "main.h"
#include "driverlib.h"
#include "io_func.h"
#include "clock_func.h"
#include "uart_common.h"
#include "mode_func.h"


#define PINGHOST              'p'

uint8_t counter = 0;                               // UART Receive byte

uint8_t RXData = 0;                               // UART Receive byte
int mode = 0;                                     // mode selection variable
int pingHost = 0;                                 // ping request from PC GUI

int _system_pre_init(void){
/* Insert your low-level initializations here */
/* Disable Watchdog timer to prevent reset during */
/* long variable initialization sequences. */
WDTCTL = WDTPW | WDTHOLD;
// Configure MPU
/*
MPUCTL0 = MPUPW; // Write PWD to access MPU registers
MPUSEGB1 = 0x0480; // B1 = 0x4800; B2 = 0x4C00
MPUSEGB2 = 0x04c0; // Borders are assigned to segments
// Segment 1 – Allows read and write only
// Segment 2 – Allows read only
// Segment 3 – Allows read and execute only
MPUSAM = (MPUSEG1WE | MPUSEG1RE | MPUSEG2RE | MPUSEG3RE | MPUSEG3XE);
MPUCTL0 = MPUPW | MPUENA | MPUSEGIE; // Enable MPU protection
*/
// MPU registers locked until BOR
/*==================================*/
/* Choose if segment initialization */
/* should be done or not. */
/* Return: 0 to omit initialization */
/* 1 to run initialization */
/*==================================*/
return 1;
}

int main(void){
   // WDT_A_hold(WDT_A_BASE);     // Stop WDT

    // 当机器启动没有异常时，IO口初始化，闪烁LED
    init_GPIO();  //将所有IO设为低
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // Toggle LED1 and LED2 to indicate OutOfBox Demo start
    int i;
    for (i=0;i<10;i++){
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);  //异或所要设置的值
        GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN6);
        __delay_cycles(200000);
    }

    init_GPIO();
    init_clock();
    init_UART();

    __delay_cycles(5000);

    __no_operation();
    while(1){
        // Acknowledge PC GUI's ping request
        if (pingHost){
            //接收指令1时，闪烁红绿LED
            twinkleRedLED();
            twinkleGreenLED();
            sendAckToPC();
            pingHost = 0;
        }

        switch(mode){
            case LIVE_MODE: {
               //工作时，亮红灯
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                liveMode();
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                break;
            }
            case FRAM_LOG_MODE: {
                //工作时，亮绿灯
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                framLogMode();
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//                break;
            }
            case FRAM_SEND_MODE: {
                //工作时，亮红绿灯
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                framSendMode_test();
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                break;
            }
            case FRAM_FULL:{
                GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                break;
            }
            default: break;
        }

        __bis_SR_register(LPM3_bits + GIE);
        __no_operation();
    }
}


/*
 * TIMER_A0 interrupt vector service routine
 * Used to trigger ADC conversion every 1/1200 seconds after a ref setup
 */

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void){
    counter = counter + 1;
    __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU

}

 /* USCI_A0 Interrupt Service Routine that receives PC GUI's commands
  */

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            RXData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            if(RXData == PINGHOST){
                pingHost = 1;
                mode = '0';
            }else{
                mode = RXData;
            }
            __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
            __no_operation();
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}


#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void){
    switch(__even_in_range(ADC12IV, ADC12IV_ADC12IFG0)){
        case ADC12IV_NONE: break;                        //No Interrupt pending
        case ADC12IV_ADC12OVIFG: break;                 // ADC12OVIFG
        case ADC12IV_ADC12TOVIFG: break;                 //ADC12TOVIFG
        case ADC12IV_ADC12HIIFG: break;                  //ADC12HIIFG
        case ADC12IV_ADC12LOIFG: break;                  //ADC12LOIFG
        case ADC12IV_ADC12INIFG: break;                  //ADC12INIFG
        case ADC12IV_ADC12IFG0:
/*            if(ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0) > 0x04b9){
                twinkleRedLED();
            }else{
                twinkleGreenLED();
            }*/
            ADC12IFGR0 &= ~ADC12IFG0;             // Clear interrupt flag
            __bic_SR_register_on_exit(LPM3_bits); // Exit active CP
            break;                   //ADC12IFG0
        case ADC12IV_ADC12IFG1:
/*            if(ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1) > 0x04b9){
                twinkleRedLED();
            }else{
                twinkleGreenLED();
            }*/
            ADC12IFGR0 &= ~ADC12IFG1;             // Clear interrupt flag
            __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
            break;                   //ADC12IFG1
        case ADC12IV_ADC12IFG2: break;                   //ADC12IFG2
        case ADC12IV_ADC12IFG3: break;                   //ADC12IFG3
        case ADC12IV_ADC12IFG4: break;                   //ADC12IFG4
        case ADC12IV_ADC12IFG5: break;                   //ADC12IFG5
        case ADC12IV_ADC12IFG6: break;                   //ADC12IFG6
        case ADC12IV_ADC12IFG7: break;                  // ADC12IFG7
        case ADC12IV_ADC12IFG8: break;                   //ADC12IFG8
        case ADC12IV_ADC12IFG9: break;                   //ADC12IFG9
        case ADC12IV_ADC12IFG10:                  //ADC12IFG10
            break;
        case ADC12IV_ADC12IFG11:                  //ADC12IFG11
            break;
        case ADC12IV_ADC12IFG12: break;                  //ADC12IFG12
        case ADC12IV_ADC12IFG13: break;                  //ADC12IFG13
        case ADC12IV_ADC12IFG14: break;                  //ADC12IFG14
        case ADC12IV_ADC12IFG15: break;                  //ADC12IFG15
        case ADC12IV_ADC12IFG16: break;                  //ADC12IFG16
        case ADC12IV_ADC12IFG17: break;                  //ADC12IFG17
        case ADC12IV_ADC12IFG18: break;                  //ADC12IFG18
        case ADC12IV_ADC12IFG19: break;                  //ADC12IFG19
        case ADC12IV_ADC12IFG20: break;                  //ADC12IFG20
        case ADC12IV_ADC12IFG21: break;                  //ADC12IFG21
        case ADC12IV_ADC12IFG22: break;                  //ADC12IFG22
        case ADC12IV_ADC12IFG23: break;                  //ADC12IFG23
        case ADC12IV_ADC12IFG24: break;                  //ADC12IFG24
        case ADC12IV_ADC12IFG25: break;                  //ADC12IFG25
        case ADC12IV_ADC12IFG26: break;                  //ADC12IFG26
        case ADC12IV_ADC12IFG27: break;                  //ADC12IFG27
        case ADC12IV_ADC12IFG28: break;                  //ADC12IFG28
        case ADC12IV_ADC12IFG29: break;                  //ADC12IFG29
        case ADC12IV_ADC12IFG30: break;                  //ADC12IFG30
        case ADC12IV_ADC12IFG31: break;                  //ADC12IFG31
        case ADC12IV_ADC12RDYIFG: break;                 //ADC12RDYIFG
        default: break;
    }
}
