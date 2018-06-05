/*
 * uart_common.c
 *
 *  Created on: Apr 2, 2018
 *      Author: wp
 */

#include "driverlib.h"
#include "uart_common.h"


/************************************************************************
 * UART Communication Initialization
 */
void init_UART(){
    // Configure UART
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    // ²¨ÌØÂÊÎª19200
/*    param.clockPrescalar = 78;
    param.firstModReg = 0;
    param.secondModReg = 0x00;*/


    //57600
    param.clockPrescalar = 13;
    param.firstModReg = 0;
    param.secondModReg = 0x25;

    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
        return;

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
                                EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

    // Enable globe interrupt
    //__enable_interrupt();
}

/************************************************************************
 * transmit 0xFF and 0xFF to acknowledge of PC GUI's Ping request
 */
void sendAckToPC(){

    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

    // Send Ackknowledgement to Host PC
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'c');
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'k');

    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
}

void sendValueToPC(uint8_t value){
    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, value);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, value);

    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);

}
