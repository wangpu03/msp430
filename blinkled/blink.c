//***************************************************************************************
//  MSP430 Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 by xor'ing P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430x5xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  Texas Instruments, Inc
//  July 2013
//***************************************************************************************

#include "msp430fr5969.h"

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    //PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings
    P1DIR |= BIT0;                          // Set P1.0 to output direction
//    PJDIR |= BIT6;


//    PJOUT &= ~BIT6;

    while(1){
        P1OUT ^= BIT0;
//      PJOUT ^= BIT6;
        _delay_cycles(100000);

    }


//    for(;;) {
//        volatile unsigned int i;            // volatile to prevent optimization
//
//
//        P4OUT ^= 0x40;                      // Toggle P1.0 using exclusive-OR
//        P1OUT ^= 0x01;                      // Toggle P1.0 using exclusive-OR
//
//        i = 50000;                          // SW Delay
//        do i--;
//        while(i != 0);
//
//
//        P4OUT ^= 0x40;                      // Toggle P1.0 using exclusive-OR
//        P1OUT ^= 0x01;                      // Toggle P1.0 using exclusive-OR
//        i = 50000;                          // SW Delay
//        do i--;
//        while(i != 0);
//    }
}
