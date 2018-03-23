#include <msp430.h> 

void sq_wave_loop();
void sq_wave_lp();

/**
 * main.c
 */
int main(void){
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 &= ~LOCKLPM5;

	sq_wave_lp();
	
	return 0;
}

void sq_wave_loop(){
    P1DIR |= BIT0;

    while(1){
        P1OUT |= 0x01;
        _delay_cycles(1000000);

        P1OUT &= ~0x01;
        _delay_cycles(1000000);
    }
}

void sq_wave_lp(){
    CTL1 = OUTMOD0_1;
    _BIS_SR(CPUOFF);
}

