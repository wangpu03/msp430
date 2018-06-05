#include <stdio.h>
#include <msp430.h> 
#include <stdint.h>

/**
 * hello.c
 */

#define CAL_ADC_12T30_L  *(int8_t *)(0x1A1E) // Temperature Sensor Calibration-30 C 2.0V ref
#define CAL_ADC_12T30_H  *(int8_t *)(0x1A1F)
#define CAL_ADC_12T85_L  *(int8_t *)(0x1A20) // Temperature Sensor Calibration-85 C 2.0V ref
#define CAL_ADC_12T85_H  *(int8_t *)(0x1A21)

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	printf("Hello World!\n");
	printf("%d,%d,%d,%d",CAL_ADC_12T30_H,CAL_ADC_12T30_L,CAL_ADC_12T85_H,CAL_ADC_12T85_L);
	
	return 0;
}
