/*
 * live_temp_mode.c
 *
 *  Created on: Apr 2, 2018
 *      Author: gmuadmin
 */

#include "mode_func.h"
#include "main.h"
#include "driverlib.h"

/*
 * FRAM Array reserved to store data memory for Temperature
 * Stores up to 6,144 Temp + 2 index data
 */

#pragma PERSISTENT(dataArray)
#pragma location = FRAM_CH1_RESULTS;
uint16_t dataArray[ARRAYLEN+2] = {0};

unsigned int *FRAM_index_ch1_ptr = (unsigned int*)(FRAM_CH1_INDEX);  // Pointer to address that stores index of data about channel 1
unsigned int *FRAM_ch1_write_ptr;

unsigned int *FRAM_index_ch2_ptr = (unsigned int*)(FRAM_CH2_INDEX);  // Pointer to address that stores index of data about channel 2
unsigned int *FRAM_ch2_write_ptr;

void liveMode() {

    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;
    //Select internal ref = 1.2V
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF1_2V);
    //Internal Reference ON
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    //Initialize the ADC12B Module
    /* Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE DCO 12MHZ Digital Oscillator as clock source
     * Use default clock divider of 1 and pre-divider of 1
     * Turn off Temperature Sensor and Battery Monitor internal channels
     * The ADC12 clock is 12MHz
     */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_SMCLK;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    //initParam.internalChannelMap = ADC12_B_TEMPSENSEMAP + ADC12_B_BATTMAP;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Sets up the sampling timer pulse mode
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_8_CYCLES,
                               ADC12_B_CYCLEHOLD_4_CYCLES,
                               ADC12_B_MULTIPLESAMPLESENABLE);

    // Maps 10 input channel to Memory 0 and select voltage references
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A10 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A7 ;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A10 ;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    // Clear memory buffer 0 interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG1);

    // Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE, ADC12_B_IE1, 0, 0);

    // Start timer
    /*
     * 首先设置的一定的时间，最开始等ADC以及reference voltage设置好，中断后将其设定为采样间隔
     * 定时器的时钟为12MHz,分频24=500k
     */
    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_24;
    param.timerPeriod = 1000;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = false;
    Timer_A_initUpMode(TIMER_A0_BASE, &param);

    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    __bis_SR_register(LPM3_bits | GIE);       // Enter LPM3. Delay for Ref to settle.
    __no_operation();

    //500的采样率,计数器值为1000(0x03E8)，中间发送时间基本上457的计时器周期,需要发送7个字节，
    //1000的采样率,计数器值为500(0x01F4)
    Timer_A_setCompareValue(TIMER_A0_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_0,
                            0x03E8
                            );

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

    counter = 0;
    while(mode == LIVE_MODE) {

        __bis_SR_register(LPM3_bits | GIE);   // Enter LPM3, wait for sec timer

        //Enable/Start sampling and conversion
        /* channel 1
         * Base address of ADC12B Module
         * Start the conversion into memory buffer 0
         * Use the single-channel, single-conversion mode
         */

        ADC12_B_startConversion(ADC12_B_BASE,
                                ADC12_B_MEMORY_0,
                                ADC12_B_SEQOFCHANNELS);
        __bis_SR_register(LPM3_bits | GIE);   // Wait for conversion to complete

        __bic_SR_register(GIE);//

        //EUSCI_A_UART_transmitData(EUSCI_A0_BASE, counter);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM0_H);             // Send higher byte of temperature data
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM0_L);             // Send higher byte of temperature data
        //EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM1_H);             // Send higher byte of temperature data
        //EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ADC12MEM1_L);             // Send higher byte of temperature data
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN2);

    }

    //发送完毕后，将UART的发送引脚设置为低
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
    __no_operation();                     // Set a breakpoint here to verify UART transmission
    // Disable ADC12 and Timer_A0
    ADC12_B_disable(ADC12_B_BASE);
    Timer_A_stop(TIMER_A0_BASE);
}

/*
 * 将采集的数据存储在FRAM中
 */
void framLogMode(void){
    while (Ref_A_isRefGenBusy(REF_A_BASE)) ;
    //Select internal ref = 1.2V
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF1_2V);
    //Internal Reference ON
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    //Initialize the ADC12B Module
    /* Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE DCO 12MHZ Digital Oscillator as clock source
     * Use default clock divider of 1 and pre-divider of 1
     * Turn off Temperature Sensor and Battery Monitor internal channels
     * The ADC12 clock is 12MHz
     */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_SMCLK;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    //initParam.internalChannelMap = ADC12_B_TEMPSENSEMAP | ADC12_B_BATTMAP;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    // Sets up the sampling timer pulse mode
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_8_CYCLES,
                               ADC12_B_CYCLEHOLD_4_CYCLES,
                               ADC12_B_MULTIPLESAMPLESENABLE);

    // Maps 10 input channel to Memory 0 and select voltage references
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A10 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A7 ;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    // Maps 10 input channel to Memory 1 and select voltage references
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A10 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A10 ;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);


    // Clear memory buffer 0 interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG1);

    // Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE, ADC12_B_IE1, 0, 0);

    // Start timer
    /*
     * 首先设置的一定的时间，最开始等ADC以及reference voltage设置好，中断后将其设定为采样间隔
     * 定时器的时钟为6MHz
     */
    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_2;
    param.timerPeriod = 1000;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &param);

    __bis_SR_register(LPM3_bits | GIE);       // Enter LPM3. Delay for Ref to settle.
    __no_operation();

    //时钟为6M，计时器即时为300, 20K的采样率
    Timer_A_setCompareValue(TIMER_A0_BASE,
                            TIMER_A_CAPTURECOMPARE_REGISTER_0,
                            0x012C
                            );

    *FRAM_index_ch1_ptr = 0;
    *FRAM_index_ch2_ptr = 0;
    while(mode == FRAM_LOG_MODE){

        __bis_SR_register(LPM3_bits | GIE);   // Enter LPM3, wait for sec timer

        //Enable/Start sampling and conversion
        /* channel 1
         * Base address of ADC12B Module
         * Start the conversion into memory buffer 0
         * Use the single-channel, single-conversion mode
         */
        ADC12_B_startConversion(ADC12_B_BASE,
                                ADC12_B_MEMORY_0,
                                ADC12_B_SEQOFCHANNELS);
        __bis_SR_register(LPM3_bits | GIE);   // Wait for conversion to complete
        __bic_SR_register(GIE);//
        // Log channel 1 ADC conversion results to FRAM
        FRAM_ch1_write_ptr = (unsigned int*)(FRAM_CH1_RESULTS + *FRAM_index_ch1_ptr);
        *FRAM_ch1_write_ptr = ADC12MEM0;
        *FRAM_index_ch1_ptr += 2;

        FRAM_ch2_write_ptr = (unsigned int*)(FRAM_CH2_RESULTS + *FRAM_index_ch2_ptr);
        *FRAM_ch2_write_ptr = ADC12MEM1;
        *FRAM_index_ch2_ptr += 2;

        // Index has reached the end of allocated FRAM
        if ((*FRAM_index_ch1_ptr)> DATALEN || (*FRAM_index_ch2_ptr) > DATALEN) {
            mode = '3';
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
            //*FRAM_index_ptr = 0;
            break;
        }

    }
    __no_operation();                     // Set a breakpoint here to verify UART transmission
    // Disable ADC12 and Timer_A0
    ADC12_B_disable(ADC12_B_BASE);
    Timer_A_stop(TIMER_A0_BASE);
    __no_operation();                     // Set a breakpoint here to verify UART transmission

}

void framSendMode(void){

    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

    __delay_cycles(50000);

    // Send FRAM Index of channel 1 即存储数组的长度
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (uint8_t)(*FRAM_index_ch1_ptr>>8));
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (uint8_t)(*FRAM_index_ch1_ptr));

    __delay_cycles(50000);

    uint16_t i;

    for (i=0;i<*FRAM_index_ch1_ptr;i+=2){
        // Send logged Temperature Sensor ata
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH1_RESULTS+i+1));
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH1_RESULTS+i));

        __delay_cycles(50000);
    }

    // Send FRAM Index of channel 2 即存储数组的长度
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (uint8_t)(*FRAM_index_ch2_ptr>>8));
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (uint8_t)(*FRAM_index_ch2_ptr));

    __delay_cycles(50000);

    for (i=0;i<*FRAM_index_ch2_ptr;i+=2){
        // Send logged Temperature Sensor ata
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH2_RESULTS+i+1));
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH2_RESULTS+i));

        __delay_cycles(50000);
    }

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    mode = '0';
}

void framSendMode_test(void){

    // Select UART TXD on P2.0
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);

    __delay_cycles(50000);

    uint16_t i;

    for (i=0;i<*FRAM_index_ch1_ptr;i+=2){
        // Send logged Temperature Sensor ata
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH1_RESULTS+i+1));
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH1_RESULTS+i));

        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH2_RESULTS+i+1));
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, *(uint8_t *)(FRAM_CH2_RESULTS+i));

        __delay_cycles(50000);
    }

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    mode = '0';
}
