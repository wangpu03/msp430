/*
 * clock_func.c
 *
 *  Created on: Apr 2, 2018
 *      Author: gmuadmin
 */


#include "clock_func.h"
#include "driverlib.h"

Calendar calendar;                                // Calendar used for RTC
/************************************************************************
 * Clock System Initialization
 */
void init_clock(){
    // Set DCO frequency to 24 MHz
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_6);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 2
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    // Set MCLK = DCO with frequency divider of 2
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_0);
}

/************************************************************************
 * Real Time Clock Initialization
 */
void init_RTC(){
    //Setup Current Time for Calendar
    calendar.Seconds    = 0x55;
    calendar.Minutes    = 0x30;
    calendar.Hours      = 0x04;
    calendar.DayOfWeek  = 0x01;
    calendar.DayOfMonth = 0x30;
    calendar.Month      = 0x04;
    calendar.Year       = 0x2014;

    // Initialize RTC with the specified Calendar above
    RTC_B_initCalendar(RTC_B_BASE,
                       &calendar,
                       RTC_B_FORMAT_BCD);

    RTC_B_setCalendarEvent(RTC_B_BASE,
                           RTC_B_CALENDAREVENT_MINUTECHANGE
                           );

    RTC_B_clearInterrupt(RTC_B_BASE,
                         RTC_B_TIME_EVENT_INTERRUPT
                         );

    RTC_B_enableInterrupt(RTC_B_BASE,
                          RTC_B_TIME_EVENT_INTERRUPT
                          );

    //Start RTC Clock
    RTC_B_startClock(RTC_B_BASE);
}

