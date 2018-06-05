/*
 * clock_func.h
 *
 *  Created on: Apr 2, 2018
 *      Author: gmuadmin
 */

#ifndef CLOCK_FUNC_H_
#define CLOCK_FUNC_H_

#include "driverlib.h"


extern Calendar calendar;                                // Calendar used for RTC

/*
 * clock system initialization
 */
void init_clock(void);

/*
 * real time clock initialization
 */
void init_RTC();

#endif /* CLOCK_FUNC_H_ */
