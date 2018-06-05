/*
 * main.h
 *
 *  Created on: Apr 2, 2018
 *      Author: gmuadmin
 */

#ifndef MAIN_H_
#define MAIN_H_
#include <stdint.h>

#define FRAM_FULL             '0'
#define LIVE_MODE             '1'
#define FRAM_LOG_MODE         '2'
#define FRAM_SEND_MODE        '3'

extern uint8_t counter;

extern uint8_t RXData;
extern int mode;
extern int pingHost;

/*
 * system pre-initialization
 */
int _system_pre_init(void);

#endif /* MAIN_H_ */
