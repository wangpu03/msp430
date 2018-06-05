/*
 * uart_common.h
 *
 *  Created on: Apr 2, 2018
 *      Author: gmuadmin
 */

#ifndef UART_COMMON_H_
#define UART_COMMON_H_

/*
 * UART communication initialization
 */
void init_UART(void);

/*
 * transmit 0xFF to acknowledge of PC GUI's Ping request
 */
void sendAckToPC(void);

/*
 * transmit a test value to acknowledge of PC GUI's mode selection
 */
void sendValueToPC(uint8_t value);
#endif /* UART_COMMON_H_ */
