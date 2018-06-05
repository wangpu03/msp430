/*
 * liveTempMode.h
 *
 *  Created on: Apr 2, 2018
 *      Author: wp
 */

#ifndef MODE_FUNC_H_
#define MODE_FUNC_H_

#include <stdint.h>

#define ARRAYLEN 16383
#define DATALEN 2499
#define FRAM_CH1_INDEX          0xF000          // FRAM location for channel 1 data index
#define FRAM_CH2_INDEX          0xF002          // FRAM location for channel 2 data index
#define FRAM_CH1_RESULTS        0x7000          // FRAM location for channel 1 results    //the first address for channel 1 the sample value
#define FRAM_CH2_RESULTS        0xB000          // FRAM location for channel 2 results    //the first address for channel 2 the sample value
#define FRAM_RESULTS_END        0xFF7F          // Address following the storage area     //the final address


extern uint16_t dataArray[];
extern uint8_t timeStamp[];
/*
 * 直接将采集的数据发送给PC
 */
void liveMode(void);

/*
 * 将采集的数据写入fram存储器中，直接写满为止
 */
void framLogMode(void);

/*
 * 将fram存储器中数据发送给PC,只发送完为止
 */
void framSendMode(void);



/*
 * 将fram存储器中数据发送给PC,只发送完为止
 */
void framSendMode_test(void);
#endif /* MODE_FUNC_H_ */
