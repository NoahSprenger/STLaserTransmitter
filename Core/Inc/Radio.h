/*
 * Radio.h
 *
 *  Created on: May 3, 2022
 *      Author: noahr
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_

#include <string.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATOMIC_BLOCK_START uint32_t primask = __get_PRIMASK(); __disable_irq(); {
#define ATOMIC_BLOCK_END } __set_PRIMASK(primask);

#ifndef NOT_AN_INTERRUPT
	#define NOT_AN_INTERRUPT -1
#endif

#define INTERRUPT_ATTR

#define YIELD

#define BROADCAST_ADDRESS 0xff

#ifdef __cplusplus
}
#endif

#endif /* INC_RADIO_H_ */
