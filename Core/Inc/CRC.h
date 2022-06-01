/*
 * CRC.h
 *
 *  Created on: May 3, 2022
 *      Author: noahr
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include <Radio.h>

extern uint16_t crc16_update(uint16_t crc, uint8_t a);
extern uint16_t crc_xmodem_update (uint16_t crc, uint8_t data);
extern uint16_t crc_ccitt_update (uint16_t crc, uint8_t data);
extern uint8_t  crc_ibutton_update(uint8_t crc, uint8_t data);

#endif /* INC_CRC_H_ */
