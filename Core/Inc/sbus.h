/*
 * sbus.h
 *
 *  Created on: Feb 22, 2026
 *      Author: guang
 */

#ifndef SRC_SBUS_H_
#define SRC_SBUS_H_

#include "main.h"

#define SBUS_FRAME_LEN 64

extern uint8_t SBUS_Failsafe;
extern uint8_t SBUS_LostFrame;

void SBUS_Init(void);

#endif /* SRC_SBUS_H_ */