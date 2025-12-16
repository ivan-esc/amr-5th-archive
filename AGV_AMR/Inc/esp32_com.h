////////////////////////////////////
////// ESP32 USART COM HEADER //////
////////////////////////////////////
/*
 INPUT: USART2_IRQ_HANDLER AND TO-SEND VARIABLES
 OUTPUT: READ BYTES
 */


#ifndef ESP32_COM_H_
#define ESP32_COM_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern volatile uint8_t LED_action;

extern volatile uint16_t temperature10;
extern volatile uint16_t voltage10;
extern volatile uint16_t valid_indexes;

#define MAX_POINTS 500
extern int16_t pathX[MAX_POINTS];
extern int16_t pathY[MAX_POINTS];

void USART1_Init(void);					//ESP32 communication init
void USART1_send(uint8_t b);			//Sent LED command
void processPoint(int16_t x, int16_t y);
void resetPath(void);

#endif /* ESP32_COM_H_ */
