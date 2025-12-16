////////////////////////////////////
////// BLUETOOTH HC-06 HEADER //////
////////////////////////////////////
/*
 INPUT: USART2_IRQ_HANDLER AND TO-SEND VARIABLES
 OUTPUT: READ BYTES
 */


#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

//recieved variables
extern volatile int16_t raw_x;  // recieved coordinate
extern volatile int16_t raw_y; // recieved coordinate
extern volatile uint8_t menu_button; //set mode
extern volatile uint8_t auto_set_speed; //line follow set speed
extern volatile uint8_t coded_recieved_bool1; //%2 = user/time %3 = go command
extern volatile uint8_t station_wait_time;
extern volatile uint8_t coded_route_recieved;

//sent variables
extern volatile uint8_t linear_velocity;
extern volatile uint8_t angular_velocity;
extern volatile uint8_t tmp_data;
extern volatile uint8_t near_obj;
extern volatile uint8_t colorin;
extern volatile uint8_t voltage_sent;

extern volatile uint8_t posx_big;
extern volatile uint8_t posx_small;
extern volatile uint8_t posy_big;
extern volatile uint8_t posy_small;

extern volatile uint8_t rx_buffer[8];


void USART2_Init(void);  		//Bluetooth init
void USART2_send(uint8_t b);	//Sent App data
void BT_SendTelemetry(void);     // Sent App data helper

#endif /* BLUETOOTH_H_ */
