/*
 * line_follow.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Ivan
 */

#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

void IR_Sensors_Init(void);
void LineFollow(void); //Logica seguidor de línea

void SmoothStop(void);
void Station_Color_Detect(void);
bool Check_All_Sensors_Black(void);
void station_wait_thing(void);

void seguir(void);

extern uint8_t linea;

extern float V_current; // acceleration ramp memory
extern float W_current;  // memory of last angular velocity command
extern float last_position_error;
extern uint16_t exit_timer;
extern bool station_done;
extern bool leaving_station;   // tracks if we're exiting a station
extern volatile uint8_t counter_stations; // fake ass mf



#endif /* LINE_FOLLOW_H_ */
