//////////////////////////////
////// ENCODERS  HEADER //////
//////////////////////////////
/*
 INPUT: EXTI-PULSES AND INIT
 OUTPUT: RPM-FILTERED
 */


#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern float rpm_1_filtered;    // GOOD PWM 1 = left
extern float rpm_2_filtered;	// GOOD PWM 2 = right

extern volatile float delta_Pos1;
extern volatile float delta_Pos2;

void update_encoder(uint8_t encoder); 	//Encoder EXTI handling
void init_GPIO_EXTI_encoders(void); 	//Encoder init
void velocidad_encoder(void);			//Transfer function for RPM filtering

#endif /* ENCODERS_H_ */
