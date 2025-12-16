///////////////////////////////
//// ULTRASONIC HEADER ////////
///////////////////////////////
/*
 INPUT: CALLED IN TIM14 IRQ HANDLER
 OUTPUT: DISTANCE_CM ARRAY
 */


#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern volatile float distance_cm[4];


void TIM3_init(void);
void init_GPIO_EXTI_ultrasonic(void);
void start_hcsr04_trigger(void);
void hcsr04_read(void);


#endif /* ULTRASONIC_H_ */
