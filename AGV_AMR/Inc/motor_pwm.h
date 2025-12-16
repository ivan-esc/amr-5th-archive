///////////////////////////////
////// MOTOR PWM  HEADER //////
///////////////////////////////
/*
 INPUT: INIT
 OUTPUT: MACROS
 */


#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

void pwm_init(void);			//PWM channels init

// Macros de PWM
#define left_LPWM(duty)  (TIM2->CCR1 = (duty)) //PA0
#define left_RPWM(duty)  (TIM2->CCR2 = (duty)) //PA1
#define right_LPWM(duty)   (TIM2->CCR3 = (duty)) //PB10
#define right_RPWM(duty)   (TIM2->CCR4 = (duty)) //PB11

// PA0 PA1 PB10 PB11

#endif /* MOTOR_PWM_H_ */
