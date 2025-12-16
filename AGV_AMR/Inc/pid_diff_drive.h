///////////////////////////////////////////////
////// PID AND DIFFERENTIAL DRIVE HEADER //////
///////////////////////////////////////////////
/*
 INPUT: V_req and W_req rpm_ref_l_r
 OUTPUT: PID FOR PWM
 */

#ifndef PID_DIFF_DRIVE_H_
#define PID_DIFF_DRIVE_H_


#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "encoders.h"
#include "motor_pwm.h"


#define WHEEL_RADIUS      0.1025f   // 20.5/2 = 10.25 cm
#define WHEEL_DISTANCE    0.362f     // 36.2 cm

void VW_DifferentialDrive(float V_req,float W_req); //Calculates w_l and w_r in rpms from a linear velocity reference and angular velocity reference
void PID_PWM(float ref_l,float ref_r); 	//Applies PID according to the previous functions rpm references and the measured rpms from encoders (filtered)

//use this ones as they are reset when mode swam
extern float V_req; //in m/s, max speed for AMR of 0.5m/s (but not constant, remember acceleration curve)
extern float W_req; //in radians
extern float rpm_ref_left;
extern float rpm_ref_right;

#endif /* PID_DIFF_DRIVE_H_ */
