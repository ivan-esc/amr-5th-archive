//////////////////////////////
////// ODOMETRY SOURCE  //////
//////////////////////////////
/*
 INPUT: DELTAPOS AND YAW PLUS MACROS
 OUTPUT: POS_X POS_Y ROUNDED_YAW
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

//extern files
#include "i2c_color_imu.h"
#include "encoders.h"
#include "pid_diff_drive.h"

#define PULSES_PER_REV   500.0f
#define TWO_PI           6.28318530718f

extern float pos_x;       // meters
extern float pos_y;       // meters
extern int16_t pos_x_mm;  // millimeters
extern int16_t pos_y_mm;  // millimeters
extern int16_t rounded_yaw; // degrees

extern float yaw_relative;
extern int16_t rounded_yaw_relative;
extern float yaw_zero;

void Odometry_Init(void);
void Odometry_Update(void);
void Reset_x_y(void);

#endif /* ODOMETRY_H_ */
