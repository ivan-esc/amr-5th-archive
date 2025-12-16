////////////////////////////////////////
////// AMR MODE LOGIC SOURCE ///////////
////////////////////////////////////////
/*
 INPUT: A TON OF STUFF
 OUTPUT: PWM ig?
 */


#ifndef AMR_MODE_H_
#define AMR_MODE_H_

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "esp32_com.h"      // provides pathX/pathY and valid_indexes
#include "odometry.h"       // provides pos_x, pos_y, rounded_yaw, Reset_x_y()
#include "pid_diff_drive.h" // provides VW_DifferentialDrive, PID_PWM, V_req, W_req
#include "i2c_color_imu.h" // provides VW_DifferentialDrive, PID_PWM, V_req, W_req

#define AMR_MAX_POINTS 500

// exported control flags & params
extern uint8_t amr_active;               // 0 = off, 1 = running
extern uint8_t amr_loop_mode;            // 0 = run once, 1 = loop
extern float position_error_threshold;   // meters (default 0.005f)
extern float amr_nominal_speed;          // m/s (default 0.25)
extern float amr_max_angular_speed;      // rad/s

// lifecycle
void AMR_Init(void);          // call once at system init
void AMR_Start(void);         // start AMR mode (resets odom & waits calibration)
void AMR_Stop(void);          // stop and reset
void resetAMR(void);          // resets internals (same as Stop but keeps inactive)
void AMR_Update(void);        // call every 10 ms to run the state machine

#endif // AMR_MODE_H
