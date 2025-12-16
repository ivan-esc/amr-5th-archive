//////////////////////////////
////// ODOMETRY SOURCE  //////
//////////////////////////////
/*
 INPUT: DELTAPOS AND YAW PLUS MACROS
 OUTPUT: POS_X POS_Y ROUNDED_YAW
 */

#include "odometry.h"

// -------- Global odometry state --------
float pos_x = 0.0f;
float pos_y = 0.0f;
int16_t pos_x_mm = 0;
int16_t pos_y_mm = 0;
int16_t rounded_yaw = 0;

float yaw_relative  = 0;
int16_t rounded_yaw_relative = 0;
float yaw_zero = 0;


// -------- Initialization --------
void Odometry_Init(void){
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_x_mm = 0;
    pos_y_mm = 0;
	rounded_yaw_relative = (int16_t)yaw_relative;

}

void Reset_x_y(void){
	yaw_zero = yaw;
    pos_x = 0.0f;
    pos_y = 0.0f;
    pos_x_mm = 0;
    pos_y_mm = 0;
	yaw_relative = yaw - yaw_zero;

}


// -------- Update every 10 ms --------
void Odometry_Update(void){
    // 1. Convert raw pulses → wheel linear distance (meters)
    //    arc = (pulses / 500) * 2πR
    float meters_per_pulse = (TWO_PI * WHEEL_RADIUS) / PULSES_PER_REV;
    float dL = delta_Pos1 * meters_per_pulse;
    float dR = delta_Pos2 * meters_per_pulse;

    // 2. Robot forward and angular displacement
    float dCenter = 0.5f * (dL + dR);
    // float dTheta  = (dR - dL) / WHEEL_DISTANCE;   // not used since IMU provides yaw

    // 6. Rounded yaw for sending
    rounded_yaw = (int16_t)yaw;
	yaw_relative = yaw - yaw_zero;
	rounded_yaw_relative = (int16_t)yaw_relative;

    // 3. Convert yaw(deg) → radians
    float yaw_rad = yaw_relative * DEG_TO_RAD;

    // 4. Update global position
    pos_x += dCenter * cosf(yaw_rad);
    pos_y += dCenter * sinf(yaw_rad);

    // 5. Convert to mm for transmission
    pos_x_mm = (int16_t)(pos_x * 1000.0f);
    pos_y_mm = (int16_t)(pos_y * 1000.0f);

}
