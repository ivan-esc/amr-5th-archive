///////////////////////////////////////////////
////// PID AND DIFFERENTIAL DRIVE SOURCE //////
///////////////////////////////////////////////
/*
 INPUT: V_req and W_req rpm_ref_l_r
 OUTPUT: PID FOR PWM
 */

#include "pid_diff_drive.h"

// Left wheel PID
float error_l     = 0.0f;
float error_l_dt   = 0.0f;
float int_error_l = 0.0f;
float der_error_l = 0.0f;

// Right wheel PID
float error_r     = 0.0f;
float error_r_dt   = 0.0f;
float int_error_r = 0.0f;
float der_error_r = 0.0f;

float kp_l = 10.0f;
float ki_l = 3.0f;
float kd_l = 0.0f;

float kp_r = 10.0f;
float ki_r = 3.0f;
float kd_r = 0.0f;

int16_t pwm_left_cmd = 0,pwm_right_cmd = 0;
float pwm_l = 0,pwm_r = 0;

//avoids killing the hbridge
float last_pwm_l = 0.0f, last_pwm_r = 0.0f;
uint8_t changed_direction_l = 0, changed_direction_r = 0;

float V_req = 0; //Linear speed MAX: 2m/s
float W_req = 0; //Angular speed MAX: 2rad/s i think
float w_l = 0, w_r = 0; // En rad/s
float rpm_ref_left = 0, rpm_ref_right = 0; //convert w_l and w_r to rpms

void PID_PWM(float ref_l,float ref_r){ //1 = left, 2 = right
    // ----- LEFT MOTOR PID -----
	error_l = ref_l - rpm_1_filtered;
	if (fabsf(pwm_l) < 650.0f && fabsf(ref_l) > 1.0f) {
		int_error_l += 0.01f * error_l;
	}
	else if (fabsf(ref_l) <= 1.0f) {
		int_error_l = 0.0f;
	}
	der_error_l = (error_l - error_l_dt) / 0.01f;
	error_l_dt = error_l;

	// ----- RIGHT MOTOR PID -----
	error_r = ref_r - rpm_2_filtered;

	if (fabsf(pwm_r) < 650.0f && fabsf(ref_r) > 1.0f) {
	    int_error_r += 0.01f * error_r;
	}
	else if (fabsf(ref_r) <= 1.0f) {
	    int_error_r = 0.0f;
	}

	der_error_r = (error_r - error_r_dt) / 0.01f;
	error_r_dt = error_r;


	pwm_l = kp_l * error_l + ki_l * int_error_l + kd_l * der_error_l;
	pwm_r = kp_r * error_r + ki_r * int_error_r + kd_r * der_error_r;

	// ----- ANTI WIND UP (MAX PWM EXPERIMENTAL AROUND 2M/S = 77% DUTY -----
	if (pwm_l > 650.0f) {
		pwm_l = 650.0f;
	} else if (pwm_l < -650.0f) {
		pwm_l = -650.0f;
	}
	if (pwm_r > 650.0f) {
		pwm_r = 650.0f;
	} else if (pwm_r < -650.0f) {
		pwm_r = -650.0f;
	}

	// Map to actual macros (BTS driver expects positive duty and direction pins)
	// ====================================================
	// SAFETY LOGIC — DIRECTION CHANGE DETECTION
	// ====================================================
	// Detect sign change (zero crossing)
	if ((last_pwm_l * pwm_l < 0.0f) || (fabsf(pwm_l) < 5.0f)) {
		left_LPWM(0);
		left_RPWM(0);
		changed_direction_l = 1;
	} else {
		changed_direction_l = 0;
	}

	if ((last_pwm_r * pwm_r < 0.0f) || (fabsf(pwm_r) < 5.0f)) {
		right_LPWM(0);
		right_RPWM(0);
		changed_direction_r = 1;
	} else {
		changed_direction_r = 0;
	}

	// ====================================================
	// APPLY PWM ONLY IF NO CHANGE
	// ====================================================
	if (!changed_direction_l) {
		if (pwm_l >= 0.0f) {
			left_RPWM((uint16_t)pwm_l);
			left_LPWM(0);
		} else {
			left_LPWM((uint16_t)(-pwm_l));
			left_RPWM(0);
		}
	}

	if (!changed_direction_r) {
		if (pwm_r >= 0.0f) {
			right_RPWM((uint16_t)pwm_r);
			right_LPWM(0);
		} else {
			right_LPWM((uint16_t)(-pwm_r));
			right_RPWM(0);
		}
	}
	//Safety delta
	last_pwm_l = pwm_l;
	last_pwm_r = pwm_r;
	 // Debug save
	pwm_left_cmd = (int)pwm_l;
	pwm_right_cmd = (int)pwm_r;
}

void VW_DifferentialDrive(float V_req,float W_req){
    w_l = (V_req / WHEEL_RADIUS) + (WHEEL_DISTANCE * W_req / (2.0f * WHEEL_RADIUS));
    w_r = (V_req / WHEEL_RADIUS) - (WHEEL_DISTANCE * W_req / (2.0f * WHEEL_RADIUS));

    rpm_ref_left  = w_l * 60.0f / (2.0f * 3.14159265358979323846f);
    rpm_ref_right = w_r * 60.0f / (2.0f * 3.14159265358979323846f);

    PID_PWM(rpm_ref_left,rpm_ref_right);

}
