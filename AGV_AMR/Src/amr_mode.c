#include "amr_mode.h"

#include <math.h>
#include <stdio.h>

// Gains (tune these)
#define ANGLE_P_GAIN     1.4f    // rad/s per radian of angle error (for W_req)
#define YAW_ANGLE_P_DEG  1.4f    // alternative gain in deg->converted below
#define LINEAR_RAMP_ACC  0.4f    // m/s^2 acceleration when ramping up
#define ANGLE_SLOW_THRESH_DEG  15.0f  // deg where linear speed reduces
#define ANGLE_FULL_SLOW_DEG    45.0f  // deg where linear speed ideally goes to 0

// limits
#define AMR_MAX_V_DEFAULT 0.25f   // m/s nominal default
#define AMR_MAX_W_DEFAULT 2.0f    // rad/s max angular speed

// exported variables
uint8_t amr_active = 0;
uint8_t amr_loop_mode = 0;
float position_error_threshold = 0.005f; // 5 mm default
float amr_nominal_speed = AMR_MAX_V_DEFAULT;
float amr_max_angular_speed = AMR_MAX_W_DEFAULT;

// internal state
typedef enum {
    AMR_IDLE = 0,
    AMR_INIT_WAIT,        // calling Reset_x_y() and waiting 1s
    AMR_ALIGN_TO_FIRST,   // rotate in place to face first point
    AMR_DRIVE_TO_FIRST,   // drive to first point
    AMR_ALIGN_NEXT,       // rotate to heading for next segment
    AMR_FOLLOW_ROUTE,     // follow the route with continuous updates
    AMR_FINISHED
} amr_state_t;

static amr_state_t state = AMR_IDLE;
static uint32_t init_wait_start_ms = 0;
static uint16_t current_index = 0;   // index of current target (0..valid_indexes-1)
static uint16_t next_index = 0;      // next index for lookahead
static float lastV = 0.0f;           // for ramping
static uint32_t now_ms = 0;

// helpers
static float mm_to_m(int16_t mm) { return ((float)mm) / 1000.0f; }
static float point_x_m(uint16_t idx) { return mm_to_m(pathX[idx]); }
static float point_y_m(uint16_t idx) { return mm_to_m(pathY[idx]); }

// normalize angle to [-180,180]
static float norm_deg(float ang) {
    while (ang > 180.0f) ang -= 360.0f;
    while (ang <= -180.0f) ang += 360.0f;
    return ang;
}

// minimal absolute value
static float abs_f(float x) { return x < 0 ? -x : x; }

// compute Euclidean distance
static float dist2d(float ax, float ay, float bx, float by) {
    float dx = bx - ax;
    float dy = by - ay;
    return sqrtf(dx*dx + dy*dy);
}

// compute angle from current odom pose to a point (degrees)
static float angle_to_point_deg(float tx, float ty) {
    // relative vector from robot origin (we assume odometry is in same frame)
    float dx = tx - 0.0f; // if origin assumed; often pos_x,pos_y zeroed earlier
    float dy = ty - 0.0f;
    // However we use global odometry later; this function used for first point from origin.
    float a = atan2f(dy, dx) * RAD_TO_DEG;
    return norm_deg(a);
}

// compute angle from current robot position to a target point (degrees)
static float angle_from_pose_to_point_deg(float rx, float ry, float tx, float ty) {
    float dx = tx - rx;
    float dy = ty - ry;
    float a = atan2f(dy, dx) * RAD_TO_DEG;
    return norm_deg(a);
}

// set low-level drive outputs using global V_req, W_req
static void apply_drive_refs(void) {
    // V_req and W_req are globals used by VW_DifferentialDrive
    // Call the mapping function to set wheel RPM references
    VW_DifferentialDrive(V_req, W_req);
}

// reset internals
void resetAMR(void) {
    // stop motion
    V_req = 0.0f;
    W_req = 0.0f;
    apply_drive_refs();

    // reset indices & flags
    current_index = 0;
    next_index = 0;
    lastV = 0.0f;

    // reset odometry & imu (user-provided function)
    Reset_x_y();

    // go to idle
    state = AMR_IDLE;
    amr_active = 0;
}

// alias stop
void AMR_Stop(void) {
    resetAMR();
}

// start sequence: resets and begin init wait (imu calibrate)
void AMR_Start(void) {
    // reset odometry & internal state
    Reset_x_y();
    current_index = 0;
    next_index = 0;
    lastV = 0.0f;

    // set flags
    amr_active = 1;

    // wait 1000 ms for imu/calibration
    state = AMR_INIT_WAIT;
    init_wait_start_ms = 0; // will be set in first Update with millis()
}

// initialize module (call once from main)
void AMR_Init(void) {
    amr_active = 0;
    amr_loop_mode = 0;
    position_error_threshold = 0.1f;
    amr_nominal_speed = AMR_MAX_V_DEFAULT;
    amr_max_angular_speed = AMR_MAX_W_DEFAULT;
    state = AMR_IDLE;
}

// Main update — call every 10 ms
void AMR_Update(void) {
    now_ms = now_ms + 1;

    if (!amr_active) {
        // ensure zero motion if not active
        V_req = 0.0f;
        W_req = 0.0f;
        apply_drive_refs();
        return;
    }

    // Safety: if no valid points, remain idle
    if (valid_indexes == 0) {
        V_req = 0.0f;
        W_req = 0.0f;
        apply_drive_refs();
        return;
    }

    switch (state) {
        case AMR_INIT_WAIT:
            if (init_wait_start_ms == 0) init_wait_start_ms = now_ms;
            if ((now_ms - init_wait_start_ms) >= 100) {
                // finished calibration/wait
                // prepare indexes and align to first
                current_index = 0;
                next_index = (valid_indexes > 1) ? 1 : 0;
                state = AMR_ALIGN_TO_FIRST;
            } else {
                // keep motors stopped during calibrate
                V_req = 0.0f; W_req = 0.0f;
                apply_drive_refs();
            }
            break;

        case AMR_ALIGN_TO_FIRST: {

        	if (current_index == 0 && pathX[0] == 0 && pathY[0] == 0) {
        	    // immediately treat point0 as "reached"
        	    if (valid_indexes > 1) {
        	        next_index = 1;
        	        state = AMR_ALIGN_NEXT;
        	    } else {
        	        // only 0,0 exists → nothing to do
        	        V_req = 0.0f;
        	        W_req = 0.0f;
        	        apply_drive_refs();
        	        state = AMR_FINISHED;
        	    }
        	    break;
        	}

            // compute first target point in meters
            float tx = point_x_m(current_index);
            float ty = point_y_m(current_index);

            // compute angle from robot origin (odometry was reset so pos_x,pos_y ~ 0)
            // but better to use current odometry (pos_x,pos_y) to be general:
            float desired_deg = angle_from_pose_to_point_deg(pos_x, pos_y, tx, ty);
            float yaw_deg = (float)rounded_yaw_relative; // degrees
            float err_deg = norm_deg(desired_deg - yaw_deg);

            // convert to radians for W_req
            float err_rad = err_deg * DEG_TO_RAD;

            // angular P controller
            float w = ANGLE_P_GAIN * err_rad;
            if (w > amr_max_angular_speed) w = amr_max_angular_speed;
            if (w < -amr_max_angular_speed) w = -amr_max_angular_speed;

            // in align stage rotate in place
            V_req = 0.0f;
            W_req = -w;
            apply_drive_refs();

            // arrival condition: yaw within a small threshold (2 degrees)
            if (fabsf(err_deg) < 6.0f) {
                // stop rotation
                V_req = 0.0f; W_req = 0.0f;
                apply_drive_refs();
                // move to drive stage
                state = AMR_DRIVE_TO_FIRST;
            }
            break;
        }

        case AMR_DRIVE_TO_FIRST: {
            float tx = point_x_m(current_index);
            float ty = point_y_m(current_index);

            // distance to target
            float dist = dist2d(pos_x, pos_y, tx, ty);

            // desired heading to target
            float desired_deg = angle_from_pose_to_point_deg(pos_x, pos_y, tx, ty);
            float yaw_deg = (float)rounded_yaw_relative;
            float err_deg = norm_deg(desired_deg - yaw_deg);
            float err_rad = err_deg * DEG_TO_RAD;

            // compute W_req (P control)
            float w = ANGLE_P_GAIN * err_rad;
            if (w > amr_max_angular_speed) w = amr_max_angular_speed;
            if (w < -amr_max_angular_speed) w = -amr_max_angular_speed;

            // set linear V_req ramping towards nominal, but reduce with yaw error
            float yaw_abs_deg = fabsf(err_deg);
            float reduce = 0.0f;
            if (yaw_abs_deg > ANGLE_SLOW_THRESH_DEG)
                reduce = (yaw_abs_deg - ANGLE_SLOW_THRESH_DEG) / (ANGLE_FULL_SLOW_DEG - ANGLE_SLOW_THRESH_DEG);
            if (reduce > 1.0f) reduce = 1.0f;
            float targetV = amr_nominal_speed * (1.0f - reduce);

            // simple ramping
            float maxInc = LINEAR_RAMP_ACC * 0.01f; // in 10ms step
            if (lastV < targetV) {
                lastV += maxInc;
                if (lastV > targetV) lastV = targetV;
            } else if (lastV > targetV) {
                lastV -= maxInc;
                if (lastV < targetV) lastV = targetV;
            }

            V_req = lastV;
            W_req = -w;
            apply_drive_refs();

            // reached target?
            if (dist <= position_error_threshold) {
                // we arrived to first point: rotate to next segment heading if exists
                // update next_index based on valid_indexes
                if (valid_indexes > 1) {
                    next_index = (current_index + 1 < valid_indexes) ? current_index + 1 : current_index;
                    state = AMR_ALIGN_NEXT;
                } else {
                    // only one point; we are done
                    V_req = 0.0f; W_req = 0.0f;
                    apply_drive_refs();
                    state = AMR_FINISHED;
                }
            }
            break;
        }

        case AMR_ALIGN_NEXT: {
            // compute heading between current point and next point
            // current target coords
            float cx = point_x_m(current_index);
            float cy = point_y_m(current_index);
            // next coords
            float nx = point_x_m(next_index);
            float ny = point_y_m(next_index);

            // angle from current to next (global)
            float desired_deg = angle_from_pose_to_point_deg(cx, cy, nx, ny);

            // but robot is at pos_x,pos_y; we just want robot yaw to equal that desired_deg
            float yaw_deg = (float)rounded_yaw_relative;
            float err_deg = norm_deg(desired_deg - yaw_deg);
            float err_rad = err_deg * DEG_TO_RAD;

            float w = ANGLE_P_GAIN * err_rad;
            if (w > amr_max_angular_speed) w = amr_max_angular_speed;
            if (w < -amr_max_angular_speed) w = -amr_max_angular_speed;

            V_req = 0.0f;
            W_req = -w;
            apply_drive_refs();

            if (fabsf(err_deg) < 6.0f) {
                // now start following the route toward next point
                V_req = 0.0f; W_req = 0.0f;
                apply_drive_refs();
                // move to follow route
                // set current_index to next target, set next for lookahead
                current_index = next_index;
                next_index = (current_index + 1 < valid_indexes) ? current_index + 1 : current_index;
                state = AMR_FOLLOW_ROUTE;
            }
            break;
        }

        case AMR_FOLLOW_ROUTE: {
            // if only one point or reached last, finish
            if (current_index >= valid_indexes) {
                V_req = 0.0f; W_req = 0.0f;
                apply_drive_refs();
                state = AMR_FINISHED;
                break;
            }

            // target is path[current_index]
            float tx = point_x_m(current_index);
            float ty = point_y_m(current_index);

            // compute distance & heading to current target
            float dist = dist2d(pos_x, pos_y, tx, ty);
            float desired_deg = angle_from_pose_to_point_deg(pos_x, pos_y, tx, ty);
            float yaw_deg = (float)rounded_yaw_relative;
            float err_deg = norm_deg(desired_deg - yaw_deg);
            float err_rad = err_deg * DEG_TO_RAD;

            // compute W_req (P)
            float w = ANGLE_P_GAIN * err_rad;
            if (w > amr_max_angular_speed) w = amr_max_angular_speed;
            if (w < -amr_max_angular_speed) w = -amr_max_angular_speed;

            // compute desired V reduced by yaw error
            float yaw_abs_deg = fabsf(err_deg);
            float reduce = 0.0f;
            if (yaw_abs_deg > ANGLE_SLOW_THRESH_DEG)
                reduce = (yaw_abs_deg - ANGLE_SLOW_THRESH_DEG) / (ANGLE_FULL_SLOW_DEG - ANGLE_SLOW_THRESH_DEG);
            if (reduce > 1.0f) reduce = 1.0f;
            float targetV = amr_nominal_speed * (1.0f - reduce);

            // ramping
            float maxInc = LINEAR_RAMP_ACC * 0.01f; // per 10ms step
            if (lastV < targetV) {
                lastV += maxInc;
                if (lastV > targetV) lastV = targetV;
            } else if (lastV > targetV) {
                lastV -= maxInc;
                if (lastV < targetV) lastV = targetV;
            }

            V_req = lastV;
            W_req = -w;
            apply_drive_refs();

            // check arrival: if within threshold, step to next index
            if (dist <= position_error_threshold) {
                // reached this waypoint
                if (current_index + 1 < valid_indexes) {
                    // advance
                    current_index++;
                    next_index = (current_index + 1 < valid_indexes) ? current_index + 1 : current_index;
                    // optionally immediately update desired heading (alpha<-beta)
                } else {
                    // reached final waypoint
                    if (amr_loop_mode) {
                        // loop: go back to first
                        current_index = 0;
                        next_index = (valid_indexes > 1) ? 1 : 0;
                        // continue
                    } else {
                        // stop
                        V_req = 0.0f; W_req = 0.0f;
                        apply_drive_refs();
                        state = AMR_FINISHED;
                    }
                }
            }

            break;
        }

        case AMR_FINISHED:
            // stopped, report done, keep motors idle
            V_req = 0.0f; W_req = 0.0f;
            apply_drive_refs();
            amr_active = 0;
            // stay in FINISHED until external reset or start
            break;
    } // switch
}
