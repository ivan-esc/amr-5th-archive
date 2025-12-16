//////////////////////////////////
//// LINE FOLLOW ALGORITHM ///////
//////////////////////////////////
/*
 INPUT: EXE FUNCTIONS
 OUTPUT: IF LINEA SEGUIR()
 */

////////////////////////////////////////////////////////////////////
/////////////// SEGUIDOR DE LINEA VARIABLES DE LOGICA //////////////
////////////////////////////////////////////////////////////////////

#include "line_follow.h"
#include "bluetooth.h"
#include "ultrasonic.h"
#include "esp32_com.h"
#include "pid_diff_drive.h"
#include "i2c_color_imu.h"

float V_current = 0.0f; // acceleration ramp memory
float W_current = 0.0f;  // memory of last angular velocity command
float last_position_error = 0.0f;
volatile uint8_t sensor[6];

bool station_done = true;
bool leaving_station = false;   // tracks if we're exiting a station
uint16_t exit_timer = 0;
uint8_t entering_station = 0;

volatile uint8_t counter_stations = 1; // fake ass mf
uint8_t linea = 1;

void seguir(void){
    // --- 1. Normal line following ---
    if (Check_All_Sensors_Black() == false && station_done == false) {
        entering_station = 1;
      	LineFollow();
       	LED_action = 10;
       	near_obj=0;
    }

    // --- 2. Entering station ---
    else if (Check_All_Sensors_Black() == true && station_done == false) {
        SmoothStop();
        LED_action = 14;
        if (fabsf(V_current) < 0.02f) {
            Station_Color_Detect();
            leaving_station = false;
            exit_timer = 0;
        }
    }

    // --- 3. Leaving station (timed exit) ---
    else if (station_done == true && leaving_station == false) {
    	LED_action = 10;
        current_color = 0;
        colorin=0;
        const float slow_exit_speed = 0.07f;
        const float V_target = slow_exit_speed;
        V_current = (V_current < V_target) ? (V_current + 0.005f) : V_target;

        VW_DifferentialDrive(V_current,0);

        // increment timer every 10 ms
        exit_timer++;
        if (exit_timer >= 300) {  // 2.5 seconds
            leaving_station = true;
            exit_timer = 0;  // reset for next phase
        }
    }

    // --- 4. After leaving station, resume normal line following ---
    else if (station_done == true && leaving_station == true) {
        V_current = 0.0f;
        exit_timer++;

        if (exit_timer >= 100) {  // 1 second "stabilization" period
        	LineFollow();
            station_done = false;     // back to normal mode
            leaving_station = false;  // fully reset
            exit_timer = 0;           // ready for next station
        }
    }

}



bool Check_All_Sensors_Black(void){
	uint8_t all_black = 0;
    int black_count = 0;
	for (int i = 0; i < 6; i++) {
		if (sensor[i]) black_count++;
	}
	if (black_count >= 5){
		all_black = 1;
	}
	return all_black;
}

void SmoothStop(void) {
    const float DECEL_RATE = 0.30f;  // m/s² — tune this for braking smoothness
    const float TS = 0.01f;          // 10ms loop
    const float MAX_DELTA_V = DECEL_RATE * TS;

    if (V_current > 0.0f) {
        V_current -= MAX_DELTA_V;
        if (V_current < 0.0f) V_current = 0.0f;
    }

    VW_DifferentialDrive(V_current,0);
}
void Station_Color_Detect(void) {
	if (entering_station == 1) {
		//color_read_flag = 1;

		///// TRAMPA POR FALSO CONTACTO DEL SENSOR DE COLOR ///////////
			counter_stations += 1;
			if (counter_stations == 4){
				(counter_stations = 1);//1green//2blue/3red
			}
			current_color = counter_stations;

		///////////////////////&///////////////////////////////////////
		///////////////////////&//////////////////////////////////////*/
		entering_station = 0;
    }

	if (current_color == 3 && coded_route_recieved % 5 == 0){ //RED
		colorin = 2;
    	LED_action = 8;
		station_wait_thing();
	}
	else if (current_color == 1 && coded_route_recieved % 7 == 0){ //GREEN
		colorin = 3;
    	LED_action = 7;
		station_wait_thing();
	}
	else if (current_color == 2 && coded_route_recieved % 3 == 0){ //BLUE
		colorin = 1;
    	LED_action = 9;
		station_wait_thing();
	}
	else{
		station_done = true;
	}
}

void station_wait_thing(void){
	static uint16_t wait_ticks = 0;

	if (coded_recieved_bool1 % 2 == 0){ //0 = User // else = timer
			if(coded_recieved_bool1 % 3 == 0){ //0 - GO_command // else = wait still
				wait_ticks = 0;
				station_done = true;
			}
			else if(coded_recieved_bool1 % 3 != 0){ //0 - GO_command // else = wait still

			}
	}
	else if (coded_recieved_bool1 % 2 != 0){
		wait_ticks += 1;
		if (wait_ticks >= (station_wait_time*10)){
			station_done = true;
			wait_ticks = 0;
		}
	}
}

//INIT SENSORS AND LINE FOLLOW
void IR_Sensors_Init(void) {
    // === 1. Enable GPIOC clock ===
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // === 2. Configure PC0–PC5 as input (00: Input mode) ===
    GPIOC->MODER &= ~(0x0000FFFF);  // Clear MODER bits for PC0–PC5 (2 bits per pin)

    // === 3. (Optional) Enable internal pull-ups ===
    // Assuming your IR sensors pull the line LOW (0) when detecting black.
    // If they output HIGH on black, use PULL-DOWN instead.
    GPIOC->PUPDR &= ~(0x0000FFFF);  // Clear PUPDR for PC0–PC5
    GPIOC->PUPDR |=  (0x00005555);  // Set pull-up (01 per pin)
}

void LineFollow(void) {
    const float sensor_weight[6] = {-3, -2, -1, 1, 2, 3};

    // Read IR sensors (your custom mapping)
    sensor[0] = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
    sensor[1] = (GPIOC->IDR & (1 << 3)) ? 1 : 0;
    sensor[2] = (GPIOC->IDR & (1 << 4)) ? 1 : 0;
    sensor[3] = (GPIOC->IDR & (1 << 0)) ? 1 : 0;
    sensor[4] = (GPIOC->IDR & (1 << 1)) ? 1 : 0;
    sensor[5] = (GPIOC->IDR & (1 << 5)) ? 1 : 0;

    // --- 1. Compute line position error ---
    float position_error = 0.0f;
    int active = 0;
    for (int i = 0; i < 6; i++) {
        if (sensor[i]) {
            position_error += sensor_weight[i];
            active++;
        }
    }

    if (active > 0) {
        position_error /= active;
        last_position_error = position_error;
    } else {
        position_error = last_position_error;
    }

    // --- 2. Angular velocity (line correction) ---
    const float K_line = 0.35f; // tune experimentally
    float W_target = K_line * position_error;

    // --- 3. Acceleration-limited linear velocity ---
    const float ACCEL_RATE = 0.2f;  // m/s²
    const float TS = 0.01f;         // 10ms loop
    const float MAX_DELTA_V = ACCEL_RATE * TS;

    float V_target = 0.25f;
    float dV = V_target - V_current;
    if (dV > MAX_DELTA_V) V_current += MAX_DELTA_V;
    else if (dV < -MAX_DELTA_V) V_current -= MAX_DELTA_V;
    else V_current = V_target;

    // --- 4. Acceleration-limited angular correction ---
    const float ANG_ACCEL_RATE = 1.25f;  // rad/s²  <<< TUNE THIS VALUE
    const float MAX_DELTA_W = ANG_ACCEL_RATE * TS;

    float dW = W_target - W_current;
    if (dW > MAX_DELTA_W) W_current += MAX_DELTA_W;
    else if (dW < -MAX_DELTA_W) W_current -= MAX_DELTA_W;
    else W_current = W_target;

    VW_DifferentialDrive(V_current,W_current);
}

