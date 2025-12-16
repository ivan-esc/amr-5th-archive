/*CODIGO EQUIPO NEGRO */

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Headers
#include "motor_pwm.h"
#include "encoders.h"
#include "ultrasonic.h"
#include "bluetooth.h"
#include "esp32_com.h"
#include "pid_diff_drive.h"
#include "i2c_color_imu.h"
#include "line_follow.h"
#include "odometry.h"
#include "amr_mode.h"

volatile uint32_t t = 0; //tiempo base
volatile uint8_t emergency_on = 0; //toggle emergencia


float w_ll = 0.0f;
float w_rr = 0.0f;
float VV = 0.0f;
float WW = 0.0f;

volatile int16_t x = 0; // Reworked mapped coordinate
volatile int16_t y = 0; // Reworked mapped coordinate


/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////    DECLARACION DE FUNCIONES    //////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

void BT_process_data(void);

void TIM14_Init(void); 			//10ms clock init
void ManualDrive(void);	//Manual mode (1) desired RPMs

static uint8_t amr_initialized = 0;   // 0 = not initialized, 1 = init done
void reset_references(void);

void emergency_stop(void);				//Actions when stopped
void check_emergency_buttons(void);		//EXTI for Emergency Pins (pending assignment)
void init_emergency_buttons(void);		//EXTI channel init for emergency
void PC7_Button_Check(void);
void PC7_Button_Init(void);

void SelectManualLEDCommand(void);		//Manual mode joystick LED aplication


int main(void)
{

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	NVIC->ISER[0] |= (1 << EXTI4_15_IRQn);

	i2c1_init(); //COLOR AND IMU

	USART2_Init(); //BLUETOOTH INIT CHECK
	USART1_Init(); //ESP32 COM INIT CHECK

	TIM14_Init(); //ROUTINE TIMER
	pwm_init(); //MOTOR CHANNELS CHECK
	IR_Sensors_Init(); //LINE FOLLOWER
	init_GPIO_EXTI_encoders(); //ENCODERS CHECK

	TIM3_init(); //ULTRASONIC TIMING CHECK
	init_GPIO_EXTI_ultrasonic(); //ULTRASONIC INIT CHECK

	__enable_irq(); //ENABLE INTERRUPTIONS CHECK
	mpu6050_init();
	//TCS34725_Init();

	Odometry_Init();
	init_emergency_buttons();
	PC7_Button_Init();

	AMR_Init();          // reset odometry, set internals, etc.

    /* Loop forever */

  	while (1){
        check_emergency_buttons();
  		if (imu_read_flag) {
			imu_read_flag = 0;
			imu_routine();
  		}
		if (color_read_flag) {
			color_read_flag = 0;
			color_detect_routine();
		}
	}
}




void TIM14_Init(void) { //10ms Timer Setup
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 8 - 1;      // 8MHz/8 = 1MHz
    TIM14->ARR = 10000 - 1;  // 10ms period
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_CEN;
	NVIC_SetPriority(TIM14_IRQn, 1);  // Lower value = higher priority
    NVIC_EnableIRQ(TIM14_IRQn);
}
void TIM14_IRQHandler(void) { //10 ms timer
    if(TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        t += 10;

        //CONTINUOS SENSOR READS
        imu_read_flag = 1;
        check_emergency_buttons();
        velocidad_encoder();
		hcsr04_read();
		Odometry_Update();

        if (rx_buffer[0]!=255){PC7_Button_Check();}

        switch(menu_button){
        case 0: // OFF - Just Connected
        	reset_references();
        	break;
        case 1: //MANUAL MODE: idle
        	reset_references();
        	LED_action = 14;
        	break;
        case 2: //MANUAL MODE: running
        	ManualDrive();
        	break;
        case 3: //LINE FOLLOW: idle configuration
        	reset_references();
        	LED_action = 7;
        	break;
        case 4: // LINE FOLLOW MODE
        {
            if((distance_cm[2]>30.0f) && (distance_cm[0]>15.0f) && (distance_cm[3]>15.0f)){
				if(linea){
					seguir();}
            break;
            }
            else{
                VW_DifferentialDrive(0.0f,0.0f);
            	near_obj=1 ;
				V_current = 0;
				LED_action = 11;
				W_current = 0;
			break;
            }
        }
		case 5: //AMR: idle configuration
			reset_references();
			if(valid_indexes > 0){
				LED_action = 21;
			}
			else{
				LED_action = 19;
			}
			break;
        case 6: //AMR: RUNNING
            if((distance_cm[2]>30.0f) && (distance_cm[0]>15.0f) && (distance_cm[3]>15.0f)){
				LED_action = 0;
				if (!amr_initialized) {
					AMR_Start();
					amr_initialized = 1;
				}
				AMR_Update();

			break;
			}
			else{
				VW_DifferentialDrive(0.0f,0.0f);
				near_obj=1 ;
				V_current = 0;
				LED_action = 11;
				W_current = 0;
			break;
			}
        case 7: //SETTINGS
        	LED_action = 12;
        	reset_references();
        	break;
        case 8: // ESTOP?
        	reset_references();
        	LED_action = 13;
        	break;

        case 20: /// RGB TEST
			reset_references();
        	LED_action = 15;
			break;

		case 21: /// song1
			reset_references();
			LED_action = 31;
			break;

		case 22: /// song1
			reset_references();
			LED_action = 30;
			break;
		}


		static uint8_t last_led_action = 0xFF;
		uint8_t cur = LED_action; // Mandar comand a ESP32 solo cuando cambia
		if (cur != last_led_action) {
			USART1_send(cur);
			last_led_action = cur;
		}

		if (t >= 70) { //MANDAR DATOS DE BLUETOOTH CADA 200ms
			BT_process_data();
			BT_SendTelemetry();
			t = 0;
		}
    }
}



void ManualDrive(void){
	//Logic calculation based on 250x250 map, where 125x125 is defined as center
	//Assume a +-30 deadzone for generous inputs

	 x = raw_x - 125; //Set initial coords to (0,0)
     y = 125- raw_y;
     if (abs(x) < 30){ x = 0;}
     if (abs(y) < 30){ y = 0;}
     //Normalize
     V_req = y*0.8f/125.0f; //Max speed will be set to 0.8m/s for plot convenience
     W_req = x*0.8f*2.5f/125.0f; // Angular speed

     VW_DifferentialDrive(V_req,W_req);
     SelectManualLEDCommand();
}


void BT_process_data(void) {
    // Convert from RPM to rad/s
    w_ll = rpm_1_filtered * 2.0f * 3.14159265359f / 60.0f;
    w_rr = rpm_2_filtered * 2.0f * 3.14159265359f / 60.0f;

    // Compute linear and angular velocity
    VV = (WHEEL_RADIUS * (w_rr + w_ll)) / 2.0f;
    WW = (WHEEL_RADIUS * (w_rr - w_ll)) / WHEEL_DISTANCE;

    linear_velocity = (uint8_t)((fabsf(VV)*10) + 0.5f);
    angular_velocity = (uint8_t)((fabsf(WW)*10) + 0.5f);

    tmp_data = temperature10;
    //voltage_sent = voltage10;
    voltage_sent = 118;

    posx_small = (uint8_t)(pos_x_mm & 0xFF);        // byte bajo  (LSB)
    posx_big   = (uint8_t)((pos_x_mm >> 8) & 0xFF); // byte alto  (MSB)

    posy_small = (uint8_t)(pos_y_mm & 0xFF);
    posy_big   = (uint8_t)((pos_y_mm >> 8) & 0xFF);

}

void init_emergency_buttons(void) {
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Set PC6 as inputs (MODER = 00)
    GPIOC->MODER &= ~(3 << (6 * 2));

    // Optional: enable pull-downs if buttons are active-high
    GPIOC->PUPDR &= ~(3 << (6 * 2)); // Clear
}
void check_emergency_buttons(void) {
    // If PC6 is HIGH
    if ((GPIOC->IDR & (1 << 6))) {
        emergency_stop();
    }
    else{
    	emergency_on = 0;
    }
}
void emergency_stop(void) {
    menu_button = 0;
    emergency_on = 1;
    reset_references();
	LED_action = 13;
}

// ----- Initialize PC7 as input -----
void PC7_Button_Init(void) {
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Set PC7 as input (MODER = 00)
    GPIOC->MODER &= ~(3 << (7*2));

    // Optional: enable pull-down or pull-up if needed
    GPIOC->PUPDR &= ~(3 << (7*2));   // clear
    GPIOC->PUPDR |=  (1 << (7*2));   // pull-up
}
// ----- Check PC7 periodically -----
void PC7_Button_Check(void) {
    if (GPIOC->IDR & (1 << 7)) {  // if PC7 is HIGH
        menu_button = 6;
    }
    else {
        menu_button = 0;          // optional: reset if low
    }
}

void SelectManualLEDCommand(void) {
    // Default = idle
    int cmd = 0;
    if (y > 30 && abs(x) < 40) {
        // Forward
        cmd = 3;
    }
    else if (y < -30 && abs(x) < 40) {
        // Backward
        cmd = 4;
    }
    else if (x < -30 /*&& abs(y) < 40*/) {
        // Left
        cmd = 5;
    }
    else if (x > 30 /*&& abs(y) < 40*/) {
        // Right
        cmd = 6;
    }
    else if (abs(x) < 30 && abs(y) < 30) {
        // Deadzone → Idle
        cmd = 0;
    }
    LED_action = cmd;
}
void reset_references(void){
	PID_PWM(0,0);
	left_RPWM(0);
	left_LPWM(0);
	right_RPWM(0);
	right_LPWM(0);
	rpm_ref_left=0;
	rpm_ref_right=0;
	V_current = 0.0f;
	W_current = 0.0f;
	last_position_error = 0.0f;
	exit_timer = 0;
	station_done = true;     // back to normal mode
	leaving_station = false;  // fully reset
	counter_stations = 1;
	AMR_Stop();             // <- resets AMR state machine, indices, odometry via Reset_x_y()
	amr_initialized = 0;    // allow re-init next time AMR is selected


}
