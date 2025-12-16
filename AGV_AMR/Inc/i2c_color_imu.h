//////////////////////////////
////// I2C PROCESS header //// //////
//////////////////////////////
/*
 INPUT: EXTI-PULSES AND INIT
 OUTPUT: CURRENT COLOR / ROLL PITCH YAW
 READ FLAGS USABLE
 */

#ifndef I2C_COLOR_IMU_H_
#define I2C_COLOR_IMU_H_

#define RAD_TO_DEG       57.2958f
#define DEG_TO_RAD       0.01745329251f
#define GYRO_SENS 131.0f   // LSB/(°/s) for ±250 dps
#define DT 0.01f          // 10ms

#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern float roll;
extern float pitch;
extern float yaw;

extern volatile uint8_t current_color;

extern volatile uint8_t imu_read_flag;
extern volatile uint8_t color_read_flag;
extern uint8_t startup_time;
extern volatile int16_t GyroX, GyroY, GyroZ;



void i2c1_init(void);
void i2c1_write(uint8_t dev, uint8_t reg, uint8_t data);
void i2c1_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

void TCS34725_Init(void);
void TCS34725_ReadRGB(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

void mpu6050_init(void);
void mpu6050_read_all(void);

void compute_angles(void);
void calibrate_init(int16_t Ax, int16_t Ay, int16_t Az, uint16_t startup_time);

void imu_routine(void); //function called when flag is raised
void color_detect_routine(void); //function called when flag is raised


#endif /* I2C_COLOR_IMU_H_ */
