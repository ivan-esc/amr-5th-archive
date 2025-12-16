//////////////////////////////
////// I2C PROCESS source //// //////
//////////////////////////////
/*
 INPUT: EXTI-PULSES AND INIT
 OUTPUT: CURRENT COLOR / ROLL PITCH YAW
 READ FLAGS USABLE
 */

#include "i2c_color_imu.h"

// #define GRAVITY 9.80665f   // m/s²

// === MPU6050 Register Map ===
#define MPU6050_ADDR       0x68   // AD0 pin low (0x68), if high → 0x69
#define WHO_AM_I           0x75
#define PWR_MGMT_1         0x6B
#define SMPLRT_DIV         0x19
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define ACCEL_XOUT_H       0x3B

// === TCS34725 Register Map ===
#define TCS34725_ADDR       0x29
#define TCS34725_ENABLE     0x00
#define TCS34725_ATIME      0x01
#define TCS34725_CONTROL    0x0F
#define TCS34725_ID         0x12
#define TCS34725_CDATAL     0x14 // RGBC datos empiezan aquí

// Comando base para registrar
#define TCS34725_COMMAND_BIT 0x80
//#define I2C_TIMEOUT_COUNT 50000U


/////////////////////////////////////////
//////////////// MPU6050 ////////////////
/////////////////////////////////////////

// === Globals for sensor data ===
volatile int16_t AccelX, AccelY, AccelZ;
volatile int16_t GyroX, GyroY, GyroZ;
volatile float gAccelX, gAccelY, gAccelZ;
//float test = 0.0f;

float roll = 0, pitch = 0, yaw = 0;

// --- Globals for yaw drift reduction ---
static float yaw_bias = 0.0f;   // Online estimated drift (deg/s)
static int yaw_bias_init = 0;   // For initialization
float acc_mag = 0.0f;
uint8_t startup_time = 0;

// ---- Global variables for calibration ----
int32_t AccelX_sum = 0;
int32_t AccelY_sum = 0;
int32_t AccelZ_sum = 0;

float AccelX_offset = 0.0f;
float AccelY_offset = 0.0f;
float AccelZ_offset = 0.0f;

int32_t GyroX_sum = 0;
int32_t GyroY_sum = 0;
int32_t GyroZ_sum = 0;

float GyroX_offset = 0.0f;
float GyroY_offset = 0.0f;
float GyroZ_offset = 0.0f;

volatile float calibX, calibY, calibZ;


uint8_t calibration_done = 0;

/////////////////////////////////////////
//////////////// TCS34725 ///////////////
/////////////////////////////////////////

volatile uint16_t ColorR = 0, ColorG = 0, ColorB = 0, ColorC = 0;
volatile uint8_t R = 0, G = 0, B = 0, C = 0, maxVal = 0;
volatile uint8_t normR = 0, normG = 0, normB = 0, normC = 0;
volatile uint8_t current_color = 0;

////////////////////////////////////////
//Flags
volatile uint8_t imu_read_flag = 0;
volatile uint8_t color_read_flag = 0;
//static uint16_t tick_counter = 0;


// === I2C helper functions ===
void i2c1_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // === Use PB6=SCL, PB7=SDA ===
    GPIOB->MODER &= ~((3 << (6*2)) | (3 << (7*2)));
    GPIOB->MODER |=  (2 << (6*2)) | (2 << (7*2));    // AF
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->AFR[0] &= ~((0xF << (6*4)) | (0xF << (7*4)));
    GPIOB->AFR[0] |=  (1 << (6*4)) | (1 << (7*4));    // AF1 = I2C1

    GPIOB->OSPEEDR |= (3 << (6*2)) | (3 << (7*2));    // High speed
    GPIOB->PUPDR &= ~((3 << (6*2)) | (3 << (7*2)));
    GPIOB->PUPDR |=  (1 << (6*2)) | (1 << (7*2)); // Pull-up both lines

    // === Reset I2C1 ===
    I2C1->CR1 &= ~I2C_CR1_PE;
   // I2C1->CR1 |= I2C_CR1_SWRST;
   // I2C1->CR1 &= ~I2C_CR1_SWRST;

    // === Timing 100 kHz @ 8 MHz ===
    I2C1->TIMINGR = 0x2000090E;

    // === Enable I2C ===
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c1_write(uint8_t dev, uint8_t reg, uint8_t data) {
	// Send reg + data
	I2C1->CR2 = (dev << 1) | (2 << 16) | I2C_CR2_START | I2C_CR2_AUTOEND;
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = reg;
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = data;
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->ICR |= I2C_ICR_STOPCF; // clear STOP flag
}

void i2c1_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len) {
	// Write register address (no STOP, use RESTART)
	I2C1->CR2 = (dev << 1) | (1 << 16) | I2C_CR2_START;
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = reg;
	while (!(I2C1->ISR & I2C_ISR_TC));

	// Restart + read
	I2C1->CR2 = (dev << 1) | (len << 16) | I2C_CR2_RD_WRN |
				I2C_CR2_START | I2C_CR2_AUTOEND;
	for (uint8_t i = 0; i < len; i++) {
		while (!(I2C1->ISR & I2C_ISR_RXNE));
		buf[i] = I2C1->RXDR;
	}
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->ICR |= I2C_ICR_STOPCF;
}

void mpu6050_init(void) {
    i2c1_write(MPU6050_ADDR, PWR_MGMT_1, 0x00); // Wake up
    i2c1_write(MPU6050_ADDR, SMPLRT_DIV, 0x07);
    i2c1_write(MPU6050_ADDR, CONFIG, 0x04); //Filtro 4 21Hz
    i2c1_write(MPU6050_ADDR, GYRO_CONFIG, 0x00);
    i2c1_write(MPU6050_ADDR, ACCEL_CONFIG, 0x00);  // ±2g
}

void mpu6050_read_all(void) {
	uint8_t data[14];
	i2c1_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, data, 14);

	AccelX = (data[0] << 8) | data[1];
	AccelY = (data[2] << 8) | data[3];
	AccelZ = (data[4] << 8) | data[5];
	GyroX  = (data[8] << 8) | data[9];
	GyroY  = (data[10] << 8) | data[11];
	GyroZ  = (data[12] << 8) | data[13];

}

void TCS34725_Init(void) {
    i2c1_write(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_ENABLE, 0x01);
	// Enable the device (power on + ADC enable)

    for (volatile uint32_t i = 0; i < 80 * 8000; i++){} // crude 1ms at 8MHz

	i2c1_write(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_ENABLE, 0x03);
	// Set integration time (e.g. 0xC0 = 100ms)
	i2c1_write(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_ATIME, 0x40);
	// Set gain (1x, 4x, 16x, 60x). Example: 0x01 = 4x
	i2c1_write(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_CONTROL, 0x02);
}

void TCS34725_ReadRGB(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    i2c1_write(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_ENABLE, 0x03);

	uint8_t data[8];
	// Read 8 bytes starting at CDATAL
	i2c1_read_bytes(TCS34725_ADDR, TCS34725_COMMAND_BIT | TCS34725_CDATAL, data, 8);

	*c = (data[1] << 8) | data[0]; // Clear channel
	*r = (data[3] << 8) | data[2];
	*g = (data[5] << 8) | data[4];
	*b = (data[7] << 8) | data[6];
}

void calibrate_init(int16_t Ax, int16_t Ay, int16_t Az, uint16_t startup_time){
    if (calibration_done) return;  // Already done

    // Accumulate the first 100 samples
    AccelX_sum += Ax;
    AccelY_sum += Ay;
    AccelZ_sum += Az;

    GyroX_sum += GyroX;
    GyroY_sum += GyroY;
    GyroZ_sum += GyroZ;

    // After 100 samples (≈ 1 second if you run every 10 ms)
    if (startup_time >= 100)
    {
        AccelX_offset = (float)AccelX_sum / 100.0f;
        AccelY_offset = (float)AccelY_sum / 100.0f;
        AccelZ_offset = ((float)AccelZ_sum / 100.0f) - 16384.0f;

        GyroX_offset = (float)GyroX_sum / 100.0f;
        GyroY_offset = (float)GyroY_sum / 100.0f;
        GyroZ_offset = (float)GyroZ_sum / 100.0f;

        calibration_done = 1;
    }
}

void compute_angles(void)
{
    gAccelX = (AccelX - AccelX_offset) / 16384.0f;
    gAccelY = (AccelY - AccelY_offset) / 16384.0f;  // assuming ±2g
    gAccelZ = (AccelZ - AccelZ_offset) / 16384.0f;  // assuming ±2g

    calibX = (GyroX - GyroX_offset) / 16384.0f;
    calibY = (GyroY - GyroY_offset) / 16384.0f;  // assuming ±2g
    calibZ = (GyroZ - GyroZ_offset) / 16384.0f;  // assuming ±2g


    // --- Convert gyro to deg/s ---
    float gx = GyroX / GYRO_SENS;
    float gy = GyroY / GYRO_SENS;
    float gz_raw = GyroZ / GYRO_SENS;

    // --- First-run initialization ---
    if (!yaw_bias_init) {
        yaw_bias = gz_raw;    // whatever drift exists at startup
        yaw_bias_init = 1;
    }

    // --- Detect stationary condition ---
    acc_mag = sqrtf(gAccelX*gAccelX + gAccelY*gAccelY + gAccelZ*gAccelZ);

    int stationary_yaw = 0;
    if (fabsf(acc_mag - 1.0f) < 0.003f &&   // accel magnitude ~1g TOLERANCE ADJUST
            fabsf(gx) < 2.0f &&               // no roll rate
            fabsf(gy) < 2.0f &&               // no pitch rate
            fabsf(gz_raw) < 2.0f) {           // no yaw rate
            stationary_yaw = 1;
     }
    //test = acc_mag - 1.0f;

    // --- Update yaw bias ONLY when stationary ---
    if (stationary_yaw) {
        const float tau = 3.0f;
        float alpha = DT / (tau + DT);
        yaw_bias += alpha * (gz_raw - yaw_bias);
    }

    // --- Bias-corrected gyro Z ---
    float gz = gz_raw - yaw_bias;

    // --- Accelerometer tilt ---
    float accRoll  = atan2f(AccelY, AccelZ) * RAD_TO_DEG;
    float accPitch = atan2f(-AccelX,
                     sqrtf(AccelY*AccelY + AccelZ*AccelZ)) * RAD_TO_DEG;

    // --- Integrate gyro angles ---
    roll  += gx * DT;
    pitch += gy * DT;
    yaw   += gz * DT;

    // --- Complementary filter for roll/pitch ---
    roll  = 0.98f * roll  + 0.02f * accRoll;
    pitch = 0.98f * pitch + 0.02f * accPitch;

    // --- Clamp yaw ---
    if (yaw > 180.0f) yaw -= 360.0f;
    else if (yaw < -180.0f) yaw += 360.0f;
}

void imu_routine(void){ //function called when flag is raised
	mpu6050_read_all();
	if(startup_time<250) startup_time += 1;
	if (startup_time <= 100){calibrate_init(AccelX,AccelY,AccelZ,startup_time);}
	else {(compute_angles());}
}

void color_detect_routine(void){ //function called when flag is raised
	TCS34725_ReadRGB((uint16_t*)&ColorR, (uint16_t*)&ColorG, (uint16_t*)&ColorB, (uint16_t*)&ColorC);

	// normalize safely
	R = ColorR / 257;
	G = ColorG / 257;
	B = ColorB / 257;
	C = ColorC / 257;

	maxVal = R;
	if (G > maxVal) maxVal = G;
	if (B > maxVal) maxVal = B;
	if (maxVal == 0) maxVal = 1;

	normR = (R * 255) / maxVal;
	normG = (G * 255) / maxVal;
	normB = (B * 255) / maxVal;

	if (normR >= normG && normR >= normB) current_color = 3; // Red
	else if (normG >= normR && normG >= normB) current_color = 1; // Green
	else current_color = 2; // Blue

}
