///////////////////////////////
//// ULTRASONIC SOURCE ////////
///////////////////////////////
/*
 INPUT: CALLED IN TIM14 IRQ HANDLER
 OUTPUT: DISTANCE_CM ARRAY
 */

#include "ultrasonic.h"
#include "encoders.h" //needs encoders because irq handling

volatile uint16_t echoStart[4] = {0,0,0,0};
volatile uint16_t echoEnd[4]   = {0,0,0,0};
volatile float distance_cm[4]  = {0,0,0,0};
volatile uint8_t currentSensor = 0; // 0..3

static const uint8_t echo_pin[4]  = { 1, 9, 12, 13 }; // PB1, PB9, PB12, PB13
#define COMMON_TRIG_C       10 // TRIG pin1: PC10
#define COMMON_TRIG_B       0 // TRIG pin2: PB0
volatile uint8_t ultrasonic_read_delay_flag = 0;
volatile uint8_t noObjectCounter[4] = {0,0,0,0};  // counts consecutive out-of-range readings

void hcsr04_read(void){
	ultrasonic_read_delay_flag += 1;
	if (ultrasonic_read_delay_flag >= 5) { // every ~50 ms trigger next sensor
		start_hcsr04_trigger();
	}

}

void TIM3_init(void)
{
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // TIM3: 1 MHz tick (1 us)
    TIM3->PSC = 8 - 1;   // 8MHz / 8 = 1 MHz
    TIM3->ARR = 0xFFFF;  // free-running 16-bit
    TIM3->CR1 |= TIM_CR1_CEN;

    // Enable TIM3 IRQ in NVIC (handler used for CCR1)
    NVIC->ISER[0] |= (1 << TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR &= ~TIM_SR_CC1IF;
        // Clear BOTH trigger pins (PC10 + PB0)
		GPIOC->BRR = (1u << COMMON_TRIG_C);
		GPIOB->BRR = (1u << COMMON_TRIG_B);
        // disable CC1 interrupt until next trigger
        TIM3->DIER &= ~TIM_DIER_CC1IE;
    }
}
void init_GPIO_EXTI_ultrasonic(void)
{
	// Enable GPIOB, GPIOC clocks and SYSCFG
	    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* -------- TRIG PINS: PC10 + PB0 -------- */

	// PC10: output, push-pull, low
	GPIOC->MODER &= ~(3u << (COMMON_TRIG_C * 2));
	GPIOC->MODER |=  (1u << (COMMON_TRIG_C * 2)); // output
	GPIOC->OTYPER &= ~(1u << COMMON_TRIG_C);
	GPIOC->OSPEEDR |= (3u << (COMMON_TRIG_C * 2)); // push-pull
	GPIOC->BRR = (1u << COMMON_TRIG_C); // ensure low

	// PB0: output, push-pull, low
	GPIOB->MODER &= ~(3u << (COMMON_TRIG_B * 2));
	GPIOB->MODER |=  (1u << (COMMON_TRIG_B * 2)); // output
	GPIOB->OTYPER &= ~(1u << COMMON_TRIG_B);
	GPIOB->OSPEEDR |= (3u << (COMMON_TRIG_B * 2)); // push-pull
	GPIOB->BRR = (1u << COMMON_TRIG_B); // ensure low

    // Echo pins PB1, PB9, PB12, PB13: inputs with pull-down, EXTI both edges
    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t pin = echo_pin[i];  // PBx
        // input mode
        GPIOB->MODER &= ~(3u << (pin * 2));
        // pull-down
        GPIOB->PUPDR &= ~(3u << (pin * 2));
        GPIOB->PUPDR |=  (2u << (pin * 2)); // pull-down

        // map EXTI line to PB via SYSCFG
        uint8_t reg = pin >> 2;           // EXTICR index 0..3
        uint8_t pos = (pin & 3u) * 4u;
        SYSCFG->EXTICR[reg] &= ~(0xFu << pos);
        SYSCFG->EXTICR[reg] |=  (1u << pos); // 1 = port B
        // unmask and both edges
        EXTI->IMR  |= (1u << pin);
        EXTI->RTSR |= (1u << pin);
        EXTI->FTSR |= (1u << pin);
    }

    // Enable EXTI IRQs used
    NVIC->ISER[0] |= (1 << EXTI0_1_IRQn);   // handles EXTI0 & EXTI1 (PB1 -> EXTI1)
    NVIC->ISER[0] |= (1 << EXTI4_15_IRQn);  // handles EXTI4..15 (encoders + PB9 PB12 PB13)
}
void start_hcsr04_trigger(void)
{
	// Ensure both TRIG pins start low
	GPIOC->BRR = (1u << COMMON_TRIG_C);
	GPIOB->BRR = (1u << COMMON_TRIG_B);

	// Set both TRIG pins high
	GPIOC->BSRR = (1u << COMMON_TRIG_C);
	GPIOB->BSRR = (1u << COMMON_TRIG_B);

    uint16_t now = (uint16_t)TIM3->CNT;
    uint16_t target = (uint16_t)(now + 10u); // 10 microseconds later
    TIM3->CCR1 = target;
    TIM3->DIER |= TIM_DIER_CC1IE; // enable CC1 interrupt

	currentSensor = (currentSensor + 1u) & 3u;
	ultrasonic_read_delay_flag = 0;
}
void EXTI0_1_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;

    //Sensor 0 is PB1
    if (pr & (1u << 1)) {
        EXTI->PR = (1u << 1); // clear pending bit

        if (GPIOB->IDR & (1u << 1)) {
            // Rising edge
            echoStart[0] = (uint16_t)TIM3->CNT;
        } else {
            // Falling edge
            if (echoStart[0] != 0) {
                echoEnd[0] = (uint16_t)TIM3->CNT;

                uint16_t s = echoStart[0];
                uint16_t e = echoEnd[0];
                uint16_t dur = (e >= s) ? (e - s) : (uint16_t)(0xFFFFu - s + e);

                float d = ((float)dur * 0.0343f) * 0.5f;
                if (d < 400.0f){  // sanity check for overflow/invalid echo
                    distance_cm[0] = d;
                    noObjectCounter[0] = 0;   // reset counter
                }
                else {                  // out of range
                	noObjectCounter[0]++;
					if (noObjectCounter[0] >= 4) {
						distance_cm[0] = 999;  // now consider no object
						noObjectCounter[0] = 4;  // cap counter
					}
                }
                echoStart[0] = 0;  // reset start
            }
        }
    }
}


// EXTI handler (EXTI lines 4..15)
void EXTI4_15_IRQHandler(void) {
    uint32_t pr = EXTI->PR;

    if (pr & (1 << 4)) {
        EXTI->PR = (1 << 4);   // clear pending
        update_encoder(1);     // encoder 1 (PB4/PB5)
    }
    if (pr & (1 << 6)) {
        EXTI->PR = (1 << 6);   // clear pending
        update_encoder(2);     // encoder 2 (PA6/PA7)
    }
    // Ultrasonic echoes:
	// PB9 -> sensor index 1 (EXTI9)
	// --- Sensor 1: PB9 ---
	if (pr & (1u << 9)) {
		EXTI->PR = (1u << 9);
		if (GPIOB->IDR & (1u << 9)) {
			echoStart[1] = (uint16_t)TIM3->CNT;
		} else if (echoStart[1] != 0) {
			echoEnd[1] = (uint16_t)TIM3->CNT;
			uint16_t s = echoStart[1], e = echoEnd[1];
			uint16_t dur = (e >= s) ? (e - s) : (uint16_t)(0xFFFFu - s + e);
			float d = ((float)dur * 0.0343f) * 0.5f;
			if (d < 400.0f){  // sanity check for overflow/invalid echo
				distance_cm[1] = d;
				noObjectCounter[1] = 0;   // reset counter
			}
			else {                  // out of range
				noObjectCounter[1]++;
				if (noObjectCounter[1] >= 4) {
					distance_cm[1] = 999;  // now consider no object
					noObjectCounter[1] = 4;  // cap counter
				}
			}
			echoStart[1] = 0;
		}
	}

	// --- Sensor 2: PB12 ---
	if (pr & (1u << 12)) {
		EXTI->PR = (1u << 12);
		if (GPIOB->IDR & (1u << 12)) {
			echoStart[2] = (uint16_t)TIM3->CNT;
		} else if (echoStart[2] != 0) {
			echoEnd[2] = (uint16_t)TIM3->CNT;
			uint16_t s = echoStart[2], e = echoEnd[2];
			uint16_t dur = (e >= s) ? (e - s) : (uint16_t)(0xFFFFu - s + e);
			float d = ((float)dur * 0.0343f) * 0.5f;
			if (d < 400.0f){  // sanity check for overflow/invalid echo
				distance_cm[2] = d;
				noObjectCounter[2] = 0;   // reset counter
			}
			else {                  // out of range
				noObjectCounter[2]++;
				if (noObjectCounter[2] >= 4) {
					distance_cm[2] = 999;  // now consider no object
					noObjectCounter[2] = 4;  // cap counter
				}
			}
			echoStart[2] = 0;
		}
	}

	// --- Sensor 3: PB13 ---
	if (pr & (1u << 13)) {
		EXTI->PR = (1u << 13);
		if (GPIOB->IDR & (1u << 13)) {
			echoStart[3] = (uint16_t)TIM3->CNT;
		} else if (echoStart[3] != 0) {
			echoEnd[3] = (uint16_t)TIM3->CNT;
			uint16_t s = echoStart[3], e = echoEnd[3];
			uint16_t dur = (e >= s) ? (e - s) : (uint16_t)(0xFFFFu - s + e);
			float d = ((float)dur * 0.0343f) * 0.5f;
			if (d < 400.0f){  // sanity check for overflow/invalid echo
				distance_cm[3] = d;
				noObjectCounter[3] = 0;   // reset counter
			}
			else {                  // out of range
				noObjectCounter[3]++;
				if (noObjectCounter[3] >= 4) {
					distance_cm[3] = 999;  // now consider no object
					noObjectCounter[3] = 4;  // cap counter
				}
			}
			echoStart[3] = 0;
		}
	}
}



