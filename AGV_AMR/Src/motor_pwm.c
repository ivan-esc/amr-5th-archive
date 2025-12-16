///////////////////////////////
////// MOTOR PWM  HEADER //////
///////////////////////////////
/*
 INPUT: INIT
 OUTPUT: MACROS
 */

#include "motor_pwm.h"

void pwm_init(void) {
    // --- Enable clocks ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN; // GPIOA + GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                      // TIM2 clock

    // --- Configure PA0, PA1 (CH1, CH2) ---
    GPIOA->MODER &= ~((0b11 << 0) | (0b11 << 2));
    GPIOA->MODER |=  ((0b10 << 0) | (0b10 << 2));            // AF mode
    GPIOA->OTYPER &= ~((1 << 0) | (1 << 1));
    GPIOA->OSPEEDR |= ((0b11 << 0) | (0b11 << 2));
    GPIOA->AFR[0] &= ~((0xF << (0*4)) | (0xF << (1*4)));
    GPIOA->AFR[0] |=  ((2 << (0*4)) | (2 << (1*4)));         // AF2 = TIM2

    // --- Configure PB10, PB11 (CH3, CH4) ---
    GPIOB->MODER &= ~((0b11 << (10*2)) | (0b11 << (11*2)));
    GPIOB->MODER |=  ((0b10 << (10*2)) | (0b10 << (11*2)));  // AF mode
    GPIOB->OTYPER &= ~((1 << 10) | (1 << 11));
    GPIOB->OSPEEDR |= ((0b11 << (10*2)) | (0b11 << (11*2)));
    GPIOB->AFR[1] &= ~((0xF << ((10-8)*4)) | (0xF << ((11-8)*4)));
    GPIOB->AFR[1] |=  ((2 << ((10-8)*4)) | (2 << ((11-8)*4))); // AF2 = TIM2

    // --- Configure TIM2 (all 4 channels) ---
    TIM2->PSC = 8 - 1;       // 1 MHz timer clock
    TIM2->ARR = 1000 - 1;    // 1 kHz PWM

    // PWM mode 1 on all channels
    TIM2->CCMR1 &= ~((TIM_CCMR1_OC1M) | (TIM_CCMR1_OC2M));
    TIM2->CCMR1 |= ((6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos));

    TIM2->CCMR2 &= ~((TIM_CCMR2_OC3M) | (TIM_CCMR2_OC4M));
    TIM2->CCMR2 |= ((6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos));

    // Enable all 4 outputs
    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    TIM2->CR1 |= TIM_CR1_CEN;  // Enable TIM2
}
