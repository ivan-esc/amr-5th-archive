//////////////////////////////
////// ENCODERS  SOURCE //////
//////////////////////////////
/*
 INPUT: EXTI-PULSES AND INIT
 OUTPUT: RPM-FILTERED
 */

#include "encoders.h"

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////      ENCODERS            ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

volatile int32_t encoderPos1 = 0, encoderPos2 = 0; //Absolute Position based on start up
volatile uint8_t direction1 = 0, direction2 = 0; // 1 cockwise, 2 counter cockwise
volatile float delta_Pos1 = 0.0f, delta_Pos2 = 0.0f; //Diferencial cada 10ms
volatile float lastPos1 = 0.0f, lastPos2 = 0.0f; //Referencia encoder pos para delta

#define Aa 0.0951625819640404f   // coefficient for input (transfer funct)
#define Bb -0.904837418035960f   // coefficient for previous output (transfer funct)
float rpm_1_filtered = 0.0f;    // GOOD PWM 1 = left
float rpm_2_filtered = 0.0f;	// GOOD PWM 2 = right
float prev_rpm_1 = 0.0f; //Referencia rpm para transfer function
float prev_rpm_2 = 0.0f; //Referencia rpm para transfer function

// --- call this once at startup (after clocks/GPIO init) ---
void init_GPIO_EXTI_encoders(void) {
    // Ensure GPIOA / GPIOB clocks enabled
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Enable SYSCFG clock (needed to route EXTI lines)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure inputs with pull-ups:
    // Encoder1: PB4 (A), PB5 (B)
    GPIOB->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->PUPDR &= ~((3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->PUPDR |=  ((1 << (4 * 2)) | (1 << (5 * 2))); // pull-up

    // Encoder2: PA6 (A), PA7 (B)
    GPIOA->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2))); // pull-up

    // Map EXTI lines:
    // SYSCFG->EXTICR[1] contains EXTI4..7 mapping (4 fields x 4 bits)
    // clear EXTI4..7 nibble fields first
    SYSCFG->EXTICR[1] &= ~((0xF << (0*4)) | (0xF << (1*4)) | (0xF << (2*4)) | (0xF << (3*4)));

    // EXTI4 -> PB  (value 1 for port B in EXTICR field)
    SYSCFG->EXTICR[1] |= (1 << (0*4));   // EXTI4 = PB
    // EXTI6 -> PA  (value 0 for port A; clearing is enough; left explicit)
    // (we already cleared, so EXTI6 = PA)

    // Unmask EXTI lines for the encoder A pins and enable rising edge trigger
    EXTI->IMR |= (1 << 4) | (1 << 6);    // EXTI4 (encoder1 A), EXTI6 (encoder2 A)
    EXTI->RTSR |= (1 << 4) | (1 << 6);   // rising edge only (keep same behavior as before)
    // Optionally enable falling edges too:
    // EXTI->FTSR |= (1<<4) | (1<<6);

    // NVIC: EXTI4..15 IRQ covers EXTI4..15
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void update_encoder(uint8_t encoder) {
    uint8_t B;
    if (encoder == 1) {
        B = (GPIOB->IDR >> 5) & 1;
    } else {
        B = (GPIOA->IDR >> 7) & 1;
    }

    if (B == 0) {
        if (encoder == 1) { encoderPos1--; direction1 = 2; }
        else { encoderPos2++; direction2 = 1; }
    } else {
        if (encoder == 1) { encoderPos1++; direction1 = 1; }
        else { encoderPos2--; direction2 = 2; }
    }
}

void velocidad_encoder(void){
    // --- Calculate raw RPM from encoder ---
    delta_Pos1 = encoderPos1 - lastPos1;
    float rpm_1_raw = (delta_Pos1 * 60.0f) / 5.0f;

    delta_Pos2 = encoderPos2 - lastPos2;
    float rpm_2_raw = (delta_Pos2 * 60.0f) / 5.0f;

    lastPos1 = encoderPos1;
    lastPos2 = encoderPos2;

    // --- Apply digital filter ---
    rpm_1_filtered = Aa * rpm_1_raw - Bb * prev_rpm_1;
    rpm_2_filtered = Aa * rpm_2_raw - Bb * prev_rpm_2;

    // --- Update states for next iteration ---
    prev_rpm_1 = rpm_1_filtered;
    prev_rpm_2 = rpm_2_filtered;
}

