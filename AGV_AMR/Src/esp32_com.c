////////////////////////////////////
////// ESP32 USART COM SOURCE //////
////////////////////////////////////
/*
 INPUT: LED_ACTION
 OUTPUT: TMP_DATA

 PENDING: ARRAY SYSTEM AND ACKNOWLEDGE
 */
#include "esp32_com.h"

volatile uint8_t LED_action = 0;  // Settear comando (byte recibido por ESP32)
volatile uint8_t debug = 0;

volatile uint16_t temperature10 = 0;
volatile uint16_t voltage10 = 0;
volatile uint16_t valid_indexes = 0;

volatile uint8_t rxBuf[16];
volatile uint8_t rxIndex = 0;
volatile uint8_t packetLen = 0;

int16_t pathX[MAX_POINTS];
int16_t pathY[MAX_POINTS];
volatile uint16_t pathCount = 0;        // How many points have been stored
volatile bool pathReady = false;        // Set true when ESP finishes sending

// ---------- USART1 init on PA9 (TX) PA10 (RX) - AF1 for USART1 on STM32F0 ----------
void USART1_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // Enable GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1

    // PA9 (TX), PA10 (RX) as AF1
    GPIOA->MODER &= ~((3 << (9*2)) | (3 << (10*2)));
    GPIOA->MODER |=  ((2 << (9*2)) | (2 << (10*2)));
    GPIOA->AFR[1] &= ~((0xF << ((9-8)*4)) | (0xF << ((10-8)*4)));
    GPIOA->AFR[1] |=  ((1 << ((9-8)*4)) | (1 << ((10-8)*4)));

    // Pull-ups
    GPIOA->PUPDR &= ~((3 << (9*2)) | (3 << (10*2)));
    GPIOA->PUPDR |=  ((1 << (9*2)) | (1 << (10*2)));

    // USART config
    USART1->BRR = (uint32_t)(8000000 / 115200); // 8 MHz clock
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
}


// Blocking transmit helper
void USART1_send(uint8_t b)
{
    // Wait for TXE flag
	if (USART1->ISR & USART_ISR_TXE) {
	        USART1->TDR = b;
	    }
}

void USART1_IRQHandler(void)
{
    if (USART1->ISR & USART_ISR_RXNE)
    {
        uint8_t data = USART1->RDR;

        // 1. Looking for SYNC
        if (rxIndex == 0) {
            if (data == 0xAA) {
                rxBuf[0] = data;
                rxIndex = 1;
            }
            return;
        }

        rxBuf[rxIndex++] = data;

        // 2. CMD position = 1 (no check needed)
        // 3. LEN position = 2 → read how many payload bytes to expect
        if (rxIndex == 3) {
            packetLen = rxBuf[2];
        }

        // 4. End when complete packet is received
        if (rxIndex >= packetLen + 3)
        {
            uint8_t cmd = rxBuf[1];

            if (cmd == 0x01) {
            	if(pathReady == true){
            		resetPath();
            	}
                // POINT packet
                int16_t X = (rxBuf[3] << 8) | rxBuf[4];
                int16_t Y = (rxBuf[5] << 8) | rxBuf[6];
                processPoint(X, Y);
                LED_action = 20; // in progress
            }
            else if (cmd == 0x02) {
                // TEMP/VOLT packet
                int16_t T = (rxBuf[3] << 8) | rxBuf[4];
                int16_t V = (rxBuf[5] << 8) | rxBuf[6];
                temperature10 = T;
                voltage10 = V;
            }
            else if (cmd == 0x03) {
                // END OF POINT STREAM
                int16_t valid = (rxBuf[3] << 8) | rxBuf[4];
                valid_indexes = valid;
                LED_action = 21; // finished
                pathReady = true;
            }

            rxIndex = 0;
        }
    }
}

void processPoint(int16_t x, int16_t y){
    if (pathCount < MAX_POINTS)
    {
        pathX[pathCount] = x;
        pathY[pathCount] = y;
        pathCount++;
    }
    else
    {
        // Overflow → clamp and mark ready
        pathReady = true;
    }
}

void resetPath(void){
    for (int i = 0; i < MAX_POINTS; i++) {
        pathX[i] = 0;
        pathY[i] = 0;
    }
    pathCount = 0;
    pathReady = false;
}


