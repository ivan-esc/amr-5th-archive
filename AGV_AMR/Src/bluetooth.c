////////////////////////////////////
////// BLUETOOTH HC-06 SOURCE //////
////////////////////////////////////
/*
 INPUT: USART2_IRQ_HANDLER AND TO-SEND VARIABLES
 OUTPUT: READ BYTES
 */

#include "bluetooth.h"
#include "esp32_com.h"

#define BAUD_RATE 38400; //CONFIGURACION DE MODULO

volatile int16_t raw_x = 125;  // recieved coordinate
volatile int16_t raw_y = 125; // recieved coordinate
volatile uint8_t menu_button = 0; //set mode
volatile uint8_t auto_set_speed = 50; //line follow set speed
volatile uint8_t coded_recieved_bool1 = 0; //%2 = user/time %3 = go command
volatile uint8_t station_wait_time = 50;
volatile uint8_t coded_route_recieved = 0;

volatile uint8_t rx_buffer[8];  // Buffer for 5-byte packet
volatile uint8_t rx_index = 0;  // Track current byte position
volatile uint8_t first_connection = 0;  // 0 = not connected yet, 1 = already connected


//////////////////////////////////////
///// HC 05 SENT TO APP VARIABLES ////
//////////////////////////////////////
volatile uint8_t linear_velocity = 0;
volatile uint8_t angular_velocity = 0;

volatile uint8_t tmp_data = 0;
volatile uint8_t near_obj = 0;
volatile uint8_t colorin = 3;
volatile uint8_t voltage_sent = 0;

volatile uint8_t posx_big = 0;
volatile uint8_t posx_small = 0;
volatile uint8_t posy_big = 0;
volatile uint8_t posy_small = 0;


void USART2_Init(void) //Bluetooth Setup
{
	// ENABLE A PINS...
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// PA2 (TX) and PA3 (RX) to alternate function
	GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));   // Clear
	GPIOA->MODER |=  (2 << (2 * 2)) | (2 << (3 * 2));     // AF mode
	GPIOA->AFR[0] |= (1 << (4 * 2)) | (1 << (4 * 3));     // AF1 (USART2)
	GPIOA->PUPDR |= (1 << (2*2)) | (1 << (3*2)); // Pull-up

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->BRR = 8000000 / BAUD_RATE; // 8 MHz clock, 38400 baud → BRR = 208
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE; // Enable TX, RX, USART
    NVIC_EnableIRQ(USART2_IRQn); // Enable USART2 interrupt in NVIC
}

void USART2_IRQHandler(void) // Bluetooth RX
{
    if (USART2->ISR & USART_ISR_RXNE)
    {
        uint8_t data = USART2->RDR;

        // --- 1. Start-of-packet sync check ---
        if (rx_index == 0 && data != 255)
        {
            // If first byte isn’t the expected header (255),
            // flush and resync.
            USART2->RQR |= USART_RQR_RXFRQ; // clear RX flag
            volatile uint8_t dummy = USART2->RDR; // read once to fully flush
            rx_index = 0;
            return;
        }

        // --- 2. Store byte ---
        rx_buffer[rx_index++] = data;

        // --- 3. Full packet received? ---
        if (rx_index == 8)
        {
            raw_x  = rx_buffer[1];
            raw_y  = rx_buffer[2];
           	menu_button = rx_buffer[3];

            coded_route_recieved = rx_buffer[4];
            coded_recieved_bool1 = rx_buffer[5];
            station_wait_time = rx_buffer[6];
            auto_set_speed = rx_buffer[7];

            // Reset for next packet
            rx_index = 0;

            // --- 4. Detect first connection packet ---
            if (first_connection == 0 && rx_buffer[0] == 255)
            {
                LED_action = 1;
                first_connection = 1;
            }
        }
    }
}



void USART2_send(uint8_t b)
{
    while (!(USART2->ISR & USART_ISR_TXE));  // Wait until TX buffer empty
    USART2->TDR = b;
}

void BT_SendTelemetry(void)
{
    // Sent Packet
	uint8_t packet[11];
	packet[0] = 0xAA;          // header
	packet[1] = (uint8_t)linear_velocity;
	packet[2] = (uint8_t)angular_velocity;
	packet[3] = (uint8_t)tmp_data;
	packet[4] = (uint8_t)near_obj;
	packet[5] = (uint8_t)colorin;
	packet[6] = (uint8_t)voltage_sent;
	packet[7] = (uint8_t)posx_big;
	packet[8] = (uint8_t)posx_small;
	packet[9] = (uint8_t)posy_big;
	packet[10] = (uint8_t)posy_small;

    for (int i = 0; i < 11; i++){
        USART2_send(packet[i]);
    }
    while (!(USART2->ISR & USART_ISR_TC));
}

