#include <Arduino.h>
#include <driver/adc.h>
#include "analogReader.h"

volatile int highest_temp10;
volatile int voltage10;

#define ADC_temp1 32
#define ADC_temp2 33
#define ADC_voltage 34

void ADCsetup(){
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
}
void ADCread_handler(){
    int raw_temp1 = analogRead(ADC_temp1);
    int raw_temp2 = analogRead(ADC_temp2);
    int raw_voltage = analogRead(ADC_voltage);

    int raw_t = (raw_temp1 > raw_temp2) ? raw_temp1 : raw_temp2;
    int mV = (raw_t * 330) / 4095;
    highest_temp10 = mV;   // Temp ×10

    int vout10 = (raw_voltage * 33) / 4095; // 3.3V → 33 (×10)
    voltage10 = (vout10 * 14) / 3;
}