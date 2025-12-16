#ifndef ANALOGREADER_H
#define ANALOGREADER_H

#include <Arduino.h>
#include <driver/adc.h>

void ADCsetup();
void ADCread_handler();

extern volatile int voltage10;
extern volatile int highest_temp10;

#endif
