#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 4
#define NUM_LEDS 103
#define BRIGHTNESS 110  // max total R+G+B per pixel

extern volatile int currentCommand;
extern int lastCommand;
extern unsigned long nowMillis;

// prototypes
void commandFirstConnectionBT(); // 1
void commandManual_idle();       // 2
void commandManual_forward();    // 3
void commandManual_backwards();  // 4
void commandManual_left();       // 5
void commandManual_right();      // 6
void commandLine_greenIdle();    // 7
void commandLine_redIdle();      // 8
void commandLine_blueIdle();     // 9
void commandLine_following();    //10
void commandObjectNearby();      //11
void commandSettingsWindow();    //12
void commandEmergencyStop();     //13
void commandAllOff();            //14 / 0
void commandRGBSlider();         //15
void commandAMR_Idle();          //19
void commandAMR_Receiving();     //20
void commandAMR_RouteLoaded();   //21
void commandAMR_Running();       //22
void commandChristmasMode();     //30
void clearAll();

void stripAck();
void stripINIT();

#endif