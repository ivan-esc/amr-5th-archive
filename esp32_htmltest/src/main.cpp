#include <WiFi.h>
#include <WebSocketsServer.h>

#include "analogReader.h"
#include "songs.h"
#include "led_control.h"

unsigned long last5s = 0;
int buzzer_setup = 0;

int life = 0;
int deck = 0;
int wish = 0;
int santa = 0;

const char* ssid = "LAPTOPESP32";
const char* password = "qwop1290";

WebSocketsServer webSocket = WebSocketsServer(81);

bool receiving = false;
int16_t pointsX[500];   // adjust if needed
int16_t pointsY[500];
int16_t countPoints = 0;

#define RXD1 16
#define TXD1 17
// current command (received via Serial1)

enum SendState {
    IDLE,
    SENDING_POINTS
};
SendState sendState = IDLE;
int sendIndex = 0;


void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t * payload, size_t length) {

    if (type == WStype_TEXT) {
        if (strcmp((char*)payload, "START") == 0) {
            receiving = true;
            countPoints = 0;
            play = 0;
            life = 1;
            Serial.println("Receiving");
            return;
        }

        if (strcmp((char*)payload, "END") == 0) {
            receiving = false;
            Serial.println("All bytes sorted");
            sendState = SENDING_POINTS;   // <<< START SENDING TO STM32
            sendIndex = 0;                // <<< Reset index

 /*           Serial.print("[");
            for (int i = 0; i < countPoints; i++) {
                Serial.print("(");
                Serial.print(pointsX[i]);
                Serial.print(", ");
                Serial.print(pointsY[i]);
                Serial.print(")");
                if (i < countPoints - 1) Serial.print(", ");
            }
            Serial.println("]");*/
            return;
        }
    }

    if (type == WStype_BIN && receiving) {
        // Expect exactly 4 bytes: X_hi, X_lo, Y_hi, Y_lo
        if (length == 4) {
            int16_t x = (int16_t)((payload[0] << 8) | payload[1]);
            int16_t y = (int16_t)((payload[2] << 8) | payload[3]);

            pointsX[countPoints] = x;
            pointsY[countPoints] = y;
            countPoints++;
        }
    }
}
void sendPointPacket(int16_t x, int16_t y) {
    uint8_t buf[7];

    buf[0] = 0xAA;       // SYNC
    buf[1] = 0x01;       // CMD = point
    buf[2] = 4;          // LEN = 4 bytes payload

    buf[3] = (x >> 8) & 0xFF;
    buf[4] = x & 0xFF;
    buf[5] = (y >> 8) & 0xFF;
    buf[6] = y & 0xFF;

    Serial1.write(buf, 7);
}
void sendEndPacket() {
    uint8_t buf[5];

    buf[0] = 0xAA;       // SYNC
    buf[1] = 0x03;       // CMD = point
    buf[2] = 2;          // LEN = 2 bytes payload

    buf[3] = (countPoints >> 8) & 0xFF;
    buf[4] = countPoints & 0xFF;

    Serial1.write(buf, 5);
}
void sendTempVoltPacket(int16_t temp, int16_t volt) {
    uint8_t buf[7];

    buf[0] = 0xAA;
    buf[1] = 0x02;  // TEMP/VOLT
    buf[2] = 4;

    buf[3] = (temp >> 8) & 0xFF;
    buf[4] = temp & 0xFF;
    buf[5] = (volt >> 8) & 0xFF;
    buf[6] = volt & 0xFF;

    Serial1.write(buf, 7);
}
void checkForACK() {
    while (Serial1.available()) {
        uint8_t v = Serial1.read();
        currentCommand = v;

        if (currentCommand != lastCommand) {
            stripAck();
            lastCommand = currentCommand;
        }
    }
}


void setup() {
    Serial.begin(115200);

    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.println(WiFi.localIP());

    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    ADCsetup();
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    buzzer_setup = 1;
}

void RandomChristmas(){
    int thing = 0;
    commandChristmasMode();
    thing = random(3);
    switch(thing){
        case 1: wish = 1;
        case 2: santa = 1;
         case 3: deck = 1;
    }
    play = 0;
}

void loop() {
    if(buzzer_setup){
        setupTone();
        stopTone();
        stripINIT();
        buzzer_setup=0;
    }

    webSocket.loop();
    nowMillis = millis();
    checkForACK();
    if (millis() - last5s >= 5000) {
        last5s = millis();
        ADCread_handler();
        if (sendState == IDLE){
            sendTempVoltPacket(highest_temp10, voltage10);
        }
    }

    // STATE MACHINE: send points
    if (sendState == SENDING_POINTS) {
        if (sendIndex < countPoints) {
            sendPointPacket(pointsX[sendIndex], pointsY[sendIndex]);
            sendIndex++;
            delay(3); // spacing (safe)
        } 
        else {
            sendEndPacket();
            Serial.println("Sent END packet");
            sendState = IDLE;
        }
    }
    switch (currentCommand) {
            case 1:  commandFirstConnectionBT(); break;
            case 2:  commandManual_idle();       break;
            case 3:  commandManual_forward();    break;
            case 4:  commandManual_backwards();  break;
            case 5:  commandManual_left();       break;
            case 6:  commandManual_right();      break;
            case 7:  commandLine_greenIdle();    break;
            case 8:  commandLine_redIdle();      break;
            case 9:  commandLine_blueIdle();     break;
            case 10: commandLine_following();    break;
            case 11: commandObjectNearby();      break;
            case 12: commandSettingsWindow();    break;
            case 13: commandEmergencyStop();     break;
            case 14: commandAllOff();            break;
            case 0:  commandAllOff();            break;
            case 15: commandRGBSlider();         break;
            case 19: commandAMR_Idle();          break;
            case 20: commandAMR_Receiving();     break;
            case 21: commandAMR_RouteLoaded();   break;
            case 22: commandAMR_Running();       break;
            case 30: RandomChristmas();         break;
            default: break;
    }
    if(!play){
        if(life){
            playMelody(LIFE_HIGHWAY_MELODY, LIFE_HIGHWAY_DURATIONS, LIFE_HIGHWAY_SIZE, 0);
            life = 0;
        }
        if(deck){
            playMelody(DECK_MELODY, DECK_DURATIONS, DECK_SIZE, 0);
            deck = 0;
        } 
        if(wish){
            playMelody(WE_WISH_MELODY, WE_WISH_DURATIONS, WE_WISH_SIZE, 0);
            wish = 0;
        } 
        if(santa){
            playMelody(SANTA_MELODY, SANTA_DURATIONS, SANTA_SIZE, 0);
            santa = 0;
        }                     
        play = 1;
    }
    updatePlayback();
}