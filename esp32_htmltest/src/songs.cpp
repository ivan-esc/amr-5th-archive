#include <Arduino.h>
#include "driver/ledc.h"

#include "songs.h"

#define SPEAKER_PIN 25

// ====== SETUP ======
//    setupTone();
/*
void loop(){
    if(!play){
      playMelody(LIFE_HIGHWAY_MELODY, LIFE_HIGHWAY_DURATIONS, LIFE_HIGHWAY_SIZE, 1);
      playMelody(DECK_MELODY, DECK_DURATIONS, DECK_SIZE, 1);
      playMelody(WE_WISH_MELODY, WE_WISH_DURATIONS, WE_WISH_SIZE, 1);
      playMelody(SANTA_MELODY, SANTA_DURATIONS, SANTA_SIZE, 1);
      play = 1;

stopPlayback() - stops completely
pausePlayback() - pauses at current note
resumePlayback() - resumes from paused position
playTestMelody() - plays the default C-D-E-C melody

    }
    updatePlayback();
}*/
int play = 1;


// ====== NOTE FREQUENCIES ======
const int NOTES[] = {
    131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247,
    262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
    523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988
};

// ====== DEFAULT MELODY ======
const Note DEFAULT_MELODY[] = {N_C4, N_D4, N_E4, N_C4};
const int DEFAULT_DURATIONS[] = {500, 500, 500, 500};
const int MELODY_SIZE = 4;

// ====== PLAYBACK VARIABLES ======
const Note* currentMelody = 0;
const int* currentDurations = 0;
int currentMelodySize = 0;
int currentNoteIndex = 0;
unsigned long noteStartTime = 0;
bool isPlaying = false;
bool loopMelody = true;
const int NOTE_GAP = 10;
unsigned long gapStartTime = 0;
bool inGap = false;

// ====== LEDC SETUP ======
void setupTone() {
    Serial.println("Setting up LEDC on pin 25");
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t channel_conf = {
        .gpio_num = SPEAKER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 64,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

void playTone(int frequency) {
    if (frequency > 0) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 64);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        stopTone();
    }
}

void stopTone() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void playNote(Note note) {
    if (note == N_REST) {
        stopTone();
        return;
    }
    
    if (note >= 0 && note < sizeof(NOTES) / sizeof(NOTES[0])) {
        playTone(NOTES[note]);
    }
}

// ====== MAIN MELODY PLAYER FUNCTION ======
void playMelody(const Note* melody, const int* durations, int size, int loop) {
    // Stop playback if all parameters are 0
    if (melody == 0 && durations == 0 && size == 0) {
        stopPlayback();
        return;
    }
    
    // If null pointers, don't change anything, just update loop setting
    if (melody == 0 || durations == 0) {
        loopMelody = (loop != 0);
        return;
    }
    
    // Set new melody
    currentMelody = melody;
    currentDurations = durations;
    currentMelodySize = size;
    loopMelody = (loop != 0);
    
    // Start playback
    currentNoteIndex = 0;
    noteStartTime = millis();
    isPlaying = true;
    inGap = false;
    
    if (currentMelody[0] != N_REST) {
        playNote(currentMelody[0]);
    } else {
        stopTone();
    }
}

void stopPlayback() {
    isPlaying = false;
    inGap = false;
    stopTone();
    currentNoteIndex = 0;
}

void pausePlayback() {
    isPlaying = false;
    stopTone();
}

void resumePlayback() {
    if (currentMelodySize > 0 && currentNoteIndex < currentMelodySize) {
        isPlaying = true;
        noteStartTime = millis();
        inGap = false;
        if (currentMelody[currentNoteIndex] != N_REST) {
            playNote(currentMelody[currentNoteIndex]);
        }
    }
}

void playTestMelody() {
    playMelody(DEFAULT_MELODY, DEFAULT_DURATIONS, MELODY_SIZE, 1);
}

// ====== PLAYBACK MANAGER ======
void updatePlayback() {
    if (!isPlaying || currentMelody == 0) return;
    
    unsigned long currentTime = millis();
    
    if (inGap) {
        // We're in a gap between notes
        if (currentTime - gapStartTime >= NOTE_GAP) {
            inGap = false;
            
            // Check if we've reached the end of the melody
            if (currentNoteIndex >= currentMelodySize) {
                if (loopMelody) {
                    currentNoteIndex = 0;  // Reset to start
                } else {
                    stopPlayback();
                    return;
                }
            }
            
            // Play the next note
            if (currentMelody[currentNoteIndex] != N_REST) {
                playNote(currentMelody[currentNoteIndex]);
            } else {
                stopTone();
            }
            noteStartTime = currentTime;
        }
    } else {
        // We're playing a note
        unsigned long noteDuration = currentDurations[currentNoteIndex];
        
        if (currentTime - noteStartTime >= noteDuration) {
            // Note finished, start gap
            stopTone();
            currentNoteIndex++;  // THIS WAS MISSING!
            inGap = true;
            gapStartTime = currentTime;
        }
    }
}