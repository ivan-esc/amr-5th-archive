#ifndef SONGS_H
#define SONGS_H

#include <Arduino.h>
#include "driver/ledc.h"

extern int play;
// ====== NOTE ENUMS ======
enum Note {
    N_C3 = 0,  N_C3s, N_D3, N_D3s, N_E3, N_F3, N_F3s, N_G3, N_G3s, N_A3, N_A3s, N_B3,
    N_C4, N_C4s, N_D4, N_D4s, N_E4, N_F4, N_F4s, N_G4, N_G4s, N_A4, N_A4s, N_B4,
    N_C5, N_C5s, N_D5, N_D5s, N_E5, N_F5, N_F5s, N_G5, N_G5s, N_A5, N_A5s, N_B5,
    N_REST = -1
};

void setupTone();

void playMelody(const Note* melody, const int* durations, int size, int loop);
void updatePlayback();

void stopTone();
void stopPlayback();
void pausePlayback();
void resumePlayback();
void playTestMelody();


const Note LIFE_HIGHWAY_MELODY[] = {N_A4, N_A4, N_A4, N_A4, N_REST, N_F4, N_REST, N_REST, N_A4, N_A4, N_A4, N_G4, N_F4, N_A3, N_A3s, N_C4, N_REST, N_REST, N_REST};
const int LIFE_HIGHWAY_DURATIONS[] = {291, 291, 291, 291, 291, 291, 582, 291, 291, 291, 291, 291, 291, 582, 582, 582, 291, 582, 2328};
const int LIFE_HIGHWAY_SIZE = 19;

// Deck the Halls - Adaptado a 130 BPM
const Note DECK_MELODY[] = {
    N_G4, N_F4, N_E4, N_D4, N_C4, N_D4, N_E4, N_C4, N_D4, N_E4, N_F4, N_D4, N_E4, N_D4, N_C4, N_B3, N_C4,
    N_G4, N_F4, N_E4, N_D4, N_C4, N_D4, N_E4, N_C4, N_D4, N_E4, N_F4, N_D4, N_E4, N_D4, N_C4, N_B3, N_C4,
    N_D4, N_E4, N_F4, N_D4, N_E4, N_F4, N_G4, N_D4, N_E3, N_F3s, N_G3, N_A3, N_B3, N_C4, N_B3, N_A3, N_G3};

const int DECK_DURATIONS[] = {
    // Línea 1 (valores: -4, 8, 4, 4, 4, 4, 4, 4, 8, 8, 8, 8, -4, 8, 4, 4, 2)
    693, 231, 462, 462, 462, 462, 462, 462, 231, 231, 231, 231, 693, 231, 462, 462, 923,
    // Línea 2 (repite los mismos valores)
    693, 231, 462, 462, 462, 462, 462, 462, 231, 231, 231, 231, 693, 231, 462, 462, 923,
    // Línea 3 (valores: -4, 8, 4, 4, -4, 8, 4, 4, 8, 8, 4, 8, 8, 4, 4, 4, 2)
    693, 231, 462, 462, 693, 231, 462, 462, 231, 231, 462, 231, 231, 462, 462, 462, 923};

const int DECK_SIZE = 51; // Número total de notas

const Note WE_WISH_MELODY[] = {
    N_C5, N_F5, N_F5, N_G5, N_F5, N_E5, N_D5, N_D5, N_D5, N_G5, N_G5, N_A5, N_G5, N_F5, N_E5, N_C5, N_C5, 
    N_A5, N_A5, N_A5s, N_A5, N_G5, N_F5, N_D5, N_C5, N_C5, N_D5, N_G5, N_E5, N_F5, N_C5,
    // Segunda mitad (repite con final diferente)
    N_F5, N_F5, N_G5, N_F5, N_E5, N_D5, N_D5, N_D5, N_G5, N_G5, N_A5, N_G5, N_F5, N_E5, N_C5, N_C5,
    N_A5, N_A5, N_A5s, N_A5, N_G5, N_F5, N_D5, N_C5, N_C5, N_D5, N_G5, N_E5, N_F5};
const int WE_WISH_DURATIONS[] = {
    // Valores: 4, 4,8,8,8,8, 4,4,4, 4,8,8,8,8, 4,4,4, 4,8,8,8,8, 4,4,8,8, 4,4,4, 2,4
    462, 462,231,231,231,231, 462,462,462, 462,231,231,231,231, 462,462,462,
    462,231,231,231,231, 462,462,231,231, 462,462,462, 923,462,
    // Repetición
    462,231,231,231,231, 462,462,462, 462,231,231,231,231, 462,462,462,
    462,231,231,231,231, 462,462,231,231, 462,462,462, 923};
const int WE_WISH_SIZE = 58;

const Note SANTA_MELODY[] = {
    N_G4, N_E4, N_F4, N_G4, N_G4, N_G4, N_A4, N_B4, N_C5, N_C5, N_C5, N_E4, N_F4, N_G4,
    N_G4, N_G4, N_A4, N_G4, N_F4, N_F4, N_E4, N_G4, N_C4, N_E4, N_D4, N_F4, N_B3, N_C4,
    N_G4, N_E4, N_F4, N_G4, N_G4, N_G4, N_A4, N_B4, N_C5, N_C5, N_C5, N_E4, N_F4, N_G4,
    N_G4, N_G4, N_A4, N_G4, N_F4, N_F4, N_E4, N_G4, N_C4, N_E4, N_D4, N_F4, N_D5, N_C5};
const int SANTA_DURATIONS[] = {
    // Valores: 8,8,8,4,4,4, 8,8,4,4,4, 8,8,4,4,4, 8,8,4,2, 4,4,4,4, 4,2,4, -2,4,
    //          8,8,8,4,4,4, 8,8,4,4,4, 8,8,4,4,4, 8,8,4,2, 4,4,4,4, 4,2,4, 1
    231,231,231,462,462,462, 231,231,462,462,462, 231,231,462,462,462,
    231,231,462,923, 462,462,462,462, 462,923,462, 1385,462,
    231,231,231,462,462,462, 231,231,462,462,462, 231,231,462,462,462,
    231,231,462,923, 462,462,462,462, 462,923,462, 1846  /*1 = Redonda (4×462)*/};

const int SANTA_SIZE = 56;



#endif