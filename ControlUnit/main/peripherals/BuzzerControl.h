#ifndef BUZZERCONTROL_H_
#define BUZZERCONTROL_H_

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "board_pins.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (BUZZER_OUTPUT_PIN) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY ((ledc_timer_bit_t)8191) // Set duty to 100%. ((2 ** 13) - 1) * 100% = 8191   // volume
#define LEDC_DUTY ((ledc_timer_bit_t)4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (5000)              // Frequency in Hertz. Set frequency at 5 kHz
#define ERROR_TUNE_BEEP_INTERVAL_MILLIS 143

const uint32_t notes[] = {
    0,    // NO_NOTE
    31,   // NOTE_A0
    33,   // NOTE_AS0
    35,   // NOTE_B0
    37,   // NOTE_C1
    39,   // NOTE_CS1
    41,   // NOTE_D1
    44,   // NOTE_DS1
    46,   // NOTE_E1
    49,   // NOTE_F1
    52,   // NOTE_FS1
    55,   // NOTE_G1
    58,   // NOTE_GS1
    62,   // NOTE_A1
    65,   // NOTE_AS1
    69,   // NOTE_B1
    73,   // NOTE_C2
    78,   // NOTE_CS2
    82,   // NOTE_D2
    87,   // NOTE_DS2
    93,   // NOTE_E2
    98,   // NOTE_F2
    104,  // NOTE_FS2
    110,  // NOTE_G2
    117,  // NOTE_GS2
    123,  // NOTE_A2
    131,  // NOTE_AS2
    139,  // NOTE_B2
    147,  // NOTE_C3
    156,  // NOTE_CS3
    165,  // NOTE_D3
    175,  // NOTE_DS3
    185,  // NOTE_E3
    196,  // NOTE_F3
    208,  // NOTE_FS3
    220,  // NOTE_G3
    233,  // NOTE_GS3
    247,  // NOTE_A3
    262,  // NOTE_AS3   // Selected
    277,  // NOTE_B3
    294,  // NOTE_C4
    311,  // NOTE_CS4
    330,  // NOTE_D4
    349,  // NOTE_DS4
    370,  // NOTE_E4    // Selected
    392,  // NOTE_F4
    415,  // NOTE_FS4
    440,  // NOTE_G4
    466,  // NOTE_GS4
    494,  // NOTE_A4
    523,  // NOTE_AS4   // Selected
    554,  // NOTE_B4
    587,  // NOTE_C5
    622,  // NOTE_CS5
    660,  // NOTE_D5    // Selected
    698,  // NOTE_DS5
    740,  // NOTE_E5
    784,  // NOTE_F5    // Selected
    831,  // NOTE_FS5
    880,  // NOTE_G5
    932,  // NOTE_GS5
    988,  // NOTE_A5
    1047, // NOTE_AS5   // Selected
    1109, // NOTE_B5
    1175, // NOTE_C6
    1245, // NOTE_CS6
    1319, // NOTE_E6    // Selected
    1397, // NOTE_F6
    1480, // NOTE_FS6
    1568, // NOTE_G6    // Selected
    1661, // NOTE_GS6   // Selected
    1760, // NOTE_A6    // Selected
    1865, // NOTE_AS6   // Selected
    1976, // NOTE_B6    // Selected
    2093, // NOTE_C7    // Selected
    2217, // NOTE_CS7   // Selected
    2349, // NOTE_D7    // Selected
    2489, // NOTE_DS7   // Selected
    2637, // NOTE_E7    // Selected
    2794, // NOTE_F7
    2960, // NOTE_FS7
    3136, // NOTE_G7
    3322, // NOTE_GS7   // Selected
    3520, // NOTE_A7    // Selected
    3729, // NOTE_AS7   // Selected
    3951, // NOTE_B7    // Selected
    4186, // NOTE_C8
};

#define NOTES_COUNT (sizeof(notes) / sizeof(uint32_t))

typedef enum
{
    NO_NOTE = 0,
    NOTE_A0,
    NOTE_AS0,
    NOTE_B0,
    NOTE_C1,
    NOTE_CS1,
    NOTE_D1,
    NOTE_DS1,
    NOTE_E1,
    NOTE_F1,
    NOTE_FS1,
    NOTE_G1,
    NOTE_GS1,
    NOTE_A1,
    NOTE_AS1,
    NOTE_B1,
    NOTE_C2,
    NOTE_CS2,
    NOTE_D2,
    NOTE_DS2,
    NOTE_E2,
    NOTE_F2,
    NOTE_FS2,
    NOTE_G2,
    NOTE_GS2,
    NOTE_A2,
    NOTE_AS2,
    NOTE_B2,
    NOTE_C3,
    NOTE_CS3,
    NOTE_D3,
    NOTE_DS3,
    NOTE_E3,
    NOTE_F3,
    NOTE_FS3,
    NOTE_G3,
    NOTE_GS3,
    NOTE_A3,
    NOTE_AS3,
    NOTE_B3,
    NOTE_C4,
    NOTE_CS4,
    NOTE_D4,
    NOTE_DS4,
    NOTE_E4,
    NOTE_F4,
    NOTE_FS4,
    NOTE_G4,
    NOTE_GS4,
    NOTE_A4,
    NOTE_AS4,
    NOTE_B4,
    NOTE_C5,
    NOTE_CS5,
    NOTE_D5,
    NOTE_DS5,
    NOTE_E5,
    NOTE_F5,
    NOTE_FS5,
    NOTE_G5,
    NOTE_GS5,
    NOTE_A5,
    NOTE_AS5,
    NOTE_B5,
    NOTE_C6,
    NOTE_CS6,
    NOTE_D6,
    NOTE_DS6,
    NOTE_E6,
    NOTE_F6,
    NOTE_FS6,
    NOTE_G6,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C7,
    NOTE_CS7,
    NOTE_D7,
    NOTE_DS7,
    NOTE_E7,
    NOTE_F7,
    NOTE_FS7,
    NOTE_G7,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
    NOTE_C8,
} piano_note_t;

const uint8_t selectedNotesIndices[] = {
    NOTE_F5,
    NOTE_AS5,
    NOTE_E6,
    NOTE_G6,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C7,
    NOTE_CS7,
    NOTE_D7,
    NOTE_DS7,
    NOTE_E7,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
};

class BuzzerControl
{
private:
public:
    void tone(piano_note_t note);
    void noTone();
    piano_note_t numberToPianoNote(int num);
    void playMotorOnTune();
    void playMotorOFFTune();
    void playFailureTune();
    void playSuccessTune();
    void playErrorTone();
};

#endif