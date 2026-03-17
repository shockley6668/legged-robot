#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// Frequencies for musical notes (approximate)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494

#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

#define NOTE_C6 1046
#define NOTE_D6 1175
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_G6 1568
#define NOTE_A6 1760
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_D7 2349
#define NOTE_E7 2637

void Buzzer_Set_Freq(uint16_t freq);
void Buzzer_Stop(void);
void Buzzer_Play_Startup_Music(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_BUZZER_H
