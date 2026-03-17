#include "bsp_buzzer.h"
#include "tim.h"

// Assuming TIM12 is defined in tim.h
// DJI startup tones are roughly standard musical notes rising in pitch.

static uint8_t buzzer_startup_music_active = 0;

void Buzzer_Set_Freq(uint16_t freq) {
  if (!buzzer_startup_music_active) {
    return;
  }

  if (freq == 0) {
    Buzzer_Stop();
    return;
  }

  // Existing config from tim.c:
  // TIM12 uses APB1 clock.
  // If HCLK = 480MHz, APB1 = 120MHz (DIV4), TIMx2 = 240MHz ?
  // Or if HCLK = 240MHz, APB1 = 120MHz (DIV2), TIMx2 = 240MHz.
  // Prescaler was set to 24-1 (23).
  // Timer Tick = 240MHz / 24 = 10 MHz.
  // ARR = (10,000,000 / freq) - 1.

  // Check bounds
  if (freq < 100)
    freq = 100;
  if (freq > 20000)
    freq = 20000;

  uint32_t arr = (10000000 / freq) - 1;

  __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,
                        (arr + 1) / 2); // 50% duty cycle
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Buzzer_Stop(void) { HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2); }

void Buzzer_Play_Startup_Music(void) {
  buzzer_startup_music_active = 1;

  // Sequence requested: C6(1/4) D6(1/4) G6(1/4) P(1/16) D6(1/4) G6(1/4) P(1/16)
  // D6(1/4) G6(1/4) Strict timing implementation. Assumed Tempo: 1/4 note =
  // 120ms => 1/16 note = 30ms

  uint32_t note_quarter = 120;
  uint32_t note_16th = 30;

  // 1. C6 1/4
  Buzzer_Set_Freq(NOTE_C6);
  HAL_Delay(note_quarter);

  // 2. D6 1/4
  Buzzer_Set_Freq(NOTE_D6);
  HAL_Delay(note_quarter);

  // 3. G6 1/4
  Buzzer_Set_Freq(NOTE_G6);
  HAL_Delay(note_quarter);

  // 4. P 1/16 (Pause)
  Buzzer_Stop();
  HAL_Delay(note_16th);

  // 5. D6 1/4
  Buzzer_Set_Freq(NOTE_D6);
  HAL_Delay(note_quarter);

  // 6. G6 1/4
  Buzzer_Set_Freq(NOTE_G6);
  HAL_Delay(note_quarter);

  // 7. P 1/16 (Pause)
  Buzzer_Stop();
  HAL_Delay(note_16th);

  // 8. D6 1/4
  Buzzer_Set_Freq(NOTE_D6);
  HAL_Delay(note_quarter);

  // 9. G6 1/4
  Buzzer_Set_Freq(NOTE_G6);
  HAL_Delay(note_quarter);

  // End
  Buzzer_Stop();
  buzzer_startup_music_active = 0;
}
