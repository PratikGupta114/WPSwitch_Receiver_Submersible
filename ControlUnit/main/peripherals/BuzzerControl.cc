
#include "BuzzerControl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define TAG "BuzzerControl"

void BuzzerControl::tone(piano_note_t note)
{
    esp_err_t ret;
    
    // Stop any existing tone first to avoid conflicts
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
    
    ledc_timer_config_t ledcTimerConfig;
    memset(&ledcTimerConfig, 0, sizeof(ledcTimerConfig));
    ledcTimerConfig.speed_mode = LEDC_MODE;
    ledcTimerConfig.duty_resolution = LEDC_DUTY_RES;
    ledcTimerConfig.timer_num = LEDC_TIMER;
    ledcTimerConfig.freq_hz = notes[(int)note];
    ledcTimerConfig.clk_cfg = LEDC_AUTO_CLK;

    ret = ledc_timer_config((const ledc_timer_config_t *)&ledcTimerConfig);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ledc timer : %s", esp_err_to_name(ret));
        return;
    }

    ledc_channel_config_t ledcChannelConfig;
    memset(&ledcChannelConfig, 0, sizeof(ledcChannelConfig));
    ledcChannelConfig.gpio_num = BUZZER_OUTPUT_PIN;
    ledcChannelConfig.speed_mode = LEDC_MODE;
    ledcChannelConfig.channel = LEDC_CHANNEL;
    ledcChannelConfig.timer_sel = LEDC_TIMER;
    ledcChannelConfig.duty = LEDC_DUTY; // Set duty to 0%
    ledcChannelConfig.hpoint = 0;

    ret = ledc_channel_config((const ledc_channel_config_t *)&ledcChannelConfig);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ledc channel : %s", esp_err_to_name(ret));
        // If channel config fails, try to reset the GPIO to a safe state
        gpio_reset_pin(BUZZER_OUTPUT_PIN);
        return;
    }
}

void BuzzerControl::noTone()
{
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
}

piano_note_t BuzzerControl::numberToPianoNote(int num)
{
    switch (num)
    {
    case NO_NOTE:
        return NO_NOTE;
    case NOTE_A0:
        return NOTE_A0;
    case NOTE_AS0:
        return NOTE_AS0;
    case NOTE_B0:
        return NOTE_B0;
    case NOTE_C1:
        return NOTE_C1;
    case NOTE_CS1:
        return NOTE_CS1;
    case NOTE_D1:
        return NOTE_D1;
    case NOTE_DS1:
        return NOTE_DS1;
    case NOTE_E1:
        return NOTE_E1;
    case NOTE_F1:
        return NOTE_F1;
    case NOTE_FS1:
        return NOTE_FS1;
    case NOTE_G1:
        return NOTE_G1;
    case NOTE_GS1:
        return NOTE_GS1;
    case NOTE_A1:
        return NOTE_A1;
    case NOTE_AS1:
        return NOTE_AS1;
    case NOTE_B1:
        return NOTE_B1;
    case NOTE_C2:
        return NOTE_C2;
    case NOTE_CS2:
        return NOTE_CS2;
    case NOTE_D2:
        return NOTE_D2;
    case NOTE_DS2:
        return NOTE_DS2;
    case NOTE_E2:
        return NOTE_E2;
    case NOTE_F2:
        return NOTE_F2;
    case NOTE_FS2:
        return NOTE_FS2;
    case NOTE_G2:
        return NOTE_G2;
    case NOTE_GS2:
        return NOTE_GS2;
    case NOTE_A2:
        return NOTE_A2;
    case NOTE_AS2:
        return NOTE_AS2;
    case NOTE_B2:
        return NOTE_B2;
    case NOTE_C3:
        return NOTE_C3;
    case NOTE_CS3:
        return NOTE_CS3;
    case NOTE_D3:
        return NOTE_D3;
    case NOTE_DS3:
        return NOTE_DS3;
    case NOTE_E3:
        return NOTE_E3;
    case NOTE_F3:
        return NOTE_F3;
    case NOTE_FS3:
        return NOTE_FS3;
    case NOTE_G3:
        return NOTE_G3;
    case NOTE_GS3:
        return NOTE_GS3;
    case NOTE_A3:
        return NOTE_A3;
    case NOTE_AS3:
        return NOTE_AS3;
    case NOTE_B3:
        return NOTE_B3;
    case NOTE_C4:
        return NOTE_C4;
    case NOTE_CS4:
        return NOTE_CS4;
    case NOTE_D4:
        return NOTE_D4;
    case NOTE_DS4:
        return NOTE_DS4;
    case NOTE_E4:
        return NOTE_E4;
    case NOTE_F4:
        return NOTE_F4;
    case NOTE_FS4:
        return NOTE_FS4;
    case NOTE_G4:
        return NOTE_G4;
    case NOTE_GS4:
        return NOTE_GS4;
    case NOTE_A4:
        return NOTE_A4;
    case NOTE_AS4:
        return NOTE_AS4;
    case NOTE_B4:
        return NOTE_B4;
    case NOTE_C5:
        return NOTE_C5;
    case NOTE_CS5:
        return NOTE_CS5;
    case NOTE_D5:
        return NOTE_D5;
    case NOTE_DS5:
        return NOTE_DS5;
    case NOTE_E5:
        return NOTE_E5;
    case NOTE_F5:
        return NOTE_F5;
    case NOTE_FS5:
        return NOTE_FS5;
    case NOTE_G5:
        return NOTE_G5;
    case NOTE_GS5:
        return NOTE_GS5;
    case NOTE_A5:
        return NOTE_A5;
    case NOTE_AS5:
        return NOTE_AS5;
    case NOTE_B5:
        return NOTE_B5;
    case NOTE_C6:
        return NOTE_C6;
    case NOTE_CS6:
        return NOTE_CS6;
    case NOTE_D6:
        return NOTE_D6;
    case NOTE_DS6:
        return NOTE_DS6;
    case NOTE_E6:
        return NOTE_E6;
    case NOTE_F6:
        return NOTE_F6;
    case NOTE_FS6:
        return NOTE_FS6;
    case NOTE_G6:
        return NOTE_G6;
    case NOTE_GS6:
        return NOTE_GS6;
    case NOTE_A6:
        return NOTE_A6;
    case NOTE_AS6:
        return NOTE_AS6;
    case NOTE_B6:
        return NOTE_B6;
    case NOTE_C7:
        return NOTE_C7;
    case NOTE_CS7:
        return NOTE_CS7;
    case NOTE_D7:
        return NOTE_D7;
    case NOTE_DS7:
        return NOTE_DS7;
    case NOTE_E7:
        return NOTE_E7;
    case NOTE_F7:
        return NOTE_F7;
    case NOTE_FS7:
        return NOTE_FS7;
    case NOTE_G7:
        return NOTE_G7;
    case NOTE_GS7:
        return NOTE_GS7;
    case NOTE_A7:
        return NOTE_A7;
    case NOTE_AS7:
        return NOTE_AS7;
    case NOTE_B7:
        return NOTE_B7;
    case NOTE_C8:
        return NOTE_C8;
    default:
        return NO_NOTE;
    }
    return NO_NOTE;
}

void BuzzerControl::playMotorOnTune()
{
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));

    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));

    this->tone(NOTE_AS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));
}

void BuzzerControl::playMotorOFFTune()
{
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));

    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));

    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(400));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(250));
}

void BuzzerControl::playFailureTune()
{
    this->tone(NOTE_A6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->tone(NOTE_GS6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->tone(NOTE_G6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(125));
}

void BuzzerControl::playSuccessTune()
{
    this->tone(NOTE_A6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->tone(NOTE_GS6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->tone(NOTE_AS6);
    vTaskDelay(pdMS_TO_TICKS(125));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(125));
}

void BuzzerControl::playErrorTone()
{
    // Pattern: 4 beeps in NOTE_GS7 tone
    // ON(143ms) OFF(143ms) ON(143ms) OFF(143ms) ON(143ms) OFF(143ms) ON(143ms) OFF
    
    // Beep 1
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    
    // Beep 2
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    
    // Beep 3
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    this->noTone();
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    
    // Beep 4 (final beep, no OFF delay after)
    this->tone(NOTE_GS7);
    vTaskDelay(pdMS_TO_TICKS(ERROR_TUNE_BEEP_INTERVAL_MILLIS));
    this->noTone();
}