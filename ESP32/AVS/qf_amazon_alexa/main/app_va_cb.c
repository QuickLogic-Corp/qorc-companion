// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include "stdio.h"
#include <va_led.h>
#include <voice_assistant_app_cb.h>
#include "va_board.h"
#include <media_hal.h>
#include "esp_log.h"
#include <esp_system.h>
#include <esp_heap_caps.h>

static const char *TAG = "[app_va_cb]";
static int prv_led_state = 1000;

//returns current va state
int get_current_va_state(void)
{
    return prv_led_state;
}

//returns 1 if the va state is speaking
int check_va_app_speaking_state(void)
{
    if(prv_led_state == VA_SPEAKING) return 1;
    
    if(prv_led_state != VA_IDLE) return 1;
    
    return 0;
}
static inline int heap_caps_get_free_size_sram()
{
    return heap_caps_get_free_size(MALLOC_CAP_8BIT) - heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}
char *va0 = "Invalid";
char *va1 = "VA_THINKING";
char *va2 = "VA_SPEAKING";
char *va4 = "VA_LISTENING";
char *va8 = "VA_IDLE";
void va_app_dialog_states(va_dialog_states_t va_state)
{
char *p1 = va0;
char *p2 = va0;
if(prv_led_state == 1) p1 = va1;
if(prv_led_state == 2) p1 = va2;
if(prv_led_state == 4) p1 = va4;
if(prv_led_state == 8) p1 = va8;
if(va_state == 1) p2 = va1;
if(va_state == 2) p2 = va2;
if(va_state == 4) p2 = va4;
if(va_state == 8) p2 = va8;

printf("%s: prev state = %d (%s) -> new = %d (%s)\n",TAG, prv_led_state, p1, va_state, p2);
extern void stopQFAudiostream(void);
    if(va_state == VA_IDLE)
    {
        if((prv_led_state == VA_SPEAKING) || (prv_led_state == VA_LISTENING) || (prv_led_state == VA_THINKING))
            stopQFAudiostream(); //this will override the timer
    
    }
    if(prv_led_state != va_state) {
        va_led_set(va_state);
        ESP_LOGI(TAG, "Dialog state is: %d", va_state);
    }
    prv_led_state = va_state;
}

int va_app_volume_is_set(int vol)
{
    volume_to_set = vol;
    va_led_set(VA_SET_VOLUME);
    va_led_set(VA_SET_VOLUME_DONE);
    return 0;
}

int va_app_mute_is_set(va_mute_state_t va_mute_state)
{
    if (va_mute_state == VA_MUTE_ENABLE) {
        va_led_set(VA_SPEAKER_MUTE_ENABLE);
        va_led_set(VA_SET_VOLUME_DONE);
    } else {
        //do nothing
    }
    ESP_LOGI(TAG, "Mute: %d", va_mute_state);
    return 0;
}

int alexa_app_raise_alert(alexa_alert_types_t alexa_alert_type, alexa_alert_state_t alexa_alert_state)
{
    if (alexa_alert_type == ALEXA_ALERT_SHORT) {
        va_led_set(VA_LED_ALERT_SHORT);
        return 0;
    } else if (alexa_alert_type != ALEXA_ALERT_NOTIFICATION) {
        if (alexa_alert_state == ALEXA_ALERT_ENABLE) {
            number_of_active_alerts++;
        } else if (number_of_active_alerts > 0) {
            number_of_active_alerts--;
        }
    }
    va_led_set_alert(alexa_alert_type, alexa_alert_state);
    va_led_set(VA_IDLE);
    return 0;
}
