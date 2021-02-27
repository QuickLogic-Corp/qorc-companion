// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"


#define TAG "LEDC-JUNLAM"


//dummy functions to remove led__driver folder 
esp_err_t ledc_lyrat_init()
{
    esp_err_t ret = 0;
    return ret;
}

void va_led_set_pwm(const uint32_t * ui_led_v)
{
    return;
}