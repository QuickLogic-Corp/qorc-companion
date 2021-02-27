/*
*
* Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/

#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include <media_hal_codec_init.h>

#define HAL_TAG "MEDIA_HAL_CODEC_INIT"

#define mutex_create() \
            xSemaphoreCreateMutex()

#define mutex_lock(x) \
            xSemaphoreTake(x, portMAX_DELAY)

#define mutex_unlock(x) \
            xSemaphoreGive(x)

#define mutex_destroy(x) \
            vSemaphoreDelete(x)

static uint8_t ql_curr_vol = 0;

esp_err_t ql_codec_init(media_hal_config_t *media_hal_conf) 
{
   esp_err_t res = 0;
   ql_curr_vol =MEDIA_HAL_VOL_DEFAULT; 
   return res; 
}
esp_err_t ql_codec_deinit(int port_num) 
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_set_state(media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state)
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_set_i2s_clk(media_hal_codec_mode_t media_hal_codec_mode, media_hal_bit_length_t media_hal_bit_length)
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_config_format(media_hal_codec_mode_t mode, media_hal_format_t fmt)
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_control_volume(uint8_t volume) 
{ 
   esp_err_t res = 0;  
   ql_curr_vol = volume;
   return res; 
}
esp_err_t ql_codec_get_volume(uint8_t *volume)
{ 
   esp_err_t res = 0;  
   *volume = ql_curr_vol;
   return res; 
}
esp_err_t ql_codec_set_mute(bool bmute)
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_powerup()
{
    esp_err_t res = 0;
    return res; 
}
esp_err_t ql_codec_powerdown()
{
    esp_err_t res = 0;
    return res; 
}
//setup dummy functions to control the audio codec
static void media_hal_func_init(media_hal_t* media_hal)
{
    media_hal->audio_codec_initialize = ql_codec_init;
    media_hal->audio_codec_deinitialize = ql_codec_deinit;
    media_hal->audio_codec_set_state = ql_codec_set_state;
    media_hal->audio_codec_set_i2s_clk = ql_codec_set_i2s_clk;
    media_hal->audio_codec_config_format = ql_codec_config_format;
    media_hal->audio_codec_control_volume = ql_codec_control_volume;
    media_hal->audio_codec_get_volume = ql_codec_get_volume;
    media_hal->audio_codec_set_mute = ql_codec_set_mute;
    media_hal->audio_codec_powerup = ql_codec_powerup;
    media_hal->audio_codec_powerdown = ql_codec_powerdown;
}

esp_err_t media_hal_codec_init(media_hal_t *media_hal, media_hal_config_t *media_hal_conf)
{
    esp_err_t ret = ESP_FAIL;
    if (media_hal && media_hal_conf) {
        media_hal_func_init(media_hal);
        ql_curr_vol =MEDIA_HAL_VOL_DEFAULT;
    } else {
        ESP_LOGW(HAL_TAG, "Codec handle or config is NULL");
    }
    return ret;
}
