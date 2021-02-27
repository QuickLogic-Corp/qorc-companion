/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <stdio.h>

#include <esp_wifi.h>
#include <esp_console.h>
#include <esp_log.h>
#include <wifi_provisioning/wifi_config.h>

static const char *TAG = "[wifi_cli]";

extern void set_server_port_in_nvs(uint16_t server_port);

static int wifi_set_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
    if (argc != 4) {
        printf("%s: Incorrect arguments\n", TAG);
        return 0;
    }
    printf("Server Port Number is %s \n", argv[3]);
    int port_num = atoi(argv[3]);
    if((port_num > 1024) && (port_num < 65530) ) {
        printf("Valid Server Port Number %d \n", port_num);
        set_server_port_in_nvs((uint16_t)port_num);
    } else {
        printf("Invalid Server Port Number %s\n", argv[3]);
        return 0;
    }
    /**
     * Initialize WiFi with default config
     * This is ignored internally if already inited
     */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) {
        printf("%s: Failed to init WiFi\n", TAG);
        return 0;
    }

    /* Stop WiFi */
    if (esp_wifi_stop() != ESP_OK) {
        printf("%s: Failed to stop wifi\n", TAG);
    }

    /* Configure WiFi as station */
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
        printf("%s: Failed to set WiFi mode\n", TAG);
        return 0;
    }

    wifi_config_t wifi_cfg = {0};
    snprintf((char*)wifi_cfg.sta.ssid, sizeof(wifi_cfg.sta.ssid), "%s", argv[1]);
    snprintf((char*)wifi_cfg.sta.password, sizeof(wifi_cfg.sta.password), "%s", argv[2]);

    printf("Setting ssid= %s, passphrase= %s\n",wifi_cfg.sta.ssid, wifi_cfg.sta.password);

    /* Configure WiFi station with provided host credentials */
    if (esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg) != ESP_OK) {
        printf("%s: Failed to set WiFi configuration\n", TAG);
        return 0;
    }
    /* (Re)Start WiFi */
    if (esp_wifi_start() != ESP_OK) {
        printf("%s: Failed to start WiFi\n", TAG);
        return 0;
    }
    /* Connect to AP */
    if (esp_wifi_connect() != ESP_OK) {
        printf("%s: Failed to connect WiFi\n", TAG);
        return 0;
    }

    return 0;
}

extern uint16_t get_server_port_number(void);

static int wifi_get_cli_handler(int argc, char *argv[])
{
    wifi_config_t wifi_config;
    
    ESP_ERROR_CHECK( esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config));
    uint16_t server_port = get_server_port_number();
    printf("ssid= %s, passphrase= %s, server port number= %d \n",wifi_config.sta.ssid, wifi_config.sta.password, server_port);

    return 0;
}

static esp_console_cmd_t wifi_cmds[] = {
    {
        .command = "wifi-set",
        .help = "wifi-set <ssid> <passphrase> <host IPv4 Address>",
        .func = wifi_set_cli_handler,
    },
    {
        .command = "wifi-get",
        .help = "Prints current <ssid> <passphrase> <host IPv4 Address>",
        .func = wifi_get_cli_handler,
    }
};

int wifi_register_cli()
{
    int cmds_num = sizeof(wifi_cmds) / sizeof(esp_console_cmd_t);
    int i;
    for (i = 0; i < cmds_num; i++) {
        ESP_LOGI(TAG, "Registering command: %s", wifi_cmds[i].command);
        esp_console_cmd_register(&wifi_cmds[i]);
    }
    return 0;
}
