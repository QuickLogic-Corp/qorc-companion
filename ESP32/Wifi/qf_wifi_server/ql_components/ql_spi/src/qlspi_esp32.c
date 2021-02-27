/*
 Ref:
 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
 
 Examples: 
 1.esp-idf\examples\peripherals\spi_master\hd_eeprom\main
 2.esp-idf\examples\peripherals\spi_master\lcd\main
 
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "esp32_board.h"

#if (USE_LYRAT_BOARD == 1)
//From LyraT schematics for V4.3
//SPI pins for SPI2 Master = HSPI
//#define PIN_NUM_MISO 12
//#define PIN_NUM_MOSI 13
//#define PIN_NUM_CLK  14
//#define PIN_NUM_CS   15

//Use I2s Pins for SPI2. Must disable calling va_board_init() in app_main.c
//pins from audio_board_lyraT.c
#define PIN_NUM_MISO  GPIO_NUM_0
#define PIN_NUM_MOSI  GPIO_NUM_5
#define PIN_NUM_CLK   GPIO_NUM_25
#define PIN_NUM_CS    GPIO_NUM_26 
#endif

#if (USE_DEVKITC_VE_BOARD == 1)

//Use 4 GPIO pins for SPI2. Must disable calling va_board_init() in app_main.c
#define PIN_NUM_MISO  GPIO_NUM_18
#define PIN_NUM_MOSI  GPIO_NUM_19
#define PIN_NUM_CLK   GPIO_NUM_21
#define PIN_NUM_CS    GPIO_NUM_22 

#endif

#if (USE_HUZZAH32_BOARD == 1)

//Use 4 GPIO pins for SPI2. 
#define PIN_NUM_MISO  GPIO_NUM_19
#define PIN_NUM_MOSI  GPIO_NUM_18
#define PIN_NUM_CLK   GPIO_NUM_5
#define PIN_NUM_CS    GPIO_NUM_14 

#endif


#define DMA_CHAN    1 //2

#define QLSPI_MAX_TRANSFER_SIZE (3*1024) // we use 2KBytes = 1K samples ~= 65ms of data

spi_device_handle_t ql_spi_handle;
static int spi_enabled = 0;
void esp32_init_ql_spi(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, //not used
        .quadhd_io_num = -1, //not used
        .max_transfer_sz = QLSPI_MAX_TRANSFER_SIZE
    };
    spi_device_interface_config_t devcfg={
        //.command_bits = 10,
        .clock_speed_hz=8*1000*1000,           //Clock out at 8 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size = 1,
        //.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        //.pre_cb = cs_high,
        //.post_cb = cs_low,
        .input_delay_ns = (1000/8)/2, //half a SPI clock at 8MHz SPI clock
   };
    
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    
    //Attach the QL_S3 to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &ql_spi_handle);
    ESP_ERROR_CHECK(ret);

//extern void send_load_firmware_cmd(void);
//    send_load_firmware_cmd();
    spi_enabled = 1;
    return;
}
//this hangs until spi is enabled
void check_esp32_spi_state(void)
{
    if(spi_enabled == 0)
    {
        vTaskDelay(1);
    }
    return;
}
#if 0
static void cs_high(spi_transaction_t* t)
{
    gpio_set_level(PIN_NUM_CS, 1);
}
static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(PIN_NUM_CS, 0);
}
#endif
int32_t esp32_tx_spi_data(uint8_t *data, uint32_t data_len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8*data_len;                 //Command is 8 bits
    t.tx_buffer=data;               //The data is the cmd itself
    t.user=(void*)0;                
//    printf("..tx start= %d ..\n",data_len);
    ret=spi_device_polling_transmit(ql_spi_handle, &t);  //Transmit!
    //assert( ret == ESP_OK );
//    printf("..tx end =%d ..\n",ret);
    return ret;
}
#if 0 //not used
int32_t esp32_txrx_spi_data(uint8_t *cmd, uint32_t cmd_len, uint8_t *data, uint32_t data_len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*cmd_len;
    t.tx_buffer=cmd;
    t.user = (void*)0;
    //first transmit the command
    ret = spi_device_polling_transmit(ql_spi_handle, &t);
    //if error no point in continueing
    if(ret != ESP_OK)
     return ret;

    memset(&t, 0, sizeof(t));
    t.length=8*data_len;
    t.tx_buffer=cmd;
    t.rx_buffer=data;
    t.rxlength =8*data_len;
    if(data_len < 4)
        t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)0;

    //next receive the data
    ret = spi_device_polling_transmit(ql_spi_handle, &t);

    //assert( ret == ESP_OK );

    return ret;
}
#endif
int32_t esp32_rx_spi_data(uint8_t *cmd, uint8_t *data, uint32_t data_len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*data_len;
    t.tx_buffer=cmd;
    t.rx_buffer=data;
    t.rxlength =8*data_len;
    if(data_len < 4)
        t.flags = SPI_TRANS_USE_RXDATA; //only for data_len is less than 32 bits
    t.user = (void*)0;

//    printf("..Rx start= %d ..\n",data_len);
    //next receive the data
    ret = spi_device_polling_transmit(ql_spi_handle, &t);
//    printf("..Rx end = %d..\n", ret);
    //assert( ret == ESP_OK );

    return ret;
}

/* This function is ised to configure and 
set pads which are connected to GPIO19 GPIO 20 of device
QL_MOSI/PAD38 of host is connected to GPIO 19 of device
This needs to to set to 0
QL_CS/ PAD23 of host is connected to GPIO 20 of device
This needs to be set to 1 for loading device firmaware from host
IMP : After firmware load, these pads should be reconfigured to be used for spi transaction
Do this by calling spi_master_pad_setup() function
*/
void config_set_pad_for_device_bootstrap(void)
{
    gpio_set_level(PIN_NUM_MOSI, 0);
    gpio_set_level(PIN_NUM_CS, 1);
    return;
}
void config_set_pad_for_device_spi(void)
{
   gpio_set_level(PIN_NUM_CS, 1);
   return;    
}