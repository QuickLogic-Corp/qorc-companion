// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <esp_wwe.h>
#include <i2s_stream.h>
#include <audio_board.h>
#include <media_hal.h>
#include <ringbuf.h>
#include <resampling.h>
#include <va_dsp.h>
#include <lyrat_init.h>

#define QL_BARGE_IN_ENABLE  (1)

#if (QL_BARGE_IN_ENABLE == 1)
#define QL_STREAM_SIZE (2*(16 + 2*8) * 1024) //~1.5 secs *2 = 3 secs 
#endif

#define WWE_TASK_STACK (8 * 1024)
#define RB_TASK_STACK (8 * 1024)
//#define RB_SIZE (4 * 1024)
#define RB_SIZE (2*4 * 1024)



#define DETECT_SAMP_RATE 16000UL
#define SAMP_RATE 48000UL
#define SAMP_BITS I2S_BITS_PER_SAMPLE_16BIT
#define SAMP_MS 20
//Sample size for 20millisec data on 48KHz/16bit sampling. Division factor is (sectomillisec * bitsinbytes)
#define SAMPLE_SZ ((SAMP_RATE * SAMP_BITS * SAMP_MS) / (1000 * 8)) //48*2*20 = 1920 

static const char *TAG = "[lyrat_init]";

static struct dsp_data {
    //int item_chunk_size;
    bool detect_wakeword;
    bool mic_mute_enabled;
    ringbuf_t *raw_mic_data;
    ringbuf_t *resampled_mic_data;
#if (QL_BARGE_IN_ENABLE == 1)
    ringbuf_t *ql_stream_data; //used to hold data from QF board while in STOP_MIC state
#endif
    audio_resample_config_t resample;
    i2s_stream_t *read_i2s_stream;
    TaskHandle_t ww_detection_task_handle;
    int16_t data_buf[SAMPLE_SZ];
} dd;

int lyrat_stream_audio(uint8_t *buffer, int size, int wait);
//to enable tap to talk only. alway returns 0
static int check_qf_dsp_state(void);
static void ww_detection_task(void *arg)
{
    int frequency = esp_wwe_get_sample_rate();
    int audio_chunksize = esp_wwe_get_sample_chunksize();

    int16_t *buffer = malloc(audio_chunksize*sizeof(int16_t));
    assert(buffer);
    int chunks=0;
    int priv_ms = 0;
printf("WWE frequency = %d, chunk size = %d \n", frequency, audio_chunksize);
    while(1) {
        if (dd.detect_wakeword) {
            lyrat_stream_audio((uint8_t *)buffer, (audio_chunksize * sizeof(int16_t)), portMAX_DELAY);
            //int r = esp_wwe_detect(buffer);
            int r = check_qf_dsp_state();// QF detects the ww and va_dsp_tap_to_talk_start() is called
            if (r && dd.detect_wakeword) {
                int new_ms = (chunks*audio_chunksize*1000)/frequency;
                //printf("%.2f: Neural network detection triggered output %d.\n", (float)new_ms/1000.0, r);
                printf("new %d: Neural network detection triggered output %d. priv=%d, chunks = %d\n", new_ms, r, priv_ms, chunks);
                int x = (new_ms - priv_ms);
                priv_ms = new_ms;
                if(x != 20) {
                    va_dsp_tap_to_talk_start();
                }
            }
            chunks++;
        } else {
            memset(buffer, 0, (audio_chunksize * sizeof(int16_t)));
            vTaskDelay(100/portTICK_RATE_MS);
        }
    }
}

static esp_err_t reader_stream_event_handler(void *arg, int event, void *data)
{
    ESP_LOGI(TAG, "Reader stream event %d", event);
    return ESP_OK;
}

static int disable_i2s_write_cb = 0;
static int16_t zero_data_buf[SAMPLE_SZ];
static int ql_dsp_state_ready = 0;
static volatile int pause_resample_task = 0;
static volatile int stream_kp_detected = 0;

static int16_t empty_data_buf[2000]; //this is dependent on the max SPI buffer from S3 we receive. so keep large

int resampler_state = 0;

void set_stream_kp_state(int state)
{
    stream_kp_detected = state;
#if 0
    printf("%s: KP stream start sizes raw = %d, resample = %d, stream = %d, taskstate = %d state = %d, dsp = %d \n", TAG, 
            dd.raw_mic_data->fill_cnt,dd.resampled_mic_data->fill_cnt, dd.ql_stream_data->fill_cnt, resampler_state, state, ql_dsp_state_ready );
#endif
    return;
}

//store the stream data in circular buffer for later use 
void write_to_ql_stream_buffer(void *data, int len)
{
   ringbuf_t *rb = dd.ql_stream_data;
   //if there is enough space, write the data
   if((rb->size - rb->fill_cnt) >= len)
   {
        rb_write(rb, data, len, 5);
        return;
   }
   //depending on the state, either wait or (empty and write)
   if(stream_kp_detected)
   {
        //wait until empty
        //rb_write(rb, data, len, portMAX_DELAY);
        int count = rb_write(rb, data, len, 15); //150ms
        if (count < len)
        {
            printf("%s: Stream buffer is full %d \n", TAG, rb->fill_cnt);
            for(int i =0; i<4; i++)
            {
                rb_read(rb, (uint8_t *)empty_data_buf, len, 5);
            }
        }
   }
   else //empty old data and fill in new data
   {
        rb_read(rb, (uint8_t *)empty_data_buf, len, 5);
        rb_read(rb, (uint8_t *)empty_data_buf, len, 5);
        rb_write(rb, data, len, 5);
   }
   
   //just a caution to prevent memory corrution
   if(len > sizeof(empty_data_buf))
        ESP_LOGE(TAG, "size of temp buf ( %d) should be larger than %d \n", sizeof(empty_data_buf), len);

   return;
}

void set_ql_spi_data_state(int ready)
{
    return;
/*    if(ready)
        disable_i2s_write_cb = 1; 
    else
        disable_i2s_write_cb = 0; //give dummy zeros
 */
}
static int phrase_length = 0;
void set_phrase_length(int length)
{
    phrase_length = length;
    return;
}
int get_phrase_length(void)
{
    return phrase_length;
}
void set_ql_dsp_detected_state(int offset)
{
    phrase_length = offset;
    ql_dsp_state_ready = 1;
    //va_dsp_tap_to_talk_start();
    set_ql_spi_data_state(1);
    set_stream_kp_state(0);
    rb_reset(dd.ql_stream_data);

    va_dsp_tap_to_talk_start();
    lyrat_start_capture();

    //fill raw mic data buffer to the half-full
    ringbuf_t *rb = dd.raw_mic_data;
    while(rb->fill_cnt < (rb->size/2) )
    {
        rb_write(rb, (uint8_t *)zero_data_buf, sizeof(zero_data_buf), 1);
        if(rb->fill_cnt >= (rb->size/2))
            break;
    }
    rb = dd.resampled_mic_data;
    while(rb->fill_cnt < (rb->size/2) )
    {
        rb_write(rb, (uint8_t *)zero_data_buf, sizeof(zero_data_buf), 1);
        if(rb->fill_cnt >= (rb->size/2))
            break;
    }
    

//    printf("%s: raw_mic_data fill_cnt=%d, resampled_mic_data=%d\n", TAG, dd.raw_mic_data->fill_cnt, dd.resampled_mic_data->fill_cnt); 

    return;
}
void set_ql_dsp_stream_state(int offset)
{
#if 1
    if(dd.detect_wakeword == false)
    {  //capture is still going on. So, should not break the data flow
        set_stream_kp_state(0); //so that raw mic data is taken instead of the stream cricular buffer
        va_dsp_tap_to_talk_start();
        printf("%s: KP stream is not inserted. raw size = %d resample size = %d \n", TAG, dd.raw_mic_data->fill_cnt,
                           dd.resampled_mic_data->fill_cnt);
        return;
    }
#endif

    //capture stopped. So stream circular buffer has to be used
    set_stream_kp_state(1);
    set_phrase_length(offset);

#if 1
    //resample task may be waiting for raw data. so write some data
    ringbuf_t *rb = dd.raw_mic_data;
    if((rb->size - rb->fill_cnt) > sizeof(zero_data_buf))
       rb_write(rb, (uint8_t *)zero_data_buf, sizeof(zero_data_buf), 1);
#endif

    printf("%s: raw_mic_data fill_cnt=%d, resampled_mic_data=%d\n", TAG, dd.raw_mic_data->fill_cnt, dd.resampled_mic_data->fill_cnt);     
    return;
}
#if 0 //just for debug, transfer the buffer to S3 via SPI
extern int h2d_transmit_buffer(uint32_t buf_address, int size, uint8_t *buf_data );
void transmit_buffer_to_s3(void)
{
    int fill_cnt = dd.ql_stream_data->fill_cnt;
    uint32_t s3_buf = 0x20064bbc; //&aucAudioBuffer3[0]
    uint32_t s3_buf_size = (12*3*960) ; //sizeof(aucAudioBuffer3)
    while(fill_cnt > 0)
    {
        int read_cnt = 512;
        if(read_cnt > fill_cnt)
            read_cnt = fill_cnt;

        rb_read(dd.ql_stream_data, (uint8_t *)empty_data_buf, read_cnt, 5);
        fill_cnt = dd.ql_stream_data->fill_cnt;
        
        if(s3_buf_size >= read_cnt)
        {
            h2d_transmit_buffer(s3_buf, read_cnt, (uint8_t *)empty_data_buf);
            s3_buf += read_cnt;
            s3_buf_size -= read_cnt;
        }
    }
    s3_buf = 0x20064bbc;
    s3_buf_size = (12*3*960) - s3_buf_size;
    printf("%s: Written %d bytes to S3 address 0x%8X \n", TAG, s3_buf_size, s3_buf); 
    
    return;
}
#endif

static int check_stream_kp_state(void)
{
    if(ql_dsp_state_ready == 2)
    {
        if(stream_kp_detected)
        {
            //printf("%s: KP stream tap start size = %d \n", TAG, dd.ql_stream_data->fill_cnt);
//transmit_buffer_to_s3();
            if (dd.detect_wakeword == true) 
            { //purge resampled data if it went back to detect state
//                rb_reset(dd.resampled_mic_data);
            }
            lyrat_start_capture();
            va_dsp_tap_to_talk_start();

            ql_dsp_state_ready = 3;
            //printf("%s: KP stream size = %d \n", TAG, dd.ql_stream_data->fill_cnt);
#if 1 //1= truncate the data to ~10K samples
            int fill_cnt = dd.ql_stream_data->fill_cnt;
            int offset = 2*get_phrase_length() + 8000;
            if(offset < (2*14000))
                offset = (2*14000);
            
            //while(fill_cnt > (2*12000))
            //if(fill_cnt > (2*14000))
            while(fill_cnt > offset)
            {
                int read_cnt = sizeof(empty_data_buf);
                int diff_cnt = fill_cnt - offset;
                if(read_cnt > diff_cnt)
                    read_cnt =  diff_cnt;

                int count = rb_read(dd.ql_stream_data, (uint8_t *)empty_data_buf, read_cnt, 1);
                if (count < read_cnt)
                    break;
                fill_cnt = dd.ql_stream_data->fill_cnt;
            }
#endif
            printf("%s: KP stream size = %d \n", TAG, dd.ql_stream_data->fill_cnt);
        }
    }
    return 0;
}

// called from resampler task
static int check_qf_dsp_state(void)
{
    if(ql_dsp_state_ready == 1)
    {
      ql_dsp_state_ready = 2;
    }
    check_stream_kp_state();
    return 0;
}

//wrapper to make a call from ql host task
void write_to_esp32_ring_buffer(void *data, int len)
{
    //write to raw buffer will be pushed into resample buffer
    rb_write(dd.raw_mic_data, data, len, 5);//5= 50ms

    //store in circular buffer will be pushed into resample buffer if KP is detected in stream
    write_to_ql_stream_buffer(data, len);
}

#if 0 //not used. but keep it as example for preventing task switching on a multi-core processor
void empty_resampled_buf(int count)
{
    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mutex);
    
    if(dd.resampled_mic_data->fill_cnt > count)
        rb_read(dd.resampled_mic_data, (uint8_t *)dd.data_buf, count, portMAX_DELAY);
    
    portEXIT_CRITICAL(&mutex);
    return;
}
#endif
 //get the data from QL SPI buff directly
 //ignore the I2S callbacks
static ssize_t dsp_write_cb(void *h, void *data, int len, uint32_t wait)
{
    if(len < 0) 
        len = 0;
    return len;
}
static void resample_rb_data_task(void *arg)
{
    size_t sent_len;
    volatile bool last_detect_state = dd.detect_wakeword;
    
    while(1) {
        resampler_state = 0;
        //first check if the DSP is ready
        check_qf_dsp_state();
        resampler_state = 1;
        //if stream KP detected, throw away raw_mic_data and read from circular stream buffer
        if(stream_kp_detected)
        {
            resampler_state = 2;
            sent_len = (SAMPLE_SZ/3);
            //first need to empty raw_mic_data filled
            while(rb_filled(dd.raw_mic_data) >= sent_len)
            {
                sent_len = rb_read(dd.raw_mic_data, (uint8_t *)dd.data_buf, (SAMPLE_SZ/3), 1);
            }
            //next read from the stream buffer
            sent_len = rb_read(dd.ql_stream_data, (uint8_t *)dd.data_buf, (SAMPLE_SZ/3), portMAX_DELAY);
        } else {
        //sent_len = rb_read(dd.raw_mic_data, (uint8_t *)dd.data_buf, SAMPLE_SZ * 2, portMAX_DELAY);
        //since SAMPLE_SZ = 48*2*20 bytes at 48K sample rate
        sent_len = rb_read(dd.raw_mic_data, (uint8_t *)dd.data_buf, (SAMPLE_SZ * 1)/3, portMAX_DELAY);
        }
        resampler_state = 3;
        //if(pause_resample_task)
        //   vTaskDelay(30/portTICK_RATE_MS);
       
        if (dd.mic_mute_enabled) {
            // Drop the data.
            vTaskDelay(200/portTICK_RATE_MS);
            printf("%s: muted ***",TAG);
        } else {
            sent_len = (SAMPLE_SZ/3);
            //write into resampled buffer first
            //rb_write(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sent_len, portMAX_DELAY);
            resampler_state = 4;
            //only if not detect state fill the buffer for va, else throw away since ww_detection_task is disabled
            if (dd.detect_wakeword == true) 
            {
                resampler_state = 5;
                //drop the data
                last_detect_state = true;
                //this is just to throw away, so should return immediately
                while(dd.resampled_mic_data->fill_cnt >= sent_len)
                    rb_read(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sent_len, 1);
            } else { 
                //if state change happened throw away the data remaining in the buffer
                if(last_detect_state == true)
                {
                    //rb_reset(dd.resampled_mic_data);
                    last_detect_state = false;
                }
                //rb_write(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sent_len, portMAX_DELAY);
            }
            resampler_state = 6;
            //write into resampled buffer last, since it is
            rb_write(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sent_len, portMAX_DELAY);
        }
        
    }
}

int lyrat_stream_audio(uint8_t *buffer, int size, int wait)
{
    return rb_read(dd.resampled_mic_data, buffer, size, wait);
}
static int stream_kp_cap_state = 0;
void lyrat_stop_capture()
{
    //portkENTER_CRITICAL();
    dd.detect_wakeword = true;
    printf("$$$");
    //if(stream_kp_cap_state == 1)
        set_stream_kp_state(0);
    if(ql_dsp_state_ready == 3)
        ql_dsp_state_ready = 2; // there could be another stream kp
    //portkEXIT_CRITICAL();
}

void lyrat_start_capture()
{
    dd.detect_wakeword = false;
    if(stream_kp_detected)
        stream_kp_cap_state = 1;
    else
        stream_kp_cap_state = 0;
    
//    printf("%s: Start- raw_mic_data fill_cnt=%d, resampled_mic_data=%d\n", TAG, dd.raw_mic_data->fill_cnt, dd.resampled_mic_data->fill_cnt); 
}

void lyrat_mic_mute()
{
    dd.mic_mute_enabled = true;
    dd.detect_wakeword = false;
}

void lyrat_mic_unmute()
{
    dd.mic_mute_enabled = false;
    dd.detect_wakeword = true;
}

void lyrat_init()
{
    dd.raw_mic_data = rb_init("raw-mic", RB_SIZE);
    dd.resampled_mic_data = rb_init("resampled-mic", RB_SIZE);
    memset(zero_data_buf, 0 , sizeof(zero_data_buf));
#if (QL_BARGE_IN_ENABLE == 1)
    dd.ql_stream_data = rb_init("ql-stream", QL_STREAM_SIZE);
#endif
#if 1 //disable this
    i2s_stream_config_t i2s_cfg;
    memset(&i2s_cfg, 0, sizeof(i2s_cfg));
    i2s_cfg.i2s_num = 0;
    audio_board_i2s_init_default(&i2s_cfg.i2s_config);
    i2s_cfg.media_hal_cfg = media_hal_get_handle();

    dd.read_i2s_stream = i2s_reader_stream_create(&i2s_cfg);
    if (dd.read_i2s_stream) {
printf("%s : I2S Read stream created \n", TAG);
        ESP_LOGI(TAG, "Created I2S audio stream");
    } else {
        ESP_LOGE(TAG, "Failed creating I2S audio stream");
    }
    i2s_stream_set_stack_size(dd.read_i2s_stream, 5000);

    audio_io_fn_arg_t stream_reader_fn = {
        .func = dsp_write_cb,
        .arg = NULL,
    };
    audio_event_fn_arg_t stream_event_fn = {
        .func = reader_stream_event_handler,
    };
    if (audio_stream_init(&dd.read_i2s_stream->base, "i2s_reader", &stream_reader_fn, &stream_event_fn) != 0) {
        ESP_LOGE(TAG, "Failed creating audio stream");
        i2s_stream_destroy(dd.read_i2s_stream);
        dd.read_i2s_stream = NULL;
    }
printf("%s : I2S Read stream initialized \n", TAG);
#endif
#if 0  
    //Initialize NN model
    if (esp_wwe_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ESP-WWE");
        return;
    }
printf("%s : After WWE initialized \n", TAG);
#endif

    //Initialize sound source
    //dd.item_chunk_size = esp_wwe_get_sample_chunksize() * sizeof(int16_t);
#if 1 //disable this
    audio_stream_start(&dd.read_i2s_stream->base);
    vTaskDelay(10/portTICK_RATE_MS);
    audio_stream_stop(&dd.read_i2s_stream->base);
printf("%s : Before I2S set clk \n", TAG);
    //This sets 48000 sample rate
    i2s_set_clk(I2S_NUM_0, SAMP_RATE, SAMP_BITS, I2S_CHANNEL_STEREO); 
    
    vTaskDelay(10/portTICK_RATE_MS);
    audio_stream_start(&dd.read_i2s_stream->base);
#endif
    dd.detect_wakeword = true;

#if 0
    memset(dd.data_buf, 0, sizeof(dd.data_buf));
    rb_write(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sizeof(dd.data_buf), 0);
    rb_write(dd.resampled_mic_data, (uint8_t *)dd.data_buf, sizeof(dd.data_buf), 0);
#endif

#if 0 //0 = completely disable detection task and absorb the functionality into resample task
    xTaskCreate(&ww_detection_task, "nn", WWE_TASK_STACK, NULL, (CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT - 1), &dd.ww_detection_task_handle);
#endif
printf("%s : After WW_detection created\n", TAG);
    xTaskCreate(&resample_rb_data_task, "rb read task", RB_TASK_STACK, NULL, (CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT - 1), NULL);
}