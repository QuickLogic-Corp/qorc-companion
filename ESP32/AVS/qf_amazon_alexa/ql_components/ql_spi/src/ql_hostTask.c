/*==========================================================
 *
 *-  Copyright Notice  -------------------------------------
 *                                                          
 *    Licensed Materials - Property of QuickLogic Corp.     
 *    Copyright (C) 2019 QuickLogic Corporation             
 *    All rights reserved                                   
 *    Use, duplication, or disclosure restricted            
 *                                                          
 *    File   : ql_hostTask.c
 *    Purpose: host task for QL smart remote test application 
 *                                                          
 *=========================================================*/
#include <string.h>

#include "ql_hostTask.h"
#include "qlspi_s3.h"
#include "qlspi_fw_loader.h"
#include "firmware_raw_image.h" 
#include "h2d_protocol.h"

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <esp_err.h>

#define DISABLE_AVS_CONNECTION  (0) //1=use standalone, without sending to Amazon cloud 

static const char *TAG = "[QL_Host]";

xTaskHandle xHandleTaskHost;
QueueHandle_t Host_MsgQ;

//extern int spi_master_init(uint32_t baud_rate);

#define HOST_TRANSPORT_CHUNK_SIZE  1024
#define NUM_TRANSPORT_CHUNKS       (50*3)
#define MAX_RX_STORAGE_BUFF_SIZE   (1*1024) //(280*1024)
//(HOST_TRANSPORT_CHUNK_SIZE * NUM_TRANSPORT_CHUNKS)

void stopQFAudiostream(void);
int host_error_count =0;
int kp_stream_count =0;
uint32_t kp_stream_time =0;
int alexa_state =0;
uint32_t alexa_state_time =0;
int wifi_state_count = 0;

static void StreamTimerCB(TimerHandle_t StreamTimerHandle);
static int data_timeout_happend = 0;
static int kp_detect_count = 0;

extern void config_set_pad_for_device_bootstrap(void);
//extern void spi_master_pad_setup();
extern void config_set_pad_for_device_spi(void);

uint8_t g_host_device_channel_num = PROTOCOL_CHANNEL_NUMBER_DEFAULT;

extern int is_wifi_connected(void);
extern void reboot_esp32(void);
void check_host_errors(void)
{
    host_error_count++;
    if(host_error_count > 10) {
      reboot_esp32();
    }
    return;
}
#define MAX_STREAM_KP_COUNT (10)
#define MAX_STREAM_TIME     (10*20*1000)  //200 secs

void check_kp_stream_state(void)
{
    //if we received many KP in stream
    if(kp_stream_count > MAX_STREAM_KP_COUNT) {
        //stop the stream
        StreamTimerCB(NULL);
        printf("%s :Too many stream kp %d. Stopping stream \n", TAG,kp_stream_count);
    } else {
        
        uint32_t time = xTaskGetTickCount() -kp_stream_time;
        //if the stream time is too long
        if(time > (pdMS_TO_TICKS(MAX_STREAM_TIME)) ) {
            //stop the stream
            StreamTimerCB(NULL);
            kp_stream_time = xTaskGetTickCount();
            printf("%s :Too long stream %d secs. Stopping stream \n", TAG,time/(pdMS_TO_TICKS(1000)) );
        }
    }

    return;
}
#define MAX_ALEXA_STATE_TIME  (100*1000) //100 sec

extern int get_current_va_state(void);
extern void va_reset(void);

//this is called every data pkt
void check_alexa_state(void)
{
    int state = get_current_va_state();
    //if state is changed. reset time
    if(alexa_state != state) {
        alexa_state = state;
        alexa_state_time =xTaskGetTickCount();
    } else {
        uint32_t time = xTaskGetTickCount() -alexa_state_time;
        //if the state has not change for too long
        if(time > (pdMS_TO_TICKS(MAX_ALEXA_STATE_TIME)) ) {
            //stop the stream
            StreamTimerCB(NULL);
            alexa_state_time = xTaskGetTickCount();
            va_reset();
            printf("%s : Alexa state %d did not change for %d secs. Stopping stream and Resetting VA \n", TAG,alexa_state, time/(pdMS_TO_TICKS(1000)));
        }
    }
    
    return;
}
//this is called every data pkt (90ms)
void check_wifi_state(void)
{
    if(is_wifi_connected() == 0)
        wifi_state_count++;
    else
        wifi_state_count = 0;
    //if no wifi connection for ~20 secs reboot
    if(wifi_state_count > 250) { 
        //if wifi is not connected reboot
         //printf("%s :No wifi connection for Too long. Rebooting .. \n", TAG );
         printf("%s :No wifi connection for Too long. .. \n", TAG );
         wifi_state_count = 0;
         //reboot_esp32(); //note: ESP32 seems to be resetting if no internet connection
    }
    return;
}
int8_t host_set_rx_channel(int8_t channel)
{
  switch(channel)
  {
  case PROTOCOL_CHANNEL_NUMBER_OPUS:
    g_host_device_channel_num = channel;
    //dbg_str_int("   INFO : OPUS channel ", g_host_device_channel_num);
    break;
  case PROTOCOL_CHANNEL_NUMBER_RAW:
    g_host_device_channel_num = channel;
    //dbg_str_int("   INFO : RAW channel ", g_host_device_channel_num);
    break;
  default:
    //dbg_str_int(" ERROR : invalid channel numner : ", channel);
    break;
  }
  return 0;
}

//unsigned char g_opus_transport_buffer[HOST_TRANSPORT_CHUNK_SIZE];
extern uint8_t g_data_buf[];

/* static overlay the rawBuff with rx buffer */
uint8_t rawData_2[MAX_RX_STORAGE_BUFF_SIZE]; //2K = 16 transfers at 16K sample rate(32k/2K)
uint8_t *g_p_rx_storage_buffer = (uint8_t *)&rawData_2[0]; //[MAX_RX_STORAGE_BUFF_SIZE];
static int storage_wr_index = 0;
static int unfilled_data_size = MAX_RX_STORAGE_BUFF_SIZE;
static int filled_data_size = 0;

/* will not overwrite the buffer. will hold the values till next session */
int opus_test_en = 1;

// to maintain the sequence number for cmds
static int seq = -1;

int g_recorded_duration = 0; 
int g_ts_last = -1;
int g_seq_num_last = -1;

void flush_opus_storage_buf(void)
{
    storage_wr_index=0;
    unfilled_data_size = MAX_RX_STORAGE_BUFF_SIZE;
    filled_data_size = 0;
    return;
}
struct  {
  int channel;
  int packet_size;
}o_channel_info =  {
  .channel = -1,
  .packet_size = -1
};
void display_rx_buf_addr_size(void)
{
    g_recorded_duration = 0;
    g_ts_last = -1;
    g_seq_num_last = -1;
}

void store_raw_transport_chunks(int32_t kbytes)
{
  uint8_t*   p_in = g_data_buf;
  
    if ( kbytes > unfilled_data_size)           // circular buffer wrap around case
    {        
        printf("     Buffer Full %d, overwriting \n", MAX_RX_STORAGE_BUFF_SIZE);
        filled_data_size = MAX_RX_STORAGE_BUFF_SIZE - unfilled_data_size;  //"filled_data_size" size will be used for file dump in jlink savebin command.
        unfilled_data_size = MAX_RX_STORAGE_BUFF_SIZE;
        storage_wr_index = 0; //We can put break point here to dump g_rx_storage_buffer to file using save bin command.
        g_recorded_duration = 0;
        //make sure kbytes is limited
        if(kbytes > unfilled_data_size)
            kbytes = unfilled_data_size;
    }
    memcpy(&g_p_rx_storage_buffer[storage_wr_index], p_in, kbytes);
    storage_wr_index += kbytes;
    unfilled_data_size -= kbytes;
}

void store_opus_transport_chunks(int length)
{
  //QAI_DataBlock_t *pdata_block_in = (QAI_DataBlock_t*)g_data_buf;
  //int32_t payload_len = pdata_block_in->dbHeader.numDataElements*pdata_block_in->dbHeader.dataElementSize;
  int32_t payload_len = length;
  uint8_t *p_in = g_data_buf;
  //p_in = prn_hdr(pdata_block_in);
    if (unfilled_data_size <= length)           // circular buffer wrap around case
    {
        if(opus_test_en)
        {
            // do nothing 
          return;
        }        
        printf("     Buffer Full %d, overwriting \n", MAX_RX_STORAGE_BUFF_SIZE);
        //filled_data_size = MAX_RX_STORAGE_BUFF_SIZE - unfilled_data_size;  //"filled_data_size" size will be used for file dump in jlink savebin command.
        unfilled_data_size = MAX_RX_STORAGE_BUFF_SIZE;
        storage_wr_index = 0; //We can put break point here to dump g_rx_storage_buffer to file using save bin command.
        g_recorded_duration = 0;
    }
    memcpy(&g_p_rx_storage_buffer[storage_wr_index], p_in, payload_len);
    storage_wr_index += payload_len;
    unfilled_data_size -= payload_len;
}

/*  Add Msg to the Host task queue */
uint32_t addPktToQueue_Host(struct xQ_Packet *pxMsg, int ctx)
{
  
	uint32_t uiErrCode = eQL_SUCCESS;
    if( CTXT_ISR == ctx)         // ISR context
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if( xQueueSendFromISR( Host_MsgQ, ( void * )pxMsg, &xHigherPriorityTaskWoken ) != pdPASS )
            uiErrCode = eQL_ERR_MSG_SEND;
        //portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        if(xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }

    }
    else                        // TASK context
    {
        if(xQueueSend( Host_MsgQ, ( void * )pxMsg, 0 ) != pdPASS)
        {
            ESP_LOGE(TAG, "timeout adding to Host_Q \n");
            uiErrCode = eQL_ERR_MSG_SEND;
            //empty the Queue 
            xQueueReset(Host_MsgQ);
            ESP_LOGE(TAG, "Reset the Host_Q \n");
            //send stop command to Host Task 
            stopQFAudiostream();
            check_host_errors();
        }
    }
	
	return uiErrCode;
}


Rx_Cb_Ret h2d_receive_callback(H2D_Cmd_Info rx_cmd_info, uint8_t data_buf_ready)
{
    Rx_Cb_Ret ret = {0};
    struct xQ_Packet rxPkt = {0};
    /* Check if the cmd received is EVT_OPUS_PKT_READY
        if yes, read the address and len from the data field and
        read that data over qlspi and store it in opus buffer
        after read is complete, send EVT_OPUS_PKT_READY msg to the host task
    */
    switch(rx_cmd_info.cmd)
    {
        case EVT_OPUS_PKT_READY :
            if(data_buf_ready){       // if data buf is read
                // store the data in opus buffer
                //uint16_t length = (rx_cmd_info.data[0]) | (rx_cmd_info.data[1] << 8 );
                // this copy data from g_data_buf in protocol layer to opus buffer
                //store_opus_transport_chunks(length);

            }
            else
            {                     // read the data
                ret.data_read_req = 1;
                ret.len = (rx_cmd_info.data[0]) | (rx_cmd_info.data[1] << 8 );
                ret.addr = ((rx_cmd_info.data[2]) | (rx_cmd_info.data[3] << 8) |     \
                (rx_cmd_info.data[4] << 16) | (rx_cmd_info.data[5] << 24));
                o_channel_info.packet_size = ret.len;
                o_channel_info.channel = rx_cmd_info.channel;

                return ret;         // return from callback with more data read request.
            }
            break;
            
        case EVT_RAW_PKT_READY :
            if(data_buf_ready){       // if data buf is read
                // store the data in opus buffer
                uint16_t length = (rx_cmd_info.data[0]) | (rx_cmd_info.data[1] << 8 );
                // this copy data from g_data_buf in protocol layer to raw buffer
                //store_raw_transport_chunks(length);
extern void write_to_esp32_ring_buffer(void *data, int len);
#if 0 //for test only: to check data read error over spi
                static int err_count1 = 0;
                for(int i1=0;i1 < length;i1++)
                {
                   //if(g_data_buf[i1] != 0xAA)
                   if(g_data_buf[i1] != 0xA3)
                       err_count1++;
                }
                //if(err_count1)
                    printf("=rderr= %d\n",err_count1);
#endif
#if (DISABLE_AVS_CONNECTION == 0)
                //write the data to esp32 buffers, only if Amazon cloud verification is enabled
                //note: length is in bytes =2880
                if(data_timeout_happend == 0)
                {
                    //write until timeout
                    write_to_esp32_ring_buffer((void *)g_data_buf, length);
                    check_alexa_state();
                    check_wifi_state();
                }
#endif

            }
            else
            {                     // read the data
                ret.data_read_req = 1;
                ret.len = (rx_cmd_info.data[0]) | (rx_cmd_info.data[1] << 8 );
                ret.addr = ((rx_cmd_info.data[2]) | (rx_cmd_info.data[3] << 8) |     \
                (rx_cmd_info.data[4] << 16) | (rx_cmd_info.data[5] << 24));
                o_channel_info.packet_size = ret.len;
                o_channel_info.channel = rx_cmd_info.channel;

                return ret;         // return from callback with more data read request.
            }
            break;
        case EVT_KP_DETECTED:
        case EVT_STREAM_KP_DETECTED:
        //default:
            if(data_buf_ready){       // if data buf is read
                //send the message to hosttaskhandler
kp_detect_count++;
            }
            else
            {                     // read the data
                ret.data_read_req = 1;
                ret.len = (rx_cmd_info.data[0]) | (rx_cmd_info.data[1] << 8 );
                ret.addr = ((rx_cmd_info.data[2]) | (rx_cmd_info.data[3] << 8) |     \
                (rx_cmd_info.data[4] << 16) | (rx_cmd_info.data[5] << 24));
                o_channel_info.packet_size = ret.len;
                o_channel_info.channel = rx_cmd_info.channel;

                return ret;         // return from callback with more data read request.
            }
            break;
    }
    
    //printf("[QL_Host]: finished storing\n");
    rxPkt.ucCommand = rx_cmd_info.cmd;
    // copy the rest of the data as it is (6 bytes in our current implementation)
    memcpy(&(rxPkt.ucData[0]), &(rx_cmd_info.data[0]), MAX_QUEUE_PACKET_DATA_LEN );
    addPktToQueue_Host(&rxPkt,CTXT_TASK);
    
    //printf("[QL_Host]: Added pkt to Q\n");
    return ret;
}

/*
function to increment the sequence number for each cmd sent
*/
static inline uint8_t increment_seq(void)
{
    seq++;
    if(0xFF < seq){
        seq = 0;
    }
    return ((uint8_t)(seq & 0xFF));
}
static TimerHandle_t StreamTimerHandle = NULL;
//static void StreamTimerCB(TimerHandle_t StreamTimerHandle);
t_ql_audio_meta_data aud_meta_data;

#define RESET_GPIO (32)
static void esp32_init_reset(void)
{
    gpio_config_t io_conf;
    
    //setup output gpio pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << RESET_GPIO;
    io_conf.mode = GPIO_MODE_OUTPUT;
    //io_conf.mode = GPIO_MODE_INPUT_OUTPUT; //since we read the state
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) RESET_GPIO, 0);
    
    return;
}
void send_load_firmware_cmd(void)
{
    struct xQ_Packet hostMsg = {0};
    
    // send msg to host task to load device firmware
    hostMsg.ucCommand = HOST_LOAD_DEVICE_FW;
    addPktToQueue_Host(&hostMsg, CTXT_TASK);
    return;
}
//this is called when alexa state changes in the app_va_cb.c function
void send_stop_streaming_cmd(void)
{
    struct xQ_Packet hostMsg = {0};
    
    // send msg to host task to load device firmware
    hostMsg.ucCommand = HOST_SEND_CMD_STOP_STREAMING;
    addPktToQueue_Host(&hostMsg, CTXT_TASK);
    return;
}
//Host task calls this, based on VA state change or timer timeout
void transmit_cmd_stop_streaming(void)
{
    /* Once ready, send  "CMD_HOST_PROCESS_OFF" to device*/
    H2D_Cmd_Info cmd_info = {0};
    cmd_info.channel = g_host_device_channel_num;
    cmd_info.seq = increment_seq();
    cmd_info.cmd = CMD_HOST_PROCESS_OFF;

    //printf(" Sending --- CMD_HOST_PROCESS_OFF cmd \n");
    if (h2d_transmit_cmd(&cmd_info)){
        ESP_LOGE(TAG, "Error returned from h2d tansmit api\n");
    }
    printf("CMD_HOST_PROCESS_OFF cmd sent\n");
    return;
}

/* Host task andler*/
void hostTaskHandler(void * parameter)
{
    BaseType_t qret;
    //unsigned int hostTaskStop = 0;
    struct xQ_Packet hostMsg = {0};

#if (DISABLE_AVS_CONNECTION == 1)
    //use 5 sec timeout when Amazon cloud is disabled
    StreamTimerHandle = xTimerCreate("StreamTimer", pdMS_TO_TICKS(5*1000), pdFALSE, (void*)0, StreamTimerCB);
#else
    //StreamTimerHandle = xTimerCreate("StreamTimer", pdMS_TO_TICKS(20000), pdFALSE, (void*)0, StreamTimerCB);
    StreamTimerHandle = xTimerCreate("StreamTimer", pdMS_TO_TICKS(20*1000), pdFALSE, (void*)0, StreamTimerCB);
#endif
    if(StreamTimerHandle == NULL)
    {
        ESP_LOGE(TAG, "Couldn't create streamTimer");
        configASSERT(0);
    }
    h2d_register_rx_callback(&h2d_receive_callback, g_host_device_channel_num );
    
    //init reset pin to Device 
    esp32_init_reset();
    // send msg to host task to load device firmware
    hostMsg.ucCommand = HOST_LOAD_DEVICE_FW;
    addPktToQueue_Host(&hostMsg, CTXT_TASK);
    
    printf("%s :ql_hostask started\n", TAG);
static int rx_block_count = 0;
    while(1) //!hostTaskStop)
    {
        //clear the Msg Q buffer 
        memset(&hostMsg, 0, sizeof(hostMsg));
        qret = xQueueReceive(Host_MsgQ, &hostMsg, HOST_MSGQ_WAIT_TIME);
        configASSERT(qret == pdTRUE);
        
        if(hostMsg.ucCommand != EVT_RAW_PKT_READY)
            printf("\n[QL_Host]: Recd pkt. cmd = %d\n", hostMsg.ucCommand);
        switch(hostMsg.ucCommand)
        {
        case HOST_LOAD_DEVICE_FW:
          {
#if 1
extern void check_esp32_spi_state(void);

            //first have to make sure SPI is setup
            check_esp32_spi_state();
            
            /* Reset the device hardware. QL_RST -> low-> high*/
            //HAL_GPIO_Write(GPIO_0, 0);
            gpio_set_level((gpio_num_t) RESET_GPIO, 0);
            
            /*set GPIO19/20 of device */
            config_set_pad_for_device_bootstrap();
            vTaskDelay(1);
            
            
            /* Release QL_RST*/
            //HAL_GPIO_Write(GPIO_0, 1);
            gpio_set_level((gpio_num_t) RESET_GPIO, 1);
            
            printf("[QL_Host]: Downloading Firmware\n");
            vTaskDelay(2);
            
            /*re configure host pads to be used for spi transaction*/
            //spi_master_pad_setup();
            
            SLAVE_DEV_FW_LOAD_T slave_fw_image_info;
            /*
                Set, slave device firmware image information structure to zero.
                Initialize slave device firmware image information structure to necessary
                image to be loaded on to slave device memory.
            */
            memset(&slave_fw_image_info, 0x00, sizeof(slave_fw_image_info));

            slave_fw_image_info.m4_fw_addr = (uint8_t *)rawData;
            slave_fw_image_info.m4_fw_size = sizeof(rawData);
            
            
            if (QL_STATUS_OK != QLSPI_fw_download(&slave_fw_image_info))
            {
                printf("Device Firmware Download Failed \n");
                ESP_LOGE(TAG,"\n*****ERROR Device Firmware Download Failed ***** \n");
                return;
            } else {
                
                printf("\n################# QL MCU Firmware Downloaded Successfully #################\r\n\r\n");
            }
            config_set_pad_for_device_spi();
#else
              printf("***************** Skipped firmware download ************************\n\n");
#endif
             //spi_master_init(SPI_BAUDRATE_5MHZ);
            break;
          }
        
        case EVT_STREAM_KP_DETECTED:
            printf("***KP detected in stream*** KP count = %d\n", kp_detect_count);
extern void set_ql_dsp_stream_state(int offset);
//extern void set_stream_kp_state(int state);
//extern void set_phrase_length(int length);
            //set_stream_kp_state(1);
            memcpy((void *)&aud_meta_data, (void *)g_data_buf, sizeof(t_ql_audio_meta_data));
#if (DISABLE_AVS_CONNECTION == 0)
            //set_phrase_length(aud_meta_data.n_rdsp_length_estimate);
            set_ql_dsp_stream_state(aud_meta_data.n_rdsp_length_estimate);
#endif
            printf("aud_meta_data: length %d, end %d, trig %d, score %d\n", aud_meta_data.n_rdsp_length_estimate,
            aud_meta_data.n_keyphrase_end_index, aud_meta_data.n_keyphrase_triggered_index, aud_meta_data.a_kephrase_score);
                
            kp_stream_count++;
            check_kp_stream_state();
            break;
            
        case EVT_KP_DETECTED:
          {
            memcpy((void *)&aud_meta_data, (void *)g_data_buf, sizeof(t_ql_audio_meta_data));
            printf("aud_meta_data: length %d, end %d, trig %d, score %d\n", aud_meta_data.n_rdsp_length_estimate,
            aud_meta_data.n_keyphrase_end_index, aud_meta_data.n_keyphrase_triggered_index, aud_meta_data.a_kephrase_score);
            
            /* Waking up process to be done*/

            /* For debug, reset save buffer to make svaebin easier */
            flush_opus_storage_buf();
            
          /* Once ready, send  "CMD_HOST_READY_TO_RECEIVE" to device*/
            H2D_Cmd_Info cmd_info = {0};
            cmd_info.channel = g_host_device_channel_num;
            cmd_info.seq = increment_seq();
            cmd_info.cmd = CMD_HOST_READY_TO_RECEIVE;
            
            if (h2d_transmit_cmd(&cmd_info)){
                ESP_LOGE(TAG, "Error returned from h2d tansmit api\n");
            }
            printf("CMD_HOST_READY_TO_RECEIVE cmd sent. KP count = %d \n", kp_detect_count);
            
            // Start timer that will trigger a stop
            xTimerStart(StreamTimerHandle, 0);
            printf("%s: Started timer\n", TAG);
            data_timeout_happend = 0; //reset timeout
#if (DISABLE_AVS_CONNECTION == 0)
extern void set_ql_dsp_detected_state(int offset);
            //set_ql_dsp_detected_state(12000); //use 12000 for the time being
            set_ql_dsp_detected_state(aud_meta_data.n_rdsp_length_estimate); //use the data from pkt 
#endif
            rx_block_count = 0;
            kp_stream_count =0;
            kp_stream_time =xTaskGetTickCount();
            alexa_state = get_current_va_state();
            alexa_state_time =xTaskGetTickCount();
            wifi_state_count = 0;

          break;
          }
        case EVT_OPUS_PKT_READY:
        case EVT_RAW_PKT_READY :
            rx_block_count++;
            /* receive callback will send this msg only after reading the opus data in the buffer*/
          printf(".%d",rx_block_count);
          check_kp_stream_state();
            break;
         
        case EVT_OPUS_PKT_END:
              /*opus packet end message*/
            printf("EVT_DATA_PKT_END\n");
            display_rx_buf_addr_size();
            break;
            
        case EVT_EOT:
            printf("EVT_EOT\n");
            break;
            
        case HOST_CMD_READ_DATA_FROM_S3 :
            // for debug/testing purpose only
            break;
            
        case HOST_CMD_WRTIE_DATA_TO_S3 :
            {
              // for debug/testing purpose only
            break;
            }
            
        case HOST_SEND_CMD_TO_DEVICE :
            {
              // for debug/testing purpose only
                break;
            }
        case HOST_SEND_CMD_STOP_STREAMING:
            transmit_cmd_stop_streaming();
            break;
        default :
            break;
            
        }
        //printf("[QL_Host]: Processed Recd pkt\n");
    }
    while(1)
    {
        vTaskDelay(10);
        ESP_LOGE(TAG," ---QL Host Exited ---\n");
    }
    return;
}

static void StreamTimerCB(TimerHandle_t StreamTimerHandle) {

extern int check_va_app_speaking_state(void);
    if((StreamTimerHandle != NULL) && (check_va_app_speaking_state()) )
    {
        //if the response is long, restart the timer and return
        xTimerStart(StreamTimerHandle, 0);
        return;
    }
    //if timeout already called, do not send the message again
    if(data_timeout_happend == 1)    {  return; }
    else { data_timeout_happend = 1; }

extern void set_stream_kp_state(int state);
extern void set_ql_spi_data_state(int ready);
    set_ql_spi_data_state(0);
    set_stream_kp_state(0);
//    data_timeout_happend = 1;

    if(StreamTimerHandle == NULL)
        printf(" ---NULL TimerHandle timeout ----\n");
    else
        printf(" ---TimerHandle timeout ----\n");
    
    //if timeout already called, do not send the message again
//    if(data_timeout_happend == 1)    {  return; }
//    else { data_timeout_happend = 1; }

#if 1
   //let the host task send the command to the device and return immediately since we are in AVS callback function
    send_stop_streaming_cmd();
#else
    /* Once ready, send  "CMD_HOST_PROCESS_OFF" to device*/
    H2D_Cmd_Info cmd_info = {0};
    cmd_info.channel = g_host_device_channel_num;
    cmd_info.seq = increment_seq();
    cmd_info.cmd = CMD_HOST_PROCESS_OFF;

    //printf(" Sending --- CMD_HOST_PROCESS_OFF cmd \n");
    if (h2d_transmit_cmd(&cmd_info)){
        ESP_LOGE(TAG, "Error returned from h2d tansmit api\n");
    }
    printf("CMD_HOST_PROCESS_OFF cmd sent\n");
#endif
    return;
}

//this is called when Voice Assitant state changes from VA_SPEAKING to VA_IDLE
// or VA_THINKING to VA_IDLE
void stopQFAudiostream(void)
{
    //first stop the timer
    xTimerStop(StreamTimerHandle, 0);
    StreamTimerCB(NULL);
}
//this is called when Voice Assitant calls va_app_speech_stop()
void stopQFAudioInput(void)
{
    //restart the timer since response may be longer
    xTimerStart(StreamTimerHandle, 0);
    
extern void set_ql_spi_data_state(int ready);
    set_ql_spi_data_state(0);

}

#define PRIORITY_TASK_HOST              (configMAX_PRIORITIES - 9) //(CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT + 1)
#define STACK_SIZE_TASK_HOST            (4*1024)

/* Setup msg queue and Task Handler for Host Task */
signed portBASE_TYPE StartRtosTaskHost( void)
{
    static uint8_t ucParameterToPass;

    /* Create queue for Host Task */
    Host_MsgQ = xQueueCreate( HOST_QUEUE_LENGTH, sizeof(struct xQ_Packet) ); 
    //vQueueAddToRegistry( Host_MsgQ, "Host_Q" );
    //configASSERT( Host_MsgQ );
    if (!Host_MsgQ) {
        ESP_LOGE(TAG, "Error creating Host Msg Q ");
        return pdFAIL;
    }
    
    /* Create BLE Task */
    //xTaskCreate ( hostTaskHandler, "HostTaskHandler", STACK_SIZE_TASK_HOST,  &ucParameterToPass, PRIORITY_TASK_HOST, &xHandleTaskHost);
    
    StackType_t *task_stack = (StackType_t *) heap_caps_calloc(1, STACK_SIZE_TASK_HOST, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    static StaticTask_t host_task_buf;

    xHandleTaskHost = xTaskCreateStatic(hostTaskHandler, "HostTaskHandler", STACK_SIZE_TASK_HOST,  &ucParameterToPass, PRIORITY_TASK_HOST, task_stack, &host_task_buf);

    configASSERT( xHandleTaskHost );
    return pdPASS;
}



