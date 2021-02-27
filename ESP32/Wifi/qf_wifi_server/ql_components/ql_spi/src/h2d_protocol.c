/*==========================================================
 *
 *-  Copyright Notice  -------------------------------------
 *                                                          
 *    Licensed Materials - Property of QuickLogic Corp.     
 *    Copyright (C) 2019 QuickLogic Corporation             
 *    All rights reserved                                   
 *    Use, duplication, or disclosure restricted            
 *                                                          
 *    File   : h2d_protocol.c
 *    Purpose: host to s3 device communication protocol implementaion 
 *                                                          
 *=========================================================*/

#include <string.h>
//#include "Fw_global_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
//#include "RtosTask.h"
//#include "eoss3_dev.h"
#include "h2d_protocol.h"
#include "qlspi_s3.h"
//#include "dbg_uart.h"
//#include "eoss3_hal_gpio.h"
#include "ql_hostTask.h"

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <esp_err.h>

//#include <stdio.h>
//#include "driver/spi_master.h"

#include "esp32_board.h"

#define DEBUG_H2D_PROTOCOL_2  (0)

static const char *TAG = "[H2D_Protocol]";

typedef struct {
	H2D_Callback rx_cb_ptr;
} H2D_Callback_Info;

typedef struct H2D_Protocol_info {
    H2D_Platform_Info pfm_info;
    H2D_Callback_Info cb_info[MAX_NUM_CHANNEL];
    uint8_t init_done;    
}H2D_Protocol_info;


/*Lock will be acquired when tx api is called and 
released in ISR after device receives the data*/ 
SemaphoreHandle_t g_h2d_transmit_lock;          // SJ : may not be required on the host side ??

TaskHandle_t xHandleTaskH2DRx;
QueueHandle_t H2DRx_MsgQ;


#define H2D_DATA_NUM_BYTES  (6)
#define H2D_SEQ_MASK        (0xF)
#define H2D_SEQ_BIT_POS     (4)
#define H2D_CHANNEL_MASK_0  (0xF)
#define H2D_CHANNEL_MASK_1  (0x3)
#define H2D_CHANNEL_MASK    (0x3F)
#define H2D_CHANNEL_BIT_POS (2)
#define H2D_CMD_MASK        (0x3F)

/* structure for msg to be sent to h2drx task*/
typedef struct {
    uint8_t msg;
}H2D_Rx_Pkt;

/*gloabal variable for h2d protocol info*/
H2D_Protocol_info g_h2d_protocol_info = {0};



/*
    buf[0]          buf[1]           buf[2]         buf[3]          buf[4]          buf[5]          buf[7]          buf[7]
*--------------------------------------------------------------------------------------------------------------------------------
*               |               |               |               |               |               |               |               |
*--------------------------------------------------------------------------------------------------------------------------------
*       |          |            |               |               |               |               |               |               |
*   seq     ch           cmd         data[0]          data[1]                                                       data[5]
   [7:4]   [3:0]         [5:0]
           +[7:6]
*/
/*global variable for tx packet*/
uint8_t g_h2d_tx_buf [H2D_PACKET_SIZE_IN_BYTES] = {0};

//#pragma data_alignment = 32
#if (USE_4PIN_D2H_PROTOCOL == 1)
uint8_t g_h2d_rx_buf [H2D_PACKET_SIZE_IN_BYTES] = {0};
#else //we will read pkt + data in one shot
uint8_t g_h2d_rx_buf [MAX_H2D_READ_SIZE] = {0};
#endif

uint8_t g_data_buf[3*1024] = {0};

//uint8_t g_data_buf_ready = 0;

/* Internal APIs*/

/*!
* \fn      H2D_packet create_tx_packet(H2D_Cmd_Info *h2d_cmd_info)
* \brief   Function to create h2d tx packet 
* \param   hh2d_cmd_info -- input as unpacked cmd packet
* \returns H2D_packet -- packed tx packet
*/
static void create_tx_packet(H2D_Cmd_Info *h2d_cmd_info){
  
    /* copy the 4 bits seq and 4 bits of channel number to byte 0*/
    g_h2d_tx_buf[0] = ((h2d_cmd_info->seq & H2D_SEQ_MASK) << H2D_SEQ_BIT_POS) |          \
                      ((h2d_cmd_info->channel & H2D_CHANNEL_MASK) >> H2D_CHANNEL_BIT_POS);
    
    /* copy the remaing 2 bits (lsb of channel num) of channel number and 6 bits cmd to byte 1*/
    g_h2d_tx_buf[1] = ((h2d_cmd_info->channel & H2D_CHANNEL_MASK) << (8-H2D_CHANNEL_BIT_POS)) |    \
                       (h2d_cmd_info->cmd & H2D_CMD_MASK);
    
    /*copy remaining data as it is*/
    memcpy( &(g_h2d_tx_buf[2]), &(h2d_cmd_info->data[0]),H2D_DATA_NUM_BYTES);
    
    return;
}

static void extract_rx_packet(H2D_Cmd_Info *h2d_cmd_info){
    
    h2d_cmd_info->seq = ((g_h2d_rx_buf[0] >> H2D_SEQ_BIT_POS) & H2D_SEQ_MASK);
    h2d_cmd_info->channel = ((g_h2d_rx_buf[0] & 0xF) << H2D_CHANNEL_BIT_POS) | \
                             (g_h2d_rx_buf[1] >> (8-H2D_CHANNEL_BIT_POS));
                             
    h2d_cmd_info->cmd = (g_h2d_rx_buf[1] & H2D_CMD_MASK );

#if (DEBUG_H2D_PROTOCOL_2 == 1)    
//if(h2d_cmd_info->cmd != EVT_RAW_PKT_READY)
{
  printf("[H2D Protocol]: Rx pkt = 0x%02X, 0x%02X, 0x%02X, 0x%02X ",g_h2d_rx_buf[0],g_h2d_rx_buf[1],g_h2d_rx_buf[2],g_h2d_rx_buf[3]);
  printf("0x%02X, 0x%02X, 0x%02X, 0x%02X \n",g_h2d_rx_buf[4],g_h2d_rx_buf[5],g_h2d_rx_buf[6],g_h2d_rx_buf[7]);
}
//else {    printf("[H2D Protocol]: Rx RAW pkt = %d, %d \n",h2d_cmd_info->seq, h2d_cmd_info->cmd); }
#endif
    /*copy remaining data as it is*/
    memcpy(&(h2d_cmd_info->data[0]),&(g_h2d_rx_buf[2]),H2D_DATA_NUM_BYTES);
    return;
}
#if (USE_4PIN_D2H_PROTOCOL == 1)
/*!
* \fn      void generate_interrupt_to_s3(void)
* \brief   function to generate interrupt to device (s3)
* \param   -
* \returns -
*/
 void generate_interrupt_to_device(void){

    uint8_t out_gpio = g_h2d_protocol_info.pfm_info.H2D_gpio;
    gpio_set_level(out_gpio, 1);
    
}

static inline uint8_t read_gpio_intr_val(uint8_t gpio_num)
{
    // This register will reflect the value of the IO regardless of the type/polarity 
    //return ((INTR_CTRL->GPIO_INTR_RAW) & (1<< gpio_num) );
    return gpio_get_level((gpio_num_t) gpio_num);
}

static inline uint8_t read_gpio_out_val(uint8_t gpio_num)
{
    // This register will reflect the value of the IO regardless of the type/polarity 
    //return ((MISC_CTRL->IO_OUTPUT >> gpio_num ) & 1 );
    return gpio_get_level((gpio_num_t) gpio_num);
}

/*!
* \fn      void clear_interrupt_to_s3(void)
* \brief   function to clear interrupt to device (s3)
* \param   -
* \returns -
*/
 void clear_interrupt_to_device(void){
  
    uint8_t out_gpio = g_h2d_protocol_info.pfm_info.H2D_gpio;
    gpio_set_level(out_gpio, 0);
}
#else //1-wire protocol
/*!
* \fn      void generate_interrupt_to_s3(void)
* \brief   function to generate interrupt to device (s3)
* \param   -
* \returns -
*/
 void generate_interrupt_to_device(void){
  QLSPI_Trigger_Intr(); //triggers SW Int1 on Device
  vTaskDelay(1); //wait 1ms for Device to read the data and clear interrupt
}
/*!
* \fn      void clear_interrupt_to_s3(void)
* \brief   function to clear interrupt to device (s3)
* \param   -
* \returns -
*/
 void clear_interrupt_to_device(void){
  //QLSPI_Clear_Intr();
  return;
}
#endif


/*!
* \fn      void generate_pulse_to_device(void)
* \brief   function to generate pulse to device (s3)
*          assumes that QL_INT is alread zero(low) when calling this api           
* \param   - 
* \returns -
*/
#if (USE_4PIN_D2H_PROTOCOL == 1)
static void generate_pulse_to_device(void){
H2D_Protocol_info *h2d = &g_h2d_protocol_info;

    /* set the ack intr to device */
    gpio_set_level(h2d->pfm_info.H2D_ack, 1);

    //int delay_in_ms = 1; // Add one ms delay
    //vTaskDelay((delay_in_ms/portTICK_PERIOD_MS));
    ets_delay_us(20); //wait 20 micro secs

    /*clear ack intr to device */
    gpio_set_level(h2d->pfm_info.H2D_ack, 0);

    return;
}
#endif
//Receiving a Data pkt is done in 2 stages. First the pkt info is read;
//Then the data is read using the pointer from the pkt info.
//In between there should not be any interruption.
//So, all the read_device_mem() are to be protected between start and end locks
static int start_recv_lock(void) {
   //take the lock
   if (xSemaphoreTake(g_h2d_transmit_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Error unable to take lock to g_h2d_transmit_lock\n");
        return H2D_ERROR;
   }
   return H2D_STATUS_OK;
}
static int end_recv_lock(void) {
    //release the lock 
    if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
        ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
    }
   return H2D_STATUS_OK;
}
/*!
* \fn      int read_device_mem(uint32_t addr, uint8_t * buf, uint32_t len)
* \brief   function to read from device memory using QLSPI
* \param   - memory address, destination buffer, length of data to be read
* \returns - status of qlspi read operation
*/
//static inline int read_device_mem(uint32_t addr, uint8_t * buf, uint32_t len)
static int read_device_mem(uint32_t addr, uint8_t * buf, uint32_t len)
{
    int ret = 0;
#if 0 //0= before calling must take lock
   //take the lock
   if (xSemaphoreTake(g_h2d_transmit_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Error unable to take lock to g_h2d_transmit_lock\n");
        return H2D_ERROR;
   }
#endif
    ret = QLSPI_Read_S3_Mem(addr, buf, len);
#if 0 //0= before calling must take lock
    //release the lock 
    if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
        ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
    }
#endif
    return ret;
}

/*!
* \fn      void get_data_buf(uint8_t * dest_buf, uint32_t len_bytes)
* \brief   function to get data from the data_buf
* \param   -  destination buffer, length of data to be copied
* \returns - 
*/
void get_data_buf(uint8_t * dest_buf, uint32_t len_bytes)
{
    if(NULL == dest_buf){
        return;
    }
    memcpy((void *)dest_buf, &g_data_buf[0], len_bytes);
    return;
}
#if (USE_4PIN_D2H_PROTOCOL == 1)
/* Receive task handler */
void h2dRxTaskHandler(void *pParameter){
  
    BaseType_t qret;
    unsigned int h2dRxTaskStop = 0;
    H2D_Rx_Pkt h2drx_msg;
    
    while(!h2dRxTaskStop){
        //clear the Msg Q buffer 
        memset(&h2drx_msg, 0, sizeof(h2drx_msg));
        qret = xQueueReceive(H2DRx_MsgQ, &h2drx_msg, H2DRX_MSGQ_WAIT_TIME);
        configASSERT(qret == pdTRUE);
        uint8_t ack_gpio;

#if 0 //print interrupts states 
        //interrupts 
        uint8_t out_gpio,in_gpio, out_gpio_val, in_gpio_val;
        in_gpio = g_h2d_protocol_info.pfm_info.D2H_gpio;
        out_gpio = g_h2d_protocol_info.pfm_info.H2D_gpio;
        out_gpio_val = read_gpio_out_val(out_gpio);
        in_gpio_val = read_gpio_out_val(in_gpio);
        //acks
        uint8_t out_ack,in_ack, out_ack_val, in_ack_val;
        in_ack = g_h2d_protocol_info.pfm_info.D2H_ack;
        out_ack = g_h2d_protocol_info.pfm_info.H2D_ack;
        out_ack_val = read_gpio_out_val(out_ack);
        in_ack_val = read_gpio_out_val(in_ack);

        printf("[H2D %d %d %d %d]", out_gpio_val, in_gpio_val, out_ack_val, in_ack_val);
#endif

        switch(h2drx_msg.msg)     {
        case H2DRX_MSG_INTR_RCVD:
            /* This is an event from device.
                read the device mem for rx buf over qlspi
                extract the ch_num and invoke the callback
                check if second read required for this event and fill data_buf
            */
            //To prevent another process to use the SPI bus. Start the lock
            start_recv_lock();
            // read rx buf from device mem
            if (read_device_mem(H2D_READ_ADDR,(uint8_t *)&(g_h2d_rx_buf[0]), (H2D_PACKET_SIZE_IN_BYTES))) {   //SJ
                ESP_LOGE(TAG, "device memory read failed\n");
            }
            else {
                // extract info from rx buf and fill info pkt to be sent to callback
                H2D_Cmd_Info h2d_cmd;
                Rx_Cb_Ret cb_ret = {0};
                extract_rx_packet(&h2d_cmd);
                uint8_t  ch_num = h2d_cmd.channel;

#if (DEBUG_H2D_PROTOCOL_2 == 1)    
                printf("[H2D Protocol]: calling Rx callback channel = %d\n", ch_num);
#endif
                /*invoke the callback*/
                if (g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr) {
                    cb_ret = g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr(h2d_cmd,cb_ret.data_read_req);
                    //printf("[H2D Protocol]: finished calling\n");
                }
                
                /* keep checking from callback ret value if second read is needed */
                /* if yes, then read the data from addr and length passed by cb ret value*/
                while(1 == cb_ret.data_read_req){
                    //printf("cb_ret.data_read_req = %d\n", cb_ret.data_read_req);
                    // need to read data buffer from device memory
                    if (read_device_mem(cb_ret.addr,(uint8_t *)&(g_data_buf[0]), cb_ret.len)) {
                        ESP_LOGE(TAG, "device memory read failed\n");
                    }
                    else{
#if (DEBUG_H2D_PROTOCOL_2 == 1)
                        printf("[H2D Protocol]: Read data for received event type.\n");
#endif
                        cb_ret = g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr(h2d_cmd,1);
                    }
                }
                generate_pulse_to_device();
#if (DEBUG_H2D_PROTOCOL_2 == 1)
                printf("[H2D Protocol]: Received event from device. Pulse sent\n");
#endif
            }
            //Release only after both pkt info and data are read 
            end_recv_lock();
            break;

        case H2DRX_MSG_ACK_RCVD:
            /* This is an ack from the device for host transmit
                Wait for intr from device to go low and then pull QL_INT low
            */
            ack_gpio = g_h2d_protocol_info.pfm_info.D2H_ack;
            // wait for ack intr to go low
            while (read_gpio_intr_val(ack_gpio)) {
                vTaskDelay(1); // Leave time for other process to execute
            }

            // pull QL_INT low
            clear_interrupt_to_device();
#if 0
            // release the lock
            if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
                ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
            }
#endif
            break;

        default:
            ESP_LOGE(TAG, "Invalid msg event received\n");
            break;
        } //end of switch
    } //end of while(1)
  
    return;
}
#else //use 1-wire protocol
/* Receive task handler */
void h2dRxTaskHandler(void *pParameter){
  
    BaseType_t qret;
    unsigned int h2dRxTaskStop = 0;
    H2D_Rx_Pkt h2drx_msg;

    while(!h2dRxTaskStop){
        //clear the Msg Q buffer 
        memset(&h2drx_msg, 0, sizeof(h2drx_msg));
        qret = xQueueReceive(H2DRx_MsgQ, &h2drx_msg, H2DRX_MSGQ_WAIT_TIME);
        configASSERT(qret == pdTRUE);

        switch(h2drx_msg.msg)     {
        case H2DRX_MSG_INTR_RCVD:
            /* This is an event from device.
               read the device mem for rx buf over qlspi
               extract the ch_num and invoke the callback
               check if data is expected and fill data_buf
            */
            //disable interrupt from Device. Do we need to do this?
            //dis_intr_from_s3();

            //To prevent another process to use the SPI bus. Start the lock
            start_recv_lock();
            // read rx buf from device mem
            if (read_device_mem(H2D_READ_ADDR,(uint8_t *)&(g_h2d_rx_buf[0]), (MAX_H2D_READ_SIZE))) {   //SJ
                ESP_LOGE(TAG, "device memory read failed\n");
            } else {
                // extract info from rx buf and fill info pkt to be sent to callback
                H2D_Cmd_Info h2d_cmd;
                Rx_Cb_Ret cb_ret = {0};
                extract_rx_packet(&h2d_cmd);
                uint8_t  ch_num = h2d_cmd.channel;

                /*invoke the callback*/
                if (g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr) {
                    cb_ret = g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr(h2d_cmd,cb_ret.data_read_req);
                    //printf("[H2D Protocol]: finished calling\n");
                }
                
                /* checking from callback ret value if second read is needed */
                /* if yes, then copy the data after the Pkt info */
                if (cb_ret.data_read_req == 1){
                    //printf("cb_ret.data_read_req = %d\n", cb_ret.data_read_req);
                    // need to just copy since whole buffer is already read
                    if(cb_ret.len > MAX_H2D_READ_DATA_SIZE) {
                        ESP_LOGE(TAG, "Error: Read length exceeds max buffer length %d\n", cb_ret.len);
                    }
                    memcpy((uint8_t *)&g_data_buf[0],(uint8_t *)&g_h2d_rx_buf[H2D_PACKET_SIZE_IN_BYTES], cb_ret.len);
                    cb_ret = g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr(h2d_cmd,1);
                }

            }
            //Release only after both pkt info and data are read 
            end_recv_lock();
                           
            //clear the Device Interrupt since we read the whole buffer
            clear_intr_sts_s3();
            //en_intr_from_s3();
            break;


        default:
            ESP_LOGE(TAG, "Invalid msg event received\n");
            break;
        } //end of switch
    } //end of while(1)
  
    return;
}
#endif //USE_4PIN_D2H_PROTOCOL

/*!
* \fn      void send_msg_to_h2drx_task_fromISR(uint8_t msg_type)
* \brief   send msg to  H2DRx_MsgQ
* \param   msg id to be sent
* \returns -
*/
void IRAM_ATTR send_msg_to_h2drx_task_fromISR(uint8_t msg_type) {
    H2D_Rx_Pkt h2d_msg;
    h2d_msg.msg = msg_type;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( xQueueSendFromISR( H2DRx_MsgQ, &(h2d_msg), &xHigherPriorityTaskWoken ) != pdPASS ) {
        ESP_LOGE(TAG,"Error : unable to send msg to H2DRx_MsgQ from ISR\n");
    }		
	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

    
    return;
}


/*!
* \fn      void service_intr_from_device(void)
* \brief   function to service interrupt received from the device (s3)
*          This function is called from ISR. should use only isr safe apis  
* \param   - 
* \returns -
*/
//void service_intr_from_device(void){
void IRAM_ATTR service_intr_from_device(void *arg){
    send_msg_to_h2drx_task_fromISR(H2DRX_MSG_INTR_RCVD);
    return;
}
/*!
* \fn      void service_ack_from_device(void)
* \brief   function to service ack interrupt received from the device (s3)
*          This function is called from ISR. should use only isr safe apis  
* \param   - 
* \returns -
*/
void IRAM_ATTR service_ack_from_device(void *arg){
    send_msg_to_h2drx_task_fromISR(H2DRX_MSG_ACK_RCVD);
    return;
}

#define PRIORITY_TASK_H2D_RX              (configMAX_PRIORITIES - 6) //(CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT + 2)
#define STACK_SIZE_TASK_H2D_RX            (8*1024)

/*!
* \fn      signed portBASE_TYPE start_rtos_task_h2drx( void)
* \brief   Setup msg queue and Task Handler for H2D rx Task 
* \param   - 
* \returns - portBASE_TYPE pdPASS on success
*/
static signed portBASE_TYPE start_rtos_task_h2drx( void) {
    static uint8_t ucParameterToPass;
 
    /* Create queue for h2d rx Task */
    H2DRx_MsgQ = xQueueCreate( H2DRX_QUEUE_LENGTH, sizeof(H2D_Rx_Pkt) );
    vQueueAddToRegistry( H2DRx_MsgQ, "H2DRx_Q" );
    configASSERT( H2DRx_MsgQ );
    
    /* Create H2D Rx Task */
    //xTaskCreate ( h2dRxTaskHandler, "H2DRxTaskHandler", STACK_SIZE_TASK_H2D_RX,  &ucParameterToPass, PRIORITY_TASK_H2D_RX, &xHandleTaskH2DRx);
    
    StackType_t *task_stack = (StackType_t *) heap_caps_calloc(1, STACK_SIZE_TASK_H2D_RX, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    static StaticTask_t task_buf;
    assert(task_stack);

    xHandleTaskH2DRx = xTaskCreateStatic(h2dRxTaskHandler, "H2DRxTaskHandler", STACK_SIZE_TASK_H2D_RX,  &ucParameterToPass, PRIORITY_TASK_H2D_RX, task_stack, &task_buf);

    if (xHandleTaskH2DRx == NULL) {
        ESP_LOGE(TAG, "Couldn't create h2d thread");
    }

    assert( xHandleTaskH2DRx );
    
    return pdPASS;
}
#if (USE_4PIN_D2H_PROTOCOL == 1)
//make sure while generating the interrupt to device no task switching happens
//Check both input gpio (interrupt from device) and output gpio (ack to device)
//are low before generating the interrupt to the device
static int generate_protected_interrupt(void)
{
    uint8_t out_gpio,in_gpio;
    in_gpio = g_h2d_protocol_info.pfm_info.D2H_gpio;
    out_gpio = g_h2d_protocol_info.pfm_info.H2D_gpio;
    int return_value = 0;

    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mutex);
    if( (read_gpio_intr_val(in_gpio) == 0) && 
        (read_gpio_out_val(out_gpio) == 0) )
        {
           // generate interrupt to device, QL_INT
           generate_interrupt_to_device();
           return_value = 1;
        }
    portEXIT_CRITICAL(&mutex);
    
    return return_value;
}
#endif
/*!
* \fn      int h2d_transmit_cmd(H2D_Cmd_Info *h2d_cmd_info)
* \brief   api to transmit cmd to device 
* \param   hh2d_cmd_info -- input as unpacked cmd packet, addre where the cmd pckt is to be written
* \returns status of tx operation
*/
int h2d_transmit_cmd(H2D_Cmd_Info *h2d_cmd_info) {
    if( !g_h2d_protocol_info.init_done )
        return H2D_ERROR;
vTaskDelay(1);    
    // create tx packet
   create_tx_packet(h2d_cmd_info);
//printf("-------before taking semaphore\n");
   if (xSemaphoreTake(g_h2d_transmit_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Error unable to take lock to g_h2d_transmit_lock\n");
        return H2D_ERROR;
   }
//printf("-----after taking semaphore\n");
#if 0   
    uint8_t in_gpio = g_h2d_protocol_info.pfm_info.D2H_gpio;
   //wait till intr from Device is low (AP_INT is low)
   //TIM!!! DOn;t see how this happens if Device is waiting for an ACK
   // wait for intr to go low
   while (read_gpio_intr_val(in_gpio)) {
        vTaskDelay(1); // Leave 2 ticks for other process to execute
        //ets_delay_us(10); //wait 10 micro secs
   }
#endif
   
   // transmit over qlspi
   if( QLSPI_Write_S3_Mem(H2D_WRITE_ADDR, (uint8_t *)&(g_h2d_tx_buf[0]), H2D_PACKET_SIZE_IN_BYTES )) {
        ESP_LOGE(TAG, "Error in h2d transmit ");
        
        //release the lock and return error
        if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
            ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
        }
        return H2D_ERROR;
   }

    //must release the lock 
    if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
        ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
    }

#if (USE_4PIN_D2H_PROTOCOL == 1)
   // generate interrupt to device independent of other states
   generate_interrupt_to_device();

#else //1-wire protocol generates SW Int1 

   generate_interrupt_to_device();
   
#endif //USE_4PIN_D2H_PROTOCOL
   return H2D_STATUS_OK;
}

/*!
* \fn      int h2d_register_rx_callback(H2D_Callback rx_cb, uint8_t ch_num)
* \brief   function to register rx callback for channel
* \param   callback function, channel number
* \returns - status of register operation
*/
int h2d_register_rx_callback(H2D_Callback rx_cb, uint8_t ch_num) {
    int ret = H2D_STATUS_OK;
    
    if ((NULL==rx_cb) || (ch_num >= MAX_NUM_CHANNEL)) {
        ESP_LOGE(TAG, "Invalid paramter for h2d register callback\n");
        ret = H2D_ERROR;
    }
    if(g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr != NULL) {
        ESP_LOGE(TAG, "callback for channel already registered. ch_num = \n"); //, ch_num);
        ret = H2D_ERROR;
    }
    else {
        g_h2d_protocol_info.cb_info[ch_num].rx_cb_ptr = rx_cb;
    }
    
    return ret;
}


/*platform configuration*/
void h2d_platform_init (H2D_Platform_Info * pfm_info) {   
    // Nothing to do as of now
    return;
}

//based on code from va_button_gpio_init() in va_button.c
//static void esp32_init_gpio(gpio_num_t H2D_gpio, gpio_num_t D2H_gpio )
static void esp32_init_gpio(H2D_Protocol_info *h2d_info)
{
    gpio_config_t io_conf;
#if (USE_4PIN_D2H_PROTOCOL == 1)
    gpio_num_t H2D_gpio = h2d_info->pfm_info.H2D_gpio;
#endif
    gpio_num_t D2H_gpio = h2d_info->pfm_info.D2H_gpio;
    
#if 0 //1 = just for testing make it an output
    //setup output gpio pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << D2H_gpio;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) D2H_gpio, 0);
#else    
    //setup input gpio pin interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << D2H_gpio;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) D2H_gpio, 0);
    gpio_isr_handler_add(D2H_gpio, service_intr_from_device, (void *) D2H_gpio);
#endif

#if (USE_4PIN_D2H_PROTOCOL == 1)
    //setup output gpio pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << H2D_gpio;
    //io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT; //since we read the state
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) H2D_gpio, 0);
    
//#if (USE_4PIN_D2H_PROTOCOL == 1)
    
    gpio_num_t H2D_ack = h2d_info->pfm_info.H2D_ack;
    gpio_num_t D2H_ack = h2d_info->pfm_info.D2H_ack;

    //setup input ack gpio pin interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << D2H_ack;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    //gpio_set_level((gpio_num_t) D2H_ack, 0);
    gpio_isr_handler_add(D2H_ack, service_ack_from_device, (void *) D2H_ack);

    //setup output ack gpio pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((uint64_t) 1) << H2D_ack;
    //io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT; //since we read the state
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) H2D_ack, 0);

#endif

    return;
    
}

#if (USE_LYRAT_BOARD == 1)

//these are from LyraT V4.3 schematic for I2C
#define PIN_NUM_SCL 23
#define PIN_NUM_SDA 18

#define H2D_GPIO (PIN_NUM_SCL)
#define D2H_GPIO (PIN_NUM_SDA)

#endif

#if (USE_DEVKITC_VE_BOARD == 1)
    
#define H2D_GPIO (GPIO_NUM_23)
#define D2H_GPIO (GPIO_NUM_27)

#define H2D_ACK (GPIO_NUM_33) //output
#define D2H_ACK (GPIO_NUM_34) //input only


#endif

#if (USE_HUZZAH32_BOARD == 1)
    
#define H2D_GPIO (GPIO_NUM_25)
#define D2H_GPIO (GPIO_NUM_34) //input only

#define H2D_ACK (GPIO_NUM_4) 
#define D2H_ACK (GPIO_NUM_36) //input only


#endif

/*!
* \fn      int h2d_protocol_init(H2D_Platform_Info * h2d_platform_info)
* \brief   function to initialize the h2d communication,
*          creates the transmit lock and h2drx task
* \param   platform info (input and output interrupt gpio)
* \returns - status of init operation
*/
//int h2d_protocol_init(H2D_Platform_Info * h2d_platform_info) {
int h2d_protocol_init(void) {

    if( g_h2d_protocol_info.init_done ) {
        ESP_LOGE(TAG, "h2d protocol already intialized.\n");
        return H2D_STATUS_OK;
    }
    //set up the esp32 gpio 
    //esp32_init_gpio(H2D_GPIO, D2H_GPIO);

    
#if (USE_4PIN_D2H_PROTOCOL == 0)    
    //for 1-wire protocol only D2H GPIO generates interrupt
    g_h2d_protocol_info.pfm_info.D2H_gpio = D2H_GPIO;
#else
    //for 4-pin protocol there 2 interrupts pins 2 ACK pins
    g_h2d_protocol_info.pfm_info.H2D_gpio = H2D_GPIO;
    g_h2d_protocol_info.pfm_info.D2H_gpio = D2H_GPIO;

//#if (USE_4PIN_D2H_PROTOCOL == 1)
    g_h2d_protocol_info.pfm_info.H2D_ack = H2D_ACK;
    g_h2d_protocol_info.pfm_info.D2H_ack = D2H_ACK;
#endif

    esp32_init_gpio(&g_h2d_protocol_info);
    
    //memcpy( &(g_h2d_protocol_info.pfm_info), h2d_platform_info, sizeof(H2D_Platform_Info)); 
    
    //uint8_t out_gpio = g_h2d_protocol_info.pfm_info.H2D_gpio;
    //gpio_set_level(out_gpio, 0);            // write 0 to the QL_INT at init
    
    //create tx lock
    if(g_h2d_transmit_lock == NULL) {
        g_h2d_transmit_lock = xSemaphoreCreateBinary();
        if( g_h2d_transmit_lock == NULL ) {
          ESP_LOGE(TAG, "Error : Unable to Create Mutex\n");
          return H2D_ERROR;
        }
        vQueueAddToRegistry(g_h2d_transmit_lock, "H2D_TX_Lock" );
        xSemaphoreGive(g_h2d_transmit_lock);
    }
    
    
    // create the rx task
    start_rtos_task_h2drx();
    
    g_h2d_protocol_info.init_done = 1;
    
    return H2D_STATUS_OK;
}
#if 0 //these are for testing only
//just for debug
void send_msg_to_h2drx_task_fromTask(void) {
    uint8_t msg_type = H2DRX_MSG_INTR_RCVD;
    H2D_Rx_Pkt h2d_msg;
    h2d_msg.msg = msg_type;
    
	if( xQueueSend( H2DRx_MsgQ, &(h2d_msg), 10 ) != pdPASS ) {
        ESP_LOGE(TAG,"Error : unable to send msg to H2DRx_MsgQ from ISR\n");
    }		
    return;
}
void read_device_mem_test(void)
{
    if (QLSPI_Read_S3_Mem(H2D_READ_ADDR,(uint8_t *)&(g_h2d_rx_buf[0]), (2*H2D_PACKET_SIZE_IN_BYTES))) {
        ESP_LOGE(TAG, "device memory read failed\n");
    }
    return;
}
void write_device_mem_test(void)
{
    static int start = 0x10;
    for(int i= 0; i < 2*H2D_PACKET_SIZE_IN_BYTES; i++)
        g_data_buf[i] = start++;
    start = 0x50;
    //if(QLSPI_Write_S3_Mem(H2D_WRITE_ADDR, (uint8_t *)&(g_h2d_tx_buf[0]), H2D_PACKET_SIZE_IN_BYTES )) {
    if(QLSPI_Write_S3_Mem(H2D_READ_ADDR, (uint8_t *)(&g_data_buf[0]), 2*H2D_PACKET_SIZE_IN_BYTES )) {
        ESP_LOGE(TAG, "device memory read failed\n");
    }
    return;
}
#endif
#if 0 //these are for testing only
int h2d_transmit_buffer(uint32_t buf_address, int size, uint8_t *buf_data ) {

   if (xSemaphoreTake(g_h2d_transmit_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Error unable to take lock to g_h2d_transmit_lock\n");
        return H2D_ERROR;
   }
   // transmit over qlspi
   if( QLSPI_Write_S3_Mem(buf_address, buf_data, size)) {
        ESP_LOGE(TAG, "Error in h2d transmit ");
        
        //release the lock and return error
        if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
            ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
        }
        return H2D_ERROR;
   }
    //must release the lock 
    if (xSemaphoreGive(g_h2d_transmit_lock) != pdTRUE) {
        ESP_LOGE(TAG, "Error : unable to release lock to g_h2d_transmit_lock\n");
    }
    printf(" *** transferred %d bytes to the buffer 0x%8X in S3\n", size, buf_address);
   return H2D_STATUS_OK;
}
#endif
