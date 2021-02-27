/*==========================================================
 *
 *-  Copyright Notice  -------------------------------------
 *                                                          
 *    Licensed Materials - Property of QuickLogic Corp.     
 *    Copyright (C) 2019 QuickLogic Corporation             
 *    All rights reserved                                   
 *    Use, duplication, or disclosure restricted            
 *                                                          
 *    File   : h2d_protocol.h
 *    Purpose: header file for host to s3 device communication protocol 
 *                                                          
 *=========================================================*/

#ifndef __H2D_PROTOCOL_H_
#define __H2D_PROTOCOL_H_

#include "stdlib.h"
#include "stdio.h"

#define DEBUG_H2D_PROTOCOL          (0)    // define this flag  to 1 to enable prints for H2D debug

//Note: This is by default 4-pin protocol 1 interrupt pin and 1 ack pin each from Device and Host
//But when there is a requirement to use only  2 pins, one pin from each of Device and Host
//there is a inherent limitation in the protocol. Since same pin is used for both interrupt generation
//and ack generation, there will be times a misinterpretation occurs which will lead to missing
//the interrupts. So, 2-pin protocol should be avoided.
#define USE_4PIN_D2H_PROTOCOL       (1) //1=use 4 pins, 2 for interrupt and 2 for ack, 0=2 pin protocol (optional)

/* Imp : Make sure these addresses are in sync with icf/ld files*/
#define H2D_WRITE_ADDR                  (0x7C800) // this is where host writes and slave reads
#define H2D_READ_ADDR                   (0x7C400) // this is whrer host reads and slave writes
#define DATA_READ_WRITE_SIZE            (128)

#define MAX_NUM_CHANNEL             (64)
#define MAX_DATA_LEN_IN_BYTES       (6)
#define H2D_PACKET_SIZE_IN_BYTES    (8)

#define H2DRX_MSGQ_WAIT_TIME	portMAX_DELAY

#define H2DRX_QUEUE_LENGTH  10  //msg queue size for h2dRxTaskHandler

/*msg types for h2d rx task */
#define H2DRX_MSG_ACK_RCVD   (0x31)
#define H2DRX_MSG_INTR_RCVD  (0x33)


typedef struct {
	uint8_t	H2D_gpio;	/* For Host to Device interrupt generation (QL_INT) */
	uint8_t	D2H_gpio;	/* For Device to Host interrupt (AP_INT) */

#if (USE_4PIN_D2H_PROTOCOL == 1)
	uint8_t	H2D_ack;	/* For Host to Device Ack (is an interrupt to Device) */
	uint8_t	D2H_ack;	/* For Device to Host interrupt (is an interrupt to Host) */
#endif    

} H2D_Platform_Info;

/* Structure to be used by user to send info for transmitting command to device */
typedef struct {
    uint8_t seq;                // seq number of command. Increases after each cmd is sent
    uint8_t channel;            // channel number 
    uint8_t cmd;                //  command id
    uint8_t data[6];            // data to be sent in the packet
} H2D_Cmd_Info;


/* structre returned by the rx callback */
typedef struct {
    uint8_t data_read_req;          // flag to convey id second data read is req by host
    uint16_t len;                   // length of data to be read
    uint32_t addr;                  // address (device memory) from where data is to be read
} Rx_Cb_Ret;

//uint8_t data_buf_ready  -> this is to convey if the rx data buf is ready in case event from device needed another read.
typedef Rx_Cb_Ret (*H2D_Callback)(H2D_Cmd_Info rx_cmd_info, uint8_t data_buf_ready); 




#define H2D_ERR_BASE    (13)
#define H2D_STATUS_OK   (0)
#define H2D_ERROR       ((H2D_ERR_BASE << 16) | 1)

/*Create tx packet api*/

/* tx api*/
int h2d_transmit_cmd(H2D_Cmd_Info *h2d_cmd_info);


/*register rx callback api*/
int h2d_register_rx_callback(H2D_Callback rx_cb, uint8_t ch_num);


/* init api */
int h2d_protocol_init(void);

void generate_interrupt_to_device(void);
void clear_interrupt_to_device(void);

extern uint8_t g_h2d_tx_buf [H2D_PACKET_SIZE_IN_BYTES];
extern uint8_t g_h2d_rx_buf [H2D_PACKET_SIZE_IN_BYTES];
extern uint8_t g_data_buf[3*1024];

#endif //__H2D_PROTOCOL_H_