/*==========================================================
 *
 *-  Copyright Notice  -------------------------------------
 *                                                          
 *    Licensed Materials - Property of QuickLogic Corp.     
 *    Copyright (C) 2019 QuickLogic Corporation             
 *    All rights reserved                                   
 *    Use, duplication, or disclosure restricted            
 *                                                          
 *    File   : ql_hostTask.h
 *    Purpose: header file for host task for QL smart remote test application 
 *                                                          
 *=========================================================*/

#ifndef __QL_HOSTTASK_H_
#define __QL_HOSTTASK_H_


#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
//=========================================
//#include "qlsh_commands.h"

/*!	\enum eERROR_CODES
\brief These values are for individual Tasks of sensorHub system
*/
enum eERROR_CODES
{
  eQL_SUCCESS = 0,
  eQL_ERR_MSG_SEND = 1
};


/*! \def MAX_QUEUE_PACKET_DATA_LEN
\brief A macro that holds maximum data length of xQ_Packet data arguments.
*/
#define MAX_QUEUE_PACKET_DATA_LEN		6

/*! \struct xQ_Packet qlsh_commands.h "inc/qlsh_commands.h"
* 	\brief packet format shared between sensorhub tasks.
* 	
* 	All sensorhub tasks will have similar queue packet format
* 	as defined below
* 	\code
* 	 =====================================================
* 	|  D5  |  D4  |  D3  |  D2  |  D1  |  D0  | Cmd | Src |
* 	|======|======|======|======|======|======|=====|=====|
* 	|      |      |      |      |      |      |     |     |
* 	 =====================================================
* 	 Src   - Source task of packet creator
* 	 Cmd   - command to process
* 	 D0-D5 - Arguments based on commands
* 	 \endcode
*/ 
struct xQ_Packet
{
  uint8_t ucSrc;								/*!< source of packet */
  uint8_t ucCommand;							/*!< command to process */
  uint8_t ucData[MAX_QUEUE_PACKET_DATA_LEN];	/*!< arguments of command */
};
//====================================================
#define CTXT_ISR     (1)
#define CTXT_TASK    (0)

#define HOST_QUEUE_LENGTH (4*10)
#define HOST_MSGQ_WAIT_TIME (portMAX_DELAY)

/* CMD and events shared between host and device
should be same on both sides.*/
#if 1 //use the Device defs file

#include "ww_metadata.h"
#include "s3_host_proto_defs.h"

//these valus conflict with s3_host_proto_defs.h so re-define them
/* internal msgs to the host task*/
#define HOST_CMD_READ_DATA_FROM_S3      (EVT_RAW_PKT_END + 1)
#define HOST_CMD_WRTIE_DATA_TO_S3       (EVT_RAW_PKT_END + 2)
#define HOST_SEND_CMD_TO_DEVICE         (EVT_RAW_PKT_END + 3)
#define HOST_LOAD_DEVICE_FW             (EVT_RAW_PKT_END + 4)

//This is added for AVS
#define HOST_SEND_CMD_STOP_STREAMING    (EVT_RAW_PKT_END + 5)


// for debug/testing purpose only
#define CMD_DUMMY_1                     (0xF)
#define CHANNEL_DUMMY_1                     (0x5)

#else //this can lead in-compatibility in definitions

/* internal msgs to the host task*/
#define HOST_CMD_READ_DATA_FROM_S3      (21)
#define HOST_CMD_WRTIE_DATA_TO_S3       (22)
#define HOST_SEND_CMD_TO_DEVICE         (23)
#define HOST_LOAD_DEVICE_FW             (24)

// for debug/testing purpose only
#define CMD_DUMMY_1                     (0xF)
#define CHANNEL_DUMMY_1                     (0x5)

// event from device to host
typedef enum HIF_EVT_
{
  EVT_KP_DETECTED     = 0x10,
  EVT_OPUS_PKT_READY  = 0x11,
  EVT_OPUS_PKT_END    = 0x12,
  EVT_RAW_PKT_READY   = 0x21,
  EVT_RAW_PKT_END     = 0x22,
  EVT_EOT
}HIF_EVT;

// cmd from host to device
#define CMD_HOST_READY_TO_RECEIVE 0x1
#define CMD_HOST_PROCESS_OFF      0x2

#endif

// channel number used for the host - device communication
// chalil - this should be read from Device when sessoin starts, as part of what ??
#define PROTOCOL_CHANNEL_NUMBER_OPUS 1
#define PROTOCOL_CHANNEL_NUMBER_RAW  10
//#define PROTOCOL_CHANNEL_NUMBER_DEFAULT PROTOCOL_CHANNEL_NUMBER_OPUS
#define PROTOCOL_CHANNEL_NUMBER_DEFAULT PROTOCOL_CHANNEL_NUMBER_RAW

int8_t host_set_rx_channel(int8_t channel);


uint32_t addPktToQueue_Host(struct xQ_Packet *pxMsg, int ctx);
signed portBASE_TYPE StartRtosTaskHost( void);

#endif  //__QL_HOSTTASK_H_