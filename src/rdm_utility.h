/**************************************************************************/
/*!
    @file     rdm_utility.h
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32
    
    defines constants and utility functions

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
/*

RDM Packet format 

	0	Start Code
	1	Sub Start Code
	2	Message Length
	3	Destination UID
	4	
	5	
	6	
	7	
	8	
	9	Source UID
	10	
	11	
	12	
	13	
	14	
	15	Transaction Number
	16	Port ID/Response Type
	17	Message Count
	18	SubDevice MSB
	19	SubDevice LSB
	20	Command Class
	21	PID MSB
	22	PID LSB
	23	Param Data Len
	24	Var len Data


	Message with zero parameter len is 24 bytes plus 2 byte checksum, 26 bytes total:
	
		byte[2] = 24 + byte[23]
		bytesSent = 26 + byte[23]
		
	

*/ 

#ifndef RDMutility_h
#define RDMutility_h

#ifdef __cplusplus
extern "C" {
#endif


#include <Arduino.h>


// start codes
#define RDM_START_CODE			0xCC
#define RDM_SUB_START_CODE		0x01

// command classes
#define RDM_DISCOVERY_COMMAND		0x10
#define RDM_DISC_COMMAND_RESPONSE	0x11
#define RDM_GET_COMMAND				0x20
#define RDM_GET_COMMAND_RESPONSE	0x21
#define RDM_SET_COMMAND				0x30
#define RDM_SET_COMMAND_RESPONSE	0x31


// response types
#define RDM_RESPONSE_TYPE_ACK		0x00

// discovery-network management Parameter IDs (PID)
#define RDM_DISC_UNIQUE_BRANCH	0x0001
#define RDM_DISC_MUTE			0x0002
#define RDM_DISC_UNMUTE			0x0003

// product information  PIDs
#define RDM_DEVICE_INFO			0x0060
#define RDM_DEVICE_START_ADDR	0x00F0
#define RDM_DEVICE_MODEL_DESC   0x0080
#define RDM_DEVICE_MFG_LABEL    0x0081
#define RDM_DEVICE_DEV_LABEL    0x0082

// control information  PIDs
#define RDM_IDENTIFY_DEVICE		0x1000

// packet heading constants
#define RDM_PORT_ONE				0x01
#define RDM_ROOT_DEVICE				0x0000

// RDM packet byte indexes
#define RDM_IDX_START_CODE				0
#define RDM_IDX_SUB_START_CODE			1
#define RDM_IDX_PACKET_SIZE				2
#define RDM_IDX_DESTINATION_UID			3
#define RDM_IDX_SOURCE_UID				9
#define RDM_IDX_TRANSACTION_NUM			15
#define RDM_IDX_PORT					16
#define RDM_IDX_RESPONSE_TYPE			16
#define RDM_IDX_MSG_COUNT				17
#define RDM_IDX_SUB_DEV_MSB				18
#define RDM_IDX_SUB_DEV_LSB				19
#define RDM_IDX_CMD_CLASS				20
#define RDM_IDX_PID_MSB					21
#define RDM_IDX_PID_LSB					22
#define RDM_IDX_PARAM_DATA_LEN			23


// packet sizes with and without checksum, an additional 2 bytes
#define RDM_PKT_BASE_MSG_LEN			24
#define RDM_PKT_BASE_TOTAL_LEN			26

#define RDM_DISC_UNIQUE_BRANCH_PKTL		38
#define RDM_DISC_UNIQUE_BRANCH_MSGL		0x24
#define RDM_DISC_UNIQUE_BRANCH_PDL		0x0C

// discover unique branch reply
#define RDM_DISC_PREAMBLE_SEPARATOR		0xAA

/*
 *  rdmChecksum calculates mod 0x10000 sum of bytes
 *  this is used in an RDM packet as follows
 *	bytes[len]   = rdmChecksum[MSB]
 *	bytes[len+1] = rdmChecksum[LSB]
 *
 */
uint16_t rdmChecksum(uint8_t* bytes, uint8_t len);

/*
 *  testRDMChecksum evaluates cksum and returns true if both
 *	bytes[index]   == cksum[MSB]
 *	bytes[index+1] == cksum[LSB]
 *
 */
uint8_t testRDMChecksum(uint16_t cksum, uint8_t* data, uint8_t index);

/*
 *  validateRDMPacket tests the RDM start code in pdata[0]
 *                    then it calls rdmChecksum using the packet length
 *                    which is the third byte of packet
 *                    checksum = rdmChecksum( pdata, pdata[2] )
 *  validateRDMPacket returns the result of testRDMChecksum( checksum, pdata, pdata[2] )
 *
 */
uint8_t validateRDMPacket(uint8_t* pdata);

#ifdef __cplusplus
}
#endif

#endif		//RDMutility_h