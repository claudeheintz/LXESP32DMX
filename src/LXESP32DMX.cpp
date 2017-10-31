/**************************************************************************/
/*!
    @file     LXESP32DMX.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    DMX Driver for ESP32

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_task_wdt.h"
#include "LXESP32DMX.h"
#include "rdm_utility.h"

LXHardwareSerial Serial2(2);	//must be initialized before ESP32DMX
LX32DMX ESP32DMX;

UID LX32DMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x02, 0x01);

// deprecated, now uses ESP32 conf0.txd_brk
#define DMX_GPIO_BREAK 0

// tx_brk configuration, see LXHardwareSerial::configureSendBreak
#define TX_BRK_ENABLE 1
#define DMX_TX_BRK_LENGTH 0x1A
#define DMX_TX_IDLE_LENGTH 0x0A

// disconnected pin definition
#define NO_PIN -1

/*
 * sendDMX is run by an task with idle priority
 * loops forever until task is ended
 */
static void IRAM_ATTR sendDMX( void * param ) {
  //LX32DMX* dmxptr = (LX32DMX*) param;
  ESP32DMX.setActiveTask(1);
  while ( ESP32DMX.continueTask() ) {
    Serial2.write(ESP32DMX.dmxData(), ESP32DMX.numberOfSlots()+1);
    Serial2.waitFIFOEmpty();
    Serial2.waitTXDone();
    				  		// break at end is actually for next packet...
#if DMX_GPIO_BREAK
    Serial2.sendBreak(150);
    delayMicroseconds(12);
#else
	Serial2.waitTXBrkDone();	  // wait for raw bit indicating transmission done
#endif
  }
  
  // signal task end and wait for task to be deleted
  ESP32DMX.setActiveTask(0);
  while( true ) {
    esp_task_wdt_feed();
  	taskYIELD();
  }
}

/*
 * receiveDMX is run by an task with idle priority
 * loops forever until task is ended
 */
static void IRAM_ATTR receiveDMX( void * param ) {
  ESP32DMX.setActiveTask(1);
  while ( ESP32DMX.continueTask() ) {
  	int c = Serial2.read();
  	if ( c >= 0 ) {
  		ESP32DMX.byteReceived(c&0xff);
  	} else {
  	    esp_task_wdt_feed();
  		taskYIELD();
  	}
  }
  
  // signal task end and wait for task to be deleted
  ESP32DMX.setActiveTask(0);
  while( true ) {
    esp_task_wdt_feed();
  	taskYIELD();
  }
}

/*
 * rdmTask is run by an task with idle priority
 * loops forever until task is ended
 */
static void IRAM_ATTR rdmTask( void * param ) {
  uint8_t task_mode;
  ESP32DMX.setActiveTask(1);
  while ( ESP32DMX.continueTask() ) {
  	esp_task_wdt_feed();
    
	int c = Serial2.read();
	if ( c >= 0 ) {								//if byte read, receive it
		ESP32DMX.byteReceived(c&0xff);
	} else {
		task_mode = ESP32DMX.rdmTaskMode();
		if ( task_mode  ) {
			if ( task_mode == DMX_TASK_SEND_RDM ) {
			    if ( ESP32DMX.rdmBreakMode() ) {
				   Serial2.sendBreak(150);					// uses hardwareSerialDelayMicroseconds
				   hardwareSerialDelayMicroseconds(12);	// <- insure no conflict on another thread/task
				   esp_task_wdt_feed();
				   Serial2.write(ESP32DMX.rdmData(), ESP32DMX.rdmPacketLength());
				} else {
				   Serial2.write(&ESP32DMX.rdmData()[1], ESP32DMX.rdmPacketLength());	//note packet length skips start code
				}
				Serial2.waitFIFOEmpty();
				Serial2.waitTXDone();
				ESP32DMX._setTaskReceiveRDM();
			} else {									// otherwise send regular DMX
				Serial2.sendBreak(150);					// send break first (uses hardwareSerialDelayMicroseconds)
				hardwareSerialDelayMicroseconds(12);    // <- insure no conflict on another thread/task
				esp_task_wdt_feed();
				Serial2.write(ESP32DMX.dmxData(), ESP32DMX.numberOfSlots()+1);
				
				Serial2.waitFIFOEmpty();
				Serial2.waitTXDone();
				if ( task_mode == DMX_TASK_SET_SEND ) {
					ESP32DMX._setTaskModeSend();
				}
			}
		} else {
		    esp_task_wdt_feed();
			taskYIELD();		// if not sending and read nothing, yield
		}
	}		//nothing read
  } 		//while
  
  // signal task end and wait for task to be deleted
  ESP32DMX.setActiveTask(0);
  while( true ) {
    esp_task_wdt_feed();
    taskYIELD();
  }
}


/*******************************************************************************
 ***********************  LX32DMX member functions  ********************/

LX32DMX::LX32DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_xHandle = NULL;
	_receive_callback = NULL;
	_rdm_receive_callback = NULL;
	clearSlots();
	_task_active = 0;
	_continue_task = 0;
	_rdm_brk_mode = 1;
}

LX32DMX::~LX32DMX ( void ) {
    stop();
    _receive_callback = NULL;
    _rdm_receive_callback = NULL;
}

void LX32DMX::startOutput ( uint8_t pin ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	Serial2.begin(250000, SERIAL_8N2, NO_PIN, pin);
#if (DMX_GPIO_BREAK == 0)
	Serial2.configureSendBreak(TX_BRK_ENABLE, DMX_TX_BRK_LENGTH, DMX_TX_IDLE_LENGTH);
#endif
	//Serial2.configureRS485(1);
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    sendDMX,            /* Function that implements the task. */
                    "DMX-Out",              /* Text name for the task. */
                    1024,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
}

void LX32DMX::startInput ( uint8_t pin ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	// disable input --if direction pin is used-- until setup complete
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	Serial2.begin(250000, SERIAL_8N2, pin, NO_PIN);
	Serial2.enableBreakDetect();
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    receiveDMX,         /* Function that implements the task. */
                    "DMX-In",              /* Text name for the task. */
                    1024,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    
    resetFrame();
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
}

void LX32DMX::startRDM ( uint8_t dirpin, uint8_t inpin, uint8_t outpin, uint8_t direction ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	pinMode(dirpin, OUTPUT);
	_direction_pin = dirpin;
	digitalWrite(_direction_pin, HIGH);//disable input until setup
	
	Serial2.begin(250000, SERIAL_8N2, inpin, outpin);
	Serial2.enableBreakDetect();

	_continue_task = 1;
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    rdmTask,         /* Function that implements the task. */
                    "DMX-Task",         /* Text name for the task. */
                    1024,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    
    _transaction = 0;
    if ( direction ) {
    	setTaskSendDMX();
    } else {
    	setTaskReceive();
    }
}

void LX32DMX::stop ( void ) {
	_continue_task = 0;
	while ( _task_active ) {
		yield();
	}
	
	// is there a better way to end task??
	// seems if task function exits, there is a crash
	// 
	if ( _xHandle != NULL ) {
		vTaskDelete( _xHandle );
	}
	
	_xHandle = NULL;
	Serial2.end();
}

void LX32DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

uint16_t LX32DMX::numberOfSlots (void) {
	return _slots;
}

void LX32DMX::setMaxSlots (int slots) {
	_slots = max(slots, DMX_MIN_SLOTS);
}

uint8_t LX32DMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LX32DMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

void LX32DMX::clearSlots (void) {
	memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}

uint8_t* LX32DMX::dmxData(void) {
	return _dmxData;
}

uint8_t* LX32DMX::rdmData( void ) {
	return _rdmPacket;
}

uint8_t* LX32DMX::receivedData( void ) {
	return _receivedData;
}

uint8_t* LX32DMX::receivedRDMData( void ) {
	return _rdmData;
}

uint8_t LX32DMX::rdmPacketLength( void ) {
	return _rdm_len;
}

uint8_t LX32DMX::continueTask() {
	return _continue_task;
}

void LX32DMX::setActiveTask(uint8_t s) {
	_task_active = s;
}

void LX32DMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == 0 ) {
			_slots = _current_slot - 1;				//_current_slot represents next slot so subtract one
			for(int j=0; j<_current_slot; j++) {	//copy dmx values from read buffer
				_dmxData[j] = _receivedData[j];
			}
	
			if ( _receive_callback != NULL ) {
				_receive_callback(_slots);
			}
		}
	} else {
		if ( _receivedData[0] == RDM_START_CODE ) {
			if ( validateRDMPacket(_receivedData) ) {
				uint8_t plen = _receivedData[2] + 2;
				for(int j=0; j<plen; j++) {
					_rdmData[j] = _receivedData[j];
					if ( _receive_callback != NULL ) {
						_rdm_receive_callback(plen);
					}
				}
			}
		} else {
			Serial.println("________________ unknown data packet ________________");
			printReceivedData();
		}
	}
	resetFrame();
}

void LX32DMX::resetFrame( void ) {		
	_dmx_state = DMX_STATE_IDLE;						// insure wait for next break
}

void LX32DMX::addReceivedByte(uint8_t value) {
	_receivedData[_current_slot] = value;
	if ( _current_slot == 2 ) {						//RDM length slot
		if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
			if ( _rdm_read_handled == 0 ) {
				_packet_length = value + 2;				//add two bytes for checksum
			}
		} else if ( _receivedData[0] == 0xFE ) {	//RDM Discovery Response
			_packet_length = DMX_MAX_FRAME;
		} else if ( _receivedData[0] != 0 ) {		// if Not Null Start Code
			_dmx_state = DMX_STATE_IDLE;			//unrecognized, ignore packet
		}
	}
	
	_current_slot++;
	if ( _current_slot >= _packet_length ) {		//reached expected end of packet
		packetComplete();
	}
}

void LX32DMX::byteReceived(uint8_t c) {
	if ( c == SLIP_END ) {			//break received
		if ( _dmx_state == DMX_STATE_RECEIVING ) {			// break has already been detected
			if ( _current_slot > 1 ) {						// break before end of maximum frame
				if ( _receivedData[0] == 0 ) {				// zero start code is DMX
					packetComplete();						// packet terminated with slots<512
				}
			}
		}
		_dmx_state = DMX_STATE_RECEIVING;
		_current_slot = 0;
		_packet_length = DMX_MAX_FRAME;						// default to receive complete frame
	} else {
		if ( _dmx_state == DMX_STATE_RECEIVING ) {
			if ( c == SLIP_ESC ) {
				_dmx_state = DMX_STATE_ESC;
			} else {
				addReceivedByte(c);
			}
		} else if ( _dmx_state == DMX_STATE_ESC ) {
			_dmx_state = DMX_STATE_RECEIVING;
			if ( c == SLIP_ESC_END ) {
				addReceivedByte(SLIP_END);
			} else if ( c == SLIP_ESC_ESC ) {
				addReceivedByte(SLIP_ESC);
			}
		}
	}
	
}

void LX32DMX::printReceivedData( void ) {
	for(int j=0; j<_current_slot; j++) {
		Serial.println(_receivedData[j]);
	}
}

void LX32DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

/************************************ RDM Methods **************************************/

void LX32DMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}

uint8_t LX32DMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
}

uint8_t LX32DMX::rdmBreakMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_brk_mode;
}

void LX32DMX::_setTaskReceiveRDM() {			// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, LOW);		// call from task loop only because receiving starts
	_current_slot = 0;						// and these flags need to be set
	_packet_length = DMX_MAX_FRAME;			// but no bytes read from fifo until next task loop
	if ( _rdm_read_handled ) {
    	_dmx_state = DMX_STATE_RECEIVING;
    } else {
    	_dmx_state = DMX_STATE_IDLE;		// if not after controller message, wait for a break
    }										// signaling start of packet
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_brk_mode = 1;
}

void LX32DMX::setTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	 _rdm_task_mode = DMX_TASK_SEND;
}

void LX32DMX::_setTaskModeSend( void ) {		// call from rdm task loop only
	 _rdm_task_mode = DMX_TASK_SEND;
}

void LX32DMX::restoreTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	 _rdm_task_mode = DMX_TASK_SET_SEND;
	 do {
	 	delay(1);
	 	taskYIELD();
	 } while ( _rdm_task_mode != DMX_TASK_SEND );
}

void LX32DMX::setTaskReceive( void ) {		// only valid if connection started using startRDM()
	_current_slot = 0;
	_packet_length = DMX_MAX_FRAME;
    _dmx_state = DMX_STATE_IDLE;
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_read_handled = 0;
    digitalWrite(_direction_pin, LOW);
}

void LX32DMX::sendRawRDMPacket( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;
	
	digitalWrite(_direction_pin, HIGH);
	_rdm_task_mode = DMX_TASK_SEND_RDM;
	
	taskYIELD();
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start
	    esp_task_wdt_feed();
		taskYIELD();
		delay(2);
	}
	
}

void  LX32DMX::setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice) {
	pdata[0] = RDM_START_CODE;		//startCode
  	pdata[1] = RDM_SUB_START_CODE;	//sub start code
  	pdata[2] = msglen;				//packet length
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, 9);
  	
  	pdata[15] = _transaction++;		//transaction number
  	pdata[16] = port;				//port
  	pdata[17] = 0x00;				//msg count (always zero for controller msgs)
  	pdata[18] = subdevice >> 8;		//sub device MSB
  	pdata[19] = subdevice & 0xFF;	//sub device LSB
  	// total always 20 bytes
}

void  LX32DMX::setupRDMDevicePacket(uint8_t* pdata, uint8_t msglen, uint8_t rtype, uint8_t msgs, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction;		//set this on read
  	pdata[RDM_IDX_RESPONSE_TYPE]	= rtype;
  	pdata[RDM_IDX_MSG_COUNT]		= msgs;
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}


void  LX32DMX::setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl) {
	pdata[RDM_IDX_CMD_CLASS] = cmdclass;	//command type
  	pdata[21] = (pid >> 8) & 0xFF;	//PID
  	pdata[22] = pid & 0xFF;
  	pdata[23] = pdl;
  	// total always 4 bytes
}

uint8_t LX32DMX::sendRDMDiscoveryPacket(UID lower, UID upper, UID* single) {
	uint8_t rv = RDM_NO_DISCOVERY;
	uint8_t j;
	
	//Build RDM packet
	setupRDMControllerPacket(_rdmPacket, RDM_DISC_UNIQUE_BRANCH_MSGL, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(BROADCAST_ALL_DEVICES_ID, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, RDM_DISC_UNIQUE_BRANCH, RDM_DISC_UNIQUE_BRANCH_PDL);
  	UID::copyFromUID(lower, _rdmPacket, 24);
  	UID::copyFromUID(upper, _rdmPacket, 30);
	
	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_DISC_UNIQUE_BRANCH_PKTL);
	delay(3);

	// any bytes read indicate response to discovery packet
	// check if a single, complete, uncorrupted packet has been received
	// otherwise, refine discovery search
	
	if ( _current_slot ) {
		rv = RDM_PARTIAL_DISCOVERY;
		
		// find preamble separator
		for(j=0; j<8; j++) {
			if ( _receivedData[j] == RDM_DISC_PREAMBLE_SEPARATOR ) {
				break;
			}
		}
		// 0-7 bytes preamble
		if ( j < 8 ) {
			if ( _current_slot == j + 17 ) { //preamble separator plus 16 byte payload
				uint8_t bindex = j + 1;
				
				//calculate checksum of 12 slots representing UID
				uint16_t checksum = rdmChecksum(&_receivedData[bindex], 12);
				
				//convert dual bytes to payload of single bytes
				uint8_t payload[8];
				for (j=0; j<8; j++) {
					payload[j] = _receivedData[bindex] & _receivedData[bindex+1];
					bindex += 2;
				}

				if ( testRDMChecksum( checksum, payload, 6 ) ) {
					//copy UID into uldata
					rv = RDM_DID_DISCOVER;
					*single = payload;
				}
			}
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}

	restoreTaskSendDMX();
	return rv;
}

uint8_t LX32DMX::sendRDMDiscoveryMute(UID target, uint8_t cmd) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, cmd, 0x00);

	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_PKT_BASE_TOTAL_LEN);
	delay(3);
	
	if ( _current_slot >= (RDM_PKT_BASE_TOTAL_LEN+2) ) {				//expected pdl 2 or 8
		if ( validateRDMPacket(_receivedData) ) {
			if ( _receivedData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
				if ( _receivedData[RDM_IDX_CMD_CLASS] == RDM_DISC_COMMAND_RESPONSE ) {
					if ( THIS_DEVICE_ID == UID(&_receivedData[RDM_IDX_DESTINATION_UID]) ) {
						rv = 1;
					}
				}
			} else {
				Serial.println("fail ACK");
			}
		} else {
			Serial.println("fail validate");
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LX32DMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = 1;
	sendRawRDMPacket(_rdmPacket[2]+2);
	delay(3);
	
	if ( _current_slot > 0 ) {
		if ( validateRDMPacket(_receivedData) ) {
			uint8_t plen = _receivedData[2] + 2;
			for(int rv=0; rv<plen; rv++) {
				_rdmData[rv] = _receivedData[rv];
			}
			rv = 1;
		} else {
		    Serial.println("fail controller packet: not valid");
		}
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
		Serial.println("fail controller packet: no response");
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LX32DMX::sendRDMControllerPacket( uint8_t* bytes, uint8_t len ) {
	for (uint8_t j=0; j<len; j++) {
		_rdmPacket[j] = bytes[j];
	}
	return sendRDMControllerPacket();
}

uint8_t LX32DMX::sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND, pid, 0x00);
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_GET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
					for(int j=0; j<len; j++) {
						info[j] = _rdmData[24+j];
					}
				}
			}
		} else {
			Serial.println("get fail: ACK");
		}
		
	} else {
		Serial.println("get fail: no valid response");
	}
	
	return rv;
}

uint8_t LX32DMX::sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 1 byte parameter is 25 (+cksum =27 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN+len, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_SET_COMMAND, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_SET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
				}
			}
		} else {
			Serial.println("fail ACK");
		}
	} else {
		Serial.println("no valid response");
	}
	
	return rv;
}

void LX32DMX::sendRDMGetResponse(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN+len;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND_RESPONSE, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX32DMX::sendAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x00);
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX32DMX::sendMuteAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN + 2;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x02);
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX32DMX::sendRDMDiscoverBranchResponse( void ) {
	// should be listening when this is called
	
	_rdmPacket[0] = 0xCC;
	_rdmPacket[1] = 0xFE;
	_rdmPacket[2] = 0xFE;
	_rdmPacket[3] = 0xFE;
	_rdmPacket[4] = 0xFE;
	_rdmPacket[5] = 0xFE;
	_rdmPacket[6] = 0xFE;
	_rdmPacket[7] = 0xFE;
	_rdmPacket[8] = 0xAA;
	
	_rdmPacket[9] = THIS_DEVICE_ID.rawbytes()[0] | 0xAA;
	_rdmPacket[10] = THIS_DEVICE_ID.rawbytes()[0] | 0x55;
	_rdmPacket[11] = THIS_DEVICE_ID.rawbytes()[1] | 0xAA;
	_rdmPacket[12] = THIS_DEVICE_ID.rawbytes()[1] | 0x55;
	
	_rdmPacket[13] = THIS_DEVICE_ID.rawbytes()[2] | 0xAA;
	_rdmPacket[14] = THIS_DEVICE_ID.rawbytes()[2] | 0x55;
	_rdmPacket[15] = THIS_DEVICE_ID.rawbytes()[3] | 0xAA;
	_rdmPacket[16] = THIS_DEVICE_ID.rawbytes()[3] | 0x55;
	_rdmPacket[17] = THIS_DEVICE_ID.rawbytes()[4] | 0xAA;
	_rdmPacket[18] = THIS_DEVICE_ID.rawbytes()[4] | 0x55;
	_rdmPacket[19] = THIS_DEVICE_ID.rawbytes()[5] | 0xAA;
	_rdmPacket[20] = THIS_DEVICE_ID.rawbytes()[5] | 0x55;
	
	uint16_t checksum = rdmChecksum(&_rdmPacket[9], 12);
	uint8_t bite = checksum >> 8;
	_rdmPacket[21] = bite | 0xAA;
	_rdmPacket[22] = bite | 0x55;
	bite = checksum & 0xFF;
	_rdmPacket[23] = bite | 0xAA;
	_rdmPacket[24] = bite | 0x55;
	
	_rdm_len = 24;							// note no start code [0]
	_rdm_brk_mode = 0;						// send (no break)
	digitalWrite(_direction_pin, HIGH); 	// could cut off receiving (?)
	delayMicroseconds(100);
	_rdm_task_mode = DMX_TASK_SEND_RDM;
	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start again
		delay(1);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}

