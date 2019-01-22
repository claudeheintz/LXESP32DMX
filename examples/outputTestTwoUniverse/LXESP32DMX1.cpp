/**************************************************************************/
/*!
    @file     LXESP32DMX1.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    DMX Driver for ESP32

    @section  HISTORY
    
    This file (and LXESP32DMX1.h) should be copied to your sketch folder
   to enable support for a second DMX serial output object, ESP32DMX1.

    v1.0 - Added January 2019 as example of second DMX output
*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "LXESP32DMX1.h"
#include "rdm_utility.h"

LXHardwareSerial LXSerial1(1);	//must be initialized before ESP32DMX
LX32DMX1 ESP32DMX1;


// disconnected pin definition
#define NO_PIN -1

/*
 * sendDMX is run by an task with idle priority
 * loops forever until task is ended
 */
static void sendDMX( void * param ) {
  //LX32DMX1* dmxptr = (LX32DMX1*) param;
  
  ESP32DMX1.setActiveTask(1);
  
  while ( ESP32DMX1.continueTask() ) {
  
    LXSerial1.sendBreak(150);
    hardwareSerialDelayMicroseconds(12);
    
    xSemaphoreTake( ESP32DMX1.lxDataLock, portMAX_DELAY );
    LXSerial1.write(ESP32DMX1.dmxData(), ESP32DMX1.numberOfSlots()+1);
    xSemaphoreGive( ESP32DMX1.lxDataLock );
    								//vTaskDelay must be called to avoid wdt and lock up issues
    vTaskDelay(5);					//use time while UART finishes sending to allow other tasks to run
    LXSerial1.waitFIFOEmpty();		//returns at about byte 384 ~128 bytes left 128 * 44 = 5.6 ms
    LXSerial1.waitTXDone();
    				  		// break at end is actually for next packet...

  }
  
  // signal task end and wait for task to be deleted
  ESP32DMX1.setActiveTask(0);
  
  vTaskDelete( NULL );	// delete this task
}

/*
 * receiveDMX is run by an task with idle priority
 * loops forever until task is ended
 */
static void receiveDMX( void * param ) {

  ESP32DMX1.setActiveTask(1);
  
  while ( ESP32DMX1.continueTask() ) {
  	int c = LXSerial1.read();
  	if ( c >= 0 ) {
  		ESP32DMX1.byteReceived(c&0xff);
  	} else {
  											//vTaskDelay must be called to avoid wdt and lock up issues
  	    vTaskDelay(1);	// can read much faster than DMX speed to catch up so 1ms delay for idle task is OK
  	}
  }
  
  // signal task end and wait for task to be deleted
  ESP32DMX1.setActiveTask(0);
  
  vTaskDelete( NULL );	// delete this task
}

/*
 * rdmTask is run by an task with idle priority
 * loops forever until task is ended
 */
static void rdmTask( void * param ) {

  uint8_t task_mode;
  int c;
  ESP32DMX1.setActiveTask(1);
  
  while ( ESP32DMX1.continueTask() ) {
  
	c = LXSerial1.read();
	
	if ( c >= 0 ) {								// if byte read, receive it...may invoke callback function
		ESP32DMX1.byteReceived(c&0xff);			// eventually will catch up, nothing to read and vTaskDelay will be called
	} else {
		task_mode = ESP32DMX1.rdmTaskMode();
		if ( task_mode  ) {
			if ( task_mode == DMX_TASK_SEND_RDM ) {
			
			   LXSerial1.sendBreak(150);					// uses hardwareSerialDelayMicroseconds
			   hardwareSerialDelayMicroseconds(12);
			   
			   LXSerial1.write(ESP32DMX1.rdmData(), ESP32DMX1.rdmPacketLength());		// data should be set from function that sets flag to get here
																					// therefore data should not need to be protected by Mutex
																					
				//vTaskDelay(1);            //   <-should not be needed here because rdm should be interleaved with regular NSC/DMX		
											//   vTaskDelay should be called next time though this loop
											//   ( at least next time bytes are not read or task_mode is send )	
				LXSerial1.waitFIFOEmpty();
				LXSerial1.waitTXDone();
				ESP32DMX1._setTaskReceiveRDM();
				
			} else {									// otherwise send regular DMX
				LXSerial1.sendBreak(150);					// send break first (uses hardwareSerialDelayMicroseconds)
				hardwareSerialDelayMicroseconds(12);    // <- insure no conflict on another thread/task
			
				xSemaphoreTake( ESP32DMX1.lxDataLock, portMAX_DELAY );
				LXSerial1.write(ESP32DMX1.dmxData(), ESP32DMX1.numberOfSlots()+1);
				xSemaphoreGive( ESP32DMX1.lxDataLock );
														//vTaskDelay must be called to avoid wdt and lock up issues
				vTaskDelay(5);							//use time while UART finishes sending to allow other tasks to run
				LXSerial1.waitFIFOEmpty();				//returns at about byte 384 ~128 bytes left 128 * 44 = 5.6 ms
				LXSerial1.waitTXDone();
				if ( task_mode == DMX_TASK_SET_SEND ) {
					ESP32DMX1._setTaskModeSend();
				}
			}
		} else {
		    vTaskDelay(1);	// receiving but nothing to read, assumes buffer can be emptied faster than it fills
		}

	}		//nothing read
  } 		//while
  
  // signal task end and wait for task to be deleted
  ESP32DMX1.setActiveTask(0);
  
  vTaskDelete( NULL );	// delete this task
}


/*******************************************************************************
 ***********************  LX32DMX1 member functions  ********************/

LX32DMX1::LX32DMX1 ( void ) {
    lxDataLock = xSemaphoreCreateMutex();
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_xHandle = NULL;
	_receive_callback = NULL;
	_rdm_receive_callback = NULL;
	clearSlots();
	_task_active = 0;
	_continue_task = 0;
	_rdm_task_mode = 0;
}

LX32DMX1::~LX32DMX1 ( void ) {
    stop();
    _receive_callback = NULL;
    _rdm_receive_callback = NULL;
}

void LX32DMX1::startOutput ( uint8_t pin ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	LXSerial1.begin(250000, SERIAL_8N2, NO_PIN, pin);

	//LXSerial1.configureRS485(1);
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    sendDMX,            /* Function that implements the task. */
                    "DMX-Out",              /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
}

void LX32DMX1::startInput ( uint8_t pin ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	// disable input --if direction pin is used-- until setup complete
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	LXSerial1.begin(250000, SERIAL_8N2, pin, NO_PIN);
	LXSerial1.enableBreakDetect();
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    receiveDMX,         /* Function that implements the task. */
                    "DMX-In",              /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    _rdm_task_mode = 0;
    
    resetFrame();
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
}

void LX32DMX1::startRDM ( uint8_t dirpin, uint8_t inpin, uint8_t outpin, uint8_t direction ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	pinMode(dirpin, OUTPUT);
	_direction_pin = dirpin;
	digitalWrite(_direction_pin, HIGH);//disable input until setup
	
	LXSerial1.begin(250000, SERIAL_8N2, inpin, outpin);
	LXSerial1.enableBreakDetect();

	_continue_task = 1;
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    rdmTask,         /* Function that implements the task. */
                    "DMX-Task",         /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle);
            
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

void LX32DMX1::stop ( void ) {
	_continue_task = 0;
	while ( _task_active ) {
		vTaskDelay(1);
	}
	
	_xHandle = NULL;
	LXSerial1.end();
}

// call from task loop!!
void LX32DMX1::sendRawRDMPacketImmediately( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;
	
	digitalWrite(_direction_pin, HIGH);
	
	LXSerial1.sendBreak(88);					// uses hardwareSerialDelayMicroseconds
	hardwareSerialDelayMicroseconds(12);
	
	LXSerial1.write(ESP32DMX1.rdmData(), ESP32DMX1.rdmPacketLength());
	LXSerial1.waitFIFOEmpty();
	LXSerial1.waitTXDone();
	digitalWrite(_direction_pin, LOW);
}

void LX32DMX1::sendRDMDiscoverBranchResponse( void ) {
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
	
	digitalWrite(_direction_pin, HIGH);
	
	LXSerial1.write(&_rdmPacket[1], 24);	// note no start code [0]
	LXSerial1.waitFIFOEmpty();
	LXSerial1.waitTXDone();
	digitalWrite(_direction_pin, LOW);
}

