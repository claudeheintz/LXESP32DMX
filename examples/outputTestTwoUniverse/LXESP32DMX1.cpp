/**************************************************************************/
/*!
    @file     LXESP32DMX1.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX1.h)
    @copyright 2017 by Claude Heintz

    DMX Driver for ESP32

    @section  HISTORY
    
    This file (and LXESP32DMX1.h) should be copied to your sketch folder
   to enable support for a second DMX serial output object, ESP32DMX1.

    v1.0 - Added January 2019 as example of second DMX output
    v2.x - Reworked for SDK v2.0.2
*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "LXESP32DMX1.h"
#include "rdm_utility.h"

LXHardwareSerial LXSerial1(1);	//must be initialized before ESP32DMX
LX32DMX1 ESP32DMX1;


static QueueHandle_t uart_queue1;

//global flag
bool initial_break1 = false;

// disconnected pin definition
#define NO_PIN -1

/*
 * sendDMX is run by a task
 * it loops forever continuously sending DMX packets
 * until  ESP32DMX1.continueTask() returns false
 *
 * Note ALT_DMX_SEND switches between two strategies for sending
 *      Yet to be determined, the pros and cons...
 */
static void send_dmx_task( void * param ) {
  
  ESP32DMX1.setActiveTask(TASK_IS_ACTIVE);
  
  while ( ESP32DMX1.continueTask() ) {

#define ALT_DMX_SEND 0
#if ALT_DMX_SEND == 1
	// ********** alt?
	//does not return until after break when tx_fifo_size == 0 see HardwareSerial begin()/instaLL_Driver
	LXSerial1.writeBytesWithBreak(ESP32DMX1.dmxData(), ESP32DMX1.numberOfSlots()+1);
	
	// minimum MAB 12µs
	hardwareSerialDelayMicroseconds(24);
	// ********** end alt?
#else	
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
    ESP32DMX1.setDMXPacketSent(1);
#endif
    
  }	//while
  
  // signal task end and wait for task to be deleted
  ESP32DMX1.setActiveTask(TASK_IS_INACTIVE);
  vTaskDelete( NULL );	// delete this task
}

static void read_queue_task(void *param) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(DMX_MAX_FRAME);	//probably can get away with 120 bytes since that seems to be event.size
    initial_break1 = false;
    int rx;
    ESP32DMX1.setActiveTask(TASK_IS_ACTIVE);
    
    while ( ESP32DMX1.continueTask() ) {
        //Waiting for UART event.
        if ( xQueueReceive(uart_queue1, (void * )&event, (portTickType)portMAX_DELAY) ) {
            switch(event.type) {
            	case UART_DATA:
            		if ( initial_break1 ) {
            			rx = ESP32DMX1.handleQueueData(event.size);
            			if ( rx >= DMX_MAX_SLOTS ) {
							ESP32DMX1.handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
            		} else {
            			rx = LXSerial1.readBytes(dtmp, event.size, portMAX_DELAY);	//drain fifo (result is ignored)
            		}
            		break;
            	case UART_BREAK:
            		rx = ESP32DMX1.handleQueueBreak();	//read remaining slots up to MAX_FRAME
            											//slots 480-512 may not read every cycle
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
                    
                    //wait until after fifo reset to process completed packet
                    if ( initial_break1 ) {
                    	if ( rx > 0 ) {
                    		ESP32DMX1.handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
                    } else {
                    	initial_break1 = true;
					}
					ESP32DMX1.handleResetQueuePacket();
            		break;
            	case UART_FIFO_OVF:
            		LXSerial1.clearFIFOOverflow();
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
            		Serial.println("fifo over");
            		initial_break1 = false;
            		break;
            	default:					// error?
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
            		Serial.print("other event ");
            		Serial.println(event.type);
            		initial_break1 = false;
            		
            
            }	// switch
        }		// x q recv
	} 			//while
	
	// signal task end and wait for task to be deleted
	ESP32DMX1.setActiveTask(TASK_IS_INACTIVE);
	free(dtmp);
	vTaskDelete( NULL );	// delete this task
}

/*
 * received_input_task loops while ESP32DMX1.continueTask() is true
 * it checks a flag to see if input has been received and
 * calls the appropriate handler function if it has.
 */

static void received_input_task(void *param) {

	ESP32DMX1.setInputReceivedTaskActive(TASK_IS_ACTIVE);

	 while ( ESP32DMX1.continueTask() ) {
	 
	 	switch ( ESP32DMX1.receiveTaskStatus() ) {
	 		case RECEIVE_STATUS_DMX:
	 			ESP32DMX1.handleQueueDMXDataReceived();
	 			ESP32DMX1.setInputReceivedTaskStatus(RECEIVE_STATUS_NONE);
	 			break;
	 		case RECEIVE_STATUS_RDM:
	 			ESP32DMX1.handleQueueRDMDataReceived();
	 			ESP32DMX1.setInputReceivedTaskStatus(RECEIVE_STATUS_NONE);
	 		default:
	 			vTaskDelay(10);
	 	}
	 
	 }

	ESP32DMX1.setInputReceivedTaskActive(TASK_IS_INACTIVE);
	vTaskDelete( NULL );	// delete this task
}

static void rdm_queue_task(void *param) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(DMX_MAX_FRAME);	//probably can get away with 120 bytes since that seems to be event.size
    initial_break1 = false;
    int rx;
    ESP32DMX1.setActiveTask(TASK_IS_ACTIVE);
    uint8_t task_mode;
    TickType_t twait;
    
    
    while ( ESP32DMX1.continueTask() ) {
		task_mode = ESP32DMX1.rdmTaskMode();
		if ( task_mode  ) {
			twait = 10;
		} else {
			twait = 1000;
		}

        //Waiting for UART event.
        if ( xQueueReceive(uart_queue1, (void * )&event, twait) ) {
            switch(event.type) {
            	case UART_DATA:
            		if ( initial_break1 ) {
            			rx = ESP32DMX1.handleQueueData(event.size);
            			if ( rx >= DMX_MAX_SLOTS ) {
							ESP32DMX1.handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
            		} else {
            			rx = LXSerial1.readBytes(dtmp, event.size, portMAX_DELAY);	//drain fifo (result is ignored)
            		}
            		break;
            	case UART_BREAK:
            		rx = ESP32DMX1.handleQueueBreak();	//read remaining slots up to MAX_FRAME
            		
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
                    
                    //wait until after fifo reset to process completed packet
                    if ( initial_break1 ) {
                    	if ( rx > 0 ) {
                    		ESP32DMX1.handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
                    } else {
                    	initial_break1 = true;
					}
					ESP32DMX1.handleResetQueuePacket();
            		break;
            	case UART_FIFO_OVF:
            		LXSerial1.clearFIFOOverflow();
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
            		Serial.println("fifo over");
            		initial_break1 = false;
            		break;
            	default:					// error?
            		LXSerial1.flushInput();
                    xQueueReset(uart_queue1);
            		Serial.print("other event ");
            		Serial.println(event.type);
            		initial_break1 = false;
            		
            
            }	        // switch
        } else {		// xQueueReceive() returned false, nothing on input

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
				vTaskDelay(1);	// receiving but nothing in the queue right now, try again...
			}

        }		// else (xQueueReceive() was false)
	} 			// while...main loop
	
	// signal task end and wait for task to be deleted
	ESP32DMX1.setActiveTask(TASK_IS_INACTIVE);
	free(dtmp);
	vTaskDelete( NULL );	// delete this task
}


/*******************************************************************************
 ***********************  LX32DMX1 member functions  ********************/

LX32DMX1::LX32DMX1 ( void ) {
    lxDataLock = xSemaphoreCreateMutex();
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_xHandle = NULL;
	_xRecvHandle = NULL;
	_receive_callback = NULL;
	_rdm_receive_callback = NULL;
	clearSlots();
	_task_active = TASK_IS_INACTIVE;
	_received_task_active = TASK_IS_INACTIVE;
	_received_task_status = RECEIVE_STATUS_NONE;
	_continue_task = 0;
	_rdm_task_mode = 0;
}

LX32DMX1::~LX32DMX1 ( void ) {
    stop();
    _receive_callback = NULL;
    _rdm_receive_callback = NULL;
}

void LX32DMX1::startOutput ( uint8_t pin, UBaseType_t priorityOverIdle ) {
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
                    send_dmx_task,      /* Function that implements the task. */
                    "DMX-Out",          /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1, /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
}

void LX32DMX1::startInput ( uint8_t pin, UBaseType_t priorityOverIdle) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	// disable input --if direction pin is used-- until setup complete
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	LXSerial1.begin(250000, SERIAL_8N2, pin, NO_PIN, false, 20000UL, 64, 513, &uart_queue1);
	LXSerial1.enableBreakDetect();

	/********** make the rx queue task **********/
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    read_queue_task,         /* Function that implements the task. */
                    "DMX-In",              /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    
    
    /********** make the input received task **********/
    
    xReturned = xTaskCreate(
                    received_input_task,  /* Function that implements the task. */
                    "Input",              /* Text name for the task. */
                    8192,                 /* Stack size in words, not bytes. */
                    this,                 /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,     /* Priority at which the task is created. */
                    &_xRecvHandle );
                     
            
    if( xReturned != pdPASS ) {
        _xRecvHandle = NULL;
    }
    _rdm_task_mode = 0;
    
    resetFrame();
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
}

void LX32DMX1::startRDM ( uint8_t dirpin, uint8_t inpin, uint8_t outpin, uint8_t direction, UBaseType_t priorityOverIdle ) {
	if ( _xHandle != NULL ) {
		stop();
	}
	
	pinMode(dirpin, OUTPUT);
	_direction_pin = dirpin;
	digitalWrite(_direction_pin, HIGH);//disable input until setup
	
	LXSerial1.begin(250000, SERIAL_8N2, inpin, outpin, false, 20000UL, 64, 513, &uart_queue1);
	LXSerial1.enableBreakDetect();

	_continue_task = 1;
	
	/********** make the rx queue task **********/

	BaseType_t xReturned;
  	xReturned = xTaskCreate(
                    rdm_queue_task,    /* Function that implements the task. */
                    "RDM-DMX-Task",         /* Text name for the task. */
                    8192,                   /* Stack size in words, not bytes. */
                    this,                   /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+priorityOverIdle,   /* Priority at which the task is created. */
                    &_xHandle);
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    
     /********** make the input received task **********/
    
    xReturned = xTaskCreate(
                    received_input_task,  /* Function that implements the task. */
                    "Input",              /* Text name for the task. */
                    8192,                 /* Stack size in words, not bytes. */
                    this,                 /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,     /* Priority at which the task is created. */
                    &_xRecvHandle );
            
    if( xReturned != pdPASS ) {
        _xRecvHandle = NULL;
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

