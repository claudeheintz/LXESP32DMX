/**************************************************************************/
/*!
    @file     LXESP32DMX.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017-2022 by Claude Heintz

    DMX Driver for ESP32

    @section  HISTORY

    v1.0 - First release
    v1.2 - improve multi-task compatibility
    v2.x - rewrite for ESP32 SDK 2.0.2
	v3.0 - Updated, see .h file		   

*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "LXESP32DMX.h"
#include "rdm_utility.h"

// disconnected pin definition
#define NO_PIN -1

// Set to use UART based hardware DMX Break
#define HARDWARE_DMX_BREAK 1

#define SEMAPHORE_TIMEOUT 50
#define SEMAPHORE_TIMEOUT_STATEMENT (TickType_t)(portTICK_PERIOD_MS * SEMAPHORE_TIMEOUT)

#if DO_NO_CREATE_DEFAULT_LXESP32DMX_CLASS_OBJECT != 1
  LX32DMX ESP32DMX;
#endif
UID LX32DMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x02, 0x01);

/*
 * sendDMX is run by a task
 * it loops forever continuously sending DMX packets
 * until  ESP32DMX.continueTask() returns false
 *
 * Note ALT_DMX_SEND switches between two strategies for sending
 *      Yet to be determined, the pros and cons...
 */
void LX32DMX::send_dmx_task( void * param )
{ 
	LX32DMX* dmx = (LX32DMX*)param;
    dmx->setActiveTask(TASK_IS_ACTIVE);
  
  while (dmx->continueTask() )
  {
#if HARDWARE_DMX_BREAK == 1
	// ********** alt?
	/*
	 * Timing test results
	 * no mark after slot 512
	 * 176µs break
	 * 115µs MAB @hsdm(24), 61µs @hsdm(10), 41µs @hsdm(5)
	 */
	//does not return until after break when tx_fifo_size == 0 see HardwareSerial begin()/instaLL_Driver
	  if (xSemaphoreTake(dmx->lxDataLock, SEMAPHORE_TIMEOUT_STATEMENT) == pdTRUE)
	  {
		  dmx->pLXSerial->writeBytesWithBreak(dmx->dmxData(), dmx->numberOfSlots() + 1);
		  xSemaphoreGive(dmx->lxDataLock);
		  hardwareSerialDelayMicroseconds(5);// minimum MAB 12µs (see timing test above)
	  }
	// ********** end alt?
#else
	/*
	 * Timing test results
	 * 281ms mark after slot 512
	 * .585ms break
	 * 57.75µs MAB
	 */

	dmx->pLXSerial->sendBreak(150);
    hardwareSerialDelayMicroseconds(12);
    
	if (xSemaphoreTake(dmx->lxDataLock, SEMAPHORE_TIMEOUT_STATEMENT) == pdTRUE)
	{
		dmx->pLXSerial->write(dmx->dmxData(), dmx->numberOfSlots() + 1);
		xSemaphoreGive(dmx->lxDataLock);
		//vTaskDelay must be called to avoid wdt and lock up issues
		vTaskDelay(5);					//use time while UART finishes sending to allow other tasks to run
		dmx->pLXSerial->waitFIFOEmpty();		//returns at about byte 384 ~128 bytes left 128 * 44 = 5.6 ms
		dmx->pLXSerial->waitTXDone();
		// break at end is actually for next packet...
		dmx->setDMXPacketSent(1);
	}
#endif
    
  }	//while
  
  // signal task end and wait for task to be deleted
  dmx->setActiveTask(TASK_IS_INACTIVE);
  vTaskDelete( NULL );	// delete this task
}

void LX32DMX::read_queue_task(void *param)
{
	LX32DMX* dmx = (LX32DMX*)param;
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(DMX_MAX_FRAME);	//probably can get away with 120 bytes since that seems to be event.size
	dmx->initial_break = false;
    int rx;
    dmx->setActiveTask(TASK_IS_ACTIVE);
    
    while ( dmx->continueTask() ) {
        //Waiting for UART event.
        if ( xQueueReceive(dmx->uart_queue, (void * )&event, SEMAPHORE_TIMEOUT_STATEMENT) ) {
            switch(event.type) {
            	case UART_DATA:
            		if (dmx->initial_break ) {
            			rx = dmx->handleQueueData(event.size);
            			if ( rx >= DMX_MAX_SLOTS ) {
							dmx->handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
            		} else {
            			rx = dmx->pLXSerial->readBytes(dtmp, event.size, portMAX_DELAY);	//drain fifo (result is ignored)
            		}
            		break;
            	case UART_BREAK:
            		rx = dmx->handleQueueBreak();	//read remaining slots up to MAX_FRAME
            											//slots 480-512 may not read every cycle
					dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
                    
                    //wait until after fifo reset to process completed packet
                    if ( dmx->initial_break ) {
                    	if ( rx > 0 ) {
                    		dmx->handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
                    } else {
						dmx->initial_break = true;
					}
					dmx->handleResetQueuePacket();
            		break;
            	case UART_FIFO_OVF:
					dmx->pLXSerial->clearFIFOOverflow();
					dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
            		Serial.println("fifo over");
					dmx->initial_break = false;
            		break;
            	default:					// error?
					dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
            		Serial.print("other event ");
            		Serial.println(event.type);
					dmx->initial_break = false;
					break;
            }	// switch
        }		// x q recv
	} 			//while
	
	// signal task end and wait for task to be deleted
	dmx->setActiveTask(TASK_IS_INACTIVE);
	free(dtmp);
	vTaskDelete( NULL );	// delete this task
}

/*
 * received_input_task loops while ESP32DMX.continueTask() is true
 * it checks a flag to see if input has been received and
 * calls the appropriate handler function if it has.
 */

void LX32DMX::received_input_task(void *param)
{
	LX32DMX* dmx = (LX32DMX*)param;
	dmx->setInputReceivedTaskActive(TASK_IS_ACTIVE);

	 while (dmx->continueTask() ) {
	 
	 	switch (dmx->receiveTaskStatus() ) {
	 		case RECEIVE_STATUS_DMX:
				dmx->handleQueueDMXDataReceived();
				dmx->setInputReceivedTaskStatus(RECEIVE_STATUS_NONE);
	 			break;
	 		case RECEIVE_STATUS_RDM:
				dmx->handleQueueRDMDataReceived();
				dmx->setInputReceivedTaskStatus(RECEIVE_STATUS_NONE);
	 		default:
	 			vTaskDelay(10);
	 	}	 
	 }

	 dmx->setInputReceivedTaskActive(TASK_IS_INACTIVE);
	vTaskDelete( NULL );	// delete this task
}

void LX32DMX::rdm_queue_task(void *param)
{
	LX32DMX* dmx = (LX32DMX*)param;
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(DMX_MAX_FRAME);	//probably can get away with 120 bytes since that seems to be event.size
	dmx->initial_break = false;
    int rx;
    dmx->setActiveTask(TASK_IS_ACTIVE);
    uint8_t task_mode;
    TickType_t twait;
    
    
    while ( dmx->continueTask() ) {
		task_mode = dmx->rdmTaskMode();
		if ( task_mode  ) {
			twait = 10;
		} else {
			twait = 1000;
		}

        //Waiting for UART event.
        if ( xQueueReceive(dmx->uart_queue, (void * )&event, twait) ) {
            switch(event.type) {
            	case UART_DATA:
            		if (dmx->initial_break) {
            			rx = dmx->handleQueueData(event.size);
            			if ( rx >= DMX_MAX_SLOTS ) {
							dmx->handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
            		} else {
            			rx = dmx->pLXSerial->readBytes(dtmp, event.size, portMAX_DELAY);	//drain fifo (result is ignored)
            		}
            		break;
            	case UART_BREAK:
            		rx = dmx->handleQueueBreak();	//read remaining slots up to MAX_FRAME
            		
            		dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
                    
                    //wait until after fifo reset to process completed packet
                    if (dmx->initial_break) {
                    	if ( rx > 0 ) {
                    		dmx->handleQueuePacketComplete(); // only copies buffer and sets input flag!
						}
                    } else {
						dmx->initial_break = true;
					}
					dmx->handleResetQueuePacket();
            		break;
            	case UART_FIFO_OVF:
            		dmx->pLXSerial->clearFIFOOverflow();
            		dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
            		Serial.println("fifo over");
					dmx->initial_break = false;
            		break;
            	default:					// error?
            		dmx->pLXSerial->flushInput();
                    xQueueReset(dmx->uart_queue);
            		Serial.print("other event ");
            		Serial.println(event.type);
					dmx->initial_break = false;
            		
            
            }	        // switch
        } else {		// xQueueReceive() returned false, nothing on input

			if ( task_mode  ) {
				if ( task_mode == DMX_TASK_SEND_RDM ) {		
				
				   dmx->pLXSerial->sendBreak(150);					// uses hardwareSerialDelayMicroseconds
				   hardwareSerialDelayMicroseconds(12);
			   
				   dmx->pLXSerial->write(dmx->rdmData(), dmx->rdmPacketLength());		// data should be set from function that sets flag to get here
																						// therefore data should not need to be protected by Mutex
					dmx->pLXSerial->waitFIFOEmpty();
					dmx->pLXSerial->waitTXDone();
					
					/*  hardware write (break follows)
					dmx->pLXSerial->writeBytesWithBreak(dmx->rdmData(), dmx->rdmPacketLength());
					hardwareSerialDelayMicroseconds(1);
					*/
					
					dmx->_setTaskReceiveRDM();
				
				} else {									// otherwise send regular DMX
					/*
				   dmx->pLXSerial->sendBreak(150);					// uses hardwareSerialDelayMicroseconds
				   hardwareSerialDelayMicroseconds(12);
			       xSemaphoreTake( dmx->lxDataLock, portMAX_DELAY );
				   dmx->pLXSerial->write(dmx->dmxData(), dmx->numberOfSlots()+1);		// data should be set from function that sets flag to get here
				   xSemaphoreGive( dmx->lxDataLock );																// therefore data should not need to be protected by Mutex
																				
					dmx->pLXSerial->waitFIFOEmpty();
					dmx->pLXSerial->waitTXDone();	
					*/
					//   hardware write (break follows)
					if (xSemaphoreTake(dmx->lxDataLock, SEMAPHORE_TIMEOUT_STATEMENT) == pdTRUE)
					{
						//xSemaphoreTake( dmx->lxDataLock, portMAX_DELAY );
						dmx->pLXSerial->writeBytesWithBreak(dmx->dmxData(), dmx->numberOfSlots() + 1);
						xSemaphoreGive(dmx->lxDataLock);
						hardwareSerialDelayMicroseconds(1);
						if (task_mode == DMX_TASK_SET_SEND)
						{
							dmx->_setTaskModeSend();
						}
					}
				}
			}
			else
			{
				vTaskDelay(1);	// receiving but nothing in the queue right now, try again...
			}
        }		// else (xQueueReceive() was false)
	} 			// while...main loop
	
	// signal task end and wait for task to be deleted
	dmx->setActiveTask(TASK_IS_INACTIVE);
	free(dtmp);
	vTaskDelete( NULL );	// delete this task
}

/*
 * rdmTask is run by an task with idle priority
 * loops forever until task is ended
 
static void rdmTask( void * param ) {

  uint8_t task_mode;
  int c;
  ESP32DMX.setActiveTask(1);
  
  while ( ESP32DMX.continueTask() ) {
  
	c = dmx->pLXSerial->read();
	
	if ( c >= 0 ) {								// if byte read, receive it...may invoke callback function
		//ESP32DMX.byteReceived(c&0xff);			// eventually will catch up, nothing to read and vTaskDelay will be called
	} else {
		task_mode = ESP32DMX.rdmTaskMode();
		if ( task_mode  ) {
			if ( task_mode == DMX_TASK_SEND_RDM ) {
			
			   dmx->pLXSerial->sendBreak(150);					// uses hardwareSerialDelayMicroseconds
			   hardwareSerialDelayMicroseconds(12);
			   
			   dmx->pLXSerial->write(ESP32DMX.rdmData(), ESP32DMX.rdmPacketLength());		// data should be set from function that sets flag to get here
																					// therefore data should not need to be protected by Mutex
																					
				//vTaskDelay(1);            //   <-should not be needed here because rdm should be interleaved with regular NSC/DMX		
											//   vTaskDelay should be called next time though this loop
											//   ( at least next time bytes are not read or task_mode is send )	
				dmx->pLXSerial->waitFIFOEmpty();
				dmx->pLXSerial->waitTXDone();
				ESP32DMX._setTaskReceiveRDM();
				
				
			} else {									// otherwise send regular DMX
				dmx->pLXSerial->sendBreak(150);					// send break first (uses hardwareSerialDelayMicroseconds)
				hardwareSerialDelayMicroseconds(12);    // <- insure no conflict on another thread/task
			
				xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );
				dmx->pLXSerial->write(ESP32DMX.dmxData(), ESP32DMX.numberOfSlots()+1);
				xSemaphoreGive( ESP32DMX.lxDataLock );
														//vTaskDelay must be called to avoid wdt and lock up issues
				vTaskDelay(5);							//use time while UART finishes sending to allow other tasks to run
				dmx->pLXSerial->waitFIFOEmpty();				//returns at about byte 384 ~128 bytes left 128 * 44 = 5.6 ms
				dmx->pLXSerial->waitTXDone();
				if ( task_mode == DMX_TASK_SET_SEND ) {
					ESP32DMX._setTaskModeSend();
				}
			}
		} else {
		    vTaskDelay(1);	// receiving but nothing to read, assumes buffer can be emptied faster than it fills
		}

	}		//nothing read
  } 		//while
  
  // signal task end and wait for task to be deleted
  ESP32DMX.setActiveTask(0);
  
  vTaskDelete( NULL );	// delete this task
}
*/


/*******************************************************************************
 ***********************  LX32DMX member functions  ********************/

LX32DMX::LX32DMX ( const uint8_t UARTID ) {
	pLXSerial = new LXHardwareSerial(UARTID);	//must be initialized before ESP32DMX
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

LX32DMX::~LX32DMX ( void ){
    stop();
    _receive_callback = NULL;
    _rdm_receive_callback = NULL;
	delete pLXSerial;
}

void LX32DMX::startOutput ( uint8_t pin, UBaseType_t priorityOverIdle, uint8_t AssignToCore)
{
	if ( _xHandle != NULL ) {
		stop();
	}

	if (AssignToCore > SOC_CPU_CORES_NUM - 1)
		AssignToCore = SOC_CPU_CORES_NUM - 1;
	
	setDMXPacketSent(0);
	
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	this->pLXSerial->begin(250000, SERIAL_8N2, NO_PIN, pin);
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreatePinnedToCore(
                    send_dmx_task,      /* Function that implements the task. */
                    "DMX-Out",          /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+priorityOverIdle,   /* Priority at which the task is created. */
                    &_xHandle,
					AssignToCore);
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;				// task create failed
    }
}

void LX32DMX::startInput ( uint8_t pin , UBaseType_t priorityOverIdle, uint8_t AssignToCore)
{
	if ( _xHandle != NULL ) {
		stop();
	}

	if (AssignToCore > SOC_CPU_CORES_NUM - 1)
		AssignToCore = SOC_CPU_CORES_NUM - 1;
	
	// disable input --if direction pin is used-- until setup complete
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	
	this->pLXSerial->begin(250000, SERIAL_8N2, pin, NO_PIN, false, 20000UL, 64, 513, &uart_queue);
	this->pLXSerial->enableBreakDetect();
	
	/********** make the rx queue task **********/
	
	_continue_task = 1;					// flag for task loop
	BaseType_t xReturned;
  	xReturned = xTaskCreatePinnedToCore(
                    read_queue_task,         /* Function that implements the task. */
                    "DMX-In",              /* Text name for the task. */
                    8192,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+priorityOverIdle,   /* Priority at which the task is created. */
                    &_xHandle, 
					AssignToCore);
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
    
    /********** make the input received task **********/
    
    xReturned = xTaskCreatePinnedToCore(
                    received_input_task,  /* Function that implements the task. */
                    "Input",              /* Text name for the task. */
                    8192,                 /* Stack size in words, not bytes. */
                    this,                 /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,     /* Priority at which the task is created. */			
                    &_xRecvHandle,
					AssignToCore);

    if( xReturned != pdPASS ) {
        _xRecvHandle = NULL;
    }

    _rdm_task_mode = 0;
    
    resetFrame();
    if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
}

void LX32DMX::startRDM ( uint8_t dirpin, uint8_t inpin, uint8_t outpin, uint8_t direction, UBaseType_t priorityOverIdle , uint8_t AssignToCore)
{
	if ( _xHandle != NULL ) {
		stop();
	}

	if (AssignToCore > SOC_CPU_CORES_NUM - 1)
		AssignToCore = SOC_CPU_CORES_NUM - 1;
	
	pinMode(dirpin, OUTPUT);
	_direction_pin = dirpin;
	digitalWrite(_direction_pin, HIGH);//disable input until setup
	
	this->pLXSerial->begin(250000, SERIAL_8N2, inpin, outpin, false, 20000UL, 64, 513, &uart_queue);
	this->pLXSerial->enableBreakDetect();
	
	/********** make the rx queue task **********/

	_continue_task = 1;
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

void LX32DMX::stop ( void ) {
	_continue_task = 0;
	
	while ( _received_task_active ) {
		vTaskDelay(1);
	}
	
	while ( _task_active ) {
		vTaskDelay(1);
	}
	
	_xHandle = NULL;	// task is deleted when while(_continue_task){...} ends
	_xRecvHandle = NULL;
	this->pLXSerial->end();
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

void LX32DMX::setInputReceivedTaskActive(uint8_t s) {
	_received_task_active = s;
}

int LX32DMX::receiveTaskStatus( void ) {
	return _received_task_status;
}

void LX32DMX::setInputReceivedTaskStatus(uint8_t s) {
	_received_task_status = s;
}

/*
void LX32DMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == ALLOW_DATA_HANDLING ) {
		    if ( _current_slot > DMX_MIN_SLOTS ) {
				_slots = _current_slot - 1;				//_current_slot represents next slot so subtract one
				
				xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );	// double buffer makes task conflict unlikely, but use mutex anyway
				for(int j=0; j<_current_slot; j++) {	//copy dmx values from read buffer
					_dmxData[j] = _receivedData[j];
				}
				xSemaphoreGive( ESP32DMX.lxDataLock );
	
				if ( _receive_callback != NULL ) {
					_receive_callback(_slots);
				}
			}
		}
	} else {
		if ( _receivedData[0] == RDM_START_CODE ) {
			if ( validateRDMPacket(_receivedData) ) {
				uint8_t plen = _receivedData[2] + 2;
				for(int j=0; j<plen; j++) {
					_rdmData[j] = _receivedData[j];
				}
				if ( _receive_callback != NULL ) {
					_rdm_receive_callback(plen);
				}
			}
		} else {
			Serial.println("________________ unknown data packet ________________");
			printReceivedData();
		}
	}
	resetFrame();
	vTaskDelay(1);	//rest and allow other tasks to run (can catch up reading later);
}
*/

// note _dmx_state is probably obsolete and can be removed

void LX32DMX::resetFrame( void ) {		
	_dmx_state = DMX_STATE_IDLE;						// insure wait for next break
	initial_break = false;
}

/*
void LX32DMX::addReceivedByte(uint8_t value) {
	if ( _current_slot == 0 ) {
		if ( _rdm_read_handled ) {
			if ( value == 0 ) {
				return;				// ignore leading zero in RDM reply
			}
		}
	}
	_receivedData[_current_slot] = value;
	if ( _current_slot == 2 ) {						//RDM length slot
		if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
			if ( _rdm_read_handled == ALLOW_DATA_HANDLING ) {
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
*/
int LX32DMX::handleQueueData(int esize) {
	if ( (_current_slot + esize) > DMX_MAX_FRAME ) {
		esize = DMX_MAX_FRAME - _current_slot;
	}
	if ( esize > 0 ) {
		int rbytes = this->pLXSerial->readBytes(&_receivedData[_current_slot], esize, portMAX_DELAY);
		_current_slot += rbytes;
	}
	return _current_slot;
}

int LX32DMX::handleQueueBreak( void ) {
	// read out remaining bytes up tp DMX_MAX_FRAME
	int rbytes = this->pLXSerial->available();
	if ( rbytes ) {
		handleQueueData(rbytes);
	}
	return _current_slot;
}

void LX32DMX::handleQueuePacketComplete( void ) {
	int i = 0;
	if ( _rdm_read_handled == DATA_HANDLED_BY_RDM_FUNCTION ) {	//ignore leading zero if DATA_HANDLED_BY_RDM_FUNCTION
		while (( _receivedData[i] == 0 ) && ( i < 10 )) {
			i++;
		}
	}
	if ( _receivedData[i] == 0 ) {
		if ( _rdm_read_handled == ALLOW_DATA_HANDLING ) {
			if ( _current_slot > DMX_MIN_SLOTS ) {
				if (xSemaphoreTake(this->lxDataLock, SEMAPHORE_TIMEOUT_STATEMENT) == pdTRUE)
				{
					//xSemaphoreTake(this->lxDataLock, portMAX_DELAY );	// double buffer makes task conflict unlikely, but use mutex anyway
					memcpy(_dmxData, _receivedData, DMX_MAX_FRAME);
					xSemaphoreGive(this->lxDataLock);
					_slots = _current_slot;
					setInputReceivedTaskStatus(RECEIVE_STATUS_DMX);
				}
			}
		}
	} else if ( _receivedData[i] == RDM_START_CODE ) {
		if ( validateRDMPacket(&_receivedData[i]) ) {
			if (xSemaphoreTake(this->lxDataLock, SEMAPHORE_TIMEOUT_STATEMENT) == pdTRUE)
			{
				uint8_t plen = _receivedData[i + 2] + 2;
				xSemaphoreGive(this->lxDataLock);
				for (int j = 0; j < plen; j++) {
					_rdmData[j] = _receivedData[j + i];
				}
				setInputReceivedTaskStatus(RECEIVE_STATUS_RDM);
			}
		}
	}
	_current_slot = 0;
}

void LX32DMX::handleResetQueuePacket( void ) {
	_current_slot = 0;
}


void LX32DMX::handleQueueDMXDataReceived( void ) {
	if ( _receive_callback != NULL ) {
		_receive_callback(_slots);
	}
}

void LX32DMX::handleQueueRDMDataReceived( void ) {

	// NOTE probably needs internal use handler for discovery and other responses

	if ( _receive_callback != NULL ) {
		uint8_t plen = _rdmData[2] + 2;
		_rdm_receive_callback(plen);
	}
}

/*
void LX32DMX::byteReceived(uint8_t c) {
	if ( _rdm_task_mode ) {				// this prevents a stray read from setting slots
		//Serial.println("warning byteReceived in send mode");
										// strange that bytes will be received when driver
		return;							// chip remains low(?)
	}

	if ( c == SLIP_END ) {			//break received
		if ( _dmx_state == DMX_STATE_RECEIVING ) {			// break has already been detected
			if ( _current_slot > 1 ) {						// break before end of maximum frame
				if ( _receivedData[0] == 0 ) {				// zero start code is DMX
					packetComplete();						// packet terminated with slots<512
				}
			}
		}
		_dmx_state = DMX_STATE_RECEIVING;
		
		if ( _rdm_read_handled == ALLOW_DATA_HANDLING ) {		//only reset if not directly handling read?
			_current_slot = 0;				//changed 2/16/18
		}
		_packet_length = DMX_MAX_FRAME;		// default to receive complete frame
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
*/

void LX32DMX::printReceivedData( void ) {
	for(int j=0; j<_current_slot; j++) {
		Serial.println(_receivedData[j]);
	}
}

void LX32DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

void LX32DMX::setDMXPacketSent( uint8_t ps) {
	_dmx_sent = ps;
}

uint8_t LX32DMX::dmxPacketSent() {
	return _dmx_sent;
}

/************************************ RDM Methods **************************************/

void LX32DMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}

uint8_t LX32DMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
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
    _rdm_read_handled = ALLOW_DATA_HANDLING;
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
		vTaskDelay(2);
	}
	
}

// call from task loop!!
void LX32DMX::sendRawRDMPacketImmediately( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;
	
	digitalWrite(_direction_pin, HIGH);
	
	this->pLXSerial->sendBreak(88);					// uses hardwareSerialDelayMicroseconds
	hardwareSerialDelayMicroseconds(12);
	
	this->pLXSerial->write(this->rdmData(), this->rdmPacketLength());
	this->pLXSerial->waitFIFOEmpty();
	this->pLXSerial->waitTXDone();
	digitalWrite(_direction_pin, LOW);
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
	
	_rdm_read_handled = DATA_HANDLED_BY_RDM_FUNCTION;
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
			if ( _current_slot == (j + 18) ) { //preamble separator plus 16 byte payload + 1
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
		
		_rdm_read_handled = ALLOW_DATA_HANDLING;
		resetFrame();
	} else {
		_rdm_read_handled = ALLOW_DATA_HANDLING;
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

	_rdm_read_handled = DATA_HANDLED_BY_RDM_FUNCTION;
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
		
		_rdm_read_handled = ALLOW_DATA_HANDLING;
		resetFrame();
	} else {
		_rdm_read_handled = ALLOW_DATA_HANDLING;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LX32DMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = DATA_HANDLED_BY_RDM_FUNCTION;
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
		_rdm_read_handled = ALLOW_DATA_HANDLING;
		resetFrame();
	} else {
		_rdm_read_handled = ALLOW_DATA_HANDLING;
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

// device methods should only be called via rdm callback
// sendRawRDMPacketImmediately and sendRDMDiscoverBranchResponse write to Serial without changing task mode
// to do this, they use a semaphore to halt the task loop

void LX32DMX::sendRDMGetResponse(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN+len;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND_RESPONSE, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	sendRawRDMPacketImmediately(plen+2);	//add 2 bytes for checksum
}

void LX32DMX::sendAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x00);
	
	sendRawRDMPacketImmediately(plen+2);	//add 2 bytes for checksum
}

void LX32DMX::sendMuteAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN + 2;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x02);
	
	sendRawRDMPacketImmediately(plen+2);	//add 2 bytes for checksum
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
	
	digitalWrite(_direction_pin, HIGH);
	
	this->pLXSerial->write(&_rdmPacket[1], 24);	// note no start code [0]
	this->pLXSerial->waitFIFOEmpty();
	this->pLXSerial->waitTXDone();
	digitalWrite(_direction_pin, LOW);
}

