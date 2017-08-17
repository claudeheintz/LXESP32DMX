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
#include "LXESP32DMX.h"

LXHardwareSerial Serial2(2);	//must be initialized before ESP32DMX
LX32DMX ESP32DMX;

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
  	yield();
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
  		taskYIELD();
  	}
  }
  
  // signal task end and wait for task to be deleted
  ESP32DMX.setActiveTask(0);
  while( true ) {
  	yield();
  }
}


/*******************************************************************************
 ***********************  LX32DMX member functions  ********************/

LX32DMX::LX32DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_xHandle = NULL;
	clearSlots();
	_task_active = 0;
	_continue_task = 0;
}

LX32DMX::~LX32DMX ( void ) {
    stop();
    _receive_callback = NULL;
}

void LX32DMX::startOutput ( uint8_t pin ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _xHandle != NULL ) {
		stop();
	}
	
	Serial2.begin(250000, SERIAL_8N2, NO_PIN, pin);
#if (DMX_GPIO_BREAK == 0)
	Serial2.configureSendBreak(TX_BRK_ENABLE, DMX_TX_BRK_LENGTH, DMX_TX_IDLE_LENGTH);
#endif

	_continue_task = 1;
	//Serial2.configureRS485(1);
	BaseType_t xReturned;

  	xReturned = xTaskCreate(
                    sendDMX,            /* Function that implements the task. */
                    "DMX-Out",              /* Text name for the task. */
                    1024,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
    }
}

void LX32DMX::startInput ( uint8_t pin ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _xHandle != NULL ) {
		stop();
	}
	
	Serial2.begin(250000, SERIAL_8N2, pin, NO_PIN);
	Serial2.enableBreakDetect();
	
	_continue_task = 1;
	BaseType_t xReturned;

  	xReturned = xTaskCreate(
                    receiveDMX,         /* Function that implements the task. */
                    "DMX-In",              /* Text name for the task. */
                    1024,               /* Stack size in words, not bytes. */
                    this,               /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,   /* Priority at which the task is created. */
                    &_xHandle );
            
    if( xReturned != pdPASS ) {
        _xHandle = NULL;
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

void LX32DMX::setCurrentSlot(uint8_t value) {
	_dmxData[_current_slot] = value;
	_current_slot++;
	if ( _current_slot >= DMX_MAX_FRAME ) {
		frameReceived();
		_dmx_state = DMX_STATE_IDLE;
	}
}

void LX32DMX::clearSlots (void) {
	memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}

uint8_t* LX32DMX::dmxData(void) {
	return _dmxData;
}

uint8_t LX32DMX::continueTask() {
	return _continue_task;
}

void LX32DMX::setActiveTask(uint8_t s) {
	_task_active = s;
}

void LX32DMX::frameReceived( void ) {
	_slots = _current_slot - 1;	//_current_slot represents next slot so subtract one
	_current_slot = 0;
	if ( _receive_callback != NULL ) {
		_receive_callback(_slots);
	}
}

void LX32DMX::byteReceived(uint8_t c) {
	if ( c == SLIP_END ) {			//break received
		if ( _dmx_state == DMX_STATE_RECEIVING ) {
			//break before end of maximum frame
			frameReceived();
		}
		_dmx_state = DMX_STATE_RECEIVING;
		_current_slot = 0;
	} else {
		if ( _dmx_state == DMX_STATE_RECEIVING ) {
			if ( c == SLIP_ESC ) {
				_dmx_state = DMX_STATE_ESC;
			} else {
				setCurrentSlot(c);
			}
		} else if ( _dmx_state == DMX_STATE_ESC ) {
			_dmx_state = DMX_STATE_RECEIVING;
			if ( c == SLIP_ESC_END ) {
				setCurrentSlot(SLIP_END);
			} else if ( c == SLIP_ESC_ESC ) {
				setCurrentSlot(SLIP_ESC);
			}
		}
	}
	
}

void LX32DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

