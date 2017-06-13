/**************************************************************************/
/*!
    @file     LXHardwareSerial.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    Exposes functionality in HardwareSerial class for LXESP32DMX driver

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "LXHardwareSerial.h"

// uart_struct_t is also defined in esp32-hal-uart.c
// defined here because to access fields in uart_t* in the following mods
struct uart_struct_t {
    uart_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    xQueueHandle queue;
};

// wait for FIFO to be empty
void uartWaitFIFOEmpty(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
	
    while(uart->dev->status.txfifo_cnt != 0x00) {
    	taskYIELD();
    }
}

void uartWaitTXDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    while (uart->dev->int_raw.tx_done == 0) {
		taskYIELD();
	}
	
}

void uartWaitTXBrkDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    while (uart->dev->int_raw.tx_brk_done == 0) {
		taskYIELD();
	}
	
}

void uartConfigureRS485(uart_t* uart, uint8_t en) {
	uart->dev->rs485_conf.en = en;
}

void uartConfigureSendBreak(uart_t* uart, uint8_t en, uint8_t len, uint16_t idle) {
	uart->dev->conf0.txd_brk = en;
	uart->dev->idle_conf.tx_brk_num=len;
	uart->dev->idle_conf.tx_idle_num = idle;
}


LXHardwareSerial::LXHardwareSerial(int uart_nr):HardwareSerial(uart_nr) {}

void LXHardwareSerial::waitFIFOEmpty() {
	uartWaitFIFOEmpty(_uart);
}

void LXHardwareSerial::waitTXDone() {
	uartWaitTXDone(_uart);
}

void LXHardwareSerial::waitTXBrkDone() {
	uartWaitTXBrkDone(_uart);
}

void LXHardwareSerial::sendBreak(uint32_t length) {
	uint8_t txPin;
	if( _uart_nr == 1 ) {
        txPin = 10;
    } else if( _uart_nr == 2 ) {
        txPin = 17;
    } else {
    	txPin = 1;
    }
    
    // detach 
    pinMatrixOutDetach(txPin, false, false);
  
  	pinMode(txPin, OUTPUT);
  	digitalWrite(txPin, LOW);
  	delayMicroseconds(length);
  	digitalWrite(txPin, HIGH);
  	
  	//reattach
  	pinMatrixOutAttach(txPin, U2TXD_OUT_IDX, false, false);
}

void LXHardwareSerial::setBaudRate(uint32_t rate) {
	uartSetBaudRate(_uart, rate);
}

void LXHardwareSerial::configureRS485(uint8_t en) {
	uartConfigureRS485(_uart, en);
}

void LXHardwareSerial::configureSendBreak(uint8_t en, uint8_t len, uint16_t idle) {
	uartConfigureSendBreak(_uart, en, len, idle);
}

