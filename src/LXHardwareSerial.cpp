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

#if CONFIG_DISABLE_HAL_LOCKS
#define UART_MUTEX_LOCK()
#define UART_MUTEX_UNLOCK()
#else
#define UART_MUTEX_LOCK()    do {} while (xSemaphoreTake(uart->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK()  xSemaphoreGive(uart->lock)
#endif

// wait for FIFO to be empty
void IRAM_ATTR uartWaitFIFOEmpty(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
	
    while(uart->dev->status.txfifo_cnt != 0x00) {
    	taskYIELD();
    }
}

void IRAM_ATTR uartWaitTXDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    while (uart->dev->int_raw.tx_done == 0) {
		taskYIELD();
	}
	uart->dev->int_clr.tx_done = 1;
}

void IRAM_ATTR uartWaitTXBrkDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    while (uart->dev->int_raw.tx_brk_idle_done == 0) {
		taskYIELD();
	}
	
	uart->dev->int_clr.tx_brk_idle_done = 1;
}

void uartConfigureRS485(uart_t* uart, uint8_t en) {
	UART_MUTEX_LOCK();
	uart->dev->rs485_conf.en = en;
	UART_MUTEX_UNLOCK();
}

void uartConfigureSendBreak(uart_t* uart, uint8_t en, uint8_t len, uint16_t idle) {
	UART_MUTEX_LOCK();
	uart->dev->conf0.txd_brk = en;
	uart->dev->idle_conf.tx_brk_num=len;
	uart->dev->idle_conf.tx_idle_num = idle;
	UART_MUTEX_UNLOCK();
}

// ****************************************************
//			uartConfigureTwoStopBits
//          known issue with two stop bits now handled in IDF and Arduino Core
//          see
//				https://github.com/espressif/esp-idf/blob/master/components/driver/uart.c
//				uart_set_stop_bits(), lines 118-127
//          see also
//				https://esp32.com/viewtopic.php?f=2&t=1431

void uartSetToTwoStopBits(uart_t* uart) {
	UART_MUTEX_LOCK();
	uart->dev->conf0.stop_bit_num = 1;
	uart->dev->rs485_conf.dl1_en = 1;
	UART_MUTEX_UNLOCK();
}

void uartEnableBreakDetect(uart_t* uart) {
	UART_MUTEX_LOCK();
	uart->dev->int_ena.brk_det = 1;
    uart->dev->conf1.rxfifo_full_thrhd = 1;
    uart->dev->auto_baud.val = 0;
    UART_MUTEX_UNLOCK();
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

void LXHardwareSerial::setToTwoStopBits() {
	uartSetToTwoStopBits(_uart);
}

void LXHardwareSerial::enableBreakDetect() {
	uartEnableBreakDetect(_uart);
}

