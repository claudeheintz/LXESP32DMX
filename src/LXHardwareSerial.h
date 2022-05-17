/**************************************************************************/
/*!
    @file     LXHardwareSerial.h
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017-2022 by Claude Heintz

    Exposes functionality in HardwareSerial class for LXESP32DMX driver

    @section  HISTORY

    v1.0 - First release
    v1.1 - Added functions here to simplify required modifications to esp32-hal-uart.c
    v2.x - rewrite for ESP32 SDK 2.0.2
*/
/**************************************************************************/

#include "HardwareSerial.h"
#include "esp32-hal-uart.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
// for 2.0.0_rc
// #include "esp_intr_alloc.h"
// for 1.0.6 and 1.0.5
// #include "esp_intr.h"		
#include "rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"

#ifndef LXHardwareSerial_H
#define LXHardwareSerial_H

class LXHardwareSerial : public HardwareSerial {
	public:
	LXHardwareSerial(int uart_nr);
	
	void end();
	
	void waitFIFOEmpty();
	void waitTXDone();
    void sendBreak(uint32_t length);
    void writeBytesWithBreak(const void* src, size_t size);
    int  readBytes(void* buf, uint32_t length, TickType_t ticks_to_wait);
	void setBaudRate(uint32_t rate);
	void setToTwoStopBits();
	void enableBreakDetect(void);
	void disableBreakDetect();
	void clearInterrupts();
	void clearFIFOOverflow();
	void flushInput();
	
	//void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false);
	void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = 112, int qSize = 0, QueueHandle_t* q = NULL);
	
	private:
	uint8_t _tx_gpio_pin;
	uint8_t _rx_gpio_pin;
};

void uartWaitFIFOEmpty(uart_t* uart);
void uartDisableInterrupts(uart_t* uart);
void uartSetInterrupts(uart_t* uart, uint32_t value);

uart_t* uartQueueBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, uint16_t rx_buf_sz, uint16_t tx_buf_sz, bool inverted, uint8_t rxfifo_full_thrhd, int qSize, QueueHandle_t* q);

void hardwareSerialDelayMicroseconds(uint32_t us);

#endif