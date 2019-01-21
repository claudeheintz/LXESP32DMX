/**************************************************************************/
/*!
    @file     LXHardwareSerial.h
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    Exposes functionality in HardwareSerial class for LXESP32DMX driver

    @section  HISTORY

    v1.0 - First release
    v1.1 - Added functions here to simplify required modifications to esp32-hal-uart.c
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
#include "esp_intr.h"
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
	void waitRxFIFOEmpty();
	void waitTXDone();
	void waitTXBrkDone();
    void sendBreak(uint32_t length);
	void setBaudRate(uint32_t rate);
	void configureRS485(uint8_t en);
	void configureSendBreak(uint8_t en, uint8_t len, uint16_t idle);
	void setToTwoStopBits();
	void enableBreakDetect();
	void disableBreakDetect();
	void clearInterrupts();
	void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false);
	
	private:
	uint8_t _tx_gpio_pin;
};

void uartWaitFIFOEmpty(uart_t* uart);
void uartWaitRxFIFOEmpty(uart_t* uart);
void uartWaitTXDone(uart_t* uart);
void uartWaitTXBrkDone(uart_t* uart);
void uartConfigureRS485(uart_t* uart, uint8_t en);
void uartConfigureSendBreak(uart_t* uart, uint8_t en, uint8_t len, uint16_t idle);
void uartSetToTwoStopBits(uart_t* uart);
void uartEnableBreakDetect(uart_t* uart);
void uartDisableBreakDetect(uart_t* uart);
void uartDisableInterrupts(uart_t* uart);
void uartSetInterrupts(uart_t* uart, uint32_t value);
void uartClearInterrupts(uart_t* uart);

void hardwareSerialDelayMicroseconds(uint32_t us);

#endif