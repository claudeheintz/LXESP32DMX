/**************************************************************************/
/*!
    @file     LXHardwareSerial.h
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    Exposes functionality in HardwareSerial class for LXESP32DMX driver

    @section  HISTORY

    v1.0 - First release
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


class LXHardwareSerial : public HardwareSerial {
	public:
	LXHardwareSerial(int uart_nr);
	
	void waitFIFOEmpty();
	void waitTXDone();
	void waitTXBrkDone();
    void sendBreak(uint32_t length);
	void setBaudRate(uint32_t rate);
	void configureRS485(uint8_t en);
	void configureSendBreak(uint8_t en, uint8_t len, uint16_t idle);
};

void uartWaitFIFOEmpty(uart_t* uart);
void uartWaitTXDone(uart_t* uart);
void uartWaitTXBrkDone(uart_t* uart);
void uartConfigureRS485(uart_t* uart, uint8_t en);
void uartConfigureSendBreak(uart_t* uart, uint8_t en, uint8_t len, uint16_t idle);
