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
#include "freertos/portmacro.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"

// uart_struct_t is also defined in esp32-hal-uart.c
// defined here because to access fields in uart_t* in the following mods
struct uart_struct_t {
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    xQueueHandle queue;
    intr_handle_t intr_handle;
};

#if CONFIG_DISABLE_HAL_LOCKS
#define UART_MUTEX_LOCK()
#define UART_MUTEX_UNLOCK()
#else
#define UART_MUTEX_LOCK()    do {} while (xSemaphoreTake(uart->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK()  xSemaphoreGive(uart->lock)
#warning CONFIG_DISABLE_HAL_LOCKS is false...
#endif


uint8_t testCtr = 0;

/***************************** privateDelayMicroseconds allows sendBreak to be called on non-main task/thread 
*
* reason for private function is
* portENTER_CRITICAL_ISR has a spin lock
* delayMicroseconds() can only be called on main loop 
* because micros() is called on main loop task
*
*/

portMUX_TYPE privateMicrosMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long IRAM_ATTR privateMicros()
{
    static unsigned long lccount = 0;     //because this depends of these static variables does each task need a private micros()/delayMicroseconds()?
    static unsigned long overflow = 0;
    unsigned long ccount;
    portENTER_CRITICAL_ISR(&privateMicrosMux);
    __asm__ __volatile__ ( "rsr     %0, ccount" : "=a" (ccount) );
    if(ccount < lccount){
        overflow += UINT32_MAX / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
    }
    lccount = ccount;
    portEXIT_CRITICAL_ISR(&privateMicrosMux);
    return overflow + (ccount / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
}

void IRAM_ATTR hardwareSerialDelayMicroseconds(uint32_t us) {
	for(int k=0; k<us; k++) {	// approximate delay, not a very portable solution
		taskYIELD();			// this may or may not allow another task to run
	}
// ccount appears to be CPU dependent and task might switch CPUs making Micros invalid
/*
    uint32_t m = privateMicros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(privateMicros() > e){
                NOP();
            }
        }
        while(privateMicros() < e){
            NOP();
        }
    }*/
}

// *****************************

// repeat these defines from HardwareSerial.cpp
#ifndef SOC_RX0
#if CONFIG_IDF_TARGET_ESP32
#define SOC_RX0 3
#elif CONFIG_IDF_TARGET_ESP32S2
#define SOC_RX0 44
#elif CONFIG_IDF_TARGET_ESP32C3
#define SOC_RX0 20
#endif
#endif

#ifndef SOC_TX0
#if CONFIG_IDF_TARGET_ESP32
#define SOC_TX0 1
#elif CONFIG_IDF_TARGET_ESP32S2
#define SOC_TX0 43
#elif CONFIG_IDF_TARGET_ESP32C3
#define SOC_TX0 21
#endif
#endif

#if SOC_UART_NUM > 1

#ifndef RX1
#if CONFIG_IDF_TARGET_ESP32
#define RX1 9
#elif CONFIG_IDF_TARGET_ESP32S2
#define RX1 18
#elif CONFIG_IDF_TARGET_ESP32C3
#define RX1 18
#endif
#endif

#ifndef TX1
#if CONFIG_IDF_TARGET_ESP32
#define TX1 10
#elif CONFIG_IDF_TARGET_ESP32S2
#define TX1 17
#elif CONFIG_IDF_TARGET_ESP32C3
#define TX1 19
#endif
#endif

#if SOC_UART_NUM > 2
#ifndef RX2
#if CONFIG_IDF_TARGET_ESP32
#define RX2 16
#endif
#endif

#ifndef TX2
#if CONFIG_IDF_TARGET_ESP32
#define TX2 17
#endif
#endif

#endif
#endif


// wait for FIFO to be empty
void IRAM_ATTR uartWaitFIFOEmpty(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
	uart_wait_tx_done(uart->num, 20000);
}

void IRAM_ATTR uartWaitRxFIFOEmpty(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
	
	uint16_t timeout = 0;
    while(uart->dev->status.rxfifo_cnt != 0x00) {
        timeout++;
        if ( timeout > 20000 ) {
        	break;
        }
        hardwareSerialDelayMicroseconds(10);
    }
}

void IRAM_ATTR uartWaitTXDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    uint16_t timeout = 0;
    while (uart->dev->int_raw.tx_done == 0) {
        timeout++;
        if ( timeout > 20000 ) {
        	break;
        }
        hardwareSerialDelayMicroseconds(10);
	}
	uart->dev->int_clr.tx_done = 1;
}

void IRAM_ATTR uartWaitTXBrkDone(uart_t* uart) {
	if ( uart == NULL ) {
		return;
	}
    
    while (uart->dev->int_raw.tx_brk_idle_done == 0) {
        hardwareSerialDelayMicroseconds(10);
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
	uart_set_stop_bits(uart->num, 2);
}

void uartEnableBreakDetect(uart_t* uart) {
	UART_MUTEX_LOCK();
	
	uart->dev->int_ena.brk_det = 1;
    uart->dev->conf1.rxfifo_full_thrhd = 1;
    uart->dev->auto_baud.val = 0;
    UART_MUTEX_UNLOCK();
}

void uartDisableBreakDetect(uart_t* uart) {
	UART_MUTEX_LOCK();
	uart->dev->int_ena.brk_det = 0;
    UART_MUTEX_UNLOCK();
}

void uartDisableInterrupts(uart_t* uart) {
    UART_MUTEX_LOCK();
    //uart->dev->conf1.val = 0;
    uart->dev->int_ena.val = 0;
    uart->dev->int_clr.val = 0xffffffff;
    UART_MUTEX_UNLOCK();
}

void uartSetInterrupts(uart_t* uart, uint32_t value) {
	UART_MUTEX_LOCK();
	uart->dev->int_ena.val = value;
	UART_MUTEX_UNLOCK();
}

void uartClearInterrupts(uart_t* uart) {
    UART_MUTEX_LOCK();
    uart->dev->int_clr.val = 0xffffffff;
    UART_MUTEX_UNLOCK();
}

void uartLockMUTEX(uart_t* uart) {
    UART_MUTEX_LOCK();
}

void uartUnlockMUTEX(uart_t* uart) {
    UART_MUTEX_UNLOCK();
}

uint32_t _get_effective_baudrate(uint32_t baudrate) 
{
    uint32_t Freq = getApbFrequency()/1000000;
    if (Freq < 80) {
        return 80 / Freq * baudrate;
     }
    else {
        return baudrate;
    }
}

uart_t* uartQueueBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, uint16_t queueLen, bool inverted, uint8_t rxfifo_full_thrhd, int qSize, QueueHandle_t* q)
{
    if(uart_nr >= SOC_UART_NUM) {
        return NULL;
    }

    if(rxPin == -1 && txPin == -1) {
        return NULL;
    }
    
    
	/* use regular uartBegin() to get pointer to uart
	 * wasteful, perhaps, but it appears that there's no other way
	 * to access _uart_bus_array which is static in esp32-hal-uart.c
	 * the struct is not destroyed with uartEnd(uart)
	 */
    uart_t* uart = uartBegin(uart_nr, baudrate, config, rxPin, txPin, queueLen, inverted, rxfifo_full_thrhd);
    
    /* continue with copy of regular uartBegin() until uart_driver_install */

    if (uart_is_driver_installed(uart_nr)) {	
        uartEnd(uart);   //we just installed a driver and now we will uninstall it
    }

#if !CONFIG_DISABLE_HAL_LOCKS
    if(uart->lock == NULL) {
        uart->lock = xSemaphoreCreateMutex();
        if(uart->lock == NULL) {
            return NULL;
        }
    }
#endif

    UART_MUTEX_LOCK();

    uart_config_t uart_config;
    uart_config.baud_rate = _get_effective_baudrate(baudrate);
    uart_config.data_bits = (uart_word_length_t)((config & 0xc) >> 2);
    uart_config.parity = (uart_parity_t)(config & 0x3);
    uart_config.stop_bits = (uart_stop_bits_t)((config & 0x30) >> 4);
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = rxfifo_full_thrhd;
    uart_config.source_clk = UART_SCLK_APB;

/* call to uart_driver_install is where to modify to create queue...
   uart_driver_install
      (
      uart_port_t uart_num,
      int rx_buffer_size,
      int tx_buffer_size,
   -> int queue_size,
   -> QueueHandle_t* uart_queue,
      int intr_alloc_flags
      )
      
      note when tx_buffer_size == 0, write will block until all data is sent
*/

    ESP_ERROR_CHECK(uart_driver_install(uart_nr, 2*queueLen, 0, qSize, q, 0));
    
/****** continue with regular uartBegin() ******/
    
    ESP_ERROR_CHECK(uart_param_config(uart_nr, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_nr, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Is it right or the idea is to swap rx and tx pins? 
    if (inverted) {
        // invert signal for both Rx and Tx
        ESP_ERROR_CHECK(uart_set_line_inverse(uart_nr, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));    
    }

    // Set RS485 half duplex mode on UART.  This shall force flush to wait up to sending all bits out
    ESP_ERROR_CHECK(uart_set_mode(uart_nr, UART_MODE_RS485_HALF_DUPLEX));

    UART_MUTEX_UNLOCK();

    uartFlush(uart);
    
    return uart;
}

LXHardwareSerial::LXHardwareSerial(int uart_nr):HardwareSerial(uart_nr) {}

void LXHardwareSerial::end() {
	uartDisableInterrupts(_uart);
	
    if(uartGetDebug() == _uart_nr) {
        uartSetDebug(0);
    }
    //uartEnd(_uart, _rx_gpio_pin, _tx_gpio_pin);		//Arduino esp32 1.0.5
    uartEnd(_uart);								        // 1.0.4, 2.0.1
    _uart = 0;
}

void LXHardwareSerial::waitFIFOEmpty() {
	uartWaitFIFOEmpty(_uart);
}

void LXHardwareSerial::waitRxFIFOEmpty() {
	uartWaitRxFIFOEmpty(_uart);
}

void LXHardwareSerial::waitTXDone() {
	uartWaitTXDone(_uart);
}

void LXHardwareSerial::waitTXBrkDone() {
	uartWaitTXBrkDone(_uart);
}

void LXHardwareSerial::sendBreak(uint32_t length) {
	uint8_t gpioSig;
	if( _uart_nr == 1 ) {
        gpioSig = U1TXD_OUT_IDX;
    } else if( _uart_nr == 2 ) {
        gpioSig = U2TXD_OUT_IDX;
    } else {
    	gpioSig = U0TXD_OUT_IDX;
    }
    
    //uint32_t save_interrupts = _uart->dev->int_ena.val;
    //uartDisableInterrupts(_uart);
    esp_intr_disable(_uart->intr_handle);
    
    // detach 
    pinMatrixOutDetach(_tx_gpio_pin, false, false);
  	pinMode(_tx_gpio_pin, OUTPUT);
  	
  	digitalWrite(_tx_gpio_pin, LOW);
  	hardwareSerialDelayMicroseconds(length);
  	digitalWrite(_tx_gpio_pin, HIGH);
  	
  	//reattach
  	pinMatrixOutAttach(_tx_gpio_pin, gpioSig, false, false);
  	
  	esp_intr_enable(_uart->intr_handle);
  	//uartSetInterrupts(_uart, save_interrupts);
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

void LXHardwareSerial::disableBreakDetect() {
	uartDisableBreakDetect(_uart);
}

void LXHardwareSerial::clearInterrupts() {
	uartClearInterrupts(_uart);
}

void LXHardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd, int qSize, QueueHandle_t* q) {
	_tx_gpio_pin = txPin;
	_rx_gpio_pin = rxPin;
	
	Serial.print("test uart");
	Serial.println((int)_uart);
	
	// SDK 2.0.2 replace call to super begin() with version installing queue
	HardwareSerial::begin(baud, config, rxPin, txPin, invert, timeout_ms, rxfifo_full_thrhd);
	
	Serial.println((int)_uart->lock);

/*
	if(0 > _uart_nr || _uart_nr >= SOC_UART_NUM) {
        log_e("Serial number is invalid, please use numers from 0 to %u", SOC_UART_NUM - 1);
        return;
    }
    if(_uart) {
        // in this case it is a begin() over a previous begin() - maybe to change baud rate
        // thus do not disable debug output
        HardwareSerial::end(false);
    }
    if(_uart_nr == 0 && rxPin < 0 && txPin < 0) {
        rxPin = SOC_RX0;
        txPin = SOC_TX0;
    }
#if SOC_UART_NUM > 1
    if(_uart_nr == 1 && rxPin < 0 && txPin < 0) {
        rxPin = RX1;
        txPin = TX1;
    }
#endif
#if SOC_UART_NUM > 2
    if(_uart_nr == 2 && rxPin < 0 && txPin < 0) {
        rxPin = RX2;
        txPin = TX2;
    }
#endif

    _uart = uartBegin(_uart_nr, baud ? baud : 9600, config, rxPin, txPin, _rxBufferSize, invert, rxfifo_full_thrhd);
    
    //NOTE does not detect Baud like HardwareSerial.begin()
*/
	
}
	
