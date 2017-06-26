/* LXESP32DMX.h
   Copyright 2017 by Claude Heintz Design

Copyright (c) 2017, Claude Heintz
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXESP32DMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------------------

   The LXESP32DMX library supports output and input of DMX using the UART
   serial output of an ESP32 microcontroller.  LXESP32DMX uses
   UART2for output.  This means that hardware Serial
   can still be used for USB communication.
   (do not use Serial1 because it is connected to flash)
   
   Input functionality requires replacing the esp32-hal-uart.c
   files in Arduino/hardware/espressif/esp32/cores/esp32/
   
   The necessary modified esp32-hal-uart file is included in the
   extras/modified_hal-uart folder of this library.
   Or use the forked Arduino Core at https://github.com/claudeheintz/arduino-esp32
   
   @section  HISTORY
   v1.0	- first release
   v1.1 - simplifies modifications to esp32-hal-uart.c required for DMX input
 */


#ifndef LX32DMX_H
#define LX32MX_H

#include <Arduino.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "LXHardwareSerial.h"

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513

#define DIRECTION_PIN_NOT_USED 255

 //***** states indicate current position in DMX stream
    #define DMX_STATE_IDLE 0
    #define DMX_STATE_RECEIVING 1
    #define DMX_STATE_ESC 2
    
typedef void (*LXRecvCallback)(int);

//Special Byte Definitions for SLIP Encoding
#define SLIP_END      0xC0
#define SLIP_ESC      0xDB
#define SLIP_ESC_ESC  0xDC
#define SLIP_ESC_END  0xDD

/*!   
@class LX32DMX
@abstract
   LX32DMX is a driver for sending or receiving DMX using an ESP32's UART2.
   
   LX32DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   DMX output uses the ESP32's UART2 TX pin (GPIO17).
   
   LX32DMX input mode receives DMX using the ESP32's UART2 RX pin (GPIO16)
   LX32DMX continuously updates its DMX buffer once its interrupts have been enabled using startInput()
   and DMX data is received by UART2.
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LX32DMX input requires replacing the uart hardware abstraction files to allow breaks to be converted
   to SLIP encoded serial to divide the incoming DMX serial into packets.
   
   LX32DMX is used with a single instance called ESP32DMX.
*/

class LX32DMX {

  public:
  
	LX32DMX ( void );
   ~LX32DMX( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt.
   */
   void startOutput( uint8_t pin=17 );
   
   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt
   */
   void startInput( uint8_t pin=16 );
   
   /*!
    * @brief disables transmission and tx interrupt
   */
	void stop( void );
	
	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.     
    * @param pin to be automatically set for input/output direction
    */
   void setDirectionPin( uint8_t pin );
   
   /*!
    * @brief the current number of slots
   */
   uint16_t numberOfSlots (void);
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	
	/*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.  
    *                   So a complete single frame is not guaranteed.  
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);
   
   /*!
	 * @brief Sets the output value of a slot  Note:slot[0] is DMX start code!
	 * @param slot number of the slot aka address or channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   
   /*!
	 * @brief when reading sets the current slot and advances the counter
	 * @param value level (0-255)
	*/
   void setCurrentSlot(uint8_t value);
   
   /*!
    * @brief zero buffer including slot[0] which is start code
   */
   void clearSlots ( void );
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData( void );
   
   /*!
    * @brief dmx frame received, call DataReceivedCallback function, if set.
   */
   void frameReceived( void );
   
   /*!
    * @brief called from read task with next character from serial
   */
   void byteReceived(uint8_t c);

  	
  	/*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
   
    
  private:

   /*!
   * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
   */
  	uint8_t  _dmx_state;
  	
  	/*!
   * @brief pin used to control direction of output driver chip
   */
  	uint8_t _direction_pin;
  	
   /*!
   * @brief number of dmx slots ~24 to 512
   */
  	uint16_t  _current_slot;
  	
   /*!
   * @brief number of dmx slots ~24 to 512
   */
  	uint16_t  _slots;
  	
   /*!
   * @brief Array of dmx data including start code
   */
  	uint8_t  _dmxData[DMX_MAX_FRAME];
  	
  	
	/*!
    * @brief send/receive task
   */
  	TaskHandle_t _xHandle;
  	
  	/*!
    * @brief Pointer to receive callback function
   */
  	LXRecvCallback _receive_callback;
  	
};

extern LX32DMX ESP32DMX;

#endif // ifndef LX32DMX_H