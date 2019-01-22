/* LXESP32DMX1.h
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

   This file (and LXESP32DMX1.cpp) should be copied to your sketch folder
   to enable support for a second DMX serial output object, ESP32DMX1.
   
   @section  HISTORY
    v1.0 - Added January 2019 as example of second DMX output
   
 */


#ifndef LX32DMX1_H
#define LX32DMX1_H

#include <Arduino.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "LXHardwareSerial.h"
#include "UID.h"
#include "LXESP32DMX.h"


/*!   
@class LX32DMX1
@abstract
   LX32DMX1 is a driver for sending or receiving DMX using an ESP32's UART2.
   
   LX32DMX1 output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   DMX output uses the ESP32's UART2 TX pin (GPIO17).
   
   LX32DMX1 input mode receives DMX using the ESP32's UART2 RX pin (GPIO16)
   LX32DMX1 continuously updates its DMX buffer once its interrupts have been enabled using startInput()
   and DMX data is received by UART2.
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LX32DMX1 input requires replacing the uart hardware abstraction files to allow breaks to be converted
   to SLIP encoded serial to divide the incoming DMX serial into packets.
   
   LX32DMX1 is used with a single instance called ESP32DMX1.
*/

class LX32DMX1 : public LX32DMX {

  public:
  
	LX32DMX1 ( void );
   ~LX32DMX1( void );
    
   /*!
    * @brief starts task that continuously sends DMX output
    * @discussion begins serial TX connection.
    			  Creates task that continuously sends dmx data.
   */
   void startOutput( uint8_t pin=17 );
   
   /*!
    * @brief starts task that continuously reads DMX data
    * @discussion begins serial RX connection.
    *             Creates task that reads bytes from RX queue.
   */
   void startInput( uint8_t pin=16 );
   
   /*!
    * @brief starts task that continuously reads and optionally sends DMX data.
    * @discussion Creates task that continuously sends and/or receives dmx data.
    *			  IMPORTANT note: switching direction requires use of direction pin.
   */
   void startRDM ( uint8_t dirpin, uint8_t inpin=16, uint8_t outpin=17, uint8_t direction=1 );
   
   /*!
    * @brief disables transmission and tx interrupt
   */
	void stop( void );
	
	
	/*!
    * @brief sends packet immediately, writes directly to Serial2 without using task loop
    */
	void sendRawRDMPacketImmediately( uint8_t len );
	
    
    /*!
    * @brief send response to RDM_DISC_UNIQUE_BRANCH packet
    */
    void sendRDMDiscoverBranchResponse( void );
    
  	
};

extern LX32DMX1 ESP32DMX1;


#endif // ifndef LX32DMX1_H