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
   v1.2 - improve multi-task compatibility
   
 */


#ifndef LX32DMX_H
#define LX32DMX_H

#include <Arduino.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "LXHardwareSerial.h"
#include "UID.h"

#define DMX_MIN_SLOTS 24
#define RDM_MAX_FRAME 257
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513

#define DIRECTION_PIN_NOT_USED 255

 //***** states indicate current position in DMX stream
    #define DMX_STATE_IDLE 0
    #define DMX_STATE_RECEIVING 1
    #define DMX_STATE_ESC 2
 
#define DMX_TASK_RECEIVE	0   
#define DMX_TASK_SEND		1
#define DMX_TASK_SEND_RDM	2
#define DMX_TASK_SET_SEND	3

#define RDM_NO_DISCOVERY		0
#define RDM_PARTIAL_DISCOVERY	1
#define RDM_DID_DISCOVER		2
    
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
    * @brief starts task that continuously sends DMX output
    * @discussion begins serial TX connection.
    			  Creates task that continuously sends dmx data.
   */
   void startOutput( uint8_t pin=17, UBaseType_t priorityOverIdle=1 );
   
   /*!
    * @brief starts task that continuously reads DMX data
    * @discussion begins serial RX connection.
    *             Creates task that reads bytes from RX queue.
    *
    *			  Warning: Calls HardwareSerial::begin which flushes input queue
    *					   until empty.
    *                      If serial data is present (DMX stream on input pin)
    *					   when startInput is called, flush and therefore
    *					   HardwareSerial::begin can hang...
   */
   void startInput( uint8_t pin=16, UBaseType_t priorityOverIdle=1 );
   
   /*!
    * @brief starts task that continuously reads and optionally sends DMX data.
    * @discussion Creates task that continuously sends and/or receives dmx data.
    *			  IMPORTANT note: switching direction requires use of direction pin.
    *
    *			  Warning: Calls HardwareSerial::begin which flushes input queue
    *					   until empty.
    *                      If serial data is present (DMX stream on input pin)
    *					   when startInput is called, flush and therefore
    *					   HardwareSerial::begin can hang...
   */
   void startRDM ( uint8_t dirpin,
   				   uint8_t inpin=16,
   				   uint8_t outpin=17,
   				   uint8_t direction=1,
   				   UBaseType_t priorityOverIdle=1);
   
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
    * @brief zero buffer including slot[0] which is start code
   */
   void clearSlots ( void );
   
   /*!
    * @brief mutex exposed as public member variable
   */
   SemaphoreHandle_t lxDataLock;
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData( void );
   
   /*!
    * @brief provides direct access to data array
    * @discussion this is used for sending RDM while
    *             receivedRDMData() is used for reading RDM
    * @return pointer to dmx array
   */
   uint8_t* rdmData( void );
   
   /*!
    * @brief provides direct access to received data array
    * @discussion receivedData is copied into either dmxData or receivedRDMData
    *             when a complete frame is read.
    *             (Exception:  RDM responses to controller packets
    *                          are handled by the sending methods.)
    * @return pointer to read buffer array
   */
   uint8_t* receivedData( void );
   
   /*!
    * @brief provides direct access to received rdm data array
    * @discussion this is used for receiving RDM while
    *             rdmData() is used for sending RDM
    * @return pointer to rdm data array
   */
   uint8_t* receivedRDMData( void );
   
	/*!
    * @brief indicate if the loop of an I/O task should continue
    * @return 0 if task should not repeat (and end)
   */
	uint8_t continueTask( void );
	
	/*!
    * @brief set a flag indicating that a task is looping
   */
	void setActiveTask(uint8_t s);
	
   /*!
    * @brief dmx frame received, call DataReceivedCallback function, if set.
   */
   void packetComplete( void );
   
   /*!
    * @brief called to reset input to wait for next break;
   */
   void resetFrame( void );
   
	/*!
	 * @brief when reading sets the current slot and advances the counter
	 * @param value level (0-255)
	 */
   void addReceivedByte(uint8_t value);
   
   /*!
    * @brief called from read task with next character from serial
   */
   void byteReceived(uint8_t c);
   
   /*!
    * @brief utility that prints receive buffer to Serial
   */
   void printReceivedData( void );
  	
  	/*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
    */
   void setDataReceivedCallback(LXRecvCallback callback);
   
   
   /*!
    * @brief Set flag indicating a dmx packet has been sent.
   */
   void setDMXPacketSent( uint8_t ps );
   
   /*!
    * @brief Flag indicating if a dmx packet has been sent.
   */
   uint8_t dmxPacketSent();
   
   /************************************ RDM Methods ***********************************/
   
   /*!
    * @brief Function called when RDM frame has been read
    * @discussion Sets a pointer to a function that is called
    *             after an RDM frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
    */
   void setRDMReceivedCallback(LXRecvCallback callback);
   
   	/*!
    * @brief indicate if dmx frame should be sent by bi-directional task loop
    * @discussion should only be called by task loop
    * @return 1 if dmx frame should be sent
    *  return 2 if RDM should be sent
    *  return 3 if RDM should be sent and set mode to 1 after first frame finished
    */
	uint8_t rdmTaskMode( void );
	
	/*!
    * @brief sets rdm task to receive mode
    * @discussion should only be called by task loop
    *             read assumes response packet starts immediately
    *             and would only be called if rxfifo is empty
    *             so task loop would send a raw controller packet
    */
	void _setTaskReceiveRDM( void );
	
	/*!
    * @brief sets rdm task to send mode and the direction pin to HIGH
	*/
	void setTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to send mode.  Should only be called by task loop.
	*/
	void _setTaskModeSend( void );
	
	
	/*!
    * @brief sets rdm task to send mode after task mode loops.
    *        Sent after sending RDM message so DMX is resumed.
    *        Blocks until task loop sets mode to send.
	*/
	void restoreTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to receive mode
    *        Prepares variables to receive starting with next break.
    *        Sets the direction pin to LOW.
	*/
	void setTaskReceive( void );
	
	
	/*!
    * @brief length of the rdm packet awaiting being sent
	*/
	uint8_t rdmPacketLength( void );
	
	/*!
    * @brief sends packet using bytes from _rdmPacket ( rdmData() )
    * @discussion sets rdm task mode to DMX_TASK_SEND_RDM which causes
    *             _rdmPacket to be sent on next opportunity from task loop.
    *             after _rdmPacket is sent, task mode switches to listen for response.
    *
    *             set _rdm_read_handled flag prior to calling sendRawRDMPacket
    *             _rdm_read_handled = 1 if reading is handled by calling function
    *             _rdm_read_handled = 0 if desired to resume passive listening for next break
    */
	void sendRawRDMPacket( uint8_t len );
	/*!
    * @brief sends packet immediately, writes directly to Serial2 without using task loop
    */
	void sendRawRDMPacketImmediately( uint8_t len );
	/*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to constant THIS_DEVICE_ID below.
	*/
	void setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice);
	
	/*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to static member THIS_DEVICE_ID
	*/
	void  setupRDMDevicePacket(uint8_t* pdata, uint8_t msglen, uint8_t rtype, uint8_t msgs, uint16_t subdevice);
	
	/*!
    * @brief convenience method for setting fields in the top bytes 20-23 of an RDM message
    *        that will be sent.
	*/
	void setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl);
	
	/*!
    * @brief send discovery packet using upper and lower bounds
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if discovered, 2 if valid packet (UID stored in uldata[12-17])
    */
    uint8_t sendRDMDiscoveryPacket(UID lower, UID upper, UID* single);
    
    /*!
    * @brief send discovery mute/un-mute packet to target UID
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack response is received.
    */
    uint8_t sendRDMDiscoveryMute(UID target, uint8_t cmd);
    
   /*!
    * @brief send previously built packet in _rdmPacket and validate response
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( void );
    
   /*!
    * @brief copies len of bytes into _rdmPacket and sends it
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( uint8_t* bytes, uint8_t len );
    
   /*!
    * @brief send RDM_GET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
   /*!
    * @brief send RDM_SET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
   
   /*!
    * @brief send RDM_GET_COMMAND_RESPONSE with RDM_RESPONSE_TYPE_ACK
	* @discussion sends data (info) of length (len)
    */
    void sendRDMGetResponse(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
   /*!
    * @brief send RDM_SET_COMMAND_RESPONSE/RDM_DISC_COMMAND_RESPONSE with RDM_RESPONSE_TYPE_ACK
	* @discussion PDL is zero
    */
    void sendAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid);
    
    
    void sendMuteAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid);
    
    /*!
    * @brief send response to RDM_DISC_UNIQUE_BRANCH packet
    */
    void sendRDMDiscoverBranchResponse( void );
    
    /*!
    * @brief static member containing UID of this device
    * @discussion call LX32DMX::THIS_DEVICE_ID.setBytes() or
    *                  ESP32DMX.THIS_DEVICE_ID.setBytes() to change default
    */
    static UID THIS_DEVICE_ID;
    
  protected:

   /*!
   * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
   */
  	uint8_t  _dmx_state;
  	
  /*!
   * @brief set to 0 in startOutput, set to 1 after first packet
   */
  	uint8_t  _dmx_sent;
  	
   /*!
   * @brief flag to continuously loop an I/O task
   */
  	uint8_t  _continue_task;
  	
   /*!
   * @brief flag indication a task loop has started
   */
  	uint8_t  _task_active;
  	
  	/*!
   * @brief flag indicating RDM task should send dmx slots
   */
  	uint8_t  _rdm_task_mode;
  	
  	
  	/*!
   * @brief flag indicating RDM task should send dmx slots
   */
  	uint8_t  _rdm_read_handled;
  	
  	/*!
	 * @brief transaction number
	 */
  	uint8_t _transaction;
  	
  	/*!
	 * @brief maximum expected length of packet
	 */
  	uint16_t  _packet_length;
  	
  	/*!
	 * @brief pin used to control direction of output driver chip
	 */
  	uint8_t _direction_pin;
  	
	/*!
	 * @brief slot index indicating position of last byte received
	 */
  	uint16_t  _current_slot;
  	
	/*!
	 * @brief number of dmx slots ~24 to 512
	 */
  	uint16_t  _slots;
  	
	/*!
	 * @brief outgoing rdm packet length
	 */
  	uint16_t  _rdm_len;
  	
	/*!
	 * @brief Array of dmx data including start code
	 */
  	uint8_t  _dmxData[DMX_MAX_FRAME];
  	
  	/*!
	 * @brief Array of received bytes first byte is start code
	 */
  	uint8_t  _receivedData[DMX_MAX_FRAME];
  	
	/*!
	 * @brief Array representing an rdm packet to be sent
	 */
	uint8_t  _rdmPacket[RDM_MAX_FRAME];
	
	/*!
	 * @brief Array representing a received rdm packet
	 */
	uint8_t  _rdmData[RDM_MAX_FRAME];
  	
	/*!
    * @brief send/receive task
    */
  	TaskHandle_t _xHandle;
  	
  	/*!
    * @brief Pointer to receive callback function
    */
  	LXRecvCallback _receive_callback;
  	
  	/*!
    * @brief Pointer to receive callback function
    */
  	LXRecvCallback _rdm_receive_callback;
  	
};

extern LX32DMX ESP32DMX;

const UID THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x00, 0x01);

#endif // ifndef LX32DMX_H