/**************************************************************************/
/*!
    @file     TOD.h
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32
    
    Implements RDM Table of Devices as array of 6 byte (48bit) UIDs
    capable of storing 200 UIDs in static array

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#ifndef RDMTOD_h
#define RDMTOD_h

#include <stdint.h>
#include <WString.h>
#include "UID.h"


#define STORAGE_SIZE 1200

/*!   
@class TOD
@abstract
   Implements an RDM table of devices.  Contains a list of up to 200 UIDs.
*/

class TOD {
private:
    uint8_t   storage[STORAGE_SIZE];
    uint16_t  next;

public:
    TOD( void );

	/*!
	 * @brief add UID to array
	 * @discussion copies 6 bytes into the next six indexes in the storage array
	 *             the next index is increased by 6
	 * @returns 0 if no more room in storage, otherwise 1
	 */
	uint8_t addUID(UID uid);
	/*!
	 * @brief Add UID to array if storage does not contain this UID.
	 * @discussion checks storage for match to uid
	 *             if not found, calls addUID
	 *             returns 0 if no more room
	 */
	uint8_t add(UID uid);
	/*!
	 * @brief Removes 6 bytes from storage starting at index.
	 */
	void    removeUIDAt(int index);
	
	/*!
	 * @brief Sets the bytes of the UID, copying from storage starting at at index.
	 */
	uint8_t getUIDAt(int index, UID* uid);
	int     getNextUID(int index, UID* uid);
	
	/*!
	 * @brief calls addUID, does nothing if not enough room
	 */
	void push (UID uid);
	/*!
	 * @brief pops the last 6 bytes from storage
	 * @discussion Sets the bytes of the UID, using last 6 bytes of storage
	 *             and decreases next index by 6
	 * @returns 0 if no more remaining, otherwise 1
	 */
	uint8_t pop (UID* uid);
	
	/*!
	 * @brief searches storage for match to UID
	 * @returns 1 if found, otherwise 0
	 */
	uint8_t contains( UID uid );
	
	/*!
	 * @brief Number of UIDs in table
	 * @returns number of 6 byte UIDs currently in storage
	 */
	uint8_t count( void );
	
	/*!
	 * @brief Zeros out the table
	 */
	void    reset( void );
	
	/*!
	 * @brief pointer to the raw storage bytes
	 */
	uint8_t* rawBytes( void );
	
	/*!
	 * @brief utility to print the table using Serial
	 */
	void    printTOD( void );
};


#endif	//RDMTOD
