/**************************************************************************/
/*!
    @file     TOD.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <Arduino.h>
#include "TOD.h"
#include "rdm_utility.h"

TOD::TOD( void ) {
    reset();
}


uint8_t TOD::addUID(UID uid) {
	if ( next < (STORAGE_SIZE-6) ) {
		UID::copyFromUID(uid, storage, next);
		next += 6;
		return 1;
	}
	
	return 0;
}

uint8_t TOD::add(UID uid) {
	if ( ! contains(uid) ) {
		return addUID(uid);
	}
	return 0;
}

void TOD::removeUIDAt(int index) {
	if ( index < next-5 ) {
		next -= 6;
		int len = next - index;
		if ( len > 0 ) {
			memcpy(&storage[index], &storage[index+6], len);
		}
	}
}

uint8_t TOD::getUIDAt(int index, UID* uid) {
	if ( index < next-5 ) {
		*uid = &storage[index];
		return 1;
	}
	
	return 0;
}

int TOD::getNextUID(int index, UID* uid) {
	if ( index < next-5 ) {
		*uid = &storage[index];
		return index + 6;
	}
	
	return -1;
}

void TOD::push (UID uid) {
	addUID(uid);
}

uint8_t TOD::pop (UID* uid) {
	if ( next > 5 ) {
		uint16_t n = next - 6;
		if ( getUIDAt(n, uid) ) {
			next = n;
			return 1;
		}
	}
	return 0;
}

uint8_t TOD::contains( UID uid ) {
	int index = 0;
	UID u(0,0,0,0,0,0);
	while ( index >= 0 ) {
		index = getNextUID(index, &u);
		if ( index > 0 )  {
			if ( uid == u ) {
				return 1;
			}
		}
	}
	return 0;
}

uint8_t TOD::count( void ) {
	return next / 6;
}

void TOD::reset( void ) {
	memset(storage, 0, 1200);
    next = 0;
}

uint8_t* TOD::rawBytes( void ) {
	return storage;
}

void TOD::printTOD( void ) {
	Serial.print("TOD-");
	Serial.println(next/6);
	int index = 0;
	UID u(0,0,0,0,0,0);
	while ( index >= 0 ) {
		index = getNextUID(index, &u);
		if ( index > 0 )  {
			Serial.println(u);
		}
	}
}


