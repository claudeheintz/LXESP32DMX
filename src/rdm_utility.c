/**************************************************************************/
/*!
    @file     rdm_utility.c
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include "rdm_utility.h"

uint16_t rdmChecksum(uint8_t* bytes, uint8_t len) {
	uint16_t rv = 0;
	for(int j=0; j<len; j++) {
		rv += bytes[j];
	}
	return rv;
}

uint8_t testRDMChecksum(uint16_t cksum, uint8_t* data, uint8_t index) {
	return (( (cksum >> 8) == data[index] ) && ( (cksum &0xFF) == data[index+1] ));
}


uint8_t validateRDMPacket(uint8_t* pdata) {
	if ( pdata[0] != RDM_START_CODE ) {
		return 0;
	}
	uint16_t checksum = rdmChecksum(pdata, pdata[2]);
	return testRDMChecksum(checksum, pdata, pdata[2]);
}