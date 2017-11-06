/**************************************************************************/
/*!
    @file     UID.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <Arduino.h>
#include <UID.h>
#include <Print.h>

UID::UID( void ) {
	setBytes((uint64_t)0);
}

UID::UID( uint64_t u ) {
    setBytes(u);
}

UID::UID(uint8_t m1, uint8_t m2, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    bytes[0] = m1;
    bytes[1] = m2;
    bytes[2] = d1;
    bytes[3] = d2;
    bytes[4] = d3;
    bytes[5] = d4;
}

UID::UID(const uint8_t *address) {
    memcpy(bytes, address, sizeof(bytes));
}

UID& UID::operator=(const uint8_t *address) {
    memcpy(bytes, address, sizeof(bytes));
    return *this;
}

UID& UID::operator=(UID address) {
    memcpy(bytes, address.bytes, sizeof(bytes));
    return *this;
}

bool UID::operator==(const UID& addr) const {
    return memcmp(addr.bytes, bytes, sizeof(bytes)) == 0;
}

bool UID::operator==(const uint8_t* addr) const {
    return memcmp(addr, bytes, sizeof(bytes)) == 0;
}

uint8_t UID::becomeMidpoint(UID a, UID b) {
	uint64_t a64 = uid_bytes2long(a.rawbytes());
	uint64_t b64 = uid_bytes2long(b.rawbytes());
	if ( a64 > b64 ) {
		if ( (a64 - b64) < 2 ) {
			return 0;
		}
	} else if ( (b64 - a64) < 2 ) {
		return 0;
	}
	a64 += b64;
	b64 = a64 >> 1;
	uid_long2Bytes(b64, bytes);
	return 1;
}

uint8_t* UID::rawbytes( void ) {
	return bytes;
}

void UID::setBytes(uint64_t u) {
	uid_long2Bytes(u, bytes);
}

void UID::setBytes(UID u) {
	memcpy(bytes, u.bytes, sizeof(bytes));
}

void UID::setBytes(uint8_t m1, uint8_t m2, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
	bytes[0] = m1;
    bytes[1] = m2;
    bytes[2] = d1;
    bytes[3] = d2;
    bytes[4] = d3;
    bytes[5] = d4;
}

uint64_t UID::getValue ( void ) {
	return uid_bytes2long(bytes);
}

size_t UID::printTo(Print& p) const {
    size_t n = 0;
    n += p.print(bytes[0], HEX);
    n += p.print(bytes[1], HEX);
    n += p.print(':');
    n += p.print(bytes[2], HEX);
    n += p.print(bytes[3], HEX);
    n += p.print(bytes[4], HEX);
    n += p.print(bytes[5], HEX);
    return n;
}

String UID::toString() const {
    char szRet[13];
    sprintf(szRet,"%u%u:%u%u%u%u", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);
    return String(szRet);
}

void print64Bit(uint64_t n) {
  uint8_t b[6];
  b[0] = ((uint64_t)(n >> 40)) & 0xFF;
  b[1] = ((uint64_t)(n >> 32)) & 0xFF;
  b[2] = ((uint64_t)(n >> 24)) & 0xFF;
  b[3] = ((uint64_t)(n >> 16)) & 0xFF;
  b[4] = ((uint64_t)(n >> 8)) & 0xFF;
  b[5] = n & 0xFF;
  Serial.print("0x ");
  Serial.print(b[0], HEX);
  Serial.print(" ");
  Serial.print(b[1], HEX);
  Serial.print(" ");
  Serial.print(b[2], HEX);
  Serial.print(" ");
  Serial.print(b[3], HEX);
  Serial.print(" ");
  Serial.print(b[4], HEX);
  Serial.print(" ");
  Serial.println(b[5], HEX);
}

uint64_t uid_bytes2long(uint8_t* b) {
	return (((uint64_t)b[0]) <<40)| (((uint64_t)b[1])<<32) | (((uint64_t)b[2])<<24) | (((uint64_t)b[3])<<16) | (((uint64_t)b[4])<<8) | b[5];
}

void uid_long2Bytes(uint64_t u, uint8_t* bytes) {
	bytes[0] = ((uint64_t)(u >> 40)) & 0xFF;
    bytes[1] = ((uint64_t)(u >> 32)) & 0xFF;
    bytes[2] = ((uint64_t)(u >> 24)) & 0xFF;
    bytes[3] = ((uint64_t)(u >> 16)) & 0xFF;
    bytes[4] = ((uint64_t)(u >> 8)) & 0xFF;
    bytes[5] = u & 0xFF;
}
