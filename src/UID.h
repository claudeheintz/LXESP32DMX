/**************************************************************************/
/*!
    @file     UID.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX.h)
    @copyright 2017 by Claude Heintz

    RDM support for DMX Driver for ESP32
    
    Implements class encapsulating 48bit RDM unique device identifier
    MM:DDDD
    Where MM is ESTA manufacturer code.

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#ifndef RDMUID_h
#define RDMUID_h

#include <stdint.h>
#include <WString.h>
#include <Printable.h>


// utility functions
void print64Bit(uint64_t n);
uint64_t uid_bytes2long(uint8_t* b);
void uid_long2Bytes(uint64_t u, uint8_t* bytes);

// A class to make it easier to handle RDM UIDs
// based on Arduino IPAddress class

class UID: public Printable
{
private:
    uint8_t bytes[6];

public:
    UID( uint64_t u );
    UID(uint8_t m1, uint8_t m2, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    UID(const uint8_t *address);

	// Overloaded equality operator
    bool operator==(const UID& addr) const;
    bool operator==(const uint8_t* addr) const;

    // Overloaded index operator to allow getting and setting individual bytes
    uint8_t operator[](int index) const {
        return bytes[index];
    }
    uint8_t& operator[](int index) {
        return bytes[index];
    }

    // Overloaded copy operators to allow initialisation of UID objects
    UID& operator=(const uint8_t *address);
    UID& operator=(UID address);
    
    static void copyFromUID(UID id, uint8_t *address, uint16_t index=0) {
    	memcpy(&address[index], id.bytes, sizeof(id.bytes));
    }
    
    static void copyToUID(UID id, uint8_t *address, uint16_t index=0) {
    	memcpy(id.bytes, &address[index], sizeof(id.bytes));
    }
    
    uint8_t becomeMidpoint(UID a, UID b);
    
    void     setBytes(uint64_t u);
    void     setBytes(UID u);
    uint8_t* rawbytes( void );

    virtual size_t printTo(Print& p) const;
    String toString() const;
};

const UID BROADCAST_ALL_DEVICES_ID(0xFFFFFFFFFFFF);

#endif	//RDMUID
