/**************************************************************************/
/*!
    @file     outputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Simple Fade test of ESP32 DMX Driver
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#include <LXESP32DMX.h>
#include "LXESP32DMX1.h"


uint8_t level;
uint8_t dmxbuffer[DMX_MAX_FRAME];

void setup() {
  Serial.begin(115200);
  Serial.print("setup");

  ESP32DMX.setDirectionPin(33);
  ESP32DMX.startOutput(14);
  ESP32DMX1.startOutput(13);
  Serial.println("setup complete");
}

void copyDMXToOutput(void) {
	for (int i=1; i<DMX_MAX_FRAME; i++) {
    	ESP32DMX.setSlot(i , dmxbuffer[i]);
    	ESP32DMX1.setSlot(i , dmxbuffer[i]);
    }
}

/************************************************************************

  The main loop fades the levels of addresses 1,7,8,510,511, and 512 from zero->full
  
*************************************************************************/

void loop() {
  dmxbuffer[1] = level;
  dmxbuffer[7] = level;
  dmxbuffer[8] = level;
  dmxbuffer[105] = level;
  dmxbuffer[106] = level;
  dmxbuffer[107] = level;
  dmxbuffer[510] = level;
  dmxbuffer[511] = level;
  dmxbuffer[512] = level;
  
  copyDMXToOutput();
  
  level++;
  delay(100);
}
