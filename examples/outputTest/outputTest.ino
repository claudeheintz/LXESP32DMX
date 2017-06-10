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


uint8_t level;

void setup() {
  Serial.begin(115200);
  Serial.print("setup");

  ESP32DMX.startOutput();
  Serial.println("setup complete");
}

/************************************************************************

  The main loop fades the levels of addresses 1,7,8,510,511, and 512 from zero->full
  
*************************************************************************/

void loop() {
  ESP32DMX.setSlot(1, level);
  ESP32DMX.setSlot(7, level);
  ESP32DMX.setSlot(8, level);
  ESP32DMX.setSlot(510, level);
  ESP32DMX.setSlot(511, level);
  ESP32DMX.setSlot(512, level);
  level++;
  delay(100);
}
