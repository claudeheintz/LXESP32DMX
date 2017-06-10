/**************************************************************************/
/*!
    @file     inputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Print changes in the levels of three slots in DMX serial input
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#include <LXESP32DMX.h>


// the addresses of the slots to observe
int slot = 10;
int slot2 = 1;
int slot3 = 512;

// the levels of those slots
uint8_t level = 0;
uint8_t level2 = 0;
uint8_t level3 = 0;


void setup() {
  Serial.begin(115200);
  Serial.print("setup");

  ESP32DMX.startInput();
  Serial.println("setup complete");
}

/************************************************************************

  The main loop checks to see if the level of the designated slot has changed
  and prints the new level to the serial monitor.
  
*************************************************************************/

void loop() {
  if ( level != ESP32DMX.getSlot(slot) ) {
    level = ESP32DMX.getSlot(slot);
    Serial.print(slot);
    Serial.print(" => ");
    Serial.println(level);
  }
  if ( level2 != ESP32DMX.getSlot(slot2) ) {
    level2 = ESP32DMX.getSlot(slot2);
    Serial.print(slot2);
    Serial.print(" => ");
    Serial.println(level2);
  }
  if ( level3 != ESP32DMX.getSlot(slot3) ) {
    level3 = ESP32DMX.getSlot(slot3);
    Serial.print(slot3);
    Serial.print(" => ");
    Serial.println(level3);
  }
  delay(2);
}
