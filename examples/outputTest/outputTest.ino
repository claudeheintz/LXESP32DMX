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
#include "esp_task_wdt.h"

#define DMX_DIRECTION_PIN 21
#define DMX_SERIAL_OUTPUT_PIN 17

uint8_t level;
uint8_t dmxbuffer[DMX_MAX_FRAME];

void setup() {
  Serial.begin(115200);
  Serial.print("setup");
  
  pinMode(DMX_DIRECTION_PIN, OUTPUT);
  digitalWrite(DMX_DIRECTION_PIN, HIGH);

  pinMode(DMX_SERIAL_OUTPUT_PIN, OUTPUT);
  ESP32DMX.startOutput(DMX_SERIAL_OUTPUT_PIN);
  Serial.println("setup complete");
}

void copyDMXToOutput(void) {
  xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );
	for (int i=1; i<DMX_MAX_FRAME; i++) {
    	ESP32DMX.setSlot(i , dmxbuffer[i]);
   }
   xSemaphoreGive( ESP32DMX.lxDataLock );
}

/************************************************************************

  The main loop fades the levels of addresses 1,7,8,510,511, and 512 from zero->full
  
*************************************************************************/

void loop() {
  dmxbuffer[1] = level;
  dmxbuffer[7] = level;
  dmxbuffer[8] = level;
  dmxbuffer[510] = level;
  dmxbuffer[511] = level;
  dmxbuffer[512] = level;
  
  copyDMXToOutput();
  
  level++;

  esp_task_wdt_feed();
  vTaskDelay(100);
}
