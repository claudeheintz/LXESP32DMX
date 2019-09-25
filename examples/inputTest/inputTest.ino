/**************************************************************************/
/*!
    @file     inputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Print changes in the levels of three slots in DMX serial input
    @section  HISTORY

    NOTE HardwareSerial::begin flushes the input queue until it is empty.
         If DMX serial is present on the input pin when ESP32DMX.startInput
         is called, it is possible that flush will fail to empty the queue
         because it is being refilled by the DMX as fast as it is emptied.
         This will result in startInput hanging.  To avoid this, plug in the
         DMX input after the sketch has called setup.

    v1.00 - First release
    v1.1  - Adds PWM output
    v1.2  - 
*/
/**************************************************************************/
#include <LXESP32DMX.h>
#include "esp_task_wdt.h"

#define DMX_DIRECTION_PIN 32
#define DMX_SERIAL_INPUT_PIN 34

// the addresses of the slots to observe
int test_slotA = 10;
int test_slotB = 1;
int test_slotC = 512;

// the levels of those slots
uint8_t test_levelA = 0;
uint8_t test_levelB = 0;
uint8_t test_levelC = 0;

//pins for PWM output
uint8_t led_pinA = 12;
uint8_t led_pinB = 18;
uint8_t led_pinC = 19;

//ledc channels (set to zero to disable)
uint8_t led_channelA = 1;
uint8_t led_channelB = 2;
uint8_t led_channelC = 3;

uint8_t dataChanged = 1;

/************************************************************************
	attach a pin to a channel and configure PWM output
*************************************************************************/
void setupPWMChannel(uint8_t pin, uint8_t channel) {
	if ( channel ) {
		ledcAttachPin(pin, channel);
		ledcSetup(channel, 12000, 16); // 12 kHz PWM, 8-bit resolution
	}
}

/************************************************************************
	gamma corrected write to a PWM channel
*************************************************************************/
void gammaCorrectedWrite(uint8_t channel, uint8_t level) {
	if ( channel ) {
		ledcWrite(channel, level*level);
	}
}

/************************************************************************

  callback for when DMX frame is received
  Note:  called from receive task
  
  Checks to see if the level of the designated slot has changed
  and sets the dataChanged flag.

  Processing in this callback should be minimal.

  Instead, use a flag and do more intense processing in a lower priority task.
  
*************************************************************************/

void receiveCallback(int slots) {
	if ( slots ) {
	    xSemaphoreTake( ESP32DMX.lxDataLock, portMAX_DELAY );
		if ( test_levelA != ESP32DMX.getSlot(test_slotA) ) {
			test_levelA = ESP32DMX.getSlot(test_slotA);
			dataChanged = 1;
		}
		if ( test_levelB != ESP32DMX.getSlot(test_slotB) ) {
			test_levelB = ESP32DMX.getSlot(test_slotB);
			dataChanged = 1;
		}
		if ( test_levelC != ESP32DMX.getSlot(test_slotC) ) {
			test_levelC = ESP32DMX.getSlot(test_slotC);
			dataChanged = 1;
		}
		xSemaphoreGive( ESP32DMX.lxDataLock );
	}
}

/************************************************************************
	setup
*************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.print("setup");
  
  ESP32DMX.setDirectionPin(DMX_DIRECTION_PIN);
  
  setupPWMChannel(led_pinA, led_channelA);
  setupPWMChannel(led_pinB, led_channelB);
  setupPWMChannel(led_pinC, led_channelC);

  Serial.print(", set callback");
  ESP32DMX.setDataReceivedCallback(receiveCallback);

  Serial.print(", start dmx input");
  ESP32DMX.startInput(DMX_SERIAL_INPUT_PIN);
  
  Serial.println(", setup complete.");
}


/************************************************************************
	main loop just idles
	vTaskDelay is called to prevent wdt timeout
*************************************************************************/

void loop() {
  if ( dataChanged ) {
    dataChanged = 0;
    Serial.print(test_slotA);
    Serial.print(" => ");
    Serial.println(test_levelA);
    gammaCorrectedWrite(led_channelA, test_levelA);
    Serial.print(test_slotB);
    Serial.print(" => ");
    Serial.println(test_levelB);
    gammaCorrectedWrite(led_channelB, test_levelB);
    Serial.print(test_slotC);
    Serial.print(" => ");
    Serial.println(test_levelC);
    gammaCorrectedWrite(led_channelC, test_levelC);
  } else {
    delay(25);
  }
  
}
