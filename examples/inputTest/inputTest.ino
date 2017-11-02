/**************************************************************************/
/*!
    @file     inputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Print changes in the levels of three slots in DMX serial input
    @section  HISTORY

    v1.00 - First release
    v1.1  - Adds PWM output
*/
/**************************************************************************/
#include <LXESP32DMX.h>
#include "esp_task_wdt.h"

// the addresses of the slots to observe
int test_slotA = 10;
int test_slotB = 1;
int test_slotC = 512;

// the levels of those slots
uint8_t test_levelA = 0;
uint8_t test_levelB = 0;
uint8_t test_levelC = 0;

//pins for PWM output
uint8_t led_pinA = 5;
uint8_t led_pinB = 18;
uint8_t led_pinC = 19;

//ledc channels (set to zero to disable)
uint8_t led_channelA = 1;
uint8_t led_channelB = 2;
uint8_t led_channelC = 3;

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
*************************************************************************/

void receiveCallback(int slots) {
	if ( slots ) {
		if ( test_levelA != ESP32DMX.getSlot(test_slotA) ) {
			test_levelA = ESP32DMX.getSlot(test_slotA);
			Serial.print(test_slotA);
			Serial.print(" => ");
			Serial.println(test_levelA);
			gammaCorrectedWrite(led_channelA, test_levelA);
		}
		if ( test_levelB != ESP32DMX.getSlot(test_slotB) ) {
			test_levelB = ESP32DMX.getSlot(test_slotB);
			Serial.print(test_slotB);
			Serial.print(" => ");
			Serial.println(test_levelB);
			gammaCorrectedWrite(led_channelB, test_levelB);
		}
		if ( test_levelC != ESP32DMX.getSlot(test_slotC) ) {
			test_levelC = ESP32DMX.getSlot(test_slotC);
			Serial.print(test_slotC);
			Serial.print(" => ");
			Serial.println(test_levelC);
			gammaCorrectedWrite(led_channelC, test_levelC);
		}
	}
}

/************************************************************************
	setup
*************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.print("setup");
  
  setupPWMChannel(led_pinA, led_channelA);
  setupPWMChannel(led_pinB, led_channelB);
  setupPWMChannel(led_pinC, led_channelC);

  ESP32DMX.startInput();
  ESP32DMX.setDataReceivedCallback(receiveCallback);
  Serial.println("setup complete");
}


/************************************************************************

  The main loop checks to see if the level of the designated slot has changed
  and prints the new level to the serial monitor.  If a PWM channel is assigned,
  it also sets the output level.
  
*************************************************************************/

void loop() {
  delay(25);
}
