/**************************************************************************/
/*!
    @file     InAndOutputTest.ino
    @author   Roeland Kluit
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2024 by Roeland Kluit

    Simple pass trough DMX test of ESP32 DMX Driver
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#define DO_NO_CREATE_DEFAULT_LXESP32DMX_CLASS_OBJECT 1
#include "LXESP32DMX.h"
#include "esp_task_wdt.h"

#define DMX_INPUT_DIRECTION_PIN    21
#define DMX_INPUT_SERIAL_PIN       17
#define PRIO_INPUT_TASK            10

#define PRIO_OUTPUT_TASK           9
#define DMX_OUTPUT_DIRECTION_PIN   22
#define DMX_OUTPUT_SERIAL_PIN      13

uint8_t level;
uint8_t dmxbuffer[DMX_MAX_FRAME];

LX32DMX DMXoutput(1);
LX32DMX DMXinput(2);

volatile bool dmx = false;
byte dmxInputSerial[DMX_MAX_FRAME];

void onLocalDMXRecieve(int slots)
{
    dmx = true;
    xSemaphoreTake(DMXinput.lxDataLock, portMAX_DELAY);
    byte* dmxDataIn = DMXinput.dmxData();
    memcpy(dmxInputSerial, dmxDataIn, DMX_MAX_FRAME);
    xSemaphoreGive(DMXinput.lxDataLock);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("setup");
  
  DMXoutput.setDirectionPin(DMX_OUTPUT_DIRECTION_PIN);
  DMXinput.setDirectionPin(DMX_INPUT_DIRECTION_PIN);

  DMXinput.setDataReceivedCallback(onLocalDMXRecieve);

  //Serial.print(", start dmx input and output");
  DMXinput.startInput(DMX_INPUT_SERIAL_PIN, PRIO_INPUT_TASK, 1);
  DMXoutput.startOutput(DMX_OUTPUT_SERIAL_PIN, PRIO_OUTPUT_TASK, 0);

  Serial.println("Done");
}


/************************************************************************

  The main loop fades the levels of addresses 1,7,8,510,511, and 512 from zero->full
  
*************************************************************************/

void PrintByteArrayHex(const byte* byteArray, const size_t& length)
{
    for (size_t i = 0; i < length; ++i) {
        if (byteArray[i] < 16) {
            Serial.print("0"); // Add leading zero if necessary
        }
        Serial.print(byteArray[i], HEX);
        Serial.print(" "); // Add space between bytes
    }
    Serial.println(); // Print newline at the end
}

unsigned long lastTimer = 0;

void loop()
{
  bool ldmx = dmx;
  vTaskDelay(25);

  if (dmx)
  {
      byte* dmxInterfaceDataOut = DMXoutput.dmxData();
      if (xSemaphoreTake(DMXoutput.lxDataLock, (TickType_t)((portTICK_PERIOD_MS) * 25)) == pdTRUE)
      {
          memcpy(dmxInterfaceDataOut, dmxInputSerial, DMX_MAX_FRAME);
          xSemaphoreGive(DMXoutput.lxDataLock);
      }
  }
  if ((millis() - lastTimer) > 500)
  {
      lastTimer = millis();

      if (ldmx)
      {
          Serial.println('!');
          PrintByteArrayHex(dmxInputSerial, 32);
          dmx = false;
      }
      else
      {
          Serial.println('.');          
      }
  }
}
