/**************************************************************************/
/*!
    @file     RDMDeviceTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP32DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Example showing LXESP32DMX RDM support for devices.
    Control brightness of LED on GPIO14 with DMX address 1 (settable via RDM)
    
    @section  HISTORY
    v1.00 - First release  
*/
/**************************************************************************/
#include <LXESP32DMX.h>
#include <rdm_utility.h>
#include <UID.h>
#include "freertos/task.h"

#define DMX_DIRECTION_PIN 21
#define DMX_SERIAL_INPUT_PIN 16
#define DMX_SERIAL_OUTPUT_PIN 17
#define LED_PIN 19

int got_dmx = 0;
int got_rdm = 0;
uint8_t discovery_enabled = 1;
uint16_t start_address = 1;



#define DEFAULT_DEVICE_LABEL  "RDM dev test v1.0"
#define MFG_LABEL             "LXDMX"
#define MODEL_DESCRIPTION     "RDMDeviceTest"
#define DEVICE_LABEL_MAX_LEN 33
uint8_t device_label[DEVICE_LABEL_MAX_LEN];

//ledc channel (set to zero to disable)
uint8_t led_channelA = 1;


void setup() {
  Serial.begin(115200);
  
  setupPWMChannel(LED_PIN, led_channelA);

  for(int i=0; i<DEVICE_LABEL_MAX_LEN; i++) {
  	device_label[i] = 0;
  }
  strcpy((char*)device_label, DEFAULT_DEVICE_LABEL);
  
  ESP32DMX.setDataReceivedCallback(&gotDMXCallback);
  ESP32DMX.setRDMReceivedCallback(&gotRDMCallback);
  LX32DMX::THIS_DEVICE_ID.setBytes(0x6C, 0x78, 0x0F, 0x0A, 0x0C, 0x0E);    //change device ID from default
  
  ESP32DMX.startRDM(DMX_DIRECTION_PIN, DMX_SERIAL_INPUT_PIN, DMX_SERIAL_OUTPUT_PIN, DMX_TASK_RECEIVE);
  Serial.println("setup complete");

  // increase the priority of this task (main.cpp sets it at 1);
 // vTaskPrioritySet(xTaskGetCurrentTaskHandle(), 2);
  
  Serial.print("number of tasks is ");
  Serial.println(uxTaskGetNumberOfTasks());
}

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


// ***************** input callback function *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

void gotRDMCallback(int len) {
  // rdm start code and checksum are validated before this is called
  
  uint8_t* rdmdata = ESP32DMX.receivedRDMData();
  uint8_t cmdclass = rdmdata[RDM_IDX_CMD_CLASS];
  uint16_t pid = (rdmdata[RDM_IDX_PID_MSB] << 8 ) | rdmdata[RDM_IDX_PID_LSB];  

  if ( cmdclass == RDM_DISCOVERY_COMMAND ) {
    if ( pid == RDM_DISC_UNIQUE_BRANCH ) {
      if ( discovery_enabled ) {
        uint64_t tv = ESP32DMX.THIS_DEVICE_ID.getValue();
        UID u;
        u.setBytes(&rdmdata[24]);                //lower 
        uint64_t uv = u.getValue();
        if ( tv >= uv ) {
          u.setBytes(&rdmdata[30]);              //upper
          uv = u.getValue();
          if ( tv <= uv ) {
            ESP32DMX.sendRDMDiscoverBranchResponse();
          }
        }
      }

    } else {  // mute RDM_DISCOVERY_COMMAND PIDs
      UID destination;
      destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
      
      if ( pid == RDM_DISC_MUTE ) {
        if ( destination == ESP32DMX.THIS_DEVICE_ID ) {
          discovery_enabled = 0;
          // send ACK
          UID source;
          source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
          ESP32DMX.sendMuteAckRDMResponse(RDM_DISC_COMMAND_RESPONSE, source, RDM_DISC_MUTE);
        }
      } else if ( pid == RDM_DISC_UNMUTE ) {
        if ( destination == BROADCAST_ALL_DEVICES_ID ) {
          // just un-mute
          discovery_enabled = 1;
        } else if ( destination == ESP32DMX.THIS_DEVICE_ID ) {
          discovery_enabled = 1;
          // send ACK
          UID source;
          source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
          ESP32DMX.sendMuteAckRDMResponse(RDM_DISC_COMMAND_RESPONSE, source, RDM_DISC_UNMUTE);
        }
      }
    }
    // <- discovery command
  } else if ( cmdclass == RDM_GET_COMMAND ) {
    UID destination;
    destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
          
    if ( destination == ESP32DMX.THIS_DEVICE_ID ) {
       UID source;
       source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
       
       if ( pid == RDM_DEVICE_START_ADDR ) {
          
          uint8_t sa[2];
          sa[0] = start_address >> 8;
          sa[1] = start_address & 0xff;
          ESP32DMX.sendRDMGetResponse(source, pid, sa, 2);
       } else if ( pid == RDM_DEVICE_MFG_LABEL ) {
          const char * label = MFG_LABEL;
          ESP32DMX.sendRDMGetResponse(source, pid, (uint8_t*)label, 5);
       } else if ( pid == RDM_DEVICE_MODEL_DESC ) {
          const char * label = MODEL_DESCRIPTION;
          ESP32DMX.sendRDMGetResponse(source, pid, (uint8_t*)label, 13);
       } else if ( pid == RDM_DEVICE_DEV_LABEL ) {
          ESP32DMX.sendRDMGetResponse(source, pid, device_label, strlen((const char*)device_label));
       }

       
    }
    // <- get command
  } else if ( cmdclass == RDM_SET_COMMAND ) {
      UID destination;
      destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
          
      if ( destination == ESP32DMX.THIS_DEVICE_ID ) {
         UID source;
         source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
         
         if ( pid == RDM_DEVICE_START_ADDR ) {
            uint16_t scratch = (rdmdata[24] << 8) + rdmdata[25];
            if (( scratch > 0 ) && ( scratch < 513 )) {
              start_address = scratch;
            }
            ESP32DMX.sendAckRDMResponse(RDM_SET_COMMAND_RESPONSE, source, pid);
            
         } else if ( pid == RDM_DEVICE_DEV_LABEL ) {
            uint8_t llen = 0;
            if ( rdmdata[2] > 24 ) {  //label not empty string
              llen = rdmdata[2] - 24;
              if ( llen > 32 ) {      //limit to max 32 characters
                llen = 32;
              }
            }
            for ( uint8_t j=0; j<33; j++) { //copy label, zero the rest of the array
              if ( j < llen ) {
                device_label[j] = rdmdata[24+j];
              } else {
                device_label[j] = 0;
              }
            }   // <-for
            ESP32DMX.sendAckRDMResponse(RDM_SET_COMMAND_RESPONSE, source, pid);
         }      // <-pid RDM_DEVICE_DEV_LABEL
      }
      // <- set command
  } else {
    got_rdm = len;  // set flag and handle in loop???
  }

}

/************************************************************************

  The main loop checks to see if dmx input is available (got_dmx>0)
  And then reads the level of dimmer 1 to set PWM level of LED connected to pin 14
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    gammaCorrectedWrite(led_channelA, ESP32DMX.getSlot(start_address));
    got_dmx = 0;  //reset
    
  } else if ( got_rdm ) {
    //Serial.println("some other rdm");
  }

  vTaskDelay(1);
}
