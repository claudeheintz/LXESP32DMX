# LXESP32DMX
DMX Driver for ESP32 using ArduinoIDE and Arduino core for the ESP32.

Driver outputs DMX using UART2 on GPIO pin 17.

Driver inputs DMX using UART2 on GPIO pin 16.

Arduino core for the ESP32 is found at https://github.com/espressif/arduino-esp32

Install Arduino core for the ESP32 into [path to sketch folder]/Arduino/hardware/.

Use the following statement PRIOR the include of the .h file, to not create a default DMX class object
 #define DO_NO_CREATE_DEFAULT_LXESP32DMX_CLASS_OBJECT 1

Create a class by specifying the class name, and serial port to use. 
 LX32DMX DMXoutput(1);
 LX32DMX DMXinput(2);


In Setup() specify the (needed) RX, TX and Direction pins:

  DMXoutput.setDirectionPin(DMX_OUTPUT_DIRECTION_PIN);
  DMXinput.setDirectionPin(DMX_INPUT_DIRECTION_PIN);

  DMXinput.setDataReceivedCallback(onLocalDMXRecieve);

  DMXinput.startInput(DMX_INPUT_SERIAL_PIN, PRIO_INPUT_TASK, 1);
  DMXoutput.startOutput(DMX_OUTPUT_SERIAL_PIN, PRIO_OUTPUT_TASK, 0);

