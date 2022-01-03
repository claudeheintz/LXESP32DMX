# LXESP32DMX
DMX Driver for ESP32 using ArduinoIDE and Arduino core for the ESP32.

Driver outputs DMX using UART2 on GPIO pin 17.

Driver inputs DMX using UART2 with modified hardware abstraction file on GPIO pin 16.

If using SDK v2.0.0 RC1 or earlier, use the master branch and replace the following file:

[path to sketch folder]/Arduino/hardware/espressif/esp32/cores/esp32/esp32-hal-uart.c

with the file included with this library in the extras/modified_hal-uart folder.


If using SDK v2.0.2 or newer use the w202 branch.  The master branch is not compatible with later SDKs.
