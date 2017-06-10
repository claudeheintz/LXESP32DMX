# LXESP32DMX
DMX Driver for ESP32 using ArduinoIDE and Arduino core for the ESP32.

Driver outputs DMX using UART2 on GPIO pin 17.

Driver inputs DMX using UART2 with modified hardware abstraction file on GPIO pin 16.

Arduino core for the ESP32 is found at https://github.com/espressif/arduino-esp32

Install Arduino core for the ESP32 into [path to sketch folder]/Arduino/hardware/.

Replace the following files:

[path to sketch folder]/Arduino/hardware/espressif/esp32/cores/esp32/esp32-hal-uart.h

and

[path to sketch folder]/Arduino/hardware/espressif/esp32/cores/esp32/esp32-hal-uart.c

with files included with this library in the extras/modified_hal-uart folder.
