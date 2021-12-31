It is necessary to replace the esp32-hal-uart.c file in .../Arduino/hardware/espressif/esp32/cores/esp32 with the version in this folder.

This makes a modification to the standard version (as of October 30th, 2017) so that break detection can be passed to the receiving task.

DMX Input by modifying esp32-hal-uart.c is only supported up to Boards Manager ESP-32 v2.0.0 RC1.