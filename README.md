## Quadruped control algorithm
<div align="center">
<img src="https://github.com/yuzeni/quadruped-control-algorithm/blob/main/misc/quadruped.jpg" alt="quadruped" width="400"/>
</div>
This is a revised and imporved upon version of the algorithm I originally wrote for my quadruped project.

## Links

YouTube: https://www.youtube.com/watch?v=Y6QYdh4bs70&t
GrabCad: https://grabcad.com/library/walking-quadruped-robot-diy-1

## Building

### Without Ardiono IDE

This option is there for testing and debugging purposes.\
Use any c++ compiler to compile `main.cpp` with the compilation define `ARDUINO_FREE 1`.
For example:
| MSVC      | `/D ARDUINO_FREE=1` |
| Clang\GCC | `-DARDUINO_FREE=1`  |

### With Arduino IDE

1. Paste the contents of `main.cpp` in the Arduino-IDE.
2. Install the board pack **esp32** *by Espressif Systems*
3. Install the following libraries:
- **Adafruit PWM Servo Driver Library** *by Adarfuit*
- **PS3 Controller Host** *by Jeffry van Pernis*
4. Install the esp32 [usb communication drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads).
5. Select the board: ESP32 Dev Module
6. Compile and upload to the esp32.
