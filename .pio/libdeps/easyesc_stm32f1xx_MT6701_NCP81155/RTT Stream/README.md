# Arduino RTTStream

## Description
Arduino library for [Segger RTT](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) on arm and risc-v Arduinos

With RTTStream an stm32duino can do screen output on a pc, and get keyboard input from a pc, without using usb or serials.

To use RTTStream you need a [Segger JLink](https://www.segger.com/products/debug-probes/j-link/) debugger probe. The JLink debugger probe is a piece of hardware that connects your arduino to the debugger program on your pc.

RTTStream only runs on arduinos with arm or risc-v processors, sorry.

## Use

Use RTTStream just like you would use Serial:

```
#include <RTTStream.h>
RTTStream rtt;
void setup() {
	rtt.println("hello world!");
}
void loop() {
}
```

## Sample RTT Session

This is an example of a debugging session of a STM32F103 *Blue Pill*  on linux.

Connect a Segger J-Link probe to the SWD port of your arduino, and to the usb of your pc.

Connections between blue pill and [J-Link](https://www.segger.com/products/debug-probes/j-link/technology/interface-description/#swd-and-swo-also-called-swv-compatibility):

| Blue pill | J-Link | J-Link Pin | J-Link Mini Pin |
| --------- | ------ | ---------- | --------------- |
| +3.3V     | VTREF  | 1          | 1               |
| SWDIO     | SWDIO  | 7          | 2               |
| SWCLK     | SWCLK  | 9          | 4               |
| GND       | GND    | 4          | 3               |

Open a window and start the gdb server:
```
JLinkGDBServer -device STM32F103R8 -if SWD -speed 4000
```
Open another window and start the RTT terminal. This is where the messages from the arduino program will appear, and where you can type keyboard input.
```
JLinkRTTClient
```
Start the debugger, download and run your program:
```
$ arm-none-eabi-gdb
(gdb) target extended-remote 127.0.0.1:2331
(gdb) file StreamTest.ino.elf
(gdb) load
(gdb) run
```
or, abbreviated:
```
$ arm-none-eabi-gdb
(gdb) tar ext :2331
(gdb) fil StreamTest.ino.elf
(gdb) lo
(gdb) r
```
Switch to the window where *JLinkRTTClient* is running to read program messages, and to type program input:

```
rtt stream test
char: a
string: the quick brown fox jumps over a lazy dog
int: 12345
float: 3.14
printf char: a signed int: -12345 unsigned int: 12345
printf hex int: 12345 string: 12345 pointer: 20000090
enter an int: 23456
value: 23456
enter a float: 2.71828
value: 2.72
enter a string and wait 5 seconds: qwerty
string: qwerty
rtt stream test end
```

## Boards with built-in RTT

RTTStream contains SEGGER RTT source files. Some Arduino boards already include SEGGER RTT as part of the board package, e.g. nrf52.

To avoid duplicates, if the Arduino board package already contains the files `SEGGER_RTT.h`, `SEGGER_RTT.c` and `SEGGER_RTT_Conf.h`, please

- either remove `SEGGER_RTT.h`, `SEGGER_RTT.c` and `SEGGER_RTT_Conf.h` from the board package
- or remove the files `libraries/RTT_Stream/src/SEGGER_RTT.h`, `libraries/RTT_Stream/src/SEGGER_RTT.c` and `libraries/RTT_Stream/src/SEGGER_RTT_Conf.h`.

This way there is only one version of SEGGER RTT.

## Links
- The Eclipse development environment has a [plugin](https://eclipse-embed-cdt.github.io/debug/jlink/) to automate using Segger J-Link.
- [OpenOCD](https://openocd.org/doc/html/General-Commands.html#Real-Time-Transfer-_0028RTT_0029) and  [Black Magic Probe](https://github.com/blackmagic-debug/blackmagic/blob/master/UsingRTT.md) support RTT.

*not truncated*
