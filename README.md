# Circuit and code for a heartbeat bouncy ball

## PCB Design

The PCB design is made in Cadsoft Eagle 7.1 (http://www.cadsoftusa.com/download-eagle/).
The free version should be fine for opening and editing the design.

It's based on the Atmel ATTINY85, though an ATTINY45 or ATTINY25 would probably also work.

## Firmware

The firmware is written against avr-gcc.

### Toolchain setup for OS X

Install XCode and command line tools. Version 6.1.1 was used for development, but this should
not be critical. The method for enabling the command line tools seems to be a moving target,
one set of instructions is here (https://developer.apple.com/library/ios/technotes/tn2339/_index.html).

Install CrossPack for AVR (http://www.obdev.at/products/crosspack/download.html). Version
CrossPack-AVR 20130212 was used for development. Most likey this code is not version-sensitive.

### Compiling

Once the toolchain is configured, this should be as simple as:

  make
  
### Uploading firmware

An AVR ISP is required to upload code. The Makefile is configured to use a USBTinyISP
(https://learn.adafruit.com/usbtinyisp), however it would be trivial to use a different
one, or even an Arduino to upload the code.

Solder the 6 ICSP test points on the board (use Eagle to inspect the pinout- the board design
is a moving target), then an automatic upload should work:

  make install

A test jig can also be built to make code upload possible without soldering.
