#!/bin/bash

avra OneWireLeds.asm

avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -V -U flash:w:OneWireLeds.hex

