#!/bin/bash

avra blink.asm

avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -B 125kHz -U flash:w:blink.hex

