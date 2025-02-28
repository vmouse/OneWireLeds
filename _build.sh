#!/bin/bash

avra -D DEVICE=ATtiny85 -W NoRegDef OneWireLeds.asm

#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -U flash:w:OneWireLeds.hex
