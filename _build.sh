#!/bin/bash

avra OneWireLeds.asm -l OneWireLeds.list -m OneWireLeds.map --listmac

#set fuses for 16Mhz Attiny85
#low = F1, high = DF, Extended = FF
#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -v -U lfuse:w:0xFF:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

#main code
avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -V -U flash:w:OneWireLeds.hex

#read fuses
#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -v -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i 
