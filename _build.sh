#!/bin/bash

<<<<<<< HEAD
avra -D DEVICE=ATtiny85 -W NoRegDef OneWireLeds.asm

#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -U flash:w:OneWireLeds.hex
=======
#avra OneWireLeds.asm -l OneWireLeds.list -m OneWireLeds.map --listmac
wine64 avrasm2 OneWireLeds.asm -fI -o OneWireLeds.hex -d OneWireLeds.debug

#set fuses for 16Mhz Attiny85
#low = F1, high = DF, Extended = FF
#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -v -U lfuse:w:0xFF:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

#main code
avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -V -U flash:w:OneWireLeds.hex

#read fuses
#avrdude -c avrisp -b 19200 -p attiny85 -P /dev/tty.usbserial-AE01AF86 -v -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i 
>>>>>>> 0994a90a4c06cf1361a4f4e08dcc571060467891
