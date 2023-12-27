;/*
; * blink.asm
; *
; *  Created: 10.12.2013 21:24:08
; *  Author: vlad
; */ 
#define F_CPU	16000000
#define	BAUD	57600
.include "tn85def.inc"

.equ	led			= PB2	; data pin
.equ	led_port	= PORTB ; data port
.equ	led_ddr		= DDRB  ; direct databort
.equ	power_led	= PB0   ; power indicator

.def	tempa	= r16
.def	tempb	= r17
.def	tempc	= r18

.macro	OutByte 	
	ldi		tempa, 8
@0_LOOP:
	sbi		led_port, Led
	rol		@0
	brcc	@0_LP1
	rcall	WAIT400
@0_LP1:
	cbi		led_port, Led
	brcs	@0_LP2
	rcall	WAIT400
@0_LP2:
	dec		tempa
	brne	@0_LOOP
.endm

.org 0

; IRQ vectors ATTINY85
rjmp RESET ; Address 0x0000
rjmp IRQ_def; INT0_ISR ; Address 0x0001
rjmp IRQ_def; PCINT0_ISR ; Address 0x0002
rjmp IRQ_def; TIM1_COMPA_ISR ; Address 0x0003
rjmp IRQ_def; TIM1_OVF_ISR ; Address 0x0004
rjmp IRQ_def; TIM0_OVF_ISR ; Address 0x0005
rjmp IRQ_def; EE_RDY_ISR ; Address 0x0006
rjmp IRQ_def; ANA_COMP_ISR ; Address 0x0007
rjmp IRQ_def; ADC_ISR ; Address 0x0008
rjmp IRQ_def; TIM1_COMPB_ISR ; Address 0x0009
rjmp IRQ_def; TIM0_COMPA_ISR ; Address 0x000A
rjmp IRQ_def; TIM0_COMPB_ISR ; Address 0x000B
rjmp IRQ_def; WDT_ISR ; Address 0x000C
rjmp IRQ_def; USI_START_ISR ; Address 0x000D
rjmp IRQ_def; USI_OVF_ISR ; Address 0x000E

.include "avr.inc"

IRQ_def:
	reti

RESET:  

; Stack init:
	outi	SPH, high(RAMEND)
	outi	SPL, low(RAMEND)

; Port init:
	outi	led_ddr, (1<<power_led|1<<led)	; output for bus port

LOOP:
    ldi		tempb, 0
	OutByte	tempb
    ldi		tempb, 0x0f
	OutByte	tempb

;    cbi		led_port, power_led
;    cbi		led_port, led
;    sbi		led_port, power_led
;    sbi		led_port, led
    rjmp    LOOP

WAIT400: ; call = ~ 400us
	nop
	ret

