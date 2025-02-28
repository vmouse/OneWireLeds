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

#define	TapeLen		3	; length of led tape. allowable values are 12 and 20

; .macro	OutByte 	
; 	ldi		tempa, 8
; @0_LOOP:
; 	sbi		led_port, Led
; 	rol		@0
; 	brcc	@0_LP1
; 	rcall	WAIT400
; @0_LP1:
; 	cbi		led_port, Led
; 	brcs	@0_LP2
; 	rcall	WAIT400
; @0_LP2:
; 	dec		tempa
; 	brne	@0_LOOP
; .endm
; OutByte	Reg (not tempa!)
.macro	OutByte 	

	ldi		tempa, 7        ; 1  (62.5ns)
@0_LOOP:
	sbi		led_port, Led   ; 2/1 (62.5ns / 125ns)
	rol		@0              ; 1  (62.5ns)
	brcc	@0_OUT_0        ; 1 - false, 2 - true (62.5ns / 125ns)
@0_OUT_1:
	rcall	WAIT400         ; 8 (500ns)
    nop                     ; 1 (62.5ns)   
    nop                     ; 1 (62.5ns)   
    nop                     ; 1 (62.5ns)   
	cbi		led_port, Led   ; 1 (62.5ns)
	rjmp	@0_NEXT_BIT     ; 2 (125ns)
@0_OUT_0:
    nop                     ; 1 (62.5ns)   
	cbi		led_port, Led   ; 1 (62.5ns)
	rcall	WAIT400         ; 8 (500ns)
    nop                     ; 1 (62.5ns)   
    nop                     ; 1 (62.5ns)   
@0_NEXT_BIT:
	dec		tempa           ; 1 (62.5ns)
	brne	@0_LOOP         ; 1 - false, 2 - true (62.5ns / 125ns)

    nop                     ; 1 (62.5ns)    
@0_LAST_BIT:
	sbi		led_port, Led   ; 2/1 (62.5ns / 125ns)
	rol		@0              ; 1  (62.5ns)
	brcs	@0_LAST_BIT_1   ; 1 - false, 2 - true (62.5ns / 125ns)
@0_LAST_BIT_0:
    nop                     ; 1 (62.5ns)   
    nop                     ; 1 (62.5ns)   
	cbi		led_port, Led   ; 1 (62.5ns)
    rjmp    @0_END
@0_LAST_BIT_1:
	rcall	WAIT400         ; 8 (500ns)
    nop                     ; 1 (62.5ns)   
	cbi		led_port, Led   ; 1 (62.5ns)
@0_END:
.endm

; .macro	OutByteUART
; @0_send_byte: 
;     ; Load data to send
;     ldi r17, 0xAA ; Example byte to send

;     ; Send byte
;     sts UDR0, r17
; @0_wait_transmit:
;     lds r18, UCSR0A
;     sbrs r18, TXC0
;     rjmp wait_transmit
; .endm

; start:
;     ; Initialize UART
;     ldi r16, 0x00 ; Set baud rate to 250000bps for 16MHz CPU clock
;     sts UBRR0H, r16
;     ldi r16, 0x03
;     sts UBRR0L, r16
;     ldi r16, (1<<TXEN0) ; Enable transmitter
;     sts UCSR0B, r16
;     ldi r16, (3<<UCSZ00) ; Set frame format: 8data, 2stop bit
;     sts UCSR0C, r16

; send_byte: 
;     ; Load data to send
;     ldi r17, 0xAA ; Example byte to send

;     ; Send byte
;     sts UDR0, r17
; wait_transmit:
;     lds r18, UCSR0A
;     sbrs r18, TXC0
;     rjmp wait_transmit


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
    sbi		led_port, power_led

LOOP:
    rcall   InitArray
    rcall   InitArray
    rcall   InitArray
    rcall   InitArray
    rcall   InitArray
    rcall   InitArray
    ; ldi		tempb, 0x10
	; OutByte	tempb
    ; ldi		tempb, 0x50
	; OutByte	tempb
    ; ldi		tempb, 0x7f
	; OutByte	tempb
    ; rjmp    LOOP

Output_ledtape:
	ldi		tempc, TapeLen * 3
	ldiw	X, StateParams + 5
outloop:
	ld		tempb, X+
	OutByte	tempb

	addiw	X, 5
	dec		tempc
	brne	outloop
	cbi		led_port, led
    rjmp    LOOP

WAIT400:    ; 4 (250ns) total rcall = 500..562.5ns
;	nop     ; 1 (62.5ns) 
	ret     ; 4 (250ns)

InitArray:
	ldiw	X, StateParams
	ldi		tempa, TapeLen*3
	ldi		tempb, 0x55
eraseloop:
	st		X+, tempb
	st		X+, tempb
	st		X+, tempb
	st		X+, tempb
	st		X+, tempb
	st		X+, tempb
    ldi     tempb, 0xaa
	dec		tempa
	brne	eraseloop
    ret

.dseg
StateParams:	.BYTE	TapeLen * 3 * 6

