/* test branch
 * OneWireLeds.asm
 *
 *  Created: 10.12.2013 21:24:08
 *   Author: vlad
 */ 

#define F_CPU	16000000

//#define	NODELAY

.def	tempa	= r16
.def	tempb	= r17
.def	tempc	= r18
.def	ShowingState = r20

.def	WaitDelay		= r25

.def	DelayCount	= r19	//  Speed regulator

.equ	TapeLen		= 16	// length of led tape

.equ	led			= PB1	// led out pin
.equ	sens		= PB2   // sensor in port
.equ	led_port	= PORTB // data port
.equ	led_ddr		= DDRB  // direct databort

.equ	DelayConst	= 150//200
.equ	StateChange = 1
.equ	StateWait	= 2

.include "..\..\libasm\avr.inc"

; OutByte	Reg (not tempa!)
; tempa - will broken
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

// IRQ vectors ATMEGA328p
.org 0
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

IRQ_def:
	reti

RESET:
// Stack init:
//	outi	SPH, high(RAMEND)
//	outi	SPL, low(RAMEND)

// WatchDog init:
	wdr
	stsi	WDTCSR, (1<<WDCE)|(1<<WDE)	// change mode enable
	stsi	WDTCSR, (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)|(0<<WDCE)|(0<<WDE) // reset after 8 sec hang

// Port init:
	outi	led_ddr, (1<<Led)	// output for bus port only
	outi	led_port, 0xff		// set pullups 
// ADC init (ADC1)
; Initial ADC
;    outi  ADCSRA,0b10000110 ; Turn On the ADC, with prescale 64
    outi  ADCSRA, (1<<ADEN)|(0<<ADPS2)|(0<<ADPS1)|(0<<ADPS0); Turn On the ADC, with prescale 2 (fast mode)
	outi  ADCSRB,0b00000000 ; Free running mode
	outi  ADMUX, (0<<REFS2)|(0<<REFS1)|(0<<REFS0)|(0<<ADLAR)|(1<<MUX0)|(0<<MUX1)|(0<<MUX2)|(0<<MUX3) ; Reference = VCC,Left Adjust, Channel: PB2 (ADC1)
	outi  DIDR0, (1<<ADC1D) ; Disable Digital Input on PB2 (ADC1)

//ADMUX; ADCSRA; DIDR0; TCCR0A; TCCR0B



	rcall	InitVariables		// clear output array and set program pointer to begin
	rcall	InitNextEffect		// read program line and set effect array pointers (BegArr, EndArr)

	ldi		ShowingState, 1
MainLoop:						// Основной цикл обработки перехода состояний

    sbi     ADCSRA,ADSC      ; Start ADC conversion
ml_adc_wait:  
	sbic    ADCSRA,ADSC      ; while (ADCSRA & (1<<ADSC))
	rjmp    ml_adc_wait
	in      tempb,ADCL         ; Read the result Ignore the last 2 bits in ADCL
	in      tempa,ADCH         ; Read the result Ignore the last 2 bits in ADCL

	cpi	    tempb, 128
	brsh    ml_no_magnet

	ldi		ShowingState, 1
	rcall	InitNextEffect		// read program line and set effect array pointers (BegArr, EndArr)

ml_no_magnet:
	cpi		ShowingState, 1
	brne	MainLoop

	rcall	ReadNextState
	brts	ml_stop_effect		// end of effect	

//	rcall	ShowNextState		// готовим массив для отображения 
//	rcall	DrawLevel
	rcall	Output_ledtape		// выводим в ленту
	
	ldi		tempa, 1
ml_1:
	rcall	delay_output		// задержка
	dec		tempa
	brne	ml_1

	rjmp	MainLoop

ml_stop_effect:
	ldi		ShowingState, 0
	rjmp	MainLoop



// Modes loop, each mode has states (led string)
InitNextEffect:				// read effect parameters from Modes array
	ldsw	Z, NextProgLine // Load program address (Modes)

	lpm		tempa, Z+		// Begin effect addr low
	sts		BegArr, tempa
	lpm		tempa, Z+		// Begin effect addr high
	cpi		tempa, 0xff		// end of program marker
	brne	ine_read_params
	stsi	NextProgLine, Low(Modes*2)		// set program to start
	stsi	NextProgLine+1, High(Modes*2)
	rjmp	InitNextEffect

ine_read_params:
	sts		BegArr+1, tempa
	lpm		tempa, Z+		// End effect addr low
	sts		EndArr, tempa
	lpm		tempa, Z+		// End effect addr high
	sts		EndArr+1, tempa
	stsw	NextProgLine, Z // Store program address (Modes)
	ldsw	Z, BegArr
	ret 

ShowNextState:				// Выводим следующий эффект
	rcall	ReadNextState
	rettc					
	
// последовательность эффекта была завершена надо читать программу дальше
	rcall	InitNextEffect
	rjmp	ShowNextState

// Erase vaiables and set to default:
InitVariables:
	ldiw	X, StateParams
	ldiw	Y, TapeLen*3
	clr		tempb
eraloop: // erase StateParams (set all leds to black)
	st		X+, tempb
	subiw	Y, 1
	brne	eraloop

	// set start values
	stsi	NextProgLine, Low(Modes*2)
	stsi	NextProgLine+1, High(Modes*2)
	ldi		DelayCount, 100
	ret


; transfer next effect line (pointed by Z) to lights array
; tempa - broken
; T-flag = clear - Ok, Set - Out of range
ReadNextState:
		wdr						// сброс таймера собаки
		set						// взводим флаг ошибки

rns_check_end:					// проверка на выходы за пределы массива эффекта, при прямом ходе
		ldsw	Y, EndArr
		cpw		Z, Y		
		brcs	rns_init		// Z < end
		ret						// выход с ошибкой (за концом списка режимов)

rns_init:
		; читаем след.состояние
		ldi		tempc, TapeLen/2// длина массива состояний
		ldiw	X, StateParams  // встаем в начало массива управления сменой состояний

rns2:
		lpm		tempa, Z+			// берем два значения цвета из одного байта
		push	tempa
		swap	tempa
		andi	tempa, 0x0f
		rcall	Set_Color			// конвертируем в цвет и пишем в массив для вывода
		pop		tempa
		andi	tempa, 0x0f
		rcall	Set_Color

rns_cnt:
		dec		tempc
		brne	rns2
		clt						// выход без ошибки
		ret

// рисует линию длиной пропорциональной значению tempb
DrawLevel:
		ldiw	X, StateParams  // встаем в начало массива управления сменой состояний
		ldi		tempc, TapeLen
		ldi		tempa, 9
dl_loop:
		subi	tempb, 255/TapeLen // value of one point
		brcc	dl_higher
		ldi		tempa, 5
dl_higher:
		rcall	Set_Color
		dec		tempc
		brne	dl_loop
ret



// calc color by index in tempa (0..15) and store 3 bytes into X pointer
// tempa broken
Set_Color:	
		pushw	Z
		push	tempa
		lsl		tempa
		lsl		tempa
		ldiw	Z, ColorTable*2
		add		ZL, tempa
		clr		tempa
		adc		ZH, tempa
		; Z address 3 byte of color
		lpm		tempa, Z+
		st		X+, tempa
		lpm		tempa, Z+
		st		X+, tempa
		lpm		tempa, Z+
		st		X+, tempa
		pop		tempa
		popw	Z
		ret


delay_output:
		push	tempa
		push	tempb
#ifndef NODELAY
		ldi		tempb, DelayConst
dl1:	
		wdr
		mov		tempa, DelayCount
		inc		tempa
dl2: 
		rcall	WAIT400
		dec		tempa
		brne	dl2
		dec		tempb
		brne	dl1
#endif

		pop		tempb
		pop		tempa
ret


// Out led array top port
Output_ledtape:
	push	tempa
	push	tempc
	pushw	X

	ldi		tempc, TapeLen * 3
	ldiw	X, StateParams
outloop:
	ld		tempb, X+
	
//  brightness reducer
	swap	tempb
	andi	tempb,0x0f

	OutByte	tempb

	dec		tempc
	brne	outloop
	cbi		led_port, Led

	popw	X
	pop		tempc
	pop		tempa
ret

stop:
	rjmp	STOP


WAIT400: // call = ~ 400us
	nop
	ret

Modes:	
// 4 bytes for mode: ModeFlag, RepeatCnt, Begin addr, End addr
//	Mode:  0bit = Reverse, 1bit = Mirror, 2,3 - reserve, 4-8 - speed 

//.DW		Ef_Rainbow_splash16*2, Ef_Rainbow_splash16_end*2
.DW		Ef_flash*2, Ef_flash_end*2
.DW		0xffff



Ef_flash:
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.DB 0x00,0x00,0x0f,0xff,0xff,0xf0,0x00,0x00
.DB 0x00,0x0f,0xff,0xff,0xff,0xff,0x00,0x00
.DB 0x00,0xff,0xff,0xff,0xff,0xff,0xf0,0x00
.DB 0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0x00
.DB 0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0x00
.DB 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0
.DB 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0
.DB 0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0x00
.DB 0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0x00
.DB 0x00,0xff,0xff,0xff,0xff,0xff,0xf0,0x00
.DB 0x00,0x0f,0xff,0xff,0xff,0xff,0x00,0x00
.DB 0x00,0x00,0x0f,0xff,0xff,0xf0,0x00,0x00
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
.DB 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
Ef_flash_end:


Ef_Rainbow_splash16:
.DB 0x12,0x34,0x56,0x71,0x23,0x45,0x67,0x12
.DB 0x23,0x45,0x67,0x12,0x34,0x56,0x71,0x23
.DB 0x34,0x56,0x71,0x23,0x45,0x67,0x12,0x34
.DB 0x45,0x67,0x12,0x34,0x56,0x71,0x23,0x45
.DB 0x56,0x71,0x23,0x45,0x67,0x12,0x34,0x56
.DB 0x67,0x12,0x34,0x56,0x71,0x23,0x45,0x67
.DB 0x71,0x23,0x45,0x67,0x12,0x34,0x56,0x71
Ef_Rainbow_splash16_end:

ColorTable:	// G, R, B, reserved
.DB	0,0,0,		0	// 0 black
.DB 0,0x83,0,	0	// 1 red
.DB 0x26,0xa7,0,	0	// 2 orange
.DB 0xb6,0xdc,0,	0	// 3 yellow
.DB 0xff,0,0,	0	// 4 green
.DB 0xa7,0x19,0xa9,	0	// 5 light blue
.DB 0,0,0xff,	0	// 6 blue
.DB 0,0x3e,0xb1,	0	// 7 violett
.DB 170,85,0,	0	// 8 brown
.DB 0,127,127,	0	// 9 rose
.DB 51,102,102, 0	// a light rose
.DB 170,0,85,	0	// b cyan
.DB 2,2,2,		0	// c dark gray
.DB 20,20,20,	0	// d gray
.DB 255,255,255,0	// e ultra white
.DB 85,85,85,	0	// f white

.dseg

// Переменные:
BegArr:			.BYTE	2	// адрес начала текущего эффекта (в авто-режиме)
EndArr:			.BYTE	2	// адрес конца текущего эффекта
NextProgLine:	.BYTE	2	// адрес след. строки программы эффектов


// выводимое значение в ленту (цвета распакованы)
StateParams:	.BYTE	TapeLen*3



