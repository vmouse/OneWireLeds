/*
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
.def	ModeFlag		= r19 // flag bits: 0 = reverse, 1 = mirror, 7 - auto next state

.def	ManualCmd		= r20 // флаги ручного ввода: bit0 - есть команда, bit7 - ручная команда в процессе выполнения, после выполнения - стоп. bit6 - непрерывный цикл выполнения стадии Change
.def	RepeatCnt		= r21		
.def	RaiseSteps		= r22
.def	FallSteps		= r23
.def	NextFallSteps	= r24
.def	WaitDelay		= r25

.def	DelayCount	= r2	//  Speed regulator
.def	StepCnt		= r8	// step counter in current state 


.equ	TapeLen		= 6	// length of led tape

.equ	led			= PB0	// led out pin
.equ	sens		= PB1   // sensor in port
.equ	led_port	= PORTB // data port
.equ	led_ddr		= DDRB  // direct databort

.equ	DelayConst	= 250//200
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
	outi	led_ddr, (1<<Led)	// output for bus port


	rcall	InitVariables
	rcall	InitEffectState
	rcall	ReadNextState

	sbr		ModeFlag, bit7

MainLoop:						// Основной цикл обработки перехода состояний

	//		Фаза перехода
	mov		StepCnt, RaiseSteps
	rcall	Change_Phase
	brts	ManualInput		// T = был ручной ввод через UART

	//		Фаза задержки
	mov		StepCnt, WaitDelay
	rcall	Change_Phase	
	brts	ManualInput		// T = был ручной ввод через UART

	mov		FallSteps, NextFallSteps	// уст. скорость затухания текущего эффекта для следующей фазы перехода

	sbrc	ModeFlag, 7		// пропуск след. если не авторежим
	rcall	NextEffectState // устанавливаем новые значения к которым переходить

	rjmp	MainLoop

ManualInput:
#ifdef	UART_USE_HARD
	cbr		ManualCmd, bit0
	rcall	UART_receive		// read cmd byte
	rcall	UpperCase_char

//	First	group of parametrized commands
	cpi		r16, 'L'
	breq	Read_manual_line_data	// запросить линейку перехода в новое состояние и дождаться выполнения к нему

	cpi		r16, 'S'
	breq	Read_manual_single_data // установить текущую яркость заданного элемента, после чего будет выполняться переход из него

	cpi		r16, 'C'
	breq	Read_manual_single_color // установть заданный цвет из таблицы для светодиода (тройка цветов)
	
	cpi		r16, 'O'
	brne	mi_next3
	rjmp	Read_manual_all_color // установить заданный цвет RGB для всей ленты и не менять его (выключить автопереход)
mi_next3:

	rcall	ShowPrompt	// Second group - immediately run commands
	cpi		r16, 'R'
	breq	RESET
	cpi		r16, 'A'
	breq	CmdAutoMode
	cpi		r16, 'T'
	breq	CmdStepMode
	cpi		r16, 'N'
	breq	CmdNextState
	cpi		r16, 'M'
	breq	CmdMirror
	rjmp	MainLoop

CmdAutoMode:
	sbr		ModeFlag, bit7
	rjmp	MainLoop
CmdStepMode:
	cbr		ModeFlag, bit7
	rjmp	MainLoop
CmdNextState:
	rcall	NextEffectState
	rjmp	MainLoop
CmdMirror:
	ldi		tempb, bit1
	eor		ModeFlag, tempb
	rjmp	MainLoop


#endif

	rjmp	MainLoop


NextEffectState:				// Переход на след.эффект
	rcall	ReadNextState
	rettc

	dec		RepeatCnt
	brne	nes_set_pointer
InitEffectState:
	rcall	InitNextEffect
nes_set_pointer:
	sbrc	ModeFlag, 0			// Skip if Reverse bit not set
	rjmp	nes_reverse
nes_forward:
	ldsw	Z, BegArr			// Reload Z to begin of effect array
	rjmp	NextEffectState
nes_reverse:
	ldsw	Z, EndArr			// Reload Z to end of effect array
	rjmp	NextEffectState

#ifdef	UART_USE_HARD

Read_manual_single_data:
	rcall	UART_get_line
	rcall	Set_Manual_Single
	rcall	ShowPrompt
	rjmp	CmdStepMode

Read_manual_single_color:
	rcall	UART_get_line
	rcall	Set_Manual_Color
	rcall	ShowPrompt
	rjmp	CmdStepMode

Read_manual_all_color:
	rcall	UART_get_line
	rcall	Set_Manual_All_Color
	rcall	ShowPrompt
	rjmp	CmdStepMode

Read_manual_line_data:
	pushw	Z
	ldiw	Z, msg_data*2		;Ready to next command message
	rcall	UART_out_str
	popw	Z
	rcall	UART_get_line
	rcall	Set_Manual_State
	rcall	ShowPrompt
	rjmp	CmdStepMode
#endif

// Erase vaiables and set to default:
InitVariables:
	ldiw	X, StateParams
	ldiw	Y, TapeLen*3*6
//	ldi		tempa, TapeLen*3*6
	clr		tempb
eraloop:
	st		X+, tempb
//	dec		tempa
	subiw	Y, 1
	brne	eraloop

	// set start values
	stsi	NextProgLine, Low(Modes*2)
	stsi	NextProgLine+1, High(Modes*2)
	ret

InitNextEffect:				// read effect parameters from Modes array
	clr		ManualCmd
	ldsw	Z, NextProgLine // Load program address (Modes)
	lpm		tempa, Z+		//Read ModeFlag  or FF - end of program
	cpi		tempa, 0xff
	brne	ine_read_params
	stsi	NextProgLine, Low(Modes*2)
	stsi	NextProgLine+1, High(Modes*2)
	rjmp	InitNextEffect
ine_read_params:
	mov		tempb, tempa
	andi	ModeFlag, 0xf0
	andi	tempb, 0x0f
	or		ModeFlag, tempb
	com		tempa
	andi	tempa, 0xf0
	mov		DelayCount, tempa
	lpm		RepeatCnt, Z+	// RepeatCnt
	lpm		tempa, Z+		// Begin addr low
	sts		BegArr, tempa
	lpm		tempa, Z+		// Begin addr high
	sts		BegArr+1, tempa
	lpm		tempa, Z+		// End addr low
	sts		EndArr, tempa
	lpm		tempa, Z+		// End addr high
	sts		EndArr+1, tempa
	stsw	NextProgLine, Z // Store program address (Modes)
	ret 


// Change Loop: raise light with raise speed, fall light with fall speed or wait
Change_Phase:
	tst		StepCnt
	reteq	// return if equal
	rcall	CalcPWM
	rcall	Output_ledtape
	rcall	delay_output	//  задержка определяющая скорость перехода (тут происходит опрос UART, T=1 если есть команда)
	retts
	dec		StepCnt
	rjmp	Change_Phase


// Out led array
Output_ledtape:
	push	tempa
	push	tempc
	pushw	X

	ldi		tempc, TapeLen * 3
	ldiw	X, StateParams + 5
outloop:
//	ld		tempb, -X
	ld		tempb, X+
	
	swap	tempb
	andi	tempb,0x0f

	OutByte	tempb

//	sbiw	X, 5
	adiw	X, 5
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

// division tempa:tempb / tempc
// out:
// r11:r10 - result
// tempa:tempb - остаток
div16x8:
	clr		r10
	clr		r11
	tst		tempc
	brne	div16_check
	dec		r10
	rjmp	div16_result
div16_check:
	sub		tempb, tempc
	sbci	tempa, 0
	brcs	div16_result
	inc		r10
	brne	div16_check
	inc		r11
	rjmp	div16_check
div16_result:
	add		tempb, tempc
	ret

// берем из r16 следующий параметр
// пишем их в [X] и считаем приращение перехода (X16) в новое значение из старого за FallSteps/RaiseSteps шагов
// Для каждого цвета RGB по 6 байт:
//	- След.состояние (8 бит), 
//	- Флаги (0 Raise, ff - fall), 
//	- Приращение перехода x 256 (16 бит): XLo, XHi
//	- Текущее состояние x 256 (16 бит): Lo, Hi

Calc_Step_params16:
		push	tempb
		push	tempc
		push	r10
		push	r11
		push	r12
		st		X+, r16		// пишем будущее значения перехода в PWM

		adiw	X, 4		// прыгаем в конец записи, забрать текущее / 256 (т.е. старший байт)
		ld		r12, X		// R12 = текущеее значения (нужно для расчета коэф.перехода X1, X2, X3)

		sbiw	X, 4		// возвращаемся
		clr		r10
		cp		r16, r12	// Определяем направление (зажигание или затухание)
		brcc	csp16_inc_mode
csp16_dec_mode:
		dec		r10	// флаг затухания = FF
		SwapReg	r16, r12, tempb // меням r11 <=> r12
		mov		tempc, FallSteps
		rjmp	csp16_calc
csp16_inc_mode:
		mov		tempc, RaiseSteps

csp16_calc:		
		st		X+, r10			// пишем направление (затухание/зажигание)

		// расчет X = Delta * 256 / Steps = (r11-r12)*256/tempc
		sub		tempa, r12		// r13 = delta (текущее - будущее)
		clr		tempb			// tempa:tempb = delta * 256
		rcall	div16x8			// tempa:tempb / tempc
		
		st		X+, r10			// пишем Xlo
		st		X+, r11			// пишем Xhi

		adiw	X, 2			// пропускаем текущее значение
		pop		r12
		pop		r11
		pop		r10
		pop		tempc
		pop		tempb
ret


; transfer next mode from lights to PWM array
; Z - pointer to mode in lights array
; tempa - broken
; T-flag = clear - Ok, Set - Out of range
ReadNextState:
		wdr						// сброс таймера собаки
		set						// взводим флаг ошибки
		sbrs	ModeFlag, 0		// пропуск если Reverse 
		rjmp	rns_check_beg
		subiw	Z, TapeLen/2+4	// в режиме реверса отходим на предыдущий шаг эффекта

rns_check_beg:					// проверка на выходы за пределы массива эффекта, при обратном ходе
		ldsw	Y, BegArr
		cpw		Z, Y
		brcc	rns_check_end	// Z >= begin
		ret

rns_check_end:					// проверка на выходы за пределы массива эффекта, при прямом ходе
		ldsw	Y, EndArr
		cpw		Z, Y		
		brcs	rns_init		// Z < end
		ret		// выход с ошибкой (за концом списка режимов)

rns_init:
		lpm		RaiseSteps, Z+
rns1:	; читаем оставшиеся байты задерек
		lpm		WaitDelay, Z+
		lpm		NextFallSteps, Z+
		adiw	Z,1				// пропускаем байт выравнивания (в ручном режиме это кол-во светодиодов в линии)

		; читаем след.состояние
		ldi		tempc, TapeLen/2	// длина массива состояний
		sbrc	ModeFlag, 1		// пропуск если бит Mirror сброшен
		rjmp	rns_set_end
		ldiw	X, StateParams  // встаем в начало массива управления сменой состояний
		rjmp	rns2

rns_set_end:
		ldiw	X, StateParams+(TapeLen-1)*3*6	// встаем в конец массива управления сменой состояний

rns2:
		lpm		tempa, Z+			// берем два значения цвета
		push	tempa
		swap	tempa
		andi	tempa, 0x0f
		rcall	Calc_Color
		pop		tempa
		andi	tempa, 0x0f
		rcall	Calc_Color

rns_cnt:
		dec		tempc
		brne	rns2
		clt						// выход без ошибки
		sbrs	ModeFlag, 0		// пропуск если Reverse
		ret
		subiw	Z, TapeLen/2+4	// в режиме реверса ходим назад
		ret

Calc_Color:	// calc color  by index in tempa (0..15)
		pushw	Z
		lsl		tempa
		lsl		tempa
		ldiw	Z, ColorTable*2
		add		ZL, tempa
		clr		tempa
		adc		ZH, tempa
		lpm		tempa, Z+
		rcall	Calc_Step_params16
		lpm		tempa, Z+
		rcall	Calc_Step_params16
		lpm		tempa, Z+
		rcall	Calc_Step_params16
		popw	Z

		sbrc	ModeFlag, 1		// пропуск если бит Mirror очищен
		sbiw	X,2*3*6			// откатываемся на предыдущий элемент если идем зеркально
		ret


// вариант расчета:
// деление на каждом шагу, храним текущее значение, будущее и где-то в регистре оставшееся кол-во шагов (нужно оптимизировать деление на 1 и 2)
// соотв. делаем деление разницы на шаги, из плюсов - не нужно проверять граничные условия
// curr + next = 2 байта на цвет
CalcPWM:
		ldiw	X, StateParams
		ldi		tempc, TapeLen*3
cp_loop16:
		ld		r14, X+	// next state
		ld		r16, X+	// direction (zerro if raise, else fall)
		ld		r11, X+	// Xlo - приращение на шаг
		ld		r12, X+	// Xhi
		ld		r13, X+	// CurLo - текущее значение
		ld		r15, X	// CurHi
		subiw	X, 1

//		sbrs	r16, 7	// direction
		cp		r14, r15 
		brcs	cp16_fall			// fall mode if next < current
		rjmp	cp16_raise

cp16_fall:
		sub		r13, r11
		sbc		r15, r12
		brcs	cp_overrange16		// exit if current < 0
		cp		r14, r15
		brcc	cp_overrange16		// exit if current <= next
		rjmp	cp_storeval16

cp16_raise:
		add		r13, r11
		adc		r15, r12
		brcs	cp_overrange16		// exit if current >255
		cp		r15, r14
		brcc	cp_overrange16		// exit if current >= next
		rjmp	cp_storeval16

cp_overrange16:
		mov		r15, r14
		clr		r13
//		mov		r13, r11

cp_storeval16:
		st		X+, r13
		st		X+, r15
cp_end16:
		dec		tempc
		brne	cp_loop16
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
#ifdef	UART_USE_HARD
		rcall	UART_check_for_data
		brtc	dl3
		sbr		ManualCmd, bit0
dl3:
#endif

		pop		tempb
		pop		tempa
ret

#ifdef	UART_USE_HARD
Set_Manual_State:				// установить текущий эффект (будущие значения в которые перейти) из строки ввода
		wdr
		ldiw	X, StateParams  // встаем в начало массива управления сменой состояний
		ldiw	Y, UART_LineBuf // встаем в начало полученной строки
		; читаем настройки перехода
		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin
		mov		RaiseSteps, r16

		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin
		mov		WaitDelay, r16

		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin
		mov		NextFallSteps, r16

		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin
		mov		r10, r16		

		; читаем состояние светодиодов
scl_1:
		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin			// [r17][r18] => r16
		rcall	Calc_Step_params16	// расчет параметров канала 
		dec		r10
		brne	scl_1
ret

Set_Manual_Single:		// установить текущее значение конкретного элемента
		wdr
		ldiw	Y, UART_LineBuf // встаем в начало полученной строки
		rcall	ByteFromHexLine
		mov		r10, r16		// номер элемента
		rcall	ByteFromHexLine	// яркость

		;переходим на элемент
		ldiw	X, StateParams + 5 // встаем в начало массива управления сменой состояний
sms_1:	tst		r10
		breq	sms_2
		dec		r10
		adiw	X, 6			// след.элемент		
		rjmp	sms_1
sms_2:
		st		X, r16
ret

Set_Manual_Color:		// установить текущее значение цвета заданной тройки (светодиода)
		wdr
		ldiw	Y, UART_LineBuf // встаем в начало полученной строки
		rcall	ByteFromHexLine
		mov		r10, r16		// номер элемента
		rcall	ByteFromHexLine // r16 = цвет

		;переходим на элемент
		ldiw	X, StateParams+5 // встаем в начало массива управления сменой состояний
smc_1:	tst		r10
		breq	smc_2
		dec		r10
		adiw	X, 18			// след.светодиод		
		rjmp	smc_1

smc_2:
// calc color  by index in tempa (0..15)
		lsl		r16
		lsl		r16
		ldiw	Z, ColorTable*2
		add		ZL, r16
		clr		r16
		adc		ZH, r16
		lpm		tempa, Z+
		st		X, r16
		adiw	X, 6			// след.светодиод		
		lpm		tempa, Z+
		st		X, r16
		adiw	X, 6			// след.светодиод		
		lpm		tempa, Z+
		st		X, r16
ret

Set_Manual_All_Color:
		ldiw	Y, UART_LineBuf // встаем в начало полученной строки
		rcall	ByteFromHexLine
		mov		r10, r16		// R
		rcall	ByteFromHexLine 
		mov		r11, r16		// G
		rcall	ByteFromHexLine 
		mov		r12, r16		// B
		ldiw	X, StateParams // встаем в начало массива управления сменой состояний
		ldi		r17, TapeLen
rmac_1:
		mov		r16, r11
		rcall	Calc_Step_params16	// расчет параметров канала 
		mov		r16, r10
		rcall	Calc_Step_params16	// расчет параметров канала 
		mov		r16, r12
		rcall	Calc_Step_params16	// расчет параметров канала 
		dec		r17
		brne	rmac_1
ret

#endif


Modes:	
// 4 bytes for mode: ModeFlag, RepeatCnt, Begin addr, End addr
//	Mode:  0bit = Reverse, 1bit = Mirror, 2,3 - reserve, 4-8 - speed 

.DW		0x10E0, Ef_Rainbow_splash16*2, Ef_Rainbow_splash16_End*2
.DW		0xffff


Ef_Rainbow_splash16:
.DB 16,10,16,1,	0x12,0x34,0x56,0x71,0x23,0x45,0x67,0x12
.DB 16,10,16,1,	0x23,0x45,0x67,0x12,0x34,0x56,0x71,0x23
.DB 16,10,16,1,	0x34,0x56,0x71,0x23,0x45,0x67,0x12,0x34
.DB 16,10,16,1,	0x45,0x67,0x12,0x34,0x56,0x71,0x23,0x45
.DB 16,10,16,1,	0x56,0x71,0x23,0x45,0x67,0x12,0x34,0x56
.DB 16,10,16,1,	0x67,0x12,0x34,0x56,0x71,0x23,0x45,0x67
.DB 16,10,16,1,	0x71,0x23,0x45,0x67,0x12,0x34,0x56,0x71
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

// Для каждого цвета RGB по 6 байт:
//	- След.состояние , 
//	- Флаги, 
//	- Параметры приращения перехода: 
//	- Текущее состояние
StateParams:	.BYTE	TapeLen*3*6



