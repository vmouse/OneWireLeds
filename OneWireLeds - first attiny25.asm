/*
 * OneWireLeds.asm
 *
 *  Created: 10.12.2013 21:24:08
 *   Author: vlad
 */ 
#define F_CPU 16000000
#define	BAUD		57600
#define USE_USART
#define	USE_UART_text
#define UART_MaxInputSize 128	// maximum length of input line
#define TERMINAL_ECHO	// - enable input echo 
//#define UART_SIMULATE
//#define	NODELAY

.def	tempa	= r16
.def	tempb	= r17
.def	tempc	= r18
.def	State	= r19		// текущее состояние

.def	ManualCmd		= r20 // флаги ручного ввода: bit0 - есть команда, bit7 - ручная команда в процессе выполнения, после выполнения - стоп. bit6 - непрерывный цикл выполнения стадии Change
.def	RepeatCnt		= r21		
.def	RaiseSteps		= r22
.def	FallSteps		= r23
.def	NextFallSteps	= r24
.def	WaitDelay		= r25

.def	DelayCount	= r2	//  Speed regulator
.def	ModeFlag	= r3	// reverse|mirror fag
.def	StepCnt		= r8	// step counter in current state 

/*
.def	BegArrLo	= r4	// begin array of current effect
.def	BegArrHi	= r5
.def	EndArrLo	= r6	// end array of current effect
.def	EndArrHi	= r7
*/


.equ	TapeLen		= 12	// length of led tape
.equ	ManualLen	= 32	// max length of led tape for manual output
.equ	led			= PB0	// data port

.equ	DelayConst	= 200 //200
.equ	StateChange = 1
.equ	StateWait	= 2

.include "..\..\libasm\avr.inc"

; OutByte	Reg (not tempa!)
.macro	OutByte 	
	ldi		tempa, 8
@0_LOOP:
	sbi		PORTB, Led
	rol		@0
	brcc	@0_LP1
	rcall	WAIT400
@0_LP1:
	cbi		PORTB, Led
	brcs	@0_LP2
	rcall	WAIT400
@0_LP2:
	dec		tempa
	brne	@0_LOOP
.endm

.macro SwapReg // меняем местами значения первых двух указанных регистров через третий
	mov		@2, @0	
	mov		@0, @1
	mov		@1, @2
.endm

// IRQ vectors ATMEGA328p
.org 0
	rjmp RESET ; Reset Handler
	reti	;jmp EXT_INT0 ; IRQ0 Handler
	reti	;jmp EXT_INT1 ; IRQ1 Handler
	reti	;jmp PCINT0 ; PCINT0 Handler
	reti	;jmp PCINT1 ; PCINT1 Handler
	reti	;jmp PCINT2 ; PCINT2 Handler
	reti	;jmp WDT ; Watchdog Timer Handler
	reti	;jmp TIM2_COMPA ; Timer2 Compare A Handler
	reti	;jmp TIM2_COMPB ; Timer2 Compare B Handler
	reti	;jmp TIM2_OVF ; Timer2 Overflow Handler
	reti	;jmp TIM1_CAPT ; Timer1 Capture Handler
	reti	;jmp TIM1_COMPA ; Timer1 Compare A Handler
	reti	;jmp TIM1_COMPB ; Timer1 Compare B Handler
	reti	;jmp TIM1_OVF ; Timer1 Overflow Handler
	reti	;jmp TIM0_COMPA ; Timer0 Compare A Handler
	reti	;jmp TIM0_COMPB ; Timer0 Compare B Handler
	reti	;jmp TIM0_OVF ; Timer0 Overflow Handler
	reti	;jmp SPI_STC ; SPI Transfer Complete Handler
	reti	;jmp USART_RXC ; USART, RX Complete Handler
	reti	;jmp USART_UDRE ; USART, UDR Empty Handler
	reti	;jmp USART_TXC ; USART, TX Complete Handler
	reti	;jmp ADC ; ADC Conversion Complete Handler
	reti	;jmp EE_RDY ; EEPROM Ready Handler
	reti	;jmp ANA_COMP ; Analog Comparator Handler
	reti	;jmp TWI ; 2-wire Serial Interface Handler
	reti	;jmp SPM_RDY ; Store Program Memory Ready Handler

RESET:

// Stack init:
	outi	SPH, high(RAMEND)
	outi	SPL, low(RAMEND)

// WatchDog init:
	wdr
	stsi	WDTCSR, (1<<WDCE)|(1<<WDE)	// change mode enable
	stsi	WDTCSR, (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)|(0<<WDCE)|(1<<WDE) // reset after 8 sec hang

// Port init:
	outi	DDRB, (1<<Led)	// output for bus port

//	sei ; Enable interrupts

#ifdef	USE_USART
	rcall	UART_init
	ldiw	Z, msg_start*2		;Start up message
	rcall	UART_out_str
#endif

	rcall	InitVariables

	rcall	InitNextEffect

RepeatEffectLoop:
	tst		RepeatCnt
	brne	RepeatGo
	rcall	InitNextEffect		// Read next mode
RepeatGo:
	dec		RepeatCnt	

	sbrc	ModeFlag, 0			// Skip if Reverse bit not set
	rjmp	rgo_reverse
	ldsw	Z, BegArr			// Reload Z to begin of effect array
	rjmp	EffectLoop
rgo_reverse:
	ldsw	Z, EndArr			// Reload Z to end of effect array

EffectLoop:						// Begin change phase of effect:
#ifdef	USE_USART
get_next_cmd:
	sbrs	ManualCmd, 0		// bit0 - need to read command
	rjmp	auto_cmd			// No data and auto mode

check_manual_cmd:
	cbr		ManualCmd, bit0
	rcall	UART_receive		// read cmd byte
	rcall	UpperCase_char

//	First	group of parametrized commands
	cpi		r16, 'L'
	breq	Read_manual_line_data	// запросить линейку перехода в новое состояние и дождаться выполнения к нему

	cpi		r16, 'S'
	breq	Read_manual_single_data // установить текущую яркость заданного элемента, после чего будет выполняться переход из него

	rcall	ShowPrompt

// Second group - immediately run commands
	cpi		r16, 'R'
	brne	PC+2
	rjmp	RESET

	cpi		r16, 'A'
	brne	PC+2
	rjmp	RepeatEffectLoop

	cpi		r16, 'N'
	brne	PC+2
	rcall	InitNextEffect
	rjmp	RepeatEffectLoop

	cpi		r16, 'M'
	brne	cmd_m_end
	ldi		r16, bit1
	eor		ModeFlag, r16
	rjmp	auto_cmd
cmd_m_end:
	rjmp	check_manual_cmd

Read_manual_single_data:
	rcall	UART_get_line
	rcall	Set_Manual_Single
	sbr		ManualCmd, bit6
	sbr		ManualCmd, bit7
	rjmp	change_init

Read_manual_line_data:
	push	ZH
	push	ZL
	ldiw	Z, msg_data*2		;Ready to next command message
	rcall	UART_out_str
	pop		ZL
	pop		ZH
	rcall	UART_get_line
	rcall	Set_Manual_State
	sbr		ManualCmd, bit7
	rjmp	change_init
#endif

auto_cmd:
	sbrc	ManualCmd, 6
	rjmp	change_init			// выполняется ручной режим с вечным циклом изменения

	clr		ManualCmd
	rcall	ReadNextState		// читаем следующее состояние по указателю в Z
	brts	RepeatEffectLoop	// если вышли за границы эффекта, уходим на следующий повтор или эффект программы

change_init:
	mov		StepCnt, RaiseSteps
	rcall	Change_Phase
	brts	EffectFinnaly		// T = был ручной ввод через UART

	mov		StepCnt, WaitDelay
	rcall	Change_Phase

EffectFinnaly:
	// Read fall speed
	mov		FallSteps, NextFallSteps

	sbrc	ManualCmd, 7
	rcall	ShowPrompt
	cbr		ManualCmd, bit7

	rjmp	EffectLoop		 

// Erase vaiables and set to default:
InitVariables:
	ldiw	X, StateParams
	ldi		tempa, TapeLen*3*6
	clr		tempb
eraloop:
	st		X+, tempb
	dec		tempa
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
	mov		ModeFlag, tempa	// Set reverse/mirror/Speed flag
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


ShowPrompt:		// UART Show prompt
#ifdef USE_USART
	push	ZH
	push	ZL
	push	tempa
	ldiw	Z, msg_prompt*2		;Ready to next command message
	rcall	UART_out_str
	pop		tempa
	pop		ZL
	pop		ZH
#endif
	ret

// Out led array
Output_ledtape:
	push	tempa
	push	tempc
	push	XH
	push	XL

	ldi		tempc, TapeLen * 3
	ldiw	X, StateParams + 5
outloop:
//	ld		tempb, -X
	ld		tempb, X+
	OutByte	tempb

//	sbiw	X, 5
	adiw	X, 5
	dec		tempc
	brne	outloop
	cbi		PORTB, Led

	pop		XL
	pop		XH
	pop		tempc
	pop		tempa
ret

Manual_Output_ledtape:
	push	tempa
	push	tempc
	push	XH
	push	XL

	ldiw	Y, StateManual * 2
	ldi		tempc, ManualLen * 3
moutlp:
	ld		tempb, -X
	OutByte	tempb

	dec		tempc
	brne	moutlp
	cbi		PORTB, Led

	pop		XL
	pop		XH
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
		subiw	Z, TapeLen*3+4	// в режиме реверса отходим на предыдущий шаг эффекта

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
		ldi		tempc, TapeLen*3	// длина массива состояний
		sbrc	ModeFlag, 1		// пропуск если бит Mirror сброшен
		rjmp	rns_set_end
		ldiw	X, StateParams  // встаем в начало массива управления сменой состояний
		rjmp	rns2

rns_set_end:
		ldiw	X, StateParams+(TapeLen-1)*3*6	// встаем в конец массива управления сменой состояний

rns2:
		lpm		tempa, Z+			// будущее значение G
		rcall	Calc_Step_params16	// расчет параметров Зеленого канала (G)
		lpm		tempa, Z+			// будущее значение R
		rcall	Calc_Step_params16	// расчет параметров Красного канала (R)
		lpm		tempa, Z+			// будущее значение B
		rcall	Calc_Step_params16	// расчет параметров Синего канала (B)

		sbrc	ModeFlag, 1		// пропуск если бит Mirror очищен
		sbiw	X,2*3*6			// откатываемся на предыдущий элемент если идем зеркально

rns_cnt:
		subi	tempc,3
		brne	rns2
		clt						// выход без ошибки
		sbrs	ModeFlag, 0		// пропуск если Reverse
		ret
		subiw	Z, TapeLen*3+4	// в режиме реверса ходим назад
		ret


CalcPWM:
		ldiw	X, StateParams
		ldi		tempc, TapeLen*3
cp_loop16:
		ld		r14, X+	// next state
		ld		r16, X+	// direction (zerro if raise, else fall)
		ld		r11, X+	// Xlo
		ld		r12, X+	// Xhi
		ld		r13, X+	// CurLo
		ld		r15, X	// CurHi
		subiw	X, 1

		sbrs	r16, 7	// directiom
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
		mov		r13, r11

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
#ifdef	USE_USART
		rcall	UART_check_for_data
		brtc	dl3
		sbr		ManualCmd, bit0
dl3:
#endif

		pop		tempb
		pop		tempa
ret

#ifdef	USE_USART
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
		; читаем настройки перехода
		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin
		mov		r10, r16		// номер элемента

		ld		r17, Y+
		ld		r18, Y+
		rcall	Hex2ToBin		// r16 = яркость

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

.include "UART.ASM"

msg_start:	
			.db	0x0d, 0x0a, "WS2812 to UART gate "
msg_prompt:
			.db 0x0d, 0x0a, "CMD>", 0, 0
msg_data:
			.db 0x0d, 0x0a, "DATA>", 0
#endif

.DW		0,0,0, 0,0,0, 0,0,0,0,0,0,0,0,0,0,0

Modes:	
// 4 bytes for mode: ModeFlag, RepeatCnt, Begin addr, End addr
//	ModeFlag:  0bit = Reverse, 1bit = Mirror, 2,3 - reserve, 4-8 - speed 

//.DW		0x01E2, Ef_White*2, Ef_White_End*2

//.DW		0x0101, Ef_Flash_BW*2, Ef_Flash_BW_End*2
//.DW		0x18E2, Ef_Rainbow*2, Ef_Rainbow_End*2


//.DW		0x0300, Ef_Flash_BW_soft*2, Ef_Flash_BW_soft_End*2

//.DW		0x0200, Ef_Light_BW_pair*2, Ef_Light_BW_pair_End*2

//.DW		0x0500, Ef_Light_BW_dark_wave*2, Ef_Light_BW_dark_wave_End*2
//.DW		0x1082, Ef_Rainbow_splash*2, Ef_Rainbow_splash_End*2
//.DW		0x1080, Ef_Rainbow_splash*2, Ef_Rainbow_splash_End*2

.DW		0x0100, Ef_Flash_BW_soft*2, Ef_Flash_BW_soft_End*2

.DW		0x0102, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x0222, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x0342, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x0462, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x0782, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x0AA2, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x15C2, Ef_Rainbow*2, Ef_Rainbow_End*2
.DW		0x18E2, Ef_Rainbow*2, Ef_Rainbow_End*2

.DW		0x0122, Ef_Rainbow_to_splash*2, Ef_Rainbow_to_splash_End*2
.DW		0x1082, Ef_Rainbow_splash*2, Ef_Rainbow_splash_End*2
.DW		0x1080, Ef_Rainbow_splash*2, Ef_Rainbow_splash_End*2

.DW		0x0100, Ef_Rainbow_splash_to_all*2, Ef_Rainbow_splash_to_all_End*2
.DW		0x0400, Ef_RainAll*2, Ef_RainAll_End*2

.DW		0x0500, Ef_Raindow_down*2, Ef_Raindow_down_End*2

.DW		0x0100, Ef_Light_BW_dark_wave_to_Rainwave*2, Ef_Light_BW_dark_wave_to_Rainwave_End*2 

.DW		0x1800, Ef_Rainbow_wave*2, Ef_Rainbow_wave_End*2
.DW		0x1801, Ef_Rainbow_wave*2, Ef_Rainbow_wave_End*2

.DB		0xff, 0xff


/*
Ef_Test:
.DB   255,50,255,1,  1,2,3,  4,5,6,  7,8,9,  10,11,12,  13,14,15,  16,17,18,  19,20,21,  22,23,24,  25,26,27,  28,29,30,  31,32,33,  34,35,36
.DB   255,50,255,1,  65,66,67,  68,69,70,  71,72,73,  74,75,76,  77,78,79,  80,81,82,  83,84,85,  86,87,88,  89,90,91,  92,93,94,  95,96,97,  98,99,100
//.DB   255,50,255,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
Ef_Test_End:
*/

Ef_Black:
.DB   1,20,1,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
Ef_Black_End:

Ef_White:
.DB   1,20,1,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
Ef_White_End:

Ef_Flash_BW:
.DB   8,0,8,1,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255,  255,255,255
//.DB   63,0,63,1,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,0,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
Ef_Flash_BW_End:


Ef_Flash_BW_soft:
.DB   127,128,127,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
.DB   127,128,127,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
Ef_Flash_BW_soft_End:

Ef_Rainbow:
.DB   16,0,16,1,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255,  128,0,128
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170,  0,0,255
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,85,170

Ef_Rainbow_End:

Ef_Rainbow_to_splash:
.DB   24,0,20,1,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0,  0,0,0
.DB   24,0,20,1,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,0,0
.DB   24,0,20,1,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0
.DB   24,0,20,1,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0
.DB   24,0,20,1,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0
Ef_Rainbow_to_splash_End:

Ef_Rainbow_splash:
.DB   64,6,64,1,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0
.DB   64,6,64,1,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128
.DB   64,6,64,1,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255
.DB   64,6,64,1,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170
.DB   64,6,64,1,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0
.DB   64,6,64,1,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0
.DB   64,6,64,1,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0,  85,170,0,  0,255,0,  0,85,170,  0,0,255,  128,0,128,  255,0,0,  128,128,0
Ef_Rainbow_splash_End:

Ef_Raindow_down:
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   128,0,8,1,  255,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  255,0,0,  255,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  255,0,0,  255,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   128,0,8,1,  0,0,255,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  0,0,255,  0,0,255,  0,0,0,  0,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   128,0,8,1,  0,128,128,  0,0,0,  0,0,0,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   16,0,16,1,  0,0,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   16,0,16,1,  0,0,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   16,0,16,1,  0,0,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   128,0,8,1,  0,255,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   16,0,16,1,  0,255,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  170,85,0,  170,85,0,  170,85,0
.DB   16,0,16,1,  0,0,0,  0,255,0,  0,255,0,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  255,0,0,  255,0,0,  255,0,0
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,255,0,  0,255,0,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,0,255,  0,0,255,  0,0,255
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,128,128,  0,128,128,  0,128,128
.DB   16,0,16,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,255,0,  0,255,0,  0,255,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   128,0,8,1,  170,85,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   8,8,8,1,  0,0,0,  170,85,0,  170,85,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   8,8,8,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  0,0,0,  0,0,0,  0,0,0
Ef_Raindow_down_End:

Ef_Rainbow_splash_to_all:
.DB   8,0,8,1,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255
.DB   8,0,8,1,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85,  0,255,0,  170,85,0,  255,0,0,  128,0,128
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85,  0,255,0,  170,85,0,  255,0,0
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85,  0,255,0,  170,85,0
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85,  0,255,0
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128,  0,170,85
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255,  0,128,128
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128,  0,0,255
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0,  128,0,128
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  255,0,0
.DB   8,0,8,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0
Ef_Rainbow_splash_to_all_End:

Ef_RainAll:
.DB   255,255,255,1,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  0,255,0
.DB   255,255,255,1,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,170,85
.DB   255,255,255,1,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,128,128
.DB   255,255,255,1,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,0,255
.DB   255,255,255,1,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  128,0,128
.DB   255,255,255,1,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0
.DB   255,255,255,1,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0
Ef_RainAll_End:

Ef_Light_BW_pair:
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  85,85,85
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  85,85,85,  0,0,0,  85,85,85,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  85,85,85
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  85,85,85,  0,0,0,  85,85,85,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  85,85,85
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  85,85,85,  0,0,0,  85,85,85,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  85,85,85
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  0,0,0,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,64,1,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
Ef_Light_BW_wave:
.DB   4,10,32,1,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,10,32,1,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,10,32,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
.DB   4,10,32,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0
.DB   4,50,32,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
Ef_Light_BW_wave_End:
Ef_Light_BW_pair_End:

Ef_Light_BW_dark_wave:
.DB   4,16,16,1,  0,0,0,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
.DB   4,16,16,1,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
.DB   4,16,16,1,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
.DB   4,16,16,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  85,85,85,  85,85,85,  85,85,85
.DB   4,16,16,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  0,0,0,  0,0,0,  0,0,0
.DB   4,16,16,1,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85,  85,85,85
Ef_Light_BW_dark_wave_End:

Ef_Light_BW_dark_wave_to_Rainwave:
.DB   8,0,16,1,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,0,16,1,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  255,0,0
.DB   8,0,16,1,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  128,0,128
.DB   8,0,16,1,  170,85,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  0,0,255,  0,0,255,  0,0,255
.DB   8,0,16,1,  170,85,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,128,128,  0,128,128,  0,128,128
.DB   8,0,16,1,  255,0,0,  128,0,128,  128,0,128,  0,0,255,  0,0,255,  0,128,128,  0,128,128,  0,128,128,  0,128,128,  0,170,85,  0,170,85,  0,170,85
Ef_Light_BW_dark_wave_to_Rainwave_End:

Ef_Rainbow_wave:
.DB   8,0,16,1,  128,0,128,  0,0,255,  0,0,255,  0,128,128,  0,128,128,  0,170,85,  0,170,85,  0,170,85,  0,170,85,  0,255,0,  0,255,0,  0,255,0
.DB   8,0,16,1,  0,0,255,  0,128,128,  0,128,128,  0,170,85,  0,170,85,  0,255,0,  0,255,0,  0,255,0,  0,255,0,  170,85,0,  170,85,0,  170,85,0
.DB   8,0,16,1,  0,128,128,  0,170,85,  0,170,85,  0,255,0,  0,255,0,  170,85,0,  170,85,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  255,0,0
.DB   8,0,16,1,  0,170,85,  0,255,0,  0,255,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  128,0,128
.DB   8,0,16,1,  0,255,0,  170,85,0,  170,85,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  128,0,128,  128,0,128,  0,0,255,  0,0,255,  0,0,255
.DB   8,0,16,1,  170,85,0,  255,0,0,  255,0,0,  128,0,128,  128,0,128,  0,0,255,  0,0,255,  0,0,255,  0,0,255,  0,128,128,  0,128,128,  0,128,128
Ef_Rainbow_wave_End:

.dseg

// Переменные:
BegArr:		.BYTE	2	// адрес начала текущего эффекта (в авто-режиме)
EndArr:		.BYTE	2	// адрес конца текущего эффекта

NextProgLine:	.BYTE	2 // адрес след. строки программы эффектов

// Для каждого цвета RGB по 6 байт:
//	- След.состояние , 
//	- Флаги, 
//	- Параметры приращения перехода: 
//	- Текущее состояние
StateParams:	.BYTE	TapeLen*3*6

// Ручной вывод через терминал
StateManual:	.BYTE	ManualLen*3+4 


