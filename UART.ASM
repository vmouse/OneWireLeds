;-----------------------------------------------------------------------------;
; Software implemented UART
;
; Send/Receive r16 to host via an ISP cable.
;
; NOTE: MOSI and MISO are inverted signals to connect to RS-232C line directly.
;       MISO pin must be configured as output before transmiting.
;       Don't enable any interrupt during these routines are being executed.
;
;				3.58M	 4MHz	4.19M	6MHz	6.144		8MHz	10MHz	12MHz	12.288	16MHz	20MHz
; 115.2kbps		 -		 -		-		-		16		21		27		33		34		44		56
;  57.6kbps			19		21		22		33		34		44		56		67		69		91		114
;  38.4kbps			29		 33		34		50		52		67		85		102		105		137		172
;  19.2kbps			 60		 67		71		102		105		138		173		208		211		-		-
;   9.6kbps			 122		144		208		211		-		-		-		-		-		-		-

; magic formula = K = ( F_CPU / (3 * BPS))-2 
; 8000000/3/38400-2 = 67.44 => 67

//#define UART_USE_SOFT		// - enable software UART
//#define UART_USE_HARD	// - enable hardware UART
//#define USE_UART_text		// - use text functions
//#define TERMINAL_ECHO	// - enable input echo 
//#define UART_MaxInputSize 128	// maximum length of input line
//#define UART_NORXBUF	// disable RX buffer
//#define SUART_INVERSE		// inverse TX signal (software UART)

#ifndef F_CPU
	#error "F_CPU must be defined!"
#endif

#ifndef BAUD
	#warning "BAUD not defined, use default = 38400"
	#define	BAUD	38400
#endif

#ifndef	UART_NORXBUF
#ifndef	UART_ReceiveBufSize
	#warning "Receive buffer not defined, use default size = 64"
	#define	UART_ReceiveBufSize	64
#endif
#endif

#ifdef UART_USE_SOFT //  software UART 
#if	(F_CPU/(3*BAUD))-2 > 255
	#error "Too low baud speed!"
#endif

#ifndef TX_PIN
	#warning "TX_PIN (MISO) pin bit not defined, use default = 1 (PB1)"
	#define TX_PIN	1
#endif

#ifndef RX_PIN
	#warning "RX_PIN (MOSI) pin bit not defined, use default = 0 (PB0)"
	#define RX_PIN	0
#endif

#ifndef PORT_OUT
	#warning "PORT_OUT not defined, use default = PORTB"
	#define PORT_OUT	PORTB
#endif

#ifndef PORT_IN
	#warning "PORT_IN not defined, use default = PINB"
	#define PORT_IN	PORTB
#endif

#ifndef PORT_OUT_DDR
	#warning "PORT_OUT_DDR not defined, use default = DDRB"
	#define PORT_OUT_DDR	DDRB
#endif

#ifndef UART_INIT
	#ifdef UART_USE_SOFT //  software UART 
		#warning "UART_INIT not defined, use default = SUART_init (software)"
		#define UART_INIT	SUART_init
	#else
		#warning "UART_INIT not defined, use default = HUART_init (hardware)"
		#define UART_INIT	HUART_init
	#endif
#endif

#ifndef UART_XMIT
	#ifdef UART_USE_SOFT //  software UART 
		#warning "UART_XMIT not defined, use default = SUART_xmit (software)"
		#define UART_XMIT	SUART_xmit
	#else
		#warning "UART_XMIT not defined, use default = HUART_xmit (hardware)"
		#define UART_XMIT	HUART_xmit
	#endif
#endif

#ifndef UART_RECEIVE
	#ifdef UART_USE_SOFT //  software UART 
		#warning "UART_RECEIVE not defined, use default = SUART_receive (software)"
		#define UART_RECEIVE	SUART_receive
	#else
		#warning "UART_RECEIVE not defined, use default = HUART_receive (hardware)"
		#define UART_RECEIVE	HUART_receive
	#endif
#endif


.equ	MOSI = RX_PIN
.equ	MISO = TX_PIN
.equ	PORTOUT = PORT_OUT
.equ	PORTIN	= PORT_IN
.equ	SBPS	= (F_CPU/(3*BAUD))-2

;.equ	SBPS	= 67	;See above.
;.equ	MOSI	= 1	;Bit position of MOSI and MISO on the I/O register
;.equ	MISO	= 0	;/
;.equ	PORTOUT	= PORTB
;.equ	PORTIN	= PINB

SUART_init:
	in		r16, PORT_OUT_DDR
	ori		r16, 1<<MISO
	out		PORT_OUT_DDR, r16
ret

SUART_xmit:	; Transmit r16. r17 and r18 will be broken. (14 words)
	cli
	com		r16			;
	sec					;Start bit
	ldi		r18,10		;bit count
	ldi		r17,SBPS-1	;Wait for full bit time
	dec		r17			;
	brne	PC-1		;loop
	brcs	PC+2		;MISO = bit for send
#ifdef		SUART_INVERSE
	cbi		PORTOUT,MISO
#else
	sbi		PORTOUT,MISO
#endif
	brcc	PC+2		

#ifdef		SUART_INVERSE
	sbi		PORTOUT,MISO
#else
	cbi		PORTOUT,MISO
#endif

	lsr		r16			;next bit
	dec		r18			;All sent ?
	brne	PC-9		; no, loop
	sei
	ret

SUART_receive: ; Receive into r16. r17 will be broken. (16 words)
	cli
	ldi		r16,0x80	;Shift register for receive
	sbic	PORTIN, MOSI	;Wait for rise edge of MOSI
	rjmp	PC-1		;
	sbis	PORTIN, MOSI	;
	rjmp	PC-1		;/
	ldi		r17,SBPS/2	;Wait for half bit time
	dec		r17		;
	brne	PC-1		;/
	ldi		r17,SBPS		;Wait for full bit time
	dec		r17		;
	brne	PC-1		;/
	lsr		r16		;Next bit
	sbis	PORTIN,MOSI	;Read MOSI into r16.7
	ori		r16,0x80	;/
	brcc	PC-6		;All bits received?  no, continue
	sei
	ret			;                    yes, exit
#endif		//  hardware USART 

#ifdef UART_USE_HARD
#define BPS (F_CPU/16/BAUD-1)

HUART_init:
	stsi	UBRR0H, high(BPS)
	stsi	UBRR0L, low(BPS)
	stsi	UCSR0B, (1<<RXEN0)|(1<<TXEN0)|(1<< RXCIE0) // enable receive, transmit and receive complete IRQ
	stsi	UCSR0C, ((1<<UCSZ00)|(1<<UCSZ01))
	
	stswi	UART_Recv_buf_head, UART_Recv_buf
	stsi	UART_Recv_buf_len, 0
	sei
	ret

// r16 - xmit data
// r17 - broken
HUART_xmit:
    lds		r17,UCSR0A
    sbrs	r17,UDRE0
    rjmp	HUART_xmit
    sts		UDR0, r16
    ret     

; T = set if data exists
; r16 - flags
UART_check_for_data:
#ifndef UART_SIMULATE
/*
	clt
	wdr
	lds		r16, UCSR0A           ;USART.WAIT(RXC)
	sbrc	r16, RXC0				;skip if no data
*/
	clt
	cli
	lds		r16, UART_Recv_buf_len
	tst		r16
	breq	uart_cfd_exit
#endif
	set
uart_cfd_exit:
	sei
	ret

// get value from round buffer into R16
// Set flag C - if no data
UART_get_from_buf:
	cli
	// check buffer length
	lds		r16, UART_Recv_buf_len
	subi	r16, 1
	brcs	uart_get_exit
uart_get_read:
	sts		UART_Recv_buf_len, r16
	pushw	Y
	ldsw	Y, UART_Recv_buf_head
	inc		r16
	sub		YL, r16
	sbci	YH, 0
	cpwi	Y, UART_Recv_buf
	brcc	urcv_readbyte
	addiw	Y, Low(UART_ReceiveBufSize)
urcv_readbyte:
	ld		r16, Y
	popw	Y
uart_get_exit:
	sei
ret

; r16 = data
HUART_receive:
#ifndef UART_SIMULATE
#ifdef	UART_NORXBUF
	rcall	UART_check_for_data
	brtc	HUART_receive			; wait data
	lds		r16, UDR0
#else	; Use RX buf
	rcall	UART_get_from_buf		; get data
	brcs	HUART_receive			; wait data
#endif
#else	; UART simulation
	ldi		r16,'O'
#endif
ret

// store value R16 into round buffer
UART_put_into_buf:
// check buffer overflow	
	push	r17
	lds		r17, UART_Recv_buf_len
	inc		r17
	cpi		r17, UART_ReceiveBufSize+1
	brcc	uart_put_exit
uart_put_store:
	sts		UART_Recv_buf_len, r17
	pushw	Y
	ldsw	Y, UART_Recv_buf_head
	st		Y+, r16
	cpwi	Y, UART_Recv_buf_end
	brcs	uart_put_in_range
	ldiw	Y, UART_Recv_buf	// reset head to begin
uart_put_in_range:
	stsw	UART_Recv_buf_head, Y
	popw	Y
uart_put_exit:
	pop		r17
ret

; IRQ handler
IRQ_UART_RX_Complete:
/*	push	r16
	lds		r16, UDR0
	sts		UDR0, r16
	pop		r16
reti
*/
	push	r16
	push	r18
	in		r18, SREG
	lds		r16, UDR0
	rcall	UART_put_into_buf
#ifdef TERMINAL_ECHO
//	lds		r16, UART_Recv_buf_len
	rcall	UART_XMIT		;Show character
#endif

	out		SREG, r18
	pop		r18
	pop		r16
reti

#endif		// hardware  UART


#ifdef USE_UART_text
;--------------------------------------;
;Send ROM string
;
;Call: Z = top of the string (ASCIZ)
;Ret:  Z = next string
UART_out_str:	
	lpm		r16, Z+
	tst		r16
	brne	PC+2
	ret
	rcall	UART_XMIT
	rjmp	UART_out_str

;--------------------------------------;
;Send DATA string
;
;Call: Y = top of the string (ASCIZ)
UART_out_str_Y:	
	ld		r16, Y+
	tst		r16
	brne	PC+2
	ret
	rcall	UART_XMIT
	rjmp	UART_out_str_Y


;Get byte line into UART_LineBuf
;r16, Y - broken
UART_get_line:
	ldiw	Y, UART_LineBuf
#ifndef UART_SIMULATE
UART_get_next_char:
	rcall	UART_RECEIVE

	cpi		r16, 0x20		;any invisible char (<0x20)= EOL
	brcc	ugl_check_char
	clr		r16
	st		Y, r16			; save into buffer
	ret
ugl_check_char:
//	cpi	r16, 0x20			; Unvisible code?
//	brcs	UART_get_line	; go to next char (not store)
	cpi		YH, high(UART_LineBuf + UART_MaxInputSize)		;Buffer overflow?
	brcs	ugl_inc_index
	cpi		YL, low(UART_LineBuf + UART_MaxInputSize)	
	brcs	ugl_inc_index
	sbiw	Y,1
ugl_inc_index:
	st		Y+, r16			; save into buffer
	rjmp	UART_get_next_char
#else	// use simulation
	push	ZH
	push	ZL
	ldiw	Z,UART_test_data*2
simtest_1:
	lpm		r16,Z+
	cpi		r16, 0x20
	brcs	simtest_end
	st		Y+,r16
	rjmp	simtest_1
simtest_end:
	pop		ZL
	pop		ZH
	ret
UART_test_data:		.db	"121314",0x0a
//UART_test_data:		.db	"111111240000FF0000FF0000FF0000FF0000FF0000FF0000FF0000FF0000FF0000FF0000FF0000FF",0x0d,0x0a
#endif	// simulation

// upper case char in r16
UpperCase_char:	
	cpi		r16,'a'
	brcs	PC+2		
	subi	r16,0x20
	ret


// conver HEX character into byte
// r16 = char
// Out: r16
//	C - if error
Hex1ToBin:
	rcall	UpperCase_char		
	subi	r16,'0'		; 
	brcs	cdtb_error	; <0? end
	cpi		r16, 10		;
	brcs	cdtb_end	; <10

	subi	r16,7		;>=10 switch to A..F => 10..15
	cpi		r16,16		;
	brcc	cdtb_error	;>=16 error

cdtb_end:
	clc
	ret

cdtb_error:	
	clr		r16
	sec					;Exit with error
	ret

;--------------------------------------;
;HEX to byte
;
;Call: r17, r18  chars of hi, lo halfbyte 
;        if    C=1: error
;        r16 - result
Hex2ToBin:
	mov		r16, r17
	rcall	Hex1ToBin
	brcs	htb_end		// error
	mov		r17,r16
	mov		r16, r18
	rcall	Hex1ToBin
	brcs	htb_end		// error
	swap	r17
	or		r16,r17
htb_end:
	ret

;Y -> text line, return converted binary in R16 
ByteFromHexLine:
	push	r17
	push	r18
	ld		r17, Y+
	ld		r18, Y+
	rcall	Hex2ToBin
	pop		r18
	pop		r17
ret


.DSEG
#ifndef UART_NORXBUF
UART_Recv_buf:		.BYTE	UART_ReceiveBufSize
UART_Recv_buf_end:

UART_Recv_buf_head:	.BYTE	2
UART_Recv_buf_len:	.BYTE	1
#endif
UART_LineBuf:	.BYTE	UART_MaxInputSize+1
.CSEG

#endif // UART text functions

