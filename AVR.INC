;------------------------------------------------;
; Constants

.equ	RAMTOP = 0x60



.equ	bit0	= 0b00000001
.equ	bit1	= 0b00000010
.equ	bit2	= 0b00000100
.equ	bit3	= 0b00001000
.equ	bit4	= 0b00010000
.equ	bit5	= 0b00100000
.equ	bit6	= 0b01000000
.equ	bit7	= 0b10000000



;------------------------------------------------;
; Load immediate word into the resister pair.
;
;	ldiw	Z,var

.macro	ldiw
	ldi	@0L,low(@1)
	ldi	@0H,high(@1)
.endm


.macro	pushw
	push	@0L
	push	@0H
.endm

.macro	popw
	pop		@0H
	pop		@0L
.endm

;------------------------------------------------;
; Load memory word into the resister pair.
;
;	ldsw	Z,mem

.macro	ldsw
	lds	@0L,@1
	lds	@0H,@1+1
.endm

.macro	lddw
	ldd	@0L,@1
	ldd	@0H,@1+1
.endm


;------------------------------------------------;
; Store resister pair into memory
;
;	stsw	mem,Z

.macro	stsw
	sts	@0,@1L
	sts	@0+1,@1H
.endm

.macro	stdw
	std	@0,@1L
	std	@0+1,@1H
.endm


;------------------------------------------------;
; Store immediate into the indirect memory
;
;	sti	Z,imm
;	stdi	Z+d,imm

.macro	sti
	ldi	r16,@1
	st	@0,r16
.endm

.macro	stdi
	ldi	r16,@1
	std	@0,r16
.endm


;------------------------------------------------;
; Add/Subtract immediate to register
;
;	addi	var,imm

.macro	addi
	subi	@0,-(@1)
.endm

.macro	addiw
	subi	@0L,low(-(@1))
	sbci	@0H,high(-(@1))
.endm

.macro	subiw
	subi	@0L,low(@1)
	sbci	@0H,high(@1)
.endm


;------------------------------------------------;
; Add/Subtract/Compare/Swap register pair
;

.macro	addw
	add	@0L,@1L
	adc	@0H,@1H
.endm

.macro	subw
	sub	@0L,@1L
	sbc	@0H,@1H
.endm

.macro	cpw
	cp	@0L,@1L
	cpc	@0H,@1H
.endm

.macro	cpwi
	cpi		@0H, High(@1)
	brcs	PC+2
	cpi		@0L, Low(@1)
.endm

.macro SwapReg ;// меняем местами значения первых двух указанных регистров через третий
	mov		@2, @0	
	mov		@0, @1
	mov		@1, @2
.endm

;------------------------------------------------;
; Store immediate into the direct memory
;
;	stsi	var,imm

.macro	stsi
	ldi	r16,@1
	sts	@0,r16
.endm

.macro stswi
	stsi	@0	, Low(@1)
	stsi	@0+1, High(@1)	
.endm

;------------------------------------------------;
; Input universal
;
;	uin	var, port

.macro    uin
  	.if	@1 < 0x40
    	in	@0,@1         
	.else
      	lds	@0,@1
.endif
.endm


;------------------------------------------------;
; Output universal
;
;	uout	port,var

.macro    uout
  	.if	@0 < 0x40
    	OUT	@0,@1         
	.else
      	STS	@0,@1
.endif
.endm

;------------------------------------------------;
; Output immediate
;
;	outi	port,var

.macro	outi
	ldi		r16,@1
	uout	@0,r16
.endm


;------------------------------------------------;
; Long branch


.macro	rjne
	breq	PC+2
	rjmp	@0
.endm

.macro	rjeq
	brne	PC+2
	rjmp	@0
.endm

.macro	rjcc
	brcs	PC+2
	rjmp	@0
.endm

.macro	rjcs
	brcc	PC+2
	rjmp	@0
.endm

.macro	retcc
	brcs	PC+2
	ret
.endm

.macro	retcs
	brcc	PC+2
	ret
.endm

.macro	reteq
	brne	PC+2
	ret
.endm

.macro	retne
	breq	PC+2
	ret
.endm

.macro	retts
	brtc	PC+2
	ret
.endm

.macro	rettc
	brts	PC+2
	ret
.endm