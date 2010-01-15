	.file	"ringbuffer.c"
__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__tmp_reg__ = 0
__zero_reg__ = 1
	.global __do_copy_data
	.global __do_clear_bss
	.stabs	"/home/triffid/ATmega-Skeleton/mendel/",100,0,2,.Ltext0
	.stabs	"ringbuffer.c",100,0,2,.Ltext0
	.text
.Ltext0:
	.stabs	"gcc2_compiled.",60,0,0,0
	.stabs	"int:t(0,1)=r(0,1);-32768;32767;",128,0,1,0
	.stabs	"char:t(0,2)=@s8;r(0,2);0;255;",128,0,1,0
	.stabs	"long int:t(0,3)=@s32;r(0,3);020000000000;017777777777;",128,0,1,0
	.stabs	"unsigned int:t(0,4)=r(0,4);0;0177777;",128,0,1,0
	.stabs	"long unsigned int:t(0,5)=@s32;r(0,5);0;037777777777;",128,0,1,0
	.stabs	"long long int:t(0,6)=@s64;r(0,6);01000000000000000000000;0777777777777777777777;",128,0,1,0
	.stabs	"long long unsigned int:t(0,7)=@s64;r(0,7);0;01777777777777777777777;",128,0,1,0
	.stabs	"short int:t(0,8)=r(0,8);-32768;32767;",128,0,1,0
	.stabs	"short unsigned int:t(0,9)=r(0,9);0;0177777;",128,0,1,0
	.stabs	"signed char:t(0,10)=@s8;r(0,10);-128;127;",128,0,1,0
	.stabs	"unsigned char:t(0,11)=@s8;r(0,11);0;255;",128,0,1,0
	.stabs	"float:t(0,12)=r(0,1);4;0;",128,0,1,0
	.stabs	"double:t(0,13)=r(0,1);4;0;",128,0,1,0
	.stabs	"long double:t(0,14)=r(0,1);4;0;",128,0,1,0
	.stabs	"void:t(0,15)=(0,15)",128,0,1,0
	.stabs	"ringbuffer.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h",130,0,0,0
	.stabs	"int8_t:t(2,1)=(0,10)",128,0,121,0
	.stabs	"uint8_t:t(2,2)=(0,11)",128,0,122,0
	.stabs	"int16_t:t(2,3)=(0,1)",128,0,123,0
	.stabs	"uint16_t:t(2,4)=(0,4)",128,0,124,0
	.stabs	"int32_t:t(2,5)=(0,3)",128,0,125,0
	.stabs	"uint32_t:t(2,6)=(0,5)",128,0,126,0
	.stabs	"int64_t:t(2,7)=(0,6)",128,0,128,0
	.stabs	"uint64_t:t(2,8)=(0,7)",128,0,129,0
	.stabs	"intptr_t:t(2,9)=(2,3)",128,0,142,0
	.stabs	"uintptr_t:t(2,10)=(2,4)",128,0,147,0
	.stabs	"int_least8_t:t(2,11)=(2,1)",128,0,159,0
	.stabs	"uint_least8_t:t(2,12)=(2,2)",128,0,164,0
	.stabs	"int_least16_t:t(2,13)=(2,3)",128,0,169,0
	.stabs	"uint_least16_t:t(2,14)=(2,4)",128,0,174,0
	.stabs	"int_least32_t:t(2,15)=(2,5)",128,0,179,0
	.stabs	"uint_least32_t:t(2,16)=(2,6)",128,0,184,0
	.stabs	"int_least64_t:t(2,17)=(2,7)",128,0,192,0
	.stabs	"uint_least64_t:t(2,18)=(2,8)",128,0,199,0
	.stabs	"int_fast8_t:t(2,19)=(2,1)",128,0,213,0
	.stabs	"uint_fast8_t:t(2,20)=(2,2)",128,0,218,0
	.stabs	"int_fast16_t:t(2,21)=(2,3)",128,0,223,0
	.stabs	"uint_fast16_t:t(2,22)=(2,4)",128,0,228,0
	.stabs	"int_fast32_t:t(2,23)=(2,5)",128,0,233,0
	.stabs	"uint_fast32_t:t(2,24)=(2,6)",128,0,238,0
	.stabs	"int_fast64_t:t(2,25)=(2,7)",128,0,246,0
	.stabs	"uint_fast64_t:t(2,26)=(2,8)",128,0,253,0
	.stabs	"intmax_t:t(2,27)=(2,7)",128,0,273,0
	.stabs	"uintmax_t:t(2,28)=(2,8)",128,0,278,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h",130,0,0,0
	.stabs	"int_farptr_t:t(6,1)=(2,5)",128,0,77,0
	.stabs	"uint_farptr_t:t(6,2)=(2,6)",128,0,81,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/fuse.h",130,0,0,0
	.stabs	"__fuse_t:t(7,1)=(7,2)=s3low:(0,11),0,8;high:(0,11),8,8;extended:(0,11),16,8;;",128,0,239,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"ringbuffer:t(1,1)=(1,2)=s6read_pointer:(2,4),0,16;write_pointer:(2,4),16,16;size:(2,4),32,16;;",128,0,12,0
	.stabn	162,0,0,0
	.section	.text._rb_mod,"ax",@progbits
	.stabs	"_rb_mod:F(2,4)",36,0,3,_rb_mod
	.stabs	"num:P(2,4)",64,0,3,18
	.stabs	"denom:P(2,4)",64,0,3,22
.global	_rb_mod
	.type	_rb_mod, @function
_rb_mod:
	.stabd	46,0,0
	.stabn	68,0,4,.LM0-.LFBB1
.LM0:
.LFBB1:
/* prologue: function */
/* frame size = 0 */
	movw r18,r24
	.stabn	68,0,5,.LM1-.LFBB1
.LM1:
	rjmp .L2
.L3:
	sub r18,r22
	sbc r19,r23
.L2:
	cp r18,r22
	cpc r19,r23
	brsh .L3
	.stabn	68,0,7,.LM2-.LFBB1
.LM2:
	movw r24,r18
/* epilogue start */
	ret
	.size	_rb_mod, .-_rb_mod
.Lscope1:
	.stabs	"",36,0,0,.Lscope1-.LFBB1
	.stabd	78,0,0
	.section	.text.ringbuffer_init,"ax",@progbits
	.stabs	"ringbuffer_init:F(0,15)",36,0,9,ringbuffer_init
	.stabs	"buf:P(0,16)=*(1,1)",64,0,9,30
	.stabs	"bufsize:P(0,1)",64,0,9,22
.global	ringbuffer_init
	.type	ringbuffer_init, @function
ringbuffer_init:
	.stabd	46,0,0
	.stabn	68,0,10,.LM3-.LFBB2
.LM3:
.LFBB2:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	.stabn	68,0,11,.LM4-.LFBB2
.LM4:
	std Z+1,__zero_reg__
	st Z,__zero_reg__
	.stabn	68,0,12,.LM5-.LFBB2
.LM5:
	std Z+3,__zero_reg__
	std Z+2,__zero_reg__
	.stabn	68,0,13,.LM6-.LFBB2
.LM6:
	subi r22,lo8(-(-6))
	sbci r23,hi8(-(-6))
	std Z+5,r23
	std Z+4,r22
/* epilogue start */
	.stabn	68,0,14,.LM7-.LFBB2
.LM7:
	ret
	.size	ringbuffer_init, .-ringbuffer_init
.Lscope2:
	.stabs	"",36,0,0,.Lscope2-.LFBB2
	.stabd	78,0,0
	.section	.text.ringbuffer_canread,"ax",@progbits
	.stabs	"ringbuffer_canread:F(2,4)",36,0,16,ringbuffer_canread
	.stabs	"buf:P(0,16)",64,0,16,30
.global	ringbuffer_canread
	.type	ringbuffer_canread, @function
ringbuffer_canread:
	.stabd	46,0,0
	.stabn	68,0,17,.LM8-.LFBB3
.LM8:
.LFBB3:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	.stabn	68,0,18,.LM9-.LFBB3
.LM9:
	ldd r20,Z+4
	ldd r21,Z+5
	ldd r18,Z+2
	ldd r19,Z+3
	add r18,r20
	adc r19,r21
	add r18,r20
	adc r19,r21
	ld r24,Z
	ldd r25,Z+1
	sub r18,r24
	sbc r19,r25
	rjmp .L9
.L10:
.LBB20:
.LBB21:
	.stabn	68,0,5,.LM10-.LFBB3
.LM10:
	sub r18,r20
	sbc r19,r21
.L9:
	cp r18,r20
	cpc r19,r21
	brsh .L10
.LBE21:
.LBE20:
	.stabn	68,0,19,.LM11-.LFBB3
.LM11:
	movw r24,r18
/* epilogue start */
	ret
	.size	ringbuffer_canread, .-ringbuffer_canread
	.stabs	"num:r(2,4)",64,0,18,18
	.stabn	192,0,0,.LBB20-.LFBB3
	.stabn	224,0,0,.LBE20-.LFBB3
.Lscope3:
	.stabs	"",36,0,0,.Lscope3-.LFBB3
	.stabd	78,0,0
	.section	.text.ringbuffer_canwrite,"ax",@progbits
	.stabs	"ringbuffer_canwrite:F(2,4)",36,0,21,ringbuffer_canwrite
	.stabs	"buf:P(0,16)",64,0,21,30
.global	ringbuffer_canwrite
	.type	ringbuffer_canwrite, @function
ringbuffer_canwrite:
	.stabd	46,0,0
	.stabn	68,0,22,.LM12-.LFBB4
.LM12:
.LFBB4:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	.stabn	68,0,23,.LM13-.LFBB4
.LM13:
	ldd r20,Z+4
	ldd r21,Z+5
	ld r18,Z
	ldd r19,Z+1
	add r18,r20
	adc r19,r21
	subi r18,lo8(-(-1))
	sbci r19,hi8(-(-1))
	add r18,r20
	adc r19,r21
	ldd r24,Z+2
	ldd r25,Z+3
	sub r18,r24
	sbc r19,r25
	rjmp .L13
.L14:
.LBB22:
.LBB23:
	.stabn	68,0,5,.LM14-.LFBB4
.LM14:
	sub r18,r20
	sbc r19,r21
.L13:
	cp r18,r20
	cpc r19,r21
	brsh .L14
.LBE23:
.LBE22:
	.stabn	68,0,24,.LM15-.LFBB4
.LM15:
	movw r24,r18
/* epilogue start */
	ret
	.size	ringbuffer_canwrite, .-ringbuffer_canwrite
	.stabs	"num:r(2,4)",64,0,23,18
	.stabn	192,0,0,.LBB22-.LFBB4
	.stabn	224,0,0,.LBE22-.LFBB4
.Lscope4:
	.stabs	"",36,0,0,.Lscope4-.LFBB4
	.stabd	78,0,0
	.section	.text.ringbuffer_readchar,"ax",@progbits
	.stabs	"ringbuffer_readchar:F(2,2)",36,0,26,ringbuffer_readchar
	.stabs	"buf:P(0,16)",64,0,26,30
.global	ringbuffer_readchar
	.type	ringbuffer_readchar, @function
ringbuffer_readchar:
	.stabd	46,0,0
	.stabn	68,0,27,.LM16-.LFBB5
.LM16:
.LFBB5:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
.LBB24:
.LBB25:
	.stabn	68,0,18,.LM17-.LFBB5
.LM17:
	ldd r20,Z+4
	ldd r21,Z+5
	ld r22,Z
	ldd r23,Z+1
	ldd r18,Z+2
	ldd r19,Z+3
	add r18,r20
	adc r19,r21
	add r18,r20
	adc r19,r21
	sub r18,r22
	sbc r19,r23
	rjmp .L17
.L18:
.LBB26:
.LBB27:
	.stabn	68,0,5,.LM18-.LFBB5
.LM18:
	sub r18,r20
	sbc r19,r21
.L17:
	cp r18,r20
	cpc r19,r21
	brsh .L18
.LBE27:
.LBE26:
.LBE25:
.LBE24:
	.stabn	68,0,29,.LM19-.LFBB5
.LM19:
	cp r18,__zero_reg__
	cpc r19,__zero_reg__
	brne .L19
	ldi r24,lo8(0)
	ret
.L19:
	.stabn	68,0,31,.LM20-.LFBB5
.LM20:
	movw r26,r30
	add r26,r22
	adc r27,r23
	adiw r26,6
	ld r24,X
	sbiw r26,6
	.stabn	68,0,32,.LM21-.LFBB5
.LM21:
	movw r18,r22
	subi r18,lo8(-(1))
	sbci r19,hi8(-(1))
	rjmp .L21
.L22:
.LBB28:
.LBB29:
	.stabn	68,0,5,.LM22-.LFBB5
.LM22:
	sub r18,r20
	sbc r19,r21
.L21:
	cp r18,r20
	cpc r19,r21
	brsh .L22
.LBE29:
.LBE28:
	.stabn	68,0,32,.LM23-.LFBB5
.LM23:
	std Z+1,r19
	st Z,r18
	.stabn	68,0,35,.LM24-.LFBB5
.LM24:
	ret
	.size	ringbuffer_readchar, .-ringbuffer_readchar
	.stabs	"r:r(2,2)",64,0,28,24
	.stabn	192,0,0,.LFBB5-.LFBB5
	.stabs	"num:r(2,4)",64,0,18,18
	.stabn	192,0,0,.LBB26-.LFBB5
	.stabn	224,0,0,.LBE26-.LFBB5
	.stabs	"num:r(2,4)",64,0,32,18
	.stabn	192,0,0,.LBB28-.LFBB5
	.stabn	224,0,0,.LBE28-.LFBB5
	.stabn	224,0,0,.Lscope5-.LFBB5
.Lscope5:
	.stabs	"",36,0,0,.Lscope5-.LFBB5
	.stabd	78,0,0
	.section	.text.ringbuffer_writechar,"ax",@progbits
	.stabs	"ringbuffer_writechar:F(0,15)",36,0,37,ringbuffer_writechar
	.stabs	"buf:P(0,16)",64,0,37,16
	.stabs	"data:P(2,2)",64,0,37,22
.global	ringbuffer_writechar
	.type	ringbuffer_writechar, @function
ringbuffer_writechar:
	.stabd	46,0,0
	.stabn	68,0,38,.LM25-.LFBB6
.LM25:
.LFBB6:
	push r16
	push r17
	push r29
	push r28
	push __tmp_reg__
	in r28,__SP_L__
	in r29,__SP_H__
/* prologue: function */
/* frame size = 1 */
	movw r16,r24
	.stabn	68,0,39,.LM26-.LFBB6
.LM26:
	std Y+1,r22
	call ringbuffer_canwrite
	ldd r22,Y+1
	sbiw r24,0
	breq .L28
	.stabn	68,0,41,.LM27-.LFBB6
.LM27:
	movw r30,r16
	ldd r24,Z+2
	ldd r25,Z+3
	add r30,r24
	adc r31,r25
	std Z+6,r22
	.stabn	68,0,42,.LM28-.LFBB6
.LM28:
	adiw r24,1
	movw r30,r16
	ldd r18,Z+4
	ldd r19,Z+5
	rjmp .L26
.L27:
.LBB30:
.LBB31:
	.stabn	68,0,5,.LM29-.LFBB6
.LM29:
	sub r24,r18
	sbc r25,r19
.L26:
	cp r24,r18
	cpc r25,r19
	brsh .L27
.LBE31:
.LBE30:
	.stabn	68,0,42,.LM30-.LFBB6
.LM30:
	movw r30,r16
	std Z+3,r25
	std Z+2,r24
.L28:
/* epilogue start */
	.stabn	68,0,44,.LM31-.LFBB6
.LM31:
	pop __tmp_reg__
	pop r28
	pop r29
	pop r17
	pop r16
	ret
	.size	ringbuffer_writechar, .-ringbuffer_writechar
	.stabs	"num:r(2,4)",64,0,42,24
	.stabn	192,0,0,.LBB30-.LFBB6
	.stabn	224,0,0,.LBE30-.LFBB6
.Lscope6:
	.stabs	"",36,0,0,.Lscope6-.LFBB6
	.stabd	78,0,0
	.section	.text.ringbuffer_peekchar,"ax",@progbits
	.stabs	"ringbuffer_peekchar:F(2,2)",36,0,47,ringbuffer_peekchar
	.stabs	"buf:P(0,16)",64,0,47,30
	.stabs	"index:P(2,4)",64,0,47,22
.global	ringbuffer_peekchar
	.type	ringbuffer_peekchar, @function
ringbuffer_peekchar:
	.stabd	46,0,0
	.stabn	68,0,48,.LM32-.LFBB7
.LM32:
.LFBB7:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	.stabn	68,0,49,.LM33-.LFBB7
.LM33:
	ld r24,Z
	ldd r25,Z+1
	add r22,r24
	adc r23,r25
	ldd r24,Z+4
	ldd r25,Z+5
	rjmp .L30
.L31:
.LBB32:
.LBB33:
	.stabn	68,0,5,.LM34-.LFBB7
.LM34:
	sub r22,r24
	sbc r23,r25
.L30:
	cp r22,r24
	cpc r23,r25
	brsh .L31
	add r30,r22
	adc r31,r23
.LBE33:
.LBE32:
	.stabn	68,0,50,.LM35-.LFBB7
.LM35:
	ldd r24,Z+6
/* epilogue start */
	ret
	.size	ringbuffer_peekchar, .-ringbuffer_peekchar
	.stabs	"num:r(2,4)",64,0,49,22
	.stabn	192,0,0,.LBB32-.LFBB7
	.stabn	224,0,0,.LBE32-.LFBB7
.Lscope7:
	.stabs	"",36,0,0,.Lscope7-.LFBB7
	.stabd	78,0,0
	.section	.text.ringbuffer_readblock,"ax",@progbits
	.stabs	"ringbuffer_readblock:F(2,4)",36,0,52,ringbuffer_readblock
	.stabs	"buf:P(0,16)",64,0,52,30
	.stabs	"newbuf:P(0,17)=*(2,2)",64,0,52,22
	.stabs	"size:P(0,1)",64,0,52,20
.global	ringbuffer_readblock
	.type	ringbuffer_readblock, @function
ringbuffer_readblock:
	.stabd	46,0,0
	.stabn	68,0,53,.LM36-.LFBB8
.LM36:
.LFBB8:
	push r12
	push r13
	push r15
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
.LBB34:
.LBB35:
	.stabn	68,0,18,.LM37-.LFBB8
.LM37:
	ldd r18,Z+4
	ldd r19,Z+5
	ld r26,Z
	ldd r27,Z+1
	ldd r24,Z+2
	ldd r25,Z+3
	add r24,r18
	adc r25,r19
	add r24,r18
	adc r25,r19
	sub r24,r26
	sbc r25,r27
	rjmp .L34
.L35:
.LBB36:
.LBB37:
	.stabn	68,0,5,.LM38-.LFBB8
.LM38:
	sub r24,r18
	sbc r25,r19
.L34:
	cp r24,r18
	cpc r25,r19
	brsh .L35
.LBE37:
.LBE36:
.LBE35:
.LBE34:
	.stabn	68,0,56,.LM39-.LFBB8
.LM39:
	cp r24,r20
	cpc r25,r21
	brsh .L36
	.stabn	68,0,57,.LM40-.LFBB8
.LM40:
	movw r20,r24
.L36:
	.stabn	68,0,58,.LM41-.LFBB8
.LM41:
	cp r20,__zero_reg__
	cpc r21,__zero_reg__
	breq .L37
	.stabn	68,0,60,.LM42-.LFBB8
.LM42:
	adiw r26,6
	add r26,r30
	adc r27,r31
	subi r18,lo8(-(6))
	sbci r19,hi8(-(6))
	add r18,r30
	adc r19,r31
	.stabn	68,0,63,.LM43-.LFBB8
.LM43:
	ldi r24,lo8(6)
	mov r12,r24
	mov r13,__zero_reg__
	add r12,r30
	adc r13,r31
	ldi r24,lo8(0)
	ldi r25,hi8(0)
	.stabn	68,0,60,.LM44-.LFBB8
.LM44:
	rjmp .L38
.L40:
	.stabn	68,0,62,.LM45-.LFBB8
.LM45:
	cp r26,r18
	cpc r27,r19
	brlo .L39
	movw r26,r12
.L39:
	.stabn	68,0,64,.LM46-.LFBB8
.LM46:
	movw r16,r22
	add r16,r24
	adc r17,r25
	ld r15,X+
	movw r28,r16
	st Y,r15
	.stabn	68,0,60,.LM47-.LFBB8
.LM47:
	adiw r24,1
.L38:
	cp r24,r20
	cpc r25,r21
	brlo .L40
	.stabn	68,0,66,.LM48-.LFBB8
.LM48:
	movw r24,r30
	adiw r24,6
	sub r26,r24
	sbc r27,r25
	std Z+1,r27
	st Z,r26
.L37:
	.stabn	68,0,69,.LM49-.LFBB8
.LM49:
	movw r24,r20
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	pop r15
	pop r13
	pop r12
	ret
	.size	ringbuffer_readblock, .-ringbuffer_readblock
	.stabs	"nc:r(2,4)",64,0,54,24
	.stabs	"i:r(2,4)",64,0,54,24
	.stabs	"rp:r(0,17)",64,0,55,26
	.stabs	"ms:r(0,17)",64,0,55,18
	.stabn	192,0,0,.LFBB8-.LFBB8
	.stabn	224,0,0,.Lscope8-.LFBB8
.Lscope8:
	.stabs	"",36,0,0,.Lscope8-.LFBB8
	.stabd	78,0,0
	.section	.text.ringbuffer_writeblock,"ax",@progbits
	.stabs	"ringbuffer_writeblock:F(2,4)",36,0,71,ringbuffer_writeblock
	.stabs	"buf:P(0,16)",64,0,71,28
	.stabs	"data:P(0,17)",64,0,71,16
	.stabs	"size:P(0,1)",64,0,71,14
.global	ringbuffer_writeblock
	.type	ringbuffer_writeblock, @function
ringbuffer_writeblock:
	.stabd	46,0,0
	.stabn	68,0,72,.LM50-.LFBB9
.LM50:
.LFBB9:
	push r14
	push r15
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
	movw r28,r24
	movw r16,r22
	movw r14,r20
	.stabn	68,0,76,.LM51-.LFBB9
.LM51:
	call ringbuffer_canwrite
	cp r24,r14
	cpc r25,r15
	brsh .L43
	.stabn	68,0,77,.LM52-.LFBB9
.LM52:
	movw r14,r24
.L43:
	.stabn	68,0,78,.LM53-.LFBB9
.LM53:
	cp r14,__zero_reg__
	cpc r15,__zero_reg__
	breq .L44
	.stabn	68,0,80,.LM54-.LFBB9
.LM54:
	ldd r30,Y+2
	ldd r31,Y+3
	adiw r30,6
	add r30,r28
	adc r31,r29
	ldd r24,Y+4
	ldd r25,Y+5
	adiw r24,6
	add r24,r28
	adc r25,r29
	.stabn	68,0,83,.LM55-.LFBB9
.LM55:
	movw r20,r28
	subi r20,lo8(-(6))
	sbci r21,hi8(-(6))
	ldi r18,lo8(0)
	ldi r19,hi8(0)
	.stabn	68,0,80,.LM56-.LFBB9
.LM56:
	rjmp .L45
.L47:
	.stabn	68,0,82,.LM57-.LFBB9
.LM57:
	cp r30,r24
	cpc r31,r25
	brlo .L46
	movw r30,r20
.L46:
	.stabn	68,0,84,.LM58-.LFBB9
.LM58:
	movw r26,r16
	add r26,r18
	adc r27,r19
	ld r22,X
	st Z+,r22
	.stabn	68,0,80,.LM59-.LFBB9
.LM59:
	subi r18,lo8(-(1))
	sbci r19,hi8(-(1))
.L45:
	cp r18,r14
	cpc r19,r15
	brlo .L47
	.stabn	68,0,86,.LM60-.LFBB9
.LM60:
	movw r24,r28
	adiw r24,6
	sub r30,r24
	sbc r31,r25
	std Y+3,r31
	std Y+2,r30
.L44:
	.stabn	68,0,89,.LM61-.LFBB9
.LM61:
	movw r24,r14
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	pop r15
	pop r14
	ret
	.size	ringbuffer_writeblock, .-ringbuffer_writeblock
	.stabs	"nc:r(2,4)",64,0,73,24
	.stabs	"i:r(2,4)",64,0,73,18
	.stabs	"wp:r(0,17)",64,0,74,30
	.stabs	"ms:r(0,17)",64,0,74,24
	.stabn	192,0,0,.LFBB9-.LFBB9
	.stabn	224,0,0,.Lscope9-.LFBB9
.Lscope9:
	.stabs	"",36,0,0,.Lscope9-.LFBB9
	.stabd	78,0,0
	.text
	.stabs	"",100,0,0,.Letext0
.Letext0:
