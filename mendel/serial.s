	.file	"serial.c"
__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__tmp_reg__ = 0
__zero_reg__ = 1
	.global __do_copy_data
	.global __do_clear_bss
	.stabs	"/home/triffid/ATmega-Skeleton/mendel/",100,0,2,.Ltext0
	.stabs	"serial.c",100,0,2,.Ltext0
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
	.stabs	"serial.h",130,0,0,0
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
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h",130,0,0,0
	.stabs	"int_farptr_t:t(5,1)=(2,5)",128,0,77,0
	.stabs	"uint_farptr_t:t(5,2)=(2,6)",128,0,81,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/fuse.h",130,0,0,0
	.stabs	"__fuse_t:t(6,1)=(6,2)=s3low:(0,11),0,8;high:(0,11),8,8;extended:(0,11),16,8;;",128,0,239,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"ringbuffer.h",130,0,0,0
	.stabs	"ringbuffer:t(7,1)=(7,2)=s6read_pointer:(2,4),0,16;write_pointer:(2,4),16,16;size:(2,4),32,16;;",128,0,12,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.section	.text.serial_writeblock,"ax",@progbits
	.stabs	"serial_writeblock:F(0,15)",36,0,68,serial_writeblock
	.stabs	"data:P(0,16)=*(2,2)",64,0,68,18
	.stabs	"datalen:P(0,1)",64,0,68,20
.global	serial_writeblock
	.type	serial_writeblock, @function
serial_writeblock:
	.stabd	46,0,0
	.stabn	68,0,69,.LM0-.LFBB1
.LM0:
.LFBB1:
/* prologue: function */
/* frame size = 0 */
	movw r18,r24
	movw r20,r22
	.stabn	68,0,70,.LM1-.LFBB1
.LM1:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	movw r22,r18
	call ringbuffer_writeblock
	.stabn	68,0,71,.LM2-.LFBB1
.LM2:
	ldi r30,lo8(193)
	ldi r31,hi8(193)
	ld r24,Z
	ori r24,lo8(32)
	st Z,r24
/* epilogue start */
	.stabn	68,0,72,.LM3-.LFBB1
.LM3:
	ret
	.size	serial_writeblock, .-serial_writeblock
.Lscope1:
	.stabs	"",36,0,0,.Lscope1-.LFBB1
	.stabd	78,0,0
	.section	.text.serial_writechar,"ax",@progbits
	.stabs	"serial_writechar:F(0,15)",36,0,62,serial_writechar
	.stabs	"data:P(2,2)",64,0,62,22
.global	serial_writechar
	.type	serial_writechar, @function
serial_writechar:
	.stabd	46,0,0
	.stabn	68,0,63,.LM4-.LFBB2
.LM4:
.LFBB2:
/* prologue: function */
/* frame size = 0 */
	mov r22,r24
	.stabn	68,0,64,.LM5-.LFBB2
.LM5:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	call ringbuffer_writechar
	.stabn	68,0,65,.LM6-.LFBB2
.LM6:
	ldi r30,lo8(193)
	ldi r31,hi8(193)
	ld r24,Z
	ori r24,lo8(32)
	st Z,r24
/* epilogue start */
	.stabn	68,0,66,.LM7-.LFBB2
.LM7:
	ret
	.size	serial_writechar, .-serial_writechar
.Lscope2:
	.stabs	"",36,0,0,.Lscope2-.LFBB2
	.stabd	78,0,0
	.section	.text.__vector_18,"ax",@progbits
	.stabs	"__vector_18:F(0,15)",36,0,25,__vector_18
.global	__vector_18
	.type	__vector_18, @function
__vector_18:
	.stabd	46,0,0
	.stabn	68,0,26,.LM8-.LFBB3
.LM8:
.LFBB3:
	push __zero_reg__
	push r0
	in r0,__SREG__
	push r0
	clr __zero_reg__
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27
	push r30
	push r31
/* prologue: Signal */
/* frame size = 0 */
	.stabn	68,0,27,.LM9-.LFBB3
.LM9:
	lds r22,198
	ldi r24,lo8(_rx_buffer)
	ldi r25,hi8(_rx_buffer)
	call ringbuffer_writechar
/* epilogue start */
	.stabn	68,0,28,.LM10-.LFBB3
.LM10:
	pop r31
	pop r30
	pop r27
	pop r26
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r0
	out __SREG__,r0
	pop r0
	pop __zero_reg__
	reti
	.size	__vector_18, .-__vector_18
.Lscope3:
	.stabs	"",36,0,0,.Lscope3-.LFBB3
	.stabd	78,0,0
	.section	.text.serial_recvblock,"ax",@progbits
	.stabs	"serial_recvblock:F(2,4)",36,0,57,serial_recvblock
	.stabs	"block:P(0,16)",64,0,57,18
	.stabs	"blocksize:P(0,1)",64,0,57,20
.global	serial_recvblock
	.type	serial_recvblock, @function
serial_recvblock:
	.stabd	46,0,0
	.stabn	68,0,58,.LM11-.LFBB4
.LM11:
.LFBB4:
/* prologue: function */
/* frame size = 0 */
	movw r18,r24
	movw r20,r22
	.stabn	68,0,59,.LM12-.LFBB4
.LM12:
	ldi r24,lo8(_rx_buffer)
	ldi r25,hi8(_rx_buffer)
	movw r22,r18
	call ringbuffer_readblock
/* epilogue start */
	.stabn	68,0,60,.LM13-.LFBB4
.LM13:
	ret
	.size	serial_recvblock, .-serial_recvblock
.Lscope4:
	.stabs	"",36,0,0,.Lscope4-.LFBB4
	.stabd	78,0,0
	.section	.text.serial_popchar,"ax",@progbits
	.stabs	"serial_popchar:F(2,2)",36,0,52,serial_popchar
.global	serial_popchar
	.type	serial_popchar, @function
serial_popchar:
	.stabd	46,0,0
	.stabn	68,0,53,.LM14-.LFBB5
.LM14:
.LFBB5:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,54,.LM15-.LFBB5
.LM15:
	ldi r24,lo8(_rx_buffer)
	ldi r25,hi8(_rx_buffer)
	call ringbuffer_readchar
/* epilogue start */
	.stabn	68,0,55,.LM16-.LFBB5
.LM16:
	ret
	.size	serial_popchar, .-serial_popchar
.Lscope5:
	.stabs	"",36,0,0,.Lscope5-.LFBB5
	.stabd	78,0,0
	.section	.text.serial_txchars,"ax",@progbits
	.stabs	"serial_txchars:F(2,4)",36,0,47,serial_txchars
.global	serial_txchars
	.type	serial_txchars, @function
serial_txchars:
	.stabd	46,0,0
	.stabn	68,0,48,.LM17-.LFBB6
.LM17:
.LFBB6:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,49,.LM18-.LFBB6
.LM18:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	call ringbuffer_canread
/* epilogue start */
	.stabn	68,0,50,.LM19-.LFBB6
.LM19:
	ret
	.size	serial_txchars, .-serial_txchars
.Lscope6:
	.stabs	"",36,0,0,.Lscope6-.LFBB6
	.stabd	78,0,0
	.section	.text.serial_rxchars,"ax",@progbits
	.stabs	"serial_rxchars:F(2,4)",36,0,42,serial_rxchars
.global	serial_rxchars
	.type	serial_rxchars, @function
serial_rxchars:
	.stabd	46,0,0
	.stabn	68,0,43,.LM20-.LFBB7
.LM20:
.LFBB7:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,44,.LM21-.LFBB7
.LM21:
	ldi r24,lo8(_rx_buffer)
	ldi r25,hi8(_rx_buffer)
	call ringbuffer_canread
/* epilogue start */
	.stabn	68,0,45,.LM22-.LFBB7
.LM22:
	ret
	.size	serial_rxchars, .-serial_rxchars
.Lscope7:
	.stabs	"",36,0,0,.Lscope7-.LFBB7
	.stabd	78,0,0
	.section	.text.__vector_19,"ax",@progbits
	.stabs	"__vector_19:F(0,15)",36,0,30,__vector_19
.global	__vector_19
	.type	__vector_19, @function
__vector_19:
	.stabd	46,0,0
	.stabn	68,0,31,.LM23-.LFBB8
.LM23:
.LFBB8:
	push __zero_reg__
	push r0
	in r0,__SREG__
	push r0
	clr __zero_reg__
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26
	push r27
	push r30
	push r31
/* prologue: Signal */
/* frame size = 0 */
	.stabn	68,0,32,.LM24-.LFBB8
.LM24:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	call ringbuffer_canread
	sbiw r24,0
	breq .L16
	.stabn	68,0,34,.LM25-.LFBB8
.LM25:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	call ringbuffer_readchar
	sts 198,r24
	rjmp .L18
.L16:
	.stabn	68,0,38,.LM26-.LFBB8
.LM26:
	lds r24,193
	andi r24,lo8(-33)
	sts 193,r24
.L18:
/* epilogue start */
	.stabn	68,0,40,.LM27-.LFBB8
.LM27:
	pop r31
	pop r30
	pop r27
	pop r26
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r0
	out __SREG__,r0
	pop r0
	pop __zero_reg__
	reti
	.size	__vector_19, .-__vector_19
.Lscope8:
	.stabs	"",36,0,0,.Lscope8-.LFBB8
	.stabd	78,0,0
	.section	.text.serial_init,"ax",@progbits
	.stabs	"serial_init:F(0,15)",36,0,11,serial_init
.global	serial_init
	.type	serial_init, @function
serial_init:
	.stabd	46,0,0
	.stabn	68,0,12,.LM28-.LFBB9
.LM28:
.LFBB9:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,13,.LM29-.LFBB9
.LM29:
	ldi r24,lo8(_rx_buffer)
	ldi r25,hi8(_rx_buffer)
	ldi r22,lo8(70)
	ldi r23,hi8(70)
	call ringbuffer_init
	.stabn	68,0,14,.LM30-.LFBB9
.LM30:
	ldi r24,lo8(_tx_buffer)
	ldi r25,hi8(_tx_buffer)
	ldi r22,lo8(70)
	ldi r23,hi8(70)
	call ringbuffer_init
	.stabn	68,0,16,.LM31-.LFBB9
.LM31:
	sts 192,__zero_reg__
	.stabn	68,0,17,.LM32-.LFBB9
.LM32:
	ldi r30,lo8(193)
	ldi r31,hi8(193)
	ldi r24,lo8(24)
	st Z,r24
	.stabn	68,0,18,.LM33-.LFBB9
.LM33:
	ldi r24,lo8(6)
	sts 194,r24
	.stabn	68,0,20,.LM34-.LFBB9
.LM34:
	ldi r24,lo8(51)
	ldi r25,hi8(51)
	sts (196)+1,r25
	sts 196,r24
	.stabn	68,0,22,.LM35-.LFBB9
.LM35:
	ld r24,Z
	ori r24,lo8(-96)
	st Z,r24
/* epilogue start */
	.stabn	68,0,23,.LM36-.LFBB9
.LM36:
	ret
	.size	serial_init, .-serial_init
.Lscope9:
	.stabs	"",36,0,0,.Lscope9-.LFBB9
	.stabd	78,0,0
	.comm _rx_buffer,70,1
	.comm _tx_buffer,70,1
	.stabs	"_rx_buffer:G(0,17)=ar(0,18)=r(0,18);0;0177777;;0;69;(0,19)=B(2,2)",32,0,8,0
	.stabs	"_tx_buffer:G(0,17)",32,0,9,0
	.text
	.stabs	"",100,0,0,.Letext0
.Letext0:
