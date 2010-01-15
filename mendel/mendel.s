	.file	"mendel.c"
__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__tmp_reg__ = 0
__zero_reg__ = 1
	.global __do_copy_data
	.global __do_clear_bss
	.stabs	"/home/triffid/ATmega-Skeleton/mendel/",100,0,2,.Ltext0
	.stabs	"mendel.c",100,0,2,.Ltext0
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
	.stabs	"/usr/lib/gcc/avr/4.4.1/include/stddef.h",130,0,0,0
	.stabs	"ptrdiff_t:t(1,1)=(0,1)",128,0,149,0
	.stabs	"size_t:t(1,2)=(0,4)",128,0,211,0
	.stabs	"wchar_t:t(1,3)=(0,1)",128,0,323,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h",130,0,0,0
	.stabs	"int8_t:t(4,1)=(0,10)",128,0,121,0
	.stabs	"uint8_t:t(4,2)=(0,11)",128,0,122,0
	.stabs	"int16_t:t(4,3)=(0,1)",128,0,123,0
	.stabs	"uint16_t:t(4,4)=(0,4)",128,0,124,0
	.stabs	"int32_t:t(4,5)=(0,3)",128,0,125,0
	.stabs	"uint32_t:t(4,6)=(0,5)",128,0,126,0
	.stabs	"int64_t:t(4,7)=(0,6)",128,0,128,0
	.stabs	"uint64_t:t(4,8)=(0,7)",128,0,129,0
	.stabs	"intptr_t:t(4,9)=(4,3)",128,0,142,0
	.stabs	"uintptr_t:t(4,10)=(4,4)",128,0,147,0
	.stabs	"int_least8_t:t(4,11)=(4,1)",128,0,159,0
	.stabs	"uint_least8_t:t(4,12)=(4,2)",128,0,164,0
	.stabs	"int_least16_t:t(4,13)=(4,3)",128,0,169,0
	.stabs	"uint_least16_t:t(4,14)=(4,4)",128,0,174,0
	.stabs	"int_least32_t:t(4,15)=(4,5)",128,0,179,0
	.stabs	"uint_least32_t:t(4,16)=(4,6)",128,0,184,0
	.stabs	"int_least64_t:t(4,17)=(4,7)",128,0,192,0
	.stabs	"uint_least64_t:t(4,18)=(4,8)",128,0,199,0
	.stabs	"int_fast8_t:t(4,19)=(4,1)",128,0,213,0
	.stabs	"uint_fast8_t:t(4,20)=(4,2)",128,0,218,0
	.stabs	"int_fast16_t:t(4,21)=(4,3)",128,0,223,0
	.stabs	"uint_fast16_t:t(4,22)=(4,4)",128,0,228,0
	.stabs	"int_fast32_t:t(4,23)=(4,5)",128,0,233,0
	.stabs	"uint_fast32_t:t(4,24)=(4,6)",128,0,238,0
	.stabs	"int_fast64_t:t(4,25)=(4,7)",128,0,246,0
	.stabs	"uint_fast64_t:t(4,26)=(4,8)",128,0,253,0
	.stabs	"intmax_t:t(4,27)=(4,7)",128,0,273,0
	.stabs	"uintmax_t:t(4,28)=(4,8)",128,0,278,0
	.stabn	162,0,0,0
	.stabs	"int_farptr_t:t(3,1)=(4,5)",128,0,77,0
	.stabs	"uint_farptr_t:t(3,2)=(4,6)",128,0,81,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/include/stdarg.h",130,0,0,0
	.stabs	"__gnuc_va_list:t(5,1)=(5,2)=*(0,15)",128,0,40,0
	.stabs	"va_list:t(5,3)=(5,1)",128,0,102,0
	.stabn	162,0,0,0
	.stabs	"__file:T(2,1)=s14buf:(2,2)=*(0,2),0,16;unget:(0,11),16,8;flags:(4,2),24,8;size:(0,1),32,16;len:(0,1),48,16;put:(2,3)=*(2,4)=f(0,1),64,16;get:(2,5)=*(2,6)=f(0,1),80,16;udata:(2,7)=*(0,15),96,16;;",128,0,0,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/fuse.h",130,0,0,0
	.stabs	"__fuse_t:t(7,1)=(7,2)=s3low:(0,11),0,8;high:(0,11),8,8;extended:(0,11),16,8;;",128,0,239,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"serial.h",130,0,0,0
	.stabs	"ringbuffer.h",130,0,0,0
	.stabs	"ringbuffer:t(9,1)=(9,2)=s6read_pointer:(4,4),0,16;write_pointer:(4,4),16,16;size:(4,4),32,16;;",128,0,12,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.section	.text.main,"ax",@progbits
	.stabs	"main:F(0,1)",36,0,24,main
.global	main
	.type	main, @function
main:
	.stabd	46,0,0
	.stabn	68,0,25,.LM0-.LFBB1
.LM0:
.LFBB1:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,27,.LM1-.LFBB1
.LM1:
	ldi r24,lo8(serio)
	ldi r25,hi8(serio)
	sts (__iob)+1,r25
	sts __iob,r24
	.stabn	68,0,28,.LM2-.LFBB1
.LM2:
	sts (__iob+2)+1,r25
	sts __iob+2,r24
	.stabn	68,0,29,.LM3-.LFBB1
.LM3:
	sts (__iob+4)+1,r25
	sts __iob+4,r24
	.stabn	68,0,32,.LM4-.LFBB1
.LM4:
	call serial_init
	.stabn	68,0,34,.LM5-.LFBB1
.LM5:
/* #APP */
 ;  34 "mendel.c" 1
	sei
 ;  0 "" 2
/* #NOAPP */
.L2:
	rjmp .L2
	.size	main, .-main
.Lscope1:
	.stabs	"",36,0,0,.Lscope1-.LFBB1
	.stabd	78,0,0
	.section	.text.serial_getc_fdev,"ax",@progbits
	.stabs	"serial_getc_fdev:F(0,1)",36,0,16,serial_getc_fdev
	.stabs	"stream:P(0,16)=*(2,1)",64,0,16,24
.global	serial_getc_fdev
	.type	serial_getc_fdev, @function
serial_getc_fdev:
	.stabd	46,0,0
	.stabn	68,0,17,.LM6-.LFBB2
.LM6:
.LFBB2:
/* prologue: function */
/* frame size = 0 */
.L6:
	.stabn	68,0,18,.LM7-.LFBB2
.LM7:
	call serial_rxchars
	sbiw r24,0
	breq .L6
	.stabn	68,0,19,.LM8-.LFBB2
.LM8:
	call serial_popchar
	.stabn	68,0,20,.LM9-.LFBB2
.LM9:
	ldi r25,lo8(0)
/* epilogue start */
	ret
	.size	serial_getc_fdev, .-serial_getc_fdev
.Lscope2:
	.stabs	"",36,0,0,.Lscope2-.LFBB2
	.stabd	78,0,0
	.section	.text.serial_putc_fdev,"ax",@progbits
	.stabs	"serial_putc_fdev:F(0,1)",36,0,10,serial_putc_fdev
	.stabs	"c:P(0,2)",64,0,10,24
	.stabs	"stream:P(0,16)",64,0,10,22
.global	serial_putc_fdev
	.type	serial_putc_fdev, @function
serial_putc_fdev:
	.stabd	46,0,0
	.stabn	68,0,11,.LM10-.LFBB3
.LM10:
.LFBB3:
/* prologue: function */
/* frame size = 0 */
	.stabn	68,0,12,.LM11-.LFBB3
.LM11:
	call serial_writechar
	.stabn	68,0,14,.LM12-.LFBB3
.LM12:
	ldi r24,lo8(0)
	ldi r25,hi8(0)
/* epilogue start */
	ret
	.size	serial_putc_fdev, .-serial_putc_fdev
.Lscope3:
	.stabs	"",36,0,0,.Lscope3-.LFBB3
	.stabd	78,0,0
	.data
	.type	serio, @object
	.size	serio, 14
serio:
	.skip 3,0
	.byte	3
	.skip 4,0
	.word	gs(serial_putc_fdev)
	.word	gs(serial_getc_fdev)
	.word	0
	.stabs	"serio:S(2,1)",38,0,22,serio
	.text
	.stabs	"",100,0,0,.Letext0
.Letext0:
