/* Linker script for STM32F411RE Cortex-M4 ARM MCU */

/* Copyright (C)2013-2018, Philip Munts, President, Munts AM Corp.		*/
/*										*/
/* Redistribution and use in source and binary forms, with or without		*/
/* modification, are permitted provided that the following conditions are met:	*/
/*										*/
/* * Redistributions of source code must retain the above copyright notice,	*/
/*   this list of conditions and the following disclaimer.			*/
/*										*/
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"	*/
/* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE	*/
/* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE	*/
/* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE	*/
/* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR		*/
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF		*/
/* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS	*/
/* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN	*/
/* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)	*/
/* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE	*/
/* POSSIBILITY OF SUCH DAMAGE.							*/

MEMORY
{
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

__rom_start__	= ORIGIN(flash);
__rom_size__	= LENGTH(flash);
__ram_start__	= ORIGIN(ram);
__ram_size__	= LENGTH(ram);
__ram_end__ 	= __ram_start__ + __ram_size__;
__stack_end__	= __ram_end__;		/* Top of RAM */
__stack_size__	= 16K;
__stack_start__	= __stack_end__ - __stack_size__;
__heap_start__	= __bss_end__;		/* Between bss and stack */
__heap_end__	= __stack_start__;

SECTIONS
{
  . = 0;

  .text : {
  KEEP(*(.startup))			/* Startup code */
  *(.text*)				/* Program code */
  KEEP(*(.rodata*))			/* Read only data */
  *(.glue_7)
  *(.glue_7t)
  *(.eh_frame)
  . = ALIGN(4);
  __ctors_start__ = .;
  KEEP(*(.init_array))			/* C++ constructors */
  KEEP(*(.ctors))			/* C++ constructors */
  __ctors_end__ = .;
  . = ALIGN(16);
  __text_end__ = .;
  } >flash

  .data : ALIGN(16) {
  __data_beg__ = .;			/* Used in crt0.S */
  *(.data*)				/* Initialized data */
  __data_end__ = .;			/* Used in crt0.S */
  } >ram AT > flash

  .bss (NOLOAD) : ALIGN(16) {
  __bss_beg__ = .;			/* Used in crt0.S */
  *(.bss*)				/* Uninitialized data */
  *(COMMON)				/* Common data */
  __bss_end__ = .;			/* Used in crt0.S */
  } >ram

/* C++ exception unwinding stuff--needed by some toolchains */

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >flash

  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >flash
  __exidx_end = .;
}

/* Firmware entry point */

ENTRY(_vectors)
