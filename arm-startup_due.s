/* File: startup_ARMCM3.S
 * Purpose: startup file for Cortex-M3 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.2
 * Date: 15 Nov 2011
 *
 * Copyright (c) 2011, ARM Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /*
  Changes for Teacup:

  Copied from file
    .arduino15/packages/arduino/hardware/sam/1.6.4/system/CMSIS/Device/
      ARM/ARMCM3/Source/Templates/GCC_ARM/startup_ARMCM3.S

    - comment out one .flash_to_ram_loop, else can not compile
*/
    .syntax unified
    .arch armv7-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0xc00
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x800
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .space    Heap_Size
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */

    .long    SUPC_Handler          /*  0  Supply Controller */
    .long    RSTC_Handler          /*  1  Reset Controller */
    .long    RTC_Handler           /*  2  Real Time Clock */
    .long    RTT_Handler           /*  3  Real Time Timer */
    .long    WDT_Handler           /*  4  Watchdog Timer */
    .long    PMC_Handler           /*  5  PMC */
    .long    EFC0_Handler          /*  6  EFC 0 */
    .long    EFC1_Handler          /*  7  EFC 1 */
    .long    UART_Handler          /*  8  UART */
    .long    Default_Handler       /*  9  Reserved */
    .long    Default_Handler       /* 10  Reserved */
    .long    PIOA_Handler          /* 11  Parallel IO Controller A */
    .long    PIOB_Handler          /* 12  Parallel IO Controller B */
    .long    PIOC_Handler          /* 13  Parallel IO Controller C */
    .long    PIOD_Handler          /* 14  Parallel IO Controller D */
    .long    Default_Handler       /* 15  Reserved */
    .long    Default_Handler       /* 16 Reserved */
    .long    USART0_Handler        /* 17 USART 0 */
    .long    USART1_Handler        /* 18 USART 1 */
    .long    USART2_Handler        /* 19 USART 2 */
    .long    USART3_Handler        /* 20 USART 3 */
    .long    HSMCI_Handler         /* 21 MCI */
    .long    TWI0_Handler          /* 22 TWI 0 */
    .long    TWI1_Handler          /* 23 TWI 1 */
    .long    SPI0_Handler          /* 24 SPI 0 */
    .long    Default_Handler       /* 25 Reserved */
    .long    SSC_Handler           /* 26 SSC */
    .long    TC0_Handler           /* 27 Timer Counter 0 */
    .long    TC1_Handler           /* 28 Timer Counter 1 */
    .long    TC2_Handler           /* 29 Timer Counter 2 */
    .long    TC3_Handler           /* 30 Timer Counter 3 */
    .long    TC4_Handler           /* 31 Timer Counter 4 */
    .long    TC5_Handler           /* 32 Timer Counter 5 */
    .long    TC6_Handler           /* 33 Timer Counter 6 */
    .long    TC7_Handler           /* 34 Timer Counter 7 */
    .long    TC8_Handler           /* 35 Timer Counter 8 */
    .long    PWM_Handler           /* 36 PWM */
    .long    ADC_Handler           /* 37 ADC controller */
    .long    DACC_Handler          /* 38 DAC controller */
    .long    DMAC_Handler          /* 39 DMA Controller */
    .long    UOTGHS_Handler        /* 40 USB OTG High Speed */
    .long    TRNG_Handler          /* 41 True Random Number Generator */
    .long    EMAC_Handler          /* 42 Ethernet MAC */
    .long    CAN0_Handler          /* 43 CAN Controller 0 */
    .long    CAN1_Handler          /* 44 CAN Controller 1 */

    /*
    .long    Default_Handler
    */

    .size    __isr_vector, . - __isr_vector

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

/*#if 1
 * Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one. 
 * Change to "#if 0" to use the second one
.flash_to_ram_loop:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .flash_to_ram_loop
#else 
*/
    subs    r3, r2
    ble    .flash_to_ram_loop_end    
.flash_to_ram_loop:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .flash_to_ram_loop
.flash_to_ram_loop_end:
/*
#endif
*/

    ldr    r0, =SystemInit
    blx    r0
    ldr    r0, =_start
    bx    r0
    .pool
    .size Reset_Handler, . - Reset_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler

    .macro    def_irq_default_handler    handler_name
    .weak     \handler_name
    .set      \handler_name, Default_Handler
    .endm

    def_irq_default_handler    SUPC_Handler
    def_irq_default_handler    RSTC_Handler
    def_irq_default_handler    RTC_Handler
    def_irq_default_handler    RTT_Handler
    def_irq_default_handler    WDT_Handler
    def_irq_default_handler    PMC_Handler
    def_irq_default_handler    EFC0_Handler
    def_irq_default_handler    EFC1_Handler
    def_irq_default_handler    UART_Handler
    def_irq_default_handler    PIOA_Handler
    def_irq_default_handler    PIOB_Handler
    def_irq_default_handler    PIOC_Handler
    def_irq_default_handler    PIOD_Handler
    def_irq_default_handler    USART0_Handler
    def_irq_default_handler    USART1_Handler
    def_irq_default_handler    USART2_Handler
    def_irq_default_handler    USART3_Handler
    def_irq_default_handler    HSMCI_Handler
    def_irq_default_handler    TWI0_Handler
    def_irq_default_handler    TWI1_Handler
    def_irq_default_handler    SPI0_Handler
    def_irq_default_handler    SSC_Handler
    def_irq_default_handler    TC0_Handler
    def_irq_default_handler    TC1_Handler
    def_irq_default_handler    TC2_Handler
    def_irq_default_handler    TC3_Handler
    def_irq_default_handler    TC4_Handler
    def_irq_default_handler    TC5_Handler
    def_irq_default_handler    TC6_Handler
    def_irq_default_handler    TC7_Handler
    def_irq_default_handler    TC8_Handler
    def_irq_default_handler    PWM_Handler
    def_irq_default_handler    ADC_Handler
    def_irq_default_handler    DACC_Handler
    def_irq_default_handler    DMAC_Handler
    def_irq_default_handler    UOTGHS_Handler
    def_irq_default_handler    TRNG_Handler
    def_irq_default_handler    EMAC_Handler
    def_irq_default_handler    CAN0_Handler
    def_irq_default_handler    CAN1_Handler
    def_irq_default_handler    DEF_IRQHandler

    .end
