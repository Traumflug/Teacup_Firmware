// Startup for STM32F4xx Cortex-M4 ARM MCU

// Copyright (C)2013-2018, Philip Munts, President, Munts AM Corp.
// Copyright (C)2019, Nico Tonnhofer
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

		.syntax 	unified
		.thumb
		.section	.startup, "x"

// Export these symbols

		.global		_startup
		.global		_vectors

// Import these symbols

		.extern		__text_end__
		.extern		__data_beg__
		.extern		__data_end__
		.extern		__bss_beg__
		.extern		__bss_end__
		.extern		__stack_end__
		.extern		__ctors_start__
		.extern		__ctors_end__
		.extern		main
		.extern		_exit

//=============================================================================

// Use Default_handler for all exceptions and interrupts, unless another
// handler is provided elsewhere.

		.macro		IRQ handler
		.word		\handler
		.weak		\handler
		.set		\handler, Default_Handler
		.endm

//=============================================================================

// Exception vector table--Common to all Cortex-M4

_vectors: 	.word		__stack_end__
		.word		_startup
		IRQ		NMI_Handler
		IRQ		HardFault_Handler
		IRQ		MemManage_Handler
		IRQ		BusFault_Handler
		IRQ		UsageFault_Handler
		.word		0
		.word		0
		.word		0
		.word		0
		IRQ		SVC_Handler
		IRQ		DebugMon_Handler
		.word		0
		IRQ		PendSV_Handler
		IRQ		SysTick_Handler

// Hardware interrupts specific to the STM32F411RE

		IRQ		WWDG_IRQHandler
		IRQ		PVD_IRQHandler
		IRQ		TAMP_STAMP_IRQHandler
		IRQ		RTC_WKUP_IRQHandler
		IRQ		FLASH_IRQHandler
		IRQ		RCC_IRQHandler
		IRQ		EXTI0_IRQHandler
		IRQ		EXTI1_IRQHandler
		IRQ		EXTI2_IRQHandler
		IRQ		EXTI3_IRQHandler
		IRQ		EXTI4_IRQHandler
		IRQ		DMA1_Stream0_IRQHandler
		IRQ		DMA1_Stream1_IRQHandler
		IRQ		DMA1_Stream2_IRQHandler
		IRQ		DMA1_Stream3_IRQHandler
		IRQ		DMA1_Stream4_IRQHandler
		IRQ		DMA1_Stream5_IRQHandler
		IRQ		DMA1_Stream6_IRQHandler
		IRQ		ADC_IRQHandler
#ifdef STM32F446xx
		IRQ		CAN1_TX_IRQHandler
		IRQ		CAN1_RX0_IRQHandler
		IRQ		CAN1_RX1_IRQHandler
		IRQ		CAN1_SCE_IRQHandler
#else
		.word		0
		.word		0
		.word		0
		.word		0
#endif
		IRQ		EXTI9_5_IRQHandler
		IRQ		TIM1_BRK_TIM9_IRQHandler
		IRQ		TIM1_UP_TIM10_IRQHandler
		IRQ		TIM1_TRG_COM_TIM11_IRQHandler
		IRQ		TIM1_CC_IRQHandler
		IRQ		TIM2_IRQHandler
		IRQ		TIM3_IRQHandler
		IRQ		TIM4_IRQHandler
		IRQ		I2C1_EV_IRQHandler
		IRQ		I2C1_ER_IRQHandler
		IRQ		I2C2_EV_IRQHandler
		IRQ		I2C2_ER_IRQHandler
		IRQ		SPI1_IRQHandler
		IRQ		SPI2_IRQHandler
		IRQ		USART1_IRQHandler
		IRQ		USART2_IRQHandler
#ifdef STM32F446xx
		IRQ		USART3_IRQHandler
#else
		.word		0
#endif
		IRQ		EXTI15_10_IRQHandler
		IRQ		RTC_Alarm_IRQHandler
#ifdef STM32F446xx
		IRQ		OTG_FS_WKUP_IRQHandler
		IRQ		TIM8_BRK_TIM12_IRQHandler
		IRQ		TIM8_UP_TIM13_IRQHandler
		IRQ		TIM8_TRG_COM_TIM14_IRQHandler
		IRQ		TIM8_CC_IRQHandler
#else
		.word		0
		.word		0
		.word		0
		.word		0
		.word		0
#endif
		IRQ		DMA1_Stream7_IRQHandler
#ifdef STM32F446xx
		IRQ		FMC_IRQHandler
#else
		.word		0
#endif
		IRQ		SDIO_IRQHandler
		IRQ		TIM5_IRQHandler
		IRQ		SPI3_IRQHandler
#ifdef STM32F446xx
		IRQ		UART4_IRQHandler
		IRQ		UART5_IRQHandler
		IRQ		TIM6_DAC_IRQHandler
		IRQ		TIM7_IRQHandler
#else
		.word		0
		.word		0
		.word		0
		.word		0
#endif
		IRQ		DMA2_Stream0_IRQHandler
		IRQ		DMA2_Stream1_IRQHandler
		IRQ		DMA2_Stream2_IRQHandler
		IRQ		DMA2_Stream3_IRQHandler
		IRQ		DMA2_Stream4_IRQHandler
		.word		0
		.word		0
#ifdef STM32F446xx
		IRQ		CAN2_TX_IRQHandler
		IRQ		CAN2_RX0_IRQHandler
		IRQ		CAN2_RX1_IRQHandler
		IRQ		CAN2_SCE_IRQHandler
#else
		.word		0
		.word		0
		.word		0
		.word		0
#endif
		IRQ		OTG_FS_IRQHandler
		IRQ		DMA2_Stream5_IRQHandler
		IRQ		DMA2_Stream6_IRQHandler
		IRQ		DMA2_Stream7_IRQHandler
		IRQ		USART6_IRQHandler
		IRQ		I2C3_EV_IRQHandler
		IRQ		I2C3_ER_IRQHandler
#ifdef STM32F446xx
		IRQ		OTG_HS_EP1_OUT_IRQHandler
		IRQ		OTG_HS_EP1_IN_IRQHandler
		IRQ		OTG_HS_WKUP_IRQHandler
		IRQ		OTG_HS_IRQHandler
		IRQ		DCMI_IRQHandler
#else
		.word		0
		.word		0
		.word		0
		.word		0
		.word		0
#endif
		.word		0
		.word		0
		IRQ		FPU_IRQHandler
		.word		0
		.word		0
		IRQ		SPI4_IRQHandler
		IRQ		SPI5_IRQHandler

//=============================================================================

// Default exception handler--does nothing but return

		.thumb_func
Default_Handler: bx		lr

//=============================================================================

// Reset vector: Set up environment to call C main()

		.thumb_func
_startup:

// Copy initialized data from flash to RAM

copy_data:	ldr		r1, DATA_BEG
		ldr 		r2, TEXT_END
		ldr 		r3, DATA_END
		subs		r3, r3, r1		// Length of initialized data
		beq		zero_bss		// Skip if none

copy_data_loop: ldrb		r4, [r2], #1		// Read byte from flash
		strb		r4, [r1], #1  		// Store byte to RAM
		subs		r3, r3, #1  		// Decrement counter
		bgt 		copy_data_loop		// Repeat until done

// Zero uninitialized data (bss)

zero_bss: 	ldr 		r1, BSS_BEG
		ldr 		r3, BSS_END
		subs 		r3, r3, r1		// Length of uninitialized data
		beq		call_ctors		// Skip if none

		mov 		r2, #0

zero_bss_loop: 	strb		r2, [r1], #1		// Store zero
		subs		r3, r3, #1		// Decrement counter
		bgt		zero_bss_loop		// Repeat until done

// Call C++ constructors.  The compiler and linker together populate the .ctors
// code section with the addresses of the constructor functions.

call_ctors:	ldr		r0, CTORS_BEG
		ldr		r1, CTORS_END
		subs		r1, r1, r0		// Length of ctors section
		beq		call_main		// Skip if no constructors

ctors_loop:	ldr		r2, [r0], #4		// Load next constructor address
		push		{r0,r1}			// Save registers
		blx		r2			// Call constructor
		pop		{r0,r1}			// Restore registers
		subs		r1, r1, #4		// Decrement counter
		bgt		ctors_loop		// Repeat until done

// Call main()

call_main:	mov		r0, #0			// argc=0
		mov		r1, #0			// argv=NULL

		bl		SystemInit		// Call SystemInit()
		bl		main			// Call C main()
		bl		_exit			// Call _exit() if main() returns

//=============================================================================

// These are filled in by the linker

		.align		4
TEXT_END:	.word		__text_end__
DATA_BEG:	.word		__data_beg__
DATA_END:	.word		__data_end__
BSS_BEG:	.word		__bss_beg__
BSS_END:	.word		__bss_end__
CTORS_BEG:	.word		__ctors_start__
CTORS_END:	.word		__ctors_end__

		.end
