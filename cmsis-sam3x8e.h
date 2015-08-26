/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
 /*
  Notes for Teacup:

  Copied from $(ARDUINO15)/ackages/arduino/hardware/sam/1.6.4/
                        system/CMSIS/Device/ATMEL/sam3xa/include/sam3x8e.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Replaced component/component-*-.h with it's structures
*/

#ifndef _SAM3X8E_
#define _SAM3X8E_

/** \addtogroup SAM3X8E_definitions SAM3X8E definitions
  This file defines all structures and symbols for SAM3X8E:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - PIO definitions
*/
/*@{*/

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#ifndef __cplusplus
typedef volatile const uint32_t RoReg; /**< Read only 32-bit register (volatile const unsigned int) */
#else
typedef volatile       uint32_t RoReg; /**< Read only 32-bit register (volatile const unsigned int) */
#endif
typedef volatile       uint32_t WoReg; /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg; /**< Read-Write 32-bit register (volatile unsigned int) */

/* ************************************************************************** */
/*   CMSIS DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_cmsis CMSIS Definitions */
/*@{*/

/**< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn   = -14, /**<  2 Non Maskable Interrupt                */
  MemoryManagement_IRQn = -12, /**<  4 Cortex-M3 Memory Management Interrupt */
  BusFault_IRQn         = -11, /**<  5 Cortex-M3 Bus Fault Interrupt         */
  UsageFault_IRQn       = -10, /**<  6 Cortex-M3 Usage Fault Interrupt       */
  SVCall_IRQn           = -5,  /**< 11 Cortex-M3 SV Call Interrupt           */
  DebugMonitor_IRQn     = -4,  /**< 12 Cortex-M3 Debug Monitor Interrupt     */
  PendSV_IRQn           = -2,  /**< 14 Cortex-M3 Pend SV Interrupt           */
  SysTick_IRQn          = -1,  /**< 15 Cortex-M3 System Tick Interrupt       */
/******  SAM3X8E specific Interrupt Numbers *********************************/

  SUPC_IRQn            =  0, /**<  0 SAM3X8E Supply Controller (SUPC) */
  RSTC_IRQn            =  1, /**<  1 SAM3X8E Reset Controller (RSTC) */
  RTC_IRQn             =  2, /**<  2 SAM3X8E Real Time Clock (RTC) */
  RTT_IRQn             =  3, /**<  3 SAM3X8E Real Time Timer (RTT) */
  WDT_IRQn             =  4, /**<  4 SAM3X8E Watchdog Timer (WDT) */
  PMC_IRQn             =  5, /**<  5 SAM3X8E Power Management Controller (PMC) */
  EFC0_IRQn            =  6, /**<  6 SAM3X8E Enhanced Flash Controller 0 (EFC0) */
  EFC1_IRQn            =  7, /**<  7 SAM3X8E Enhanced Flash Controller 1 (EFC1) */
  UART_IRQn            =  8, /**<  8 SAM3X8E Universal Asynchronous Receiver Transceiver (UART) */
  SMC_IRQn             =  9, /**<  9 SAM3X8E Static Memory Controller (SMC) */
  PIOA_IRQn            = 11, /**< 11 SAM3X8E Parallel I/O Controller A, (PIOA) */
  PIOB_IRQn            = 12, /**< 12 SAM3X8E Parallel I/O Controller B (PIOB) */
  PIOC_IRQn            = 13, /**< 13 SAM3X8E Parallel I/O Controller C (PIOC) */
  PIOD_IRQn            = 14, /**< 14 SAM3X8E Parallel I/O Controller D (PIOD) */
  USART0_IRQn          = 17, /**< 17 SAM3X8E USART 0 (USART0) */
  USART1_IRQn          = 18, /**< 18 SAM3X8E USART 1 (USART1) */
  USART2_IRQn          = 19, /**< 19 SAM3X8E USART 2 (USART2) */
  USART3_IRQn          = 20, /**< 20 SAM3X8E USART 3 (USART3) */
  HSMCI_IRQn           = 21, /**< 21 SAM3X8E Multimedia Card Interface (HSMCI) */
  TWI0_IRQn            = 22, /**< 22 SAM3X8E Two-Wire Interface 0 (TWI0) */
  TWI1_IRQn            = 23, /**< 23 SAM3X8E Two-Wire Interface 1 (TWI1) */
  SPI0_IRQn            = 24, /**< 24 SAM3X8E Serial Peripheral Interface (SPI0) */
  SSC_IRQn             = 26, /**< 26 SAM3X8E Synchronous Serial Controller (SSC) */
  TC0_IRQn             = 27, /**< 27 SAM3X8E Timer Counter 0 (TC0) */
  TC1_IRQn             = 28, /**< 28 SAM3X8E Timer Counter 1 (TC1) */
  TC2_IRQn             = 29, /**< 29 SAM3X8E Timer Counter 2 (TC2) */
  TC3_IRQn             = 30, /**< 30 SAM3X8E Timer Counter 3 (TC3) */
  TC4_IRQn             = 31, /**< 31 SAM3X8E Timer Counter 4 (TC4) */
  TC5_IRQn             = 32, /**< 32 SAM3X8E Timer Counter 5 (TC5) */
  TC6_IRQn             = 33, /**< 33 SAM3X8E Timer Counter 6 (TC6) */
  TC7_IRQn             = 34, /**< 34 SAM3X8E Timer Counter 7 (TC7) */
  TC8_IRQn             = 35, /**< 35 SAM3X8E Timer Counter 8 (TC8) */
  PWM_IRQn             = 36, /**< 36 SAM3X8E Pulse Width Modulation Controller (PWM) */
  ADC_IRQn             = 37, /**< 37 SAM3X8E ADC Controller (ADC) */
  DACC_IRQn            = 38, /**< 38 SAM3X8E DAC Controller (DACC) */
  DMAC_IRQn            = 39, /**< 39 SAM3X8E DMA Controller (DMAC) */
  UOTGHS_IRQn          = 40, /**< 40 SAM3X8E USB OTG High Speed (UOTGHS) */
  TRNG_IRQn            = 41, /**< 41 SAM3X8E True Random Number Generator (TRNG) */
  EMAC_IRQn            = 42, /**< 42 SAM3X8E Ethernet MAC (EMAC) */
  CAN0_IRQn            = 43, /**< 43 SAM3X8E CAN Controller 0 (CAN0) */
  CAN1_IRQn            = 44, /**< 44 SAM3X8E CAN Controller 1 (CAN1) */

  PERIPH_COUNT_IRQn    = 45  /**< Number of peripheral IDs */
} IRQn_Type;

typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnMemManage_Handler;
  void* pfnBusFault_Handler;
  void* pfnUsageFault_Handler;
  void* pfnReserved1_Handler;
  void* pfnReserved2_Handler;
  void* pfnReserved3_Handler;
  void* pfnReserved4_Handler;
  void* pfnSVC_Handler;
  void* pfnDebugMon_Handler;
  void* pfnReserved5_Handler;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;

  /* Peripheral handlers */
  void* pfnSUPC_Handler;   /*  0 Supply Controller */
  void* pfnRSTC_Handler;   /*  1 Reset Controller */
  void* pfnRTC_Handler;    /*  2 Real Time Clock */
  void* pfnRTT_Handler;    /*  3 Real Time Timer */
  void* pfnWDT_Handler;    /*  4 Watchdog Timer */
  void* pfnPMC_Handler;    /*  5 Power Management Controller */
  void* pfnEFC0_Handler;   /*  6 Enhanced Flash Controller 0 */
  void* pfnEFC1_Handler;   /*  7 Enhanced Flash Controller 1 */
  void* pfnUART_Handler;   /*  8 Universal Asynchronous Receiver Transceiver */
  void* pfnSMC_Handler;    /*  9 Static Memory Controller */
  void* pvReserved10;
  void* pfnPIOA_Handler;   /* 11 Parallel I/O Controller A, */
  void* pfnPIOB_Handler;   /* 12 Parallel I/O Controller B */
  void* pfnPIOC_Handler;   /* 13 Parallel I/O Controller C */
  void* pfnPIOD_Handler;   /* 14 Parallel I/O Controller D */
  void* pvReserved15;
  void* pvReserved16;
  void* pfnUSART0_Handler; /* 17 USART 0 */
  void* pfnUSART1_Handler; /* 18 USART 1 */
  void* pfnUSART2_Handler; /* 19 USART 2 */
  void* pfnUSART3_Handler; /* 20 USART 3 */
  void* pfnHSMCI_Handler;  /* 21 Multimedia Card Interface */
  void* pfnTWI0_Handler;   /* 22 Two-Wire Interface 0 */
  void* pfnTWI1_Handler;   /* 23 Two-Wire Interface 1 */
  void* pfnSPI0_Handler;   /* 24 Serial Peripheral Interface */
  void* pvReserved25;
  void* pfnSSC_Handler;    /* 26 Synchronous Serial Controller */
  void* pfnTC0_Handler;    /* 27 Timer Counter 0 */
  void* pfnTC1_Handler;    /* 28 Timer Counter 1 */
  void* pfnTC2_Handler;    /* 29 Timer Counter 2 */
  void* pfnTC3_Handler;    /* 30 Timer Counter 3 */
  void* pfnTC4_Handler;    /* 31 Timer Counter 4 */
  void* pfnTC5_Handler;    /* 32 Timer Counter 5 */
  void* pfnTC6_Handler;    /* 33 Timer Counter 6 */
  void* pfnTC7_Handler;    /* 34 Timer Counter 7 */
  void* pfnTC8_Handler;    /* 35 Timer Counter 8 */
  void* pfnPWM_Handler;    /* 36 Pulse Width Modulation Controller */
  void* pfnADC_Handler;    /* 37 ADC Controller */
  void* pfnDACC_Handler;   /* 38 DAC Controller */
  void* pfnDMAC_Handler;   /* 39 DMA Controller */
  void* pfnUOTGHS_Handler; /* 40 USB OTG High Speed */
  void* pfnTRNG_Handler;   /* 41 True Random Number Generator */
  void* pfnEMAC_Handler;   /* 42 Ethernet MAC */
  void* pfnCAN0_Handler;   /* 43 CAN Controller 0 */
  void* pfnCAN1_Handler;   /* 44 CAN Controller 1 */
} DeviceVectors;

/* Cortex-M3 core handlers */
void Reset_Handler      ( void );
void NMI_Handler        ( void );
void HardFault_Handler  ( void );
void MemManage_Handler  ( void );
void BusFault_Handler   ( void );
void UsageFault_Handler ( void );
void SVC_Handler        ( void );
void DebugMon_Handler   ( void );
void PendSV_Handler     ( void );
void SysTick_Handler    ( void );

/* Peripherals handlers */
void ADC_Handler        ( void );
void CAN0_Handler       ( void );
void CAN1_Handler       ( void );
void DACC_Handler       ( void );
void DMAC_Handler       ( void );
void EFC0_Handler       ( void );
void EFC1_Handler       ( void );
void EMAC_Handler       ( void );
void HSMCI_Handler      ( void );
void PIOA_Handler       ( void );
void PIOB_Handler       ( void );
void PIOC_Handler       ( void );
void PIOD_Handler       ( void );
void PMC_Handler        ( void );
void PWM_Handler        ( void );
void RSTC_Handler       ( void );
void RTC_Handler        ( void );
void RTT_Handler        ( void );
void SMC_Handler        ( void );
void SPI0_Handler       ( void );
void SSC_Handler        ( void );
void SUPC_Handler       ( void );
void TC0_Handler        ( void );
void TC1_Handler        ( void );
void TC2_Handler        ( void );
void TC3_Handler        ( void );
void TC4_Handler        ( void );
void TC5_Handler        ( void );
void TC6_Handler        ( void );
void TC7_Handler        ( void );
void TC8_Handler        ( void );
void TRNG_Handler       ( void );
void TWI0_Handler       ( void );
void TWI1_Handler       ( void );
void UART_Handler       ( void );
void UOTGHS_Handler     ( void );
void USART0_Handler     ( void );
void USART1_Handler     ( void );
void USART2_Handler     ( void );
void USART3_Handler     ( void );
void WDT_Handler        ( void );

/**
 * \brief Configuration of the Cortex-M3 Processor and Core Peripherals
 */

#define __CM3_REV              0x0200 /**< SAM3X8E core revision number ([15:8] revision number, [7:0] patch number) */
#define __MPU_PRESENT          1      /**< SAM3X8E does provide a MPU */
#define __NVIC_PRIO_BITS       4      /**< SAM3X8E uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0      /**< Set to 1 if different SysTick Config is used */

/*
 * \brief CMSIS includes
 */

#include "cmsis-core_cm3.h"
#include "cmsis-system_sam3xa.h"

/*@}*/

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_api Peripheral Software API */
/*@{*/

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Analog-to-digital Converter */
/* ============================================================================= */
/** \addtogroup SAM3XA_ADC Analog-to-digital Converter */
/*@{*/

/** \brief Adc hardware registers */
typedef struct {
  WoReg ADC_CR;        /**< \brief (Adc Offset: 0x00) Control Register */
  RwReg ADC_MR;        /**< \brief (Adc Offset: 0x04) Mode Register */
  RwReg ADC_SEQR1;     /**< \brief (Adc Offset: 0x08) Channel Sequence Register 1 */
  RwReg ADC_SEQR2;     /**< \brief (Adc Offset: 0x0C) Channel Sequence Register 2 */
  WoReg ADC_CHER;      /**< \brief (Adc Offset: 0x10) Channel Enable Register */
  WoReg ADC_CHDR;      /**< \brief (Adc Offset: 0x14) Channel Disable Register */
  RoReg ADC_CHSR;      /**< \brief (Adc Offset: 0x18) Channel Status Register */
  RoReg Reserved1[1];
  RoReg ADC_LCDR;      /**< \brief (Adc Offset: 0x20) Last Converted Data Register */
  WoReg ADC_IER;       /**< \brief (Adc Offset: 0x24) Interrupt Enable Register */
  WoReg ADC_IDR;       /**< \brief (Adc Offset: 0x28) Interrupt Disable Register */
  RoReg ADC_IMR;       /**< \brief (Adc Offset: 0x2C) Interrupt Mask Register */
  RoReg ADC_ISR;       /**< \brief (Adc Offset: 0x30) Interrupt Status Register */
  RoReg Reserved2[2];
  RoReg ADC_OVER;      /**< \brief (Adc Offset: 0x3C) Overrun Status Register */
  RwReg ADC_EMR;       /**< \brief (Adc Offset: 0x40) Extended Mode Register */
  RwReg ADC_CWR;       /**< \brief (Adc Offset: 0x44) Compare Window Register */
  RwReg ADC_CGR;       /**< \brief (Adc Offset: 0x48) Channel Gain Register */
  RwReg ADC_COR;       /**< \brief (Adc Offset: 0x4C) Channel Offset Register */
  RoReg ADC_CDR[16];   /**< \brief (Adc Offset: 0x50) Channel Data Register */
  RoReg Reserved3[1];
  RwReg ADC_ACR;       /**< \brief (Adc Offset: 0x94) Analog Control Register */
  RoReg Reserved4[19];
  RwReg ADC_WPMR;      /**< \brief (Adc Offset: 0xE4) Write Protect Mode Register */
  RoReg ADC_WPSR;      /**< \brief (Adc Offset: 0xE8) Write Protect Status Register */
  RoReg Reserved5[5];
  RwReg ADC_RPR;       /**< \brief (Adc Offset: 0x100) Receive Pointer Register */
  RwReg ADC_RCR;       /**< \brief (Adc Offset: 0x104) Receive Counter Register */
  RoReg Reserved6[2];
  RwReg ADC_RNPR;      /**< \brief (Adc Offset: 0x110) Receive Next Pointer Register */
  RwReg ADC_RNCR;      /**< \brief (Adc Offset: 0x114) Receive Next Counter Register */
  RoReg Reserved7[2];
  WoReg ADC_PTCR;      /**< \brief (Adc Offset: 0x120) Transfer Control Register */
  RoReg ADC_PTSR;      /**< \brief (Adc Offset: 0x124) Transfer Status Register */
} Adc;
/* -------- ADC_CR : (ADC Offset: 0x00) Control Register -------- */
#define ADC_CR_SWRST (0x1u << 0) /**< \brief (ADC_CR) Software Reset */
#define ADC_CR_START (0x1u << 1) /**< \brief (ADC_CR) Start Conversion */
/* -------- ADC_MR : (ADC Offset: 0x04) Mode Register -------- */
#define ADC_MR_TRGEN (0x1u << 0) /**< \brief (ADC_MR) Trigger Enable */
#define   ADC_MR_TRGEN_DIS (0x0u << 0) /**< \brief (ADC_MR) Hardware triggers are disabled. Starting a conversion is only possible by software. */
#define   ADC_MR_TRGEN_EN (0x1u << 0) /**< \brief (ADC_MR) Hardware trigger selected by TRGSEL field is enabled. */
#define ADC_MR_TRGSEL_Pos 1
#define ADC_MR_TRGSEL_Msk (0x7u << ADC_MR_TRGSEL_Pos) /**< \brief (ADC_MR) Trigger Selection */
#define   ADC_MR_TRGSEL_ADC_TRIG0 (0x0u << 1) /**< \brief (ADC_MR) External : ADCTRG */
#define   ADC_MR_TRGSEL_ADC_TRIG1 (0x1u << 1) /**< \brief (ADC_MR) TIOA Output of the Timer Counter Channel 0 */
#define   ADC_MR_TRGSEL_ADC_TRIG2 (0x2u << 1) /**< \brief (ADC_MR) TIOA Output of the Timer Counter Channel 1 */
#define   ADC_MR_TRGSEL_ADC_TRIG3 (0x3u << 1) /**< \brief (ADC_MR) TIOA Output of the Timer Counter Channel 2 */
#define   ADC_MR_TRGSEL_ADC_TRIG4 (0x4u << 1) /**< \brief (ADC_MR) PWM Event Line 0 */
#define   ADC_MR_TRGSEL_ADC_TRIG5 (0x5u << 1) /**< \brief (ADC_MR) PWM Event Line 0 */
#define ADC_MR_LOWRES (0x1u << 4) /**< \brief (ADC_MR) Resolution */
#define   ADC_MR_LOWRES_BITS_12 (0x0u << 4) /**< \brief (ADC_MR) 12-bit resolution */
#define   ADC_MR_LOWRES_BITS_10 (0x1u << 4) /**< \brief (ADC_MR) 10-bit resolution */
#define ADC_MR_SLEEP (0x1u << 5) /**< \brief (ADC_MR) Sleep Mode */
#define   ADC_MR_SLEEP_NORMAL (0x0u << 5) /**< \brief (ADC_MR) Normal Mode: The ADC Core and reference voltage circuitry are kept ON between conversions */
#define   ADC_MR_SLEEP_SLEEP (0x1u << 5) /**< \brief (ADC_MR) Sleep Mode: The ADC Core and reference voltage circuitry are OFF between conversions */
#define ADC_MR_FWUP (0x1u << 6) /**< \brief (ADC_MR) Fast Wake Up */
#define   ADC_MR_FWUP_OFF (0x0u << 6) /**< \brief (ADC_MR) Normal Sleep Mode: The sleep mode is defined by the SLEEP bit */
#define   ADC_MR_FWUP_ON (0x1u << 6) /**< \brief (ADC_MR) Fast Wake Up Sleep Mode: The Voltage reference is ON between conversions and ADC Core is OFF */
#define ADC_MR_FREERUN (0x1u << 7) /**< \brief (ADC_MR) Free Run Mode */
#define   ADC_MR_FREERUN_OFF (0x0u << 7) /**< \brief (ADC_MR) Normal Mode */
#define   ADC_MR_FREERUN_ON (0x1u << 7) /**< \brief (ADC_MR) Free Run Mode: Never wait for any trigger. */
#define ADC_MR_PRESCAL_Pos 8
#define ADC_MR_PRESCAL_Msk (0xffu << ADC_MR_PRESCAL_Pos) /**< \brief (ADC_MR) Prescaler Rate Selection */
#define ADC_MR_PRESCAL(value) ((ADC_MR_PRESCAL_Msk & ((value) << ADC_MR_PRESCAL_Pos)))
#define ADC_MR_STARTUP_Pos 16
#define ADC_MR_STARTUP_Msk (0xfu << ADC_MR_STARTUP_Pos) /**< \brief (ADC_MR) Start Up Time */
#define   ADC_MR_STARTUP_SUT0 (0x0u << 16) /**< \brief (ADC_MR) 0 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT8 (0x1u << 16) /**< \brief (ADC_MR) 8 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT16 (0x2u << 16) /**< \brief (ADC_MR) 16 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT24 (0x3u << 16) /**< \brief (ADC_MR) 24 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT64 (0x4u << 16) /**< \brief (ADC_MR) 64 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT80 (0x5u << 16) /**< \brief (ADC_MR) 80 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT96 (0x6u << 16) /**< \brief (ADC_MR) 96 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT112 (0x7u << 16) /**< \brief (ADC_MR) 112 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT512 (0x8u << 16) /**< \brief (ADC_MR) 512 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT576 (0x9u << 16) /**< \brief (ADC_MR) 576 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT640 (0xAu << 16) /**< \brief (ADC_MR) 640 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT704 (0xBu << 16) /**< \brief (ADC_MR) 704 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT768 (0xCu << 16) /**< \brief (ADC_MR) 768 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT832 (0xDu << 16) /**< \brief (ADC_MR) 832 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT896 (0xEu << 16) /**< \brief (ADC_MR) 896 periods of ADCClock */
#define   ADC_MR_STARTUP_SUT960 (0xFu << 16) /**< \brief (ADC_MR) 960 periods of ADCClock */
#define ADC_MR_SETTLING_Pos 20
#define ADC_MR_SETTLING_Msk (0x3u << ADC_MR_SETTLING_Pos) /**< \brief (ADC_MR) Analog Settling Time */
#define   ADC_MR_SETTLING_AST3 (0x0u << 20) /**< \brief (ADC_MR) 3 periods of ADCClock */
#define   ADC_MR_SETTLING_AST5 (0x1u << 20) /**< \brief (ADC_MR) 5 periods of ADCClock */
#define   ADC_MR_SETTLING_AST9 (0x2u << 20) /**< \brief (ADC_MR) 9 periods of ADCClock */
#define   ADC_MR_SETTLING_AST17 (0x3u << 20) /**< \brief (ADC_MR) 17 periods of ADCClock */
#define ADC_MR_ANACH (0x1u << 23) /**< \brief (ADC_MR) Analog Change */
#define   ADC_MR_ANACH_NONE (0x0u << 23) /**< \brief (ADC_MR) No analog change on channel switching: DIFF0, GAIN0 and OFF0 are used for all channels */
#define   ADC_MR_ANACH_ALLOWED (0x1u << 23) /**< \brief (ADC_MR) Allows different analog settings for each channel. See ADC_CGR and ADC_COR Registers */
#define ADC_MR_TRACKTIM_Pos 24
#define ADC_MR_TRACKTIM_Msk (0xfu << ADC_MR_TRACKTIM_Pos) /**< \brief (ADC_MR) Tracking Time */
#define ADC_MR_TRACKTIM(value) ((ADC_MR_TRACKTIM_Msk & ((value) << ADC_MR_TRACKTIM_Pos)))
#define ADC_MR_TRANSFER_Pos 28
#define ADC_MR_TRANSFER_Msk (0x3u << ADC_MR_TRANSFER_Pos) /**< \brief (ADC_MR) Transfer Period */
#define ADC_MR_TRANSFER(value) ((ADC_MR_TRANSFER_Msk & ((value) << ADC_MR_TRANSFER_Pos)))
#define ADC_MR_USEQ (0x1u << 31) /**< \brief (ADC_MR) Use Sequence Enable */
#define   ADC_MR_USEQ_NUM_ORDER (0x0u << 31) /**< \brief (ADC_MR) Normal Mode: The controller converts channels in a simple numeric order. */
#define   ADC_MR_USEQ_REG_ORDER (0x1u << 31) /**< \brief (ADC_MR) User Sequence Mode: The sequence respects what is defined in ADC_SEQR1 and ADC_SEQR2 registers. */
/* -------- ADC_SEQR1 : (ADC Offset: 0x08) Channel Sequence Register 1 -------- */
#define ADC_SEQR1_USCH1_Pos 0
#define ADC_SEQR1_USCH1_Msk (0xfu << ADC_SEQR1_USCH1_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 1 */
#define ADC_SEQR1_USCH1(value) ((ADC_SEQR1_USCH1_Msk & ((value) << ADC_SEQR1_USCH1_Pos)))
#define ADC_SEQR1_USCH2_Pos 4
#define ADC_SEQR1_USCH2_Msk (0xfu << ADC_SEQR1_USCH2_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 2 */
#define ADC_SEQR1_USCH2(value) ((ADC_SEQR1_USCH2_Msk & ((value) << ADC_SEQR1_USCH2_Pos)))
#define ADC_SEQR1_USCH3_Pos 8
#define ADC_SEQR1_USCH3_Msk (0xfu << ADC_SEQR1_USCH3_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 3 */
#define ADC_SEQR1_USCH3(value) ((ADC_SEQR1_USCH3_Msk & ((value) << ADC_SEQR1_USCH3_Pos)))
#define ADC_SEQR1_USCH4_Pos 12
#define ADC_SEQR1_USCH4_Msk (0xfu << ADC_SEQR1_USCH4_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 4 */
#define ADC_SEQR1_USCH4(value) ((ADC_SEQR1_USCH4_Msk & ((value) << ADC_SEQR1_USCH4_Pos)))
#define ADC_SEQR1_USCH5_Pos 16
#define ADC_SEQR1_USCH5_Msk (0xfu << ADC_SEQR1_USCH5_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 5 */
#define ADC_SEQR1_USCH5(value) ((ADC_SEQR1_USCH5_Msk & ((value) << ADC_SEQR1_USCH5_Pos)))
#define ADC_SEQR1_USCH6_Pos 20
#define ADC_SEQR1_USCH6_Msk (0xfu << ADC_SEQR1_USCH6_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 6 */
#define ADC_SEQR1_USCH6(value) ((ADC_SEQR1_USCH6_Msk & ((value) << ADC_SEQR1_USCH6_Pos)))
#define ADC_SEQR1_USCH7_Pos 24
#define ADC_SEQR1_USCH7_Msk (0xfu << ADC_SEQR1_USCH7_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 7 */
#define ADC_SEQR1_USCH7(value) ((ADC_SEQR1_USCH7_Msk & ((value) << ADC_SEQR1_USCH7_Pos)))
#define ADC_SEQR1_USCH8_Pos 28
#define ADC_SEQR1_USCH8_Msk (0xfu << ADC_SEQR1_USCH8_Pos) /**< \brief (ADC_SEQR1) User Sequence Number 8 */
#define ADC_SEQR1_USCH8(value) ((ADC_SEQR1_USCH8_Msk & ((value) << ADC_SEQR1_USCH8_Pos)))
/* -------- ADC_SEQR2 : (ADC Offset: 0x0C) Channel Sequence Register 2 -------- */
#define ADC_SEQR2_USCH9_Pos 0
#define ADC_SEQR2_USCH9_Msk (0xfu << ADC_SEQR2_USCH9_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 9 */
#define ADC_SEQR2_USCH9(value) ((ADC_SEQR2_USCH9_Msk & ((value) << ADC_SEQR2_USCH9_Pos)))
#define ADC_SEQR2_USCH10_Pos 4
#define ADC_SEQR2_USCH10_Msk (0xfu << ADC_SEQR2_USCH10_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 10 */
#define ADC_SEQR2_USCH10(value) ((ADC_SEQR2_USCH10_Msk & ((value) << ADC_SEQR2_USCH10_Pos)))
#define ADC_SEQR2_USCH11_Pos 8
#define ADC_SEQR2_USCH11_Msk (0xfu << ADC_SEQR2_USCH11_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 11 */
#define ADC_SEQR2_USCH11(value) ((ADC_SEQR2_USCH11_Msk & ((value) << ADC_SEQR2_USCH11_Pos)))
#define ADC_SEQR2_USCH12_Pos 12
#define ADC_SEQR2_USCH12_Msk (0xfu << ADC_SEQR2_USCH12_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 12 */
#define ADC_SEQR2_USCH12(value) ((ADC_SEQR2_USCH12_Msk & ((value) << ADC_SEQR2_USCH12_Pos)))
#define ADC_SEQR2_USCH13_Pos 16
#define ADC_SEQR2_USCH13_Msk (0xfu << ADC_SEQR2_USCH13_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 13 */
#define ADC_SEQR2_USCH13(value) ((ADC_SEQR2_USCH13_Msk & ((value) << ADC_SEQR2_USCH13_Pos)))
#define ADC_SEQR2_USCH14_Pos 20
#define ADC_SEQR2_USCH14_Msk (0xfu << ADC_SEQR2_USCH14_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 14 */
#define ADC_SEQR2_USCH14(value) ((ADC_SEQR2_USCH14_Msk & ((value) << ADC_SEQR2_USCH14_Pos)))
#define ADC_SEQR2_USCH15_Pos 24
#define ADC_SEQR2_USCH15_Msk (0xfu << ADC_SEQR2_USCH15_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 15 */
#define ADC_SEQR2_USCH15(value) ((ADC_SEQR2_USCH15_Msk & ((value) << ADC_SEQR2_USCH15_Pos)))
#define ADC_SEQR2_USCH16_Pos 28
#define ADC_SEQR2_USCH16_Msk (0xfu << ADC_SEQR2_USCH16_Pos) /**< \brief (ADC_SEQR2) User Sequence Number 16 */
#define ADC_SEQR2_USCH16(value) ((ADC_SEQR2_USCH16_Msk & ((value) << ADC_SEQR2_USCH16_Pos)))
/* -------- ADC_CHER : (ADC Offset: 0x10) Channel Enable Register -------- */
#define ADC_CHER_CH0 (0x1u << 0) /**< \brief (ADC_CHER) Channel 0 Enable */
#define ADC_CHER_CH1 (0x1u << 1) /**< \brief (ADC_CHER) Channel 1 Enable */
#define ADC_CHER_CH2 (0x1u << 2) /**< \brief (ADC_CHER) Channel 2 Enable */
#define ADC_CHER_CH3 (0x1u << 3) /**< \brief (ADC_CHER) Channel 3 Enable */
#define ADC_CHER_CH4 (0x1u << 4) /**< \brief (ADC_CHER) Channel 4 Enable */
#define ADC_CHER_CH5 (0x1u << 5) /**< \brief (ADC_CHER) Channel 5 Enable */
#define ADC_CHER_CH6 (0x1u << 6) /**< \brief (ADC_CHER) Channel 6 Enable */
#define ADC_CHER_CH7 (0x1u << 7) /**< \brief (ADC_CHER) Channel 7 Enable */
#define ADC_CHER_CH8 (0x1u << 8) /**< \brief (ADC_CHER) Channel 8 Enable */
#define ADC_CHER_CH9 (0x1u << 9) /**< \brief (ADC_CHER) Channel 9 Enable */
#define ADC_CHER_CH10 (0x1u << 10) /**< \brief (ADC_CHER) Channel 10 Enable */
#define ADC_CHER_CH11 (0x1u << 11) /**< \brief (ADC_CHER) Channel 11 Enable */
#define ADC_CHER_CH12 (0x1u << 12) /**< \brief (ADC_CHER) Channel 12 Enable */
#define ADC_CHER_CH13 (0x1u << 13) /**< \brief (ADC_CHER) Channel 13 Enable */
#define ADC_CHER_CH14 (0x1u << 14) /**< \brief (ADC_CHER) Channel 14 Enable */
#define ADC_CHER_CH15 (0x1u << 15) /**< \brief (ADC_CHER) Channel 15 Enable */
/* -------- ADC_CHDR : (ADC Offset: 0x14) Channel Disable Register -------- */
#define ADC_CHDR_CH0 (0x1u << 0) /**< \brief (ADC_CHDR) Channel 0 Disable */
#define ADC_CHDR_CH1 (0x1u << 1) /**< \brief (ADC_CHDR) Channel 1 Disable */
#define ADC_CHDR_CH2 (0x1u << 2) /**< \brief (ADC_CHDR) Channel 2 Disable */
#define ADC_CHDR_CH3 (0x1u << 3) /**< \brief (ADC_CHDR) Channel 3 Disable */
#define ADC_CHDR_CH4 (0x1u << 4) /**< \brief (ADC_CHDR) Channel 4 Disable */
#define ADC_CHDR_CH5 (0x1u << 5) /**< \brief (ADC_CHDR) Channel 5 Disable */
#define ADC_CHDR_CH6 (0x1u << 6) /**< \brief (ADC_CHDR) Channel 6 Disable */
#define ADC_CHDR_CH7 (0x1u << 7) /**< \brief (ADC_CHDR) Channel 7 Disable */
#define ADC_CHDR_CH8 (0x1u << 8) /**< \brief (ADC_CHDR) Channel 8 Disable */
#define ADC_CHDR_CH9 (0x1u << 9) /**< \brief (ADC_CHDR) Channel 9 Disable */
#define ADC_CHDR_CH10 (0x1u << 10) /**< \brief (ADC_CHDR) Channel 10 Disable */
#define ADC_CHDR_CH11 (0x1u << 11) /**< \brief (ADC_CHDR) Channel 11 Disable */
#define ADC_CHDR_CH12 (0x1u << 12) /**< \brief (ADC_CHDR) Channel 12 Disable */
#define ADC_CHDR_CH13 (0x1u << 13) /**< \brief (ADC_CHDR) Channel 13 Disable */
#define ADC_CHDR_CH14 (0x1u << 14) /**< \brief (ADC_CHDR) Channel 14 Disable */
#define ADC_CHDR_CH15 (0x1u << 15) /**< \brief (ADC_CHDR) Channel 15 Disable */
/* -------- ADC_CHSR : (ADC Offset: 0x18) Channel Status Register -------- */
#define ADC_CHSR_CH0 (0x1u << 0) /**< \brief (ADC_CHSR) Channel 0 Status */
#define ADC_CHSR_CH1 (0x1u << 1) /**< \brief (ADC_CHSR) Channel 1 Status */
#define ADC_CHSR_CH2 (0x1u << 2) /**< \brief (ADC_CHSR) Channel 2 Status */
#define ADC_CHSR_CH3 (0x1u << 3) /**< \brief (ADC_CHSR) Channel 3 Status */
#define ADC_CHSR_CH4 (0x1u << 4) /**< \brief (ADC_CHSR) Channel 4 Status */
#define ADC_CHSR_CH5 (0x1u << 5) /**< \brief (ADC_CHSR) Channel 5 Status */
#define ADC_CHSR_CH6 (0x1u << 6) /**< \brief (ADC_CHSR) Channel 6 Status */
#define ADC_CHSR_CH7 (0x1u << 7) /**< \brief (ADC_CHSR) Channel 7 Status */
#define ADC_CHSR_CH8 (0x1u << 8) /**< \brief (ADC_CHSR) Channel 8 Status */
#define ADC_CHSR_CH9 (0x1u << 9) /**< \brief (ADC_CHSR) Channel 9 Status */
#define ADC_CHSR_CH10 (0x1u << 10) /**< \brief (ADC_CHSR) Channel 10 Status */
#define ADC_CHSR_CH11 (0x1u << 11) /**< \brief (ADC_CHSR) Channel 11 Status */
#define ADC_CHSR_CH12 (0x1u << 12) /**< \brief (ADC_CHSR) Channel 12 Status */
#define ADC_CHSR_CH13 (0x1u << 13) /**< \brief (ADC_CHSR) Channel 13 Status */
#define ADC_CHSR_CH14 (0x1u << 14) /**< \brief (ADC_CHSR) Channel 14 Status */
#define ADC_CHSR_CH15 (0x1u << 15) /**< \brief (ADC_CHSR) Channel 15 Status */
/* -------- ADC_LCDR : (ADC Offset: 0x20) Last Converted Data Register -------- */
#define ADC_LCDR_LDATA_Pos 0
#define ADC_LCDR_LDATA_Msk (0xfffu << ADC_LCDR_LDATA_Pos) /**< \brief (ADC_LCDR) Last Data Converted */
#define ADC_LCDR_CHNB_Pos 12
#define ADC_LCDR_CHNB_Msk (0xfu << ADC_LCDR_CHNB_Pos) /**< \brief (ADC_LCDR) Channel Number */
/* -------- ADC_IER : (ADC Offset: 0x24) Interrupt Enable Register -------- */
#define ADC_IER_EOC0 (0x1u << 0) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 0 */
#define ADC_IER_EOC1 (0x1u << 1) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 1 */
#define ADC_IER_EOC2 (0x1u << 2) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 2 */
#define ADC_IER_EOC3 (0x1u << 3) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 3 */
#define ADC_IER_EOC4 (0x1u << 4) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 4 */
#define ADC_IER_EOC5 (0x1u << 5) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 5 */
#define ADC_IER_EOC6 (0x1u << 6) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 6 */
#define ADC_IER_EOC7 (0x1u << 7) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 7 */
#define ADC_IER_EOC8 (0x1u << 8) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 8 */
#define ADC_IER_EOC9 (0x1u << 9) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 9 */
#define ADC_IER_EOC10 (0x1u << 10) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 10 */
#define ADC_IER_EOC11 (0x1u << 11) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 11 */
#define ADC_IER_EOC12 (0x1u << 12) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 12 */
#define ADC_IER_EOC13 (0x1u << 13) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 13 */
#define ADC_IER_EOC14 (0x1u << 14) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 14 */
#define ADC_IER_EOC15 (0x1u << 15) /**< \brief (ADC_IER) End of Conversion Interrupt Enable 15 */
#define ADC_IER_DRDY (0x1u << 24) /**< \brief (ADC_IER) Data Ready Interrupt Enable */
#define ADC_IER_GOVRE (0x1u << 25) /**< \brief (ADC_IER) General Overrun Error Interrupt Enable */
#define ADC_IER_COMPE (0x1u << 26) /**< \brief (ADC_IER) Comparison Event Interrupt Enable */
#define ADC_IER_ENDRX (0x1u << 27) /**< \brief (ADC_IER) End of Receive Buffer Interrupt Enable */
#define ADC_IER_RXBUFF (0x1u << 28) /**< \brief (ADC_IER) Receive Buffer Full Interrupt Enable */
/* -------- ADC_IDR : (ADC Offset: 0x28) Interrupt Disable Register -------- */
#define ADC_IDR_EOC0 (0x1u << 0) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 0 */
#define ADC_IDR_EOC1 (0x1u << 1) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 1 */
#define ADC_IDR_EOC2 (0x1u << 2) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 2 */
#define ADC_IDR_EOC3 (0x1u << 3) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 3 */
#define ADC_IDR_EOC4 (0x1u << 4) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 4 */
#define ADC_IDR_EOC5 (0x1u << 5) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 5 */
#define ADC_IDR_EOC6 (0x1u << 6) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 6 */
#define ADC_IDR_EOC7 (0x1u << 7) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 7 */
#define ADC_IDR_EOC8 (0x1u << 8) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 8 */
#define ADC_IDR_EOC9 (0x1u << 9) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 9 */
#define ADC_IDR_EOC10 (0x1u << 10) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 10 */
#define ADC_IDR_EOC11 (0x1u << 11) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 11 */
#define ADC_IDR_EOC12 (0x1u << 12) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 12 */
#define ADC_IDR_EOC13 (0x1u << 13) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 13 */
#define ADC_IDR_EOC14 (0x1u << 14) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 14 */
#define ADC_IDR_EOC15 (0x1u << 15) /**< \brief (ADC_IDR) End of Conversion Interrupt Disable 15 */
#define ADC_IDR_DRDY (0x1u << 24) /**< \brief (ADC_IDR) Data Ready Interrupt Disable */
#define ADC_IDR_GOVRE (0x1u << 25) /**< \brief (ADC_IDR) General Overrun Error Interrupt Disable */
#define ADC_IDR_COMPE (0x1u << 26) /**< \brief (ADC_IDR) Comparison Event Interrupt Disable */
#define ADC_IDR_ENDRX (0x1u << 27) /**< \brief (ADC_IDR) End of Receive Buffer Interrupt Disable */
#define ADC_IDR_RXBUFF (0x1u << 28) /**< \brief (ADC_IDR) Receive Buffer Full Interrupt Disable */
/* -------- ADC_IMR : (ADC Offset: 0x2C) Interrupt Mask Register -------- */
#define ADC_IMR_EOC0 (0x1u << 0) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 0 */
#define ADC_IMR_EOC1 (0x1u << 1) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 1 */
#define ADC_IMR_EOC2 (0x1u << 2) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 2 */
#define ADC_IMR_EOC3 (0x1u << 3) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 3 */
#define ADC_IMR_EOC4 (0x1u << 4) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 4 */
#define ADC_IMR_EOC5 (0x1u << 5) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 5 */
#define ADC_IMR_EOC6 (0x1u << 6) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 6 */
#define ADC_IMR_EOC7 (0x1u << 7) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 7 */
#define ADC_IMR_EOC8 (0x1u << 8) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 8 */
#define ADC_IMR_EOC9 (0x1u << 9) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 9 */
#define ADC_IMR_EOC10 (0x1u << 10) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 10 */
#define ADC_IMR_EOC11 (0x1u << 11) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 11 */
#define ADC_IMR_EOC12 (0x1u << 12) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 12 */
#define ADC_IMR_EOC13 (0x1u << 13) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 13 */
#define ADC_IMR_EOC14 (0x1u << 14) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 14 */
#define ADC_IMR_EOC15 (0x1u << 15) /**< \brief (ADC_IMR) End of Conversion Interrupt Mask 15 */
#define ADC_IMR_DRDY (0x1u << 24) /**< \brief (ADC_IMR) Data Ready Interrupt Mask */
#define ADC_IMR_GOVRE (0x1u << 25) /**< \brief (ADC_IMR) General Overrun Error Interrupt Mask */
#define ADC_IMR_COMPE (0x1u << 26) /**< \brief (ADC_IMR) Comparison Event Interrupt Mask */
#define ADC_IMR_ENDRX (0x1u << 27) /**< \brief (ADC_IMR) End of Receive Buffer Interrupt Mask */
#define ADC_IMR_RXBUFF (0x1u << 28) /**< \brief (ADC_IMR) Receive Buffer Full Interrupt Mask */
/* -------- ADC_ISR : (ADC Offset: 0x30) Interrupt Status Register -------- */
#define ADC_ISR_EOC0 (0x1u << 0) /**< \brief (ADC_ISR) End of Conversion 0 */
#define ADC_ISR_EOC1 (0x1u << 1) /**< \brief (ADC_ISR) End of Conversion 1 */
#define ADC_ISR_EOC2 (0x1u << 2) /**< \brief (ADC_ISR) End of Conversion 2 */
#define ADC_ISR_EOC3 (0x1u << 3) /**< \brief (ADC_ISR) End of Conversion 3 */
#define ADC_ISR_EOC4 (0x1u << 4) /**< \brief (ADC_ISR) End of Conversion 4 */
#define ADC_ISR_EOC5 (0x1u << 5) /**< \brief (ADC_ISR) End of Conversion 5 */
#define ADC_ISR_EOC6 (0x1u << 6) /**< \brief (ADC_ISR) End of Conversion 6 */
#define ADC_ISR_EOC7 (0x1u << 7) /**< \brief (ADC_ISR) End of Conversion 7 */
#define ADC_ISR_EOC8 (0x1u << 8) /**< \brief (ADC_ISR) End of Conversion 8 */
#define ADC_ISR_EOC9 (0x1u << 9) /**< \brief (ADC_ISR) End of Conversion 9 */
#define ADC_ISR_EOC10 (0x1u << 10) /**< \brief (ADC_ISR) End of Conversion 10 */
#define ADC_ISR_EOC11 (0x1u << 11) /**< \brief (ADC_ISR) End of Conversion 11 */
#define ADC_ISR_EOC12 (0x1u << 12) /**< \brief (ADC_ISR) End of Conversion 12 */
#define ADC_ISR_EOC13 (0x1u << 13) /**< \brief (ADC_ISR) End of Conversion 13 */
#define ADC_ISR_EOC14 (0x1u << 14) /**< \brief (ADC_ISR) End of Conversion 14 */
#define ADC_ISR_EOC15 (0x1u << 15) /**< \brief (ADC_ISR) End of Conversion 15 */
#define ADC_ISR_DRDY (0x1u << 24) /**< \brief (ADC_ISR) Data Ready */
#define ADC_ISR_GOVRE (0x1u << 25) /**< \brief (ADC_ISR) General Overrun Error */
#define ADC_ISR_COMPE (0x1u << 26) /**< \brief (ADC_ISR) Comparison Error */
#define ADC_ISR_ENDRX (0x1u << 27) /**< \brief (ADC_ISR) End of RX Buffer */
#define ADC_ISR_RXBUFF (0x1u << 28) /**< \brief (ADC_ISR) RX Buffer Full */
/* -------- ADC_OVER : (ADC Offset: 0x3C) Overrun Status Register -------- */
#define ADC_OVER_OVRE0 (0x1u << 0) /**< \brief (ADC_OVER) Overrun Error 0 */
#define ADC_OVER_OVRE1 (0x1u << 1) /**< \brief (ADC_OVER) Overrun Error 1 */
#define ADC_OVER_OVRE2 (0x1u << 2) /**< \brief (ADC_OVER) Overrun Error 2 */
#define ADC_OVER_OVRE3 (0x1u << 3) /**< \brief (ADC_OVER) Overrun Error 3 */
#define ADC_OVER_OVRE4 (0x1u << 4) /**< \brief (ADC_OVER) Overrun Error 4 */
#define ADC_OVER_OVRE5 (0x1u << 5) /**< \brief (ADC_OVER) Overrun Error 5 */
#define ADC_OVER_OVRE6 (0x1u << 6) /**< \brief (ADC_OVER) Overrun Error 6 */
#define ADC_OVER_OVRE7 (0x1u << 7) /**< \brief (ADC_OVER) Overrun Error 7 */
#define ADC_OVER_OVRE8 (0x1u << 8) /**< \brief (ADC_OVER) Overrun Error 8 */
#define ADC_OVER_OVRE9 (0x1u << 9) /**< \brief (ADC_OVER) Overrun Error 9 */
#define ADC_OVER_OVRE10 (0x1u << 10) /**< \brief (ADC_OVER) Overrun Error 10 */
#define ADC_OVER_OVRE11 (0x1u << 11) /**< \brief (ADC_OVER) Overrun Error 11 */
#define ADC_OVER_OVRE12 (0x1u << 12) /**< \brief (ADC_OVER) Overrun Error 12 */
#define ADC_OVER_OVRE13 (0x1u << 13) /**< \brief (ADC_OVER) Overrun Error 13 */
#define ADC_OVER_OVRE14 (0x1u << 14) /**< \brief (ADC_OVER) Overrun Error 14 */
#define ADC_OVER_OVRE15 (0x1u << 15) /**< \brief (ADC_OVER) Overrun Error 15 */
/* -------- ADC_EMR : (ADC Offset: 0x40) Extended Mode Register -------- */
#define ADC_EMR_CMPMODE_Pos 0
#define ADC_EMR_CMPMODE_Msk (0x3u << ADC_EMR_CMPMODE_Pos) /**< \brief (ADC_EMR) Comparison Mode */
#define   ADC_EMR_CMPMODE_LOW (0x0u << 0) /**< \brief (ADC_EMR) Generates an event when the converted data is lower than the low threshold of the window. */
#define   ADC_EMR_CMPMODE_HIGH (0x1u << 0) /**< \brief (ADC_EMR) Generates an event when the converted data is higher than the high threshold of the window. */
#define   ADC_EMR_CMPMODE_IN (0x2u << 0) /**< \brief (ADC_EMR) Generates an event when the converted data is in the comparison window. */
#define   ADC_EMR_CMPMODE_OUT (0x3u << 0) /**< \brief (ADC_EMR) Generates an event when the converted data is out of the comparison window. */
#define ADC_EMR_CMPSEL_Pos 4
#define ADC_EMR_CMPSEL_Msk (0xfu << ADC_EMR_CMPSEL_Pos) /**< \brief (ADC_EMR) Comparison Selected Channel */
#define ADC_EMR_CMPSEL(value) ((ADC_EMR_CMPSEL_Msk & ((value) << ADC_EMR_CMPSEL_Pos)))
#define ADC_EMR_CMPALL (0x1u << 9) /**< \brief (ADC_EMR) Compare All Channels */
#define ADC_EMR_CMPFILTER_Pos 12
#define ADC_EMR_CMPFILTER_Msk (0x3u << ADC_EMR_CMPFILTER_Pos) /**< \brief (ADC_EMR) Compare Event Filtering */
#define ADC_EMR_CMPFILTER(value) ((ADC_EMR_CMPFILTER_Msk & ((value) << ADC_EMR_CMPFILTER_Pos)))
#define ADC_EMR_TAG (0x1u << 24) /**< \brief (ADC_EMR) TAG of ADC_LDCR register */
/* -------- ADC_CWR : (ADC Offset: 0x44) Compare Window Register -------- */
#define ADC_CWR_LOWTHRES_Pos 0
#define ADC_CWR_LOWTHRES_Msk (0xfffu << ADC_CWR_LOWTHRES_Pos) /**< \brief (ADC_CWR) Low Threshold */
#define ADC_CWR_LOWTHRES(value) ((ADC_CWR_LOWTHRES_Msk & ((value) << ADC_CWR_LOWTHRES_Pos)))
#define ADC_CWR_HIGHTHRES_Pos 16
#define ADC_CWR_HIGHTHRES_Msk (0xfffu << ADC_CWR_HIGHTHRES_Pos) /**< \brief (ADC_CWR) High Threshold */
#define ADC_CWR_HIGHTHRES(value) ((ADC_CWR_HIGHTHRES_Msk & ((value) << ADC_CWR_HIGHTHRES_Pos)))
/* -------- ADC_CGR : (ADC Offset: 0x48) Channel Gain Register -------- */
#define ADC_CGR_GAIN0_Pos 0
#define ADC_CGR_GAIN0_Msk (0x3u << ADC_CGR_GAIN0_Pos) /**< \brief (ADC_CGR) Gain for channel 0 */
#define ADC_CGR_GAIN0(value) ((ADC_CGR_GAIN0_Msk & ((value) << ADC_CGR_GAIN0_Pos)))
#define ADC_CGR_GAIN1_Pos 2
#define ADC_CGR_GAIN1_Msk (0x3u << ADC_CGR_GAIN1_Pos) /**< \brief (ADC_CGR) Gain for channel 1 */
#define ADC_CGR_GAIN1(value) ((ADC_CGR_GAIN1_Msk & ((value) << ADC_CGR_GAIN1_Pos)))
#define ADC_CGR_GAIN2_Pos 4
#define ADC_CGR_GAIN2_Msk (0x3u << ADC_CGR_GAIN2_Pos) /**< \brief (ADC_CGR) Gain for channel 2 */
#define ADC_CGR_GAIN2(value) ((ADC_CGR_GAIN2_Msk & ((value) << ADC_CGR_GAIN2_Pos)))
#define ADC_CGR_GAIN3_Pos 6
#define ADC_CGR_GAIN3_Msk (0x3u << ADC_CGR_GAIN3_Pos) /**< \brief (ADC_CGR) Gain for channel 3 */
#define ADC_CGR_GAIN3(value) ((ADC_CGR_GAIN3_Msk & ((value) << ADC_CGR_GAIN3_Pos)))
#define ADC_CGR_GAIN4_Pos 8
#define ADC_CGR_GAIN4_Msk (0x3u << ADC_CGR_GAIN4_Pos) /**< \brief (ADC_CGR) Gain for channel 4 */
#define ADC_CGR_GAIN4(value) ((ADC_CGR_GAIN4_Msk & ((value) << ADC_CGR_GAIN4_Pos)))
#define ADC_CGR_GAIN5_Pos 10
#define ADC_CGR_GAIN5_Msk (0x3u << ADC_CGR_GAIN5_Pos) /**< \brief (ADC_CGR) Gain for channel 5 */
#define ADC_CGR_GAIN5(value) ((ADC_CGR_GAIN5_Msk & ((value) << ADC_CGR_GAIN5_Pos)))
#define ADC_CGR_GAIN6_Pos 12
#define ADC_CGR_GAIN6_Msk (0x3u << ADC_CGR_GAIN6_Pos) /**< \brief (ADC_CGR) Gain for channel 6 */
#define ADC_CGR_GAIN6(value) ((ADC_CGR_GAIN6_Msk & ((value) << ADC_CGR_GAIN6_Pos)))
#define ADC_CGR_GAIN7_Pos 14
#define ADC_CGR_GAIN7_Msk (0x3u << ADC_CGR_GAIN7_Pos) /**< \brief (ADC_CGR) Gain for channel 7 */
#define ADC_CGR_GAIN7(value) ((ADC_CGR_GAIN7_Msk & ((value) << ADC_CGR_GAIN7_Pos)))
#define ADC_CGR_GAIN8_Pos 16
#define ADC_CGR_GAIN8_Msk (0x3u << ADC_CGR_GAIN8_Pos) /**< \brief (ADC_CGR) Gain for channel 8 */
#define ADC_CGR_GAIN8(value) ((ADC_CGR_GAIN8_Msk & ((value) << ADC_CGR_GAIN8_Pos)))
#define ADC_CGR_GAIN9_Pos 18
#define ADC_CGR_GAIN9_Msk (0x3u << ADC_CGR_GAIN9_Pos) /**< \brief (ADC_CGR) Gain for channel 9 */
#define ADC_CGR_GAIN9(value) ((ADC_CGR_GAIN9_Msk & ((value) << ADC_CGR_GAIN9_Pos)))
#define ADC_CGR_GAIN10_Pos 20
#define ADC_CGR_GAIN10_Msk (0x3u << ADC_CGR_GAIN10_Pos) /**< \brief (ADC_CGR) Gain for channel 10 */
#define ADC_CGR_GAIN10(value) ((ADC_CGR_GAIN10_Msk & ((value) << ADC_CGR_GAIN10_Pos)))
#define ADC_CGR_GAIN11_Pos 22
#define ADC_CGR_GAIN11_Msk (0x3u << ADC_CGR_GAIN11_Pos) /**< \brief (ADC_CGR) Gain for channel 11 */
#define ADC_CGR_GAIN11(value) ((ADC_CGR_GAIN11_Msk & ((value) << ADC_CGR_GAIN11_Pos)))
#define ADC_CGR_GAIN12_Pos 24
#define ADC_CGR_GAIN12_Msk (0x3u << ADC_CGR_GAIN12_Pos) /**< \brief (ADC_CGR) Gain for channel 12 */
#define ADC_CGR_GAIN12(value) ((ADC_CGR_GAIN12_Msk & ((value) << ADC_CGR_GAIN12_Pos)))
#define ADC_CGR_GAIN13_Pos 26
#define ADC_CGR_GAIN13_Msk (0x3u << ADC_CGR_GAIN13_Pos) /**< \brief (ADC_CGR) Gain for channel 13 */
#define ADC_CGR_GAIN13(value) ((ADC_CGR_GAIN13_Msk & ((value) << ADC_CGR_GAIN13_Pos)))
#define ADC_CGR_GAIN14_Pos 28
#define ADC_CGR_GAIN14_Msk (0x3u << ADC_CGR_GAIN14_Pos) /**< \brief (ADC_CGR) Gain for channel 14 */
#define ADC_CGR_GAIN14(value) ((ADC_CGR_GAIN14_Msk & ((value) << ADC_CGR_GAIN14_Pos)))
#define ADC_CGR_GAIN15_Pos 30
#define ADC_CGR_GAIN15_Msk (0x3u << ADC_CGR_GAIN15_Pos) /**< \brief (ADC_CGR) Gain for channel 15 */
#define ADC_CGR_GAIN15(value) ((ADC_CGR_GAIN15_Msk & ((value) << ADC_CGR_GAIN15_Pos)))
/* -------- ADC_COR : (ADC Offset: 0x4C) Channel Offset Register -------- */
#define ADC_COR_OFF0 (0x1u << 0) /**< \brief (ADC_COR) Offset for channel 0 */
#define ADC_COR_OFF1 (0x1u << 1) /**< \brief (ADC_COR) Offset for channel 1 */
#define ADC_COR_OFF2 (0x1u << 2) /**< \brief (ADC_COR) Offset for channel 2 */
#define ADC_COR_OFF3 (0x1u << 3) /**< \brief (ADC_COR) Offset for channel 3 */
#define ADC_COR_OFF4 (0x1u << 4) /**< \brief (ADC_COR) Offset for channel 4 */
#define ADC_COR_OFF5 (0x1u << 5) /**< \brief (ADC_COR) Offset for channel 5 */
#define ADC_COR_OFF6 (0x1u << 6) /**< \brief (ADC_COR) Offset for channel 6 */
#define ADC_COR_OFF7 (0x1u << 7) /**< \brief (ADC_COR) Offset for channel 7 */
#define ADC_COR_OFF8 (0x1u << 8) /**< \brief (ADC_COR) Offset for channel 8 */
#define ADC_COR_OFF9 (0x1u << 9) /**< \brief (ADC_COR) Offset for channel 9 */
#define ADC_COR_OFF10 (0x1u << 10) /**< \brief (ADC_COR) Offset for channel 10 */
#define ADC_COR_OFF11 (0x1u << 11) /**< \brief (ADC_COR) Offset for channel 11 */
#define ADC_COR_OFF12 (0x1u << 12) /**< \brief (ADC_COR) Offset for channel 12 */
#define ADC_COR_OFF13 (0x1u << 13) /**< \brief (ADC_COR) Offset for channel 13 */
#define ADC_COR_OFF14 (0x1u << 14) /**< \brief (ADC_COR) Offset for channel 14 */
#define ADC_COR_OFF15 (0x1u << 15) /**< \brief (ADC_COR) Offset for channel 15 */
#define ADC_COR_DIFF0 (0x1u << 16) /**< \brief (ADC_COR) Differential inputs for channel 0 */
#define ADC_COR_DIFF1 (0x1u << 17) /**< \brief (ADC_COR) Differential inputs for channel 1 */
#define ADC_COR_DIFF2 (0x1u << 18) /**< \brief (ADC_COR) Differential inputs for channel 2 */
#define ADC_COR_DIFF3 (0x1u << 19) /**< \brief (ADC_COR) Differential inputs for channel 3 */
#define ADC_COR_DIFF4 (0x1u << 20) /**< \brief (ADC_COR) Differential inputs for channel 4 */
#define ADC_COR_DIFF5 (0x1u << 21) /**< \brief (ADC_COR) Differential inputs for channel 5 */
#define ADC_COR_DIFF6 (0x1u << 22) /**< \brief (ADC_COR) Differential inputs for channel 6 */
#define ADC_COR_DIFF7 (0x1u << 23) /**< \brief (ADC_COR) Differential inputs for channel 7 */
#define ADC_COR_DIFF8 (0x1u << 24) /**< \brief (ADC_COR) Differential inputs for channel 8 */
#define ADC_COR_DIFF9 (0x1u << 25) /**< \brief (ADC_COR) Differential inputs for channel 9 */
#define ADC_COR_DIFF10 (0x1u << 26) /**< \brief (ADC_COR) Differential inputs for channel 10 */
#define ADC_COR_DIFF11 (0x1u << 27) /**< \brief (ADC_COR) Differential inputs for channel 11 */
#define ADC_COR_DIFF12 (0x1u << 28) /**< \brief (ADC_COR) Differential inputs for channel 12 */
#define ADC_COR_DIFF13 (0x1u << 29) /**< \brief (ADC_COR) Differential inputs for channel 13 */
#define ADC_COR_DIFF14 (0x1u << 30) /**< \brief (ADC_COR) Differential inputs for channel 14 */
#define ADC_COR_DIFF15 (0x1u << 31) /**< \brief (ADC_COR) Differential inputs for channel 15 */
/* -------- ADC_CDR[16] : (ADC Offset: 0x50) Channel Data Register -------- */
#define ADC_CDR_DATA_Pos 0
#define ADC_CDR_DATA_Msk (0xfffu << ADC_CDR_DATA_Pos) /**< \brief (ADC_CDR[16]) Converted Data */
/* -------- ADC_ACR : (ADC Offset: 0x94) Analog Control Register -------- */
#define ADC_ACR_TSON (0x1u << 4) /**< \brief (ADC_ACR) Temperature Sensor On */
#define ADC_ACR_IBCTL_Pos 8
#define ADC_ACR_IBCTL_Msk (0x3u << ADC_ACR_IBCTL_Pos) /**< \brief (ADC_ACR) ADC Bias Current Control */
#define ADC_ACR_IBCTL(value) ((ADC_ACR_IBCTL_Msk & ((value) << ADC_ACR_IBCTL_Pos)))
/* -------- ADC_WPMR : (ADC Offset: 0xE4) Write Protect Mode Register -------- */
#define ADC_WPMR_WPEN (0x1u << 0) /**< \brief (ADC_WPMR) Write Protect Enable */
#define ADC_WPMR_WPKEY_Pos 8
#define ADC_WPMR_WPKEY_Msk (0xffffffu << ADC_WPMR_WPKEY_Pos) /**< \brief (ADC_WPMR) Write Protect KEY */
#define ADC_WPMR_WPKEY(value) ((ADC_WPMR_WPKEY_Msk & ((value) << ADC_WPMR_WPKEY_Pos)))
/* -------- ADC_WPSR : (ADC Offset: 0xE8) Write Protect Status Register -------- */
#define ADC_WPSR_WPVS (0x1u << 0) /**< \brief (ADC_WPSR) Write Protect Violation Status */
#define ADC_WPSR_WPVSRC_Pos 8
#define ADC_WPSR_WPVSRC_Msk (0xffffu << ADC_WPSR_WPVSRC_Pos) /**< \brief (ADC_WPSR) Write Protect Violation Source */
/* -------- ADC_RPR : (ADC Offset: 0x100) Receive Pointer Register -------- */
#define ADC_RPR_RXPTR_Pos 0
#define ADC_RPR_RXPTR_Msk (0xffffffffu << ADC_RPR_RXPTR_Pos) /**< \brief (ADC_RPR) Receive Pointer Register */
#define ADC_RPR_RXPTR(value) ((ADC_RPR_RXPTR_Msk & ((value) << ADC_RPR_RXPTR_Pos)))
/* -------- ADC_RCR : (ADC Offset: 0x104) Receive Counter Register -------- */
#define ADC_RCR_RXCTR_Pos 0
#define ADC_RCR_RXCTR_Msk (0xffffu << ADC_RCR_RXCTR_Pos) /**< \brief (ADC_RCR) Receive Counter Register */
#define ADC_RCR_RXCTR(value) ((ADC_RCR_RXCTR_Msk & ((value) << ADC_RCR_RXCTR_Pos)))
/* -------- ADC_RNPR : (ADC Offset: 0x110) Receive Next Pointer Register -------- */
#define ADC_RNPR_RXNPTR_Pos 0
#define ADC_RNPR_RXNPTR_Msk (0xffffffffu << ADC_RNPR_RXNPTR_Pos) /**< \brief (ADC_RNPR) Receive Next Pointer */
#define ADC_RNPR_RXNPTR(value) ((ADC_RNPR_RXNPTR_Msk & ((value) << ADC_RNPR_RXNPTR_Pos)))
/* -------- ADC_RNCR : (ADC Offset: 0x114) Receive Next Counter Register -------- */
#define ADC_RNCR_RXNCTR_Pos 0
#define ADC_RNCR_RXNCTR_Msk (0xffffu << ADC_RNCR_RXNCTR_Pos) /**< \brief (ADC_RNCR) Receive Next Counter */
#define ADC_RNCR_RXNCTR(value) ((ADC_RNCR_RXNCTR_Msk & ((value) << ADC_RNCR_RXNCTR_Pos)))
/* -------- ADC_PTCR : (ADC Offset: 0x120) Transfer Control Register -------- */
#define ADC_PTCR_RXTEN (0x1u << 0) /**< \brief (ADC_PTCR) Receiver Transfer Enable */
#define ADC_PTCR_RXTDIS (0x1u << 1) /**< \brief (ADC_PTCR) Receiver Transfer Disable */
#define ADC_PTCR_TXTEN (0x1u << 8) /**< \brief (ADC_PTCR) Transmitter Transfer Enable */
#define ADC_PTCR_TXTDIS (0x1u << 9) /**< \brief (ADC_PTCR) Transmitter Transfer Disable */
/* -------- ADC_PTSR : (ADC Offset: 0x124) Transfer Status Register -------- */
#define ADC_PTSR_RXTEN (0x1u << 0) /**< \brief (ADC_PTSR) Receiver Transfer Enable */
#define ADC_PTSR_TXTEN (0x1u << 8) /**< \brief (ADC_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_ADC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Controller Area Network */
/* ============================================================================= */
/** \addtogroup SAM3XA_CAN Controller Area Network */
/*@{*/

/** \brief CanMb hardware registers */
typedef struct {
  RwReg  CAN_MMR;       /**< \brief (CanMb Offset: 0x0) Mailbox Mode Register */
  RwReg  CAN_MAM;       /**< \brief (CanMb Offset: 0x4) Mailbox Acceptance Mask Register */
  RwReg  CAN_MID;       /**< \brief (CanMb Offset: 0x8) Mailbox ID Register */
  RwReg  CAN_MFID;      /**< \brief (CanMb Offset: 0xC) Mailbox Family ID Register */
  RwReg  CAN_MSR;       /**< \brief (CanMb Offset: 0x10) Mailbox Status Register */
  RwReg  CAN_MDL;       /**< \brief (CanMb Offset: 0x14) Mailbox Data Low Register */
  RwReg  CAN_MDH;       /**< \brief (CanMb Offset: 0x18) Mailbox Data High Register */
  RwReg  CAN_MCR;       /**< \brief (CanMb Offset: 0x1C) Mailbox Control Register */
} CanMb;
/** \brief Can hardware registers */
#define CANMB_NUMBER 8
typedef struct {
  RwReg  CAN_MR;        /**< \brief (Can Offset: 0x0000) Mode Register */
  WoReg  CAN_IER;       /**< \brief (Can Offset: 0x0004) Interrupt Enable Register */
  WoReg  CAN_IDR;       /**< \brief (Can Offset: 0x0008) Interrupt Disable Register */
  RoReg  CAN_IMR;       /**< \brief (Can Offset: 0x000C) Interrupt Mask Register */
  RoReg  CAN_SR;        /**< \brief (Can Offset: 0x0010) Status Register */
  RwReg  CAN_BR;        /**< \brief (Can Offset: 0x0014) Baudrate Register */
  RoReg  CAN_TIM;       /**< \brief (Can Offset: 0x0018) Timer Register */
  RoReg  CAN_TIMESTP;   /**< \brief (Can Offset: 0x001C) Timestamp Register */
  RoReg  CAN_ECR;       /**< \brief (Can Offset: 0x0020) Error Counter Register */
  WoReg  CAN_TCR;       /**< \brief (Can Offset: 0x0024) Transfer Command Register */
  WoReg  CAN_ACR;       /**< \brief (Can Offset: 0x0028) Abort Command Register */
  RoReg  Reserved1[46];
  RwReg  CAN_WPMR;      /**< \brief (Can Offset: 0x00E4) Write Protect Mode Register */
  RoReg  CAN_WPSR;      /**< \brief (Can Offset: 0x00E8) Write Protect Status Register */
  RoReg  Reserved2[69];
  CanMb  CAN_MB[CANMB_NUMBER]; /**< \brief (Can Offset: 0x200) MB = 0 .. 7 */
} Can;
/* -------- CAN_MR : (CAN Offset: 0x0000) Mode Register -------- */
#define CAN_MR_CANEN (0x1u << 0) /**< \brief (CAN_MR) CAN Controller Enable */
#define CAN_MR_LPM (0x1u << 1) /**< \brief (CAN_MR) Disable/Enable Low Power Mode */
#define CAN_MR_ABM (0x1u << 2) /**< \brief (CAN_MR) Disable/Enable Autobaud/Listen mode */
#define CAN_MR_OVL (0x1u << 3) /**< \brief (CAN_MR) Disable/Enable Overload Frame */
#define CAN_MR_TEOF (0x1u << 4) /**< \brief (CAN_MR) Timestamp messages at each end of Frame */
#define CAN_MR_TTM (0x1u << 5) /**< \brief (CAN_MR) Disable/Enable Time Triggered Mode */
#define CAN_MR_TIMFRZ (0x1u << 6) /**< \brief (CAN_MR) Enable Timer Freeze */
#define CAN_MR_DRPT (0x1u << 7) /**< \brief (CAN_MR) Disable Repeat */
#define CAN_MR_RXSYNC_Pos 24
#define CAN_MR_RXSYNC_Msk (0x7u << CAN_MR_RXSYNC_Pos) /**< \brief (CAN_MR) Reception Synchronization Stage (not readable) */
#define   CAN_MR_RXSYNC_DOUBLE_PP (0x0u << 24) /**< \brief (CAN_MR) Rx Signal with Double Synchro Stages (2 Positive Edges) */
#define   CAN_MR_RXSYNC_DOUBLE_PN (0x1u << 24) /**< \brief (CAN_MR) Rx Signal with Double Synchro Stages (One Positive Edge and One Negative Edge) */
#define   CAN_MR_RXSYNC_SINGLE_P (0x2u << 24) /**< \brief (CAN_MR) Rx Signal with Single Synchro Stage (Positive Edge) */
#define   CAN_MR_RXSYNC_NONE (0x3u << 24) /**< \brief (CAN_MR) Rx Signal with No Synchro Stage */
/* -------- CAN_IER : (CAN Offset: 0x0004) Interrupt Enable Register -------- */
#define CAN_IER_MB0 (0x1u << 0) /**< \brief (CAN_IER) Mailbox 0 Interrupt Enable */
#define CAN_IER_MB1 (0x1u << 1) /**< \brief (CAN_IER) Mailbox 1 Interrupt Enable */
#define CAN_IER_MB2 (0x1u << 2) /**< \brief (CAN_IER) Mailbox 2 Interrupt Enable */
#define CAN_IER_MB3 (0x1u << 3) /**< \brief (CAN_IER) Mailbox 3 Interrupt Enable */
#define CAN_IER_MB4 (0x1u << 4) /**< \brief (CAN_IER) Mailbox 4 Interrupt Enable */
#define CAN_IER_MB5 (0x1u << 5) /**< \brief (CAN_IER) Mailbox 5 Interrupt Enable */
#define CAN_IER_MB6 (0x1u << 6) /**< \brief (CAN_IER) Mailbox 6 Interrupt Enable */
#define CAN_IER_MB7 (0x1u << 7) /**< \brief (CAN_IER) Mailbox 7 Interrupt Enable */
#define CAN_IER_ERRA (0x1u << 16) /**< \brief (CAN_IER) Error Active Mode Interrupt Enable */
#define CAN_IER_WARN (0x1u << 17) /**< \brief (CAN_IER) Warning Limit Interrupt Enable */
#define CAN_IER_ERRP (0x1u << 18) /**< \brief (CAN_IER) Error Passive Mode Interrupt Enable */
#define CAN_IER_BOFF (0x1u << 19) /**< \brief (CAN_IER) Bus Off Mode Interrupt Enable */
#define CAN_IER_SLEEP (0x1u << 20) /**< \brief (CAN_IER) Sleep Interrupt Enable */
#define CAN_IER_WAKEUP (0x1u << 21) /**< \brief (CAN_IER) Wakeup Interrupt Enable */
#define CAN_IER_TOVF (0x1u << 22) /**< \brief (CAN_IER) Timer Overflow Interrupt Enable */
#define CAN_IER_TSTP (0x1u << 23) /**< \brief (CAN_IER) TimeStamp Interrupt Enable */
#define CAN_IER_CERR (0x1u << 24) /**< \brief (CAN_IER) CRC Error Interrupt Enable */
#define CAN_IER_SERR (0x1u << 25) /**< \brief (CAN_IER) Stuffing Error Interrupt Enable */
#define CAN_IER_AERR (0x1u << 26) /**< \brief (CAN_IER) Acknowledgment Error Interrupt Enable */
#define CAN_IER_FERR (0x1u << 27) /**< \brief (CAN_IER) Form Error Interrupt Enable */
#define CAN_IER_BERR (0x1u << 28) /**< \brief (CAN_IER) Bit Error Interrupt Enable */
/* -------- CAN_IDR : (CAN Offset: 0x0008) Interrupt Disable Register -------- */
#define CAN_IDR_MB0 (0x1u << 0) /**< \brief (CAN_IDR) Mailbox 0 Interrupt Disable */
#define CAN_IDR_MB1 (0x1u << 1) /**< \brief (CAN_IDR) Mailbox 1 Interrupt Disable */
#define CAN_IDR_MB2 (0x1u << 2) /**< \brief (CAN_IDR) Mailbox 2 Interrupt Disable */
#define CAN_IDR_MB3 (0x1u << 3) /**< \brief (CAN_IDR) Mailbox 3 Interrupt Disable */
#define CAN_IDR_MB4 (0x1u << 4) /**< \brief (CAN_IDR) Mailbox 4 Interrupt Disable */
#define CAN_IDR_MB5 (0x1u << 5) /**< \brief (CAN_IDR) Mailbox 5 Interrupt Disable */
#define CAN_IDR_MB6 (0x1u << 6) /**< \brief (CAN_IDR) Mailbox 6 Interrupt Disable */
#define CAN_IDR_MB7 (0x1u << 7) /**< \brief (CAN_IDR) Mailbox 7 Interrupt Disable */
#define CAN_IDR_ERRA (0x1u << 16) /**< \brief (CAN_IDR) Error Active Mode Interrupt Disable */
#define CAN_IDR_WARN (0x1u << 17) /**< \brief (CAN_IDR) Warning Limit Interrupt Disable */
#define CAN_IDR_ERRP (0x1u << 18) /**< \brief (CAN_IDR) Error Passive Mode Interrupt Disable */
#define CAN_IDR_BOFF (0x1u << 19) /**< \brief (CAN_IDR) Bus Off Mode Interrupt Disable */
#define CAN_IDR_SLEEP (0x1u << 20) /**< \brief (CAN_IDR) Sleep Interrupt Disable */
#define CAN_IDR_WAKEUP (0x1u << 21) /**< \brief (CAN_IDR) Wakeup Interrupt Disable */
#define CAN_IDR_TOVF (0x1u << 22) /**< \brief (CAN_IDR) Timer Overflow Interrupt */
#define CAN_IDR_TSTP (0x1u << 23) /**< \brief (CAN_IDR) TimeStamp Interrupt Disable */
#define CAN_IDR_CERR (0x1u << 24) /**< \brief (CAN_IDR) CRC Error Interrupt Disable */
#define CAN_IDR_SERR (0x1u << 25) /**< \brief (CAN_IDR) Stuffing Error Interrupt Disable */
#define CAN_IDR_AERR (0x1u << 26) /**< \brief (CAN_IDR) Acknowledgment Error Interrupt Disable */
#define CAN_IDR_FERR (0x1u << 27) /**< \brief (CAN_IDR) Form Error Interrupt Disable */
#define CAN_IDR_BERR (0x1u << 28) /**< \brief (CAN_IDR) Bit Error Interrupt Disable */
/* -------- CAN_IMR : (CAN Offset: 0x000C) Interrupt Mask Register -------- */
#define CAN_IMR_MB0 (0x1u << 0) /**< \brief (CAN_IMR) Mailbox 0 Interrupt Mask */
#define CAN_IMR_MB1 (0x1u << 1) /**< \brief (CAN_IMR) Mailbox 1 Interrupt Mask */
#define CAN_IMR_MB2 (0x1u << 2) /**< \brief (CAN_IMR) Mailbox 2 Interrupt Mask */
#define CAN_IMR_MB3 (0x1u << 3) /**< \brief (CAN_IMR) Mailbox 3 Interrupt Mask */
#define CAN_IMR_MB4 (0x1u << 4) /**< \brief (CAN_IMR) Mailbox 4 Interrupt Mask */
#define CAN_IMR_MB5 (0x1u << 5) /**< \brief (CAN_IMR) Mailbox 5 Interrupt Mask */
#define CAN_IMR_MB6 (0x1u << 6) /**< \brief (CAN_IMR) Mailbox 6 Interrupt Mask */
#define CAN_IMR_MB7 (0x1u << 7) /**< \brief (CAN_IMR) Mailbox 7 Interrupt Mask */
#define CAN_IMR_ERRA (0x1u << 16) /**< \brief (CAN_IMR) Error Active Mode Interrupt Mask */
#define CAN_IMR_WARN (0x1u << 17) /**< \brief (CAN_IMR) Warning Limit Interrupt Mask */
#define CAN_IMR_ERRP (0x1u << 18) /**< \brief (CAN_IMR) Error Passive Mode Interrupt Mask */
#define CAN_IMR_BOFF (0x1u << 19) /**< \brief (CAN_IMR) Bus Off Mode Interrupt Mask */
#define CAN_IMR_SLEEP (0x1u << 20) /**< \brief (CAN_IMR) Sleep Interrupt Mask */
#define CAN_IMR_WAKEUP (0x1u << 21) /**< \brief (CAN_IMR) Wakeup Interrupt Mask */
#define CAN_IMR_TOVF (0x1u << 22) /**< \brief (CAN_IMR) Timer Overflow Interrupt Mask */
#define CAN_IMR_TSTP (0x1u << 23) /**< \brief (CAN_IMR) Timestamp Interrupt Mask */
#define CAN_IMR_CERR (0x1u << 24) /**< \brief (CAN_IMR) CRC Error Interrupt Mask */
#define CAN_IMR_SERR (0x1u << 25) /**< \brief (CAN_IMR) Stuffing Error Interrupt Mask */
#define CAN_IMR_AERR (0x1u << 26) /**< \brief (CAN_IMR) Acknowledgment Error Interrupt Mask */
#define CAN_IMR_FERR (0x1u << 27) /**< \brief (CAN_IMR) Form Error Interrupt Mask */
#define CAN_IMR_BERR (0x1u << 28) /**< \brief (CAN_IMR) Bit Error Interrupt Mask */
/* -------- CAN_SR : (CAN Offset: 0x0010) Status Register -------- */
#define CAN_SR_MB0 (0x1u << 0) /**< \brief (CAN_SR) Mailbox 0 Event */
#define CAN_SR_MB1 (0x1u << 1) /**< \brief (CAN_SR) Mailbox 1 Event */
#define CAN_SR_MB2 (0x1u << 2) /**< \brief (CAN_SR) Mailbox 2 Event */
#define CAN_SR_MB3 (0x1u << 3) /**< \brief (CAN_SR) Mailbox 3 Event */
#define CAN_SR_MB4 (0x1u << 4) /**< \brief (CAN_SR) Mailbox 4 Event */
#define CAN_SR_MB5 (0x1u << 5) /**< \brief (CAN_SR) Mailbox 5 Event */
#define CAN_SR_MB6 (0x1u << 6) /**< \brief (CAN_SR) Mailbox 6 Event */
#define CAN_SR_MB7 (0x1u << 7) /**< \brief (CAN_SR) Mailbox 7 Event */
#define CAN_SR_ERRA (0x1u << 16) /**< \brief (CAN_SR) Error Active Mode */
#define CAN_SR_WARN (0x1u << 17) /**< \brief (CAN_SR) Warning Limit */
#define CAN_SR_ERRP (0x1u << 18) /**< \brief (CAN_SR) Error Passive Mode */
#define CAN_SR_BOFF (0x1u << 19) /**< \brief (CAN_SR) Bus Off Mode */
#define CAN_SR_SLEEP (0x1u << 20) /**< \brief (CAN_SR) CAN controller in Low power Mode */
#define CAN_SR_WAKEUP (0x1u << 21) /**< \brief (CAN_SR) CAN controller is not in Low power Mode */
#define CAN_SR_TOVF (0x1u << 22) /**< \brief (CAN_SR) Timer Overflow */
#define CAN_SR_TSTP (0x1u << 23) /**< \brief (CAN_SR)  */
#define CAN_SR_CERR (0x1u << 24) /**< \brief (CAN_SR) Mailbox CRC Error */
#define CAN_SR_SERR (0x1u << 25) /**< \brief (CAN_SR) Mailbox Stuffing Error */
#define CAN_SR_AERR (0x1u << 26) /**< \brief (CAN_SR) Acknowledgment Error */
#define CAN_SR_FERR (0x1u << 27) /**< \brief (CAN_SR) Form Error */
#define CAN_SR_BERR (0x1u << 28) /**< \brief (CAN_SR) Bit Error */
#define CAN_SR_RBSY (0x1u << 29) /**< \brief (CAN_SR) Receiver busy */
#define CAN_SR_TBSY (0x1u << 30) /**< \brief (CAN_SR) Transmitter busy */
#define CAN_SR_OVLSY (0x1u << 31) /**< \brief (CAN_SR) Overload busy */
/* -------- CAN_BR : (CAN Offset: 0x0014) Baudrate Register -------- */
#define CAN_BR_PHASE2_Pos 0
#define CAN_BR_PHASE2_Msk (0x7u << CAN_BR_PHASE2_Pos) /**< \brief (CAN_BR) Phase 2 segment */
#define CAN_BR_PHASE2(value) ((CAN_BR_PHASE2_Msk & ((value) << CAN_BR_PHASE2_Pos)))
#define CAN_BR_PHASE1_Pos 4
#define CAN_BR_PHASE1_Msk (0x7u << CAN_BR_PHASE1_Pos) /**< \brief (CAN_BR) Phase 1 segment */
#define CAN_BR_PHASE1(value) ((CAN_BR_PHASE1_Msk & ((value) << CAN_BR_PHASE1_Pos)))
#define CAN_BR_PROPAG_Pos 8
#define CAN_BR_PROPAG_Msk (0x7u << CAN_BR_PROPAG_Pos) /**< \brief (CAN_BR) Programming time segment */
#define CAN_BR_PROPAG(value) ((CAN_BR_PROPAG_Msk & ((value) << CAN_BR_PROPAG_Pos)))
#define CAN_BR_SJW_Pos 12
#define CAN_BR_SJW_Msk (0x3u << CAN_BR_SJW_Pos) /**< \brief (CAN_BR) Re-synchronization jump width */
#define CAN_BR_SJW(value) ((CAN_BR_SJW_Msk & ((value) << CAN_BR_SJW_Pos)))
#define CAN_BR_BRP_Pos 16
#define CAN_BR_BRP_Msk (0x7fu << CAN_BR_BRP_Pos) /**< \brief (CAN_BR) Baudrate Prescaler. */
#define CAN_BR_BRP(value) ((CAN_BR_BRP_Msk & ((value) << CAN_BR_BRP_Pos)))
#define CAN_BR_SMP (0x1u << 24) /**< \brief (CAN_BR) Sampling Mode */
#define   CAN_BR_SMP_ONCE (0x0u << 24) /**< \brief (CAN_BR) The incoming bit stream is sampled once at sample point. */
#define   CAN_BR_SMP_THREE (0x1u << 24) /**< \brief (CAN_BR) The incoming bit stream is sampled three times with a period of a MCK clock period, centered on sample point. */
/* -------- CAN_TIM : (CAN Offset: 0x0018) Timer Register -------- */
#define CAN_TIM_TIMER_Pos 0
#define CAN_TIM_TIMER_Msk (0xffffu << CAN_TIM_TIMER_Pos) /**< \brief (CAN_TIM) Timer */
/* -------- CAN_TIMESTP : (CAN Offset: 0x001C) Timestamp Register -------- */
#define CAN_TIMESTP_MTIMESTAMP_Pos 0
#define CAN_TIMESTP_MTIMESTAMP_Msk (0xffffu << CAN_TIMESTP_MTIMESTAMP_Pos) /**< \brief (CAN_TIMESTP) Timestamp */
/* -------- CAN_ECR : (CAN Offset: 0x0020) Error Counter Register -------- */
#define CAN_ECR_REC_Pos 0
#define CAN_ECR_REC_Msk (0xffu << CAN_ECR_REC_Pos) /**< \brief (CAN_ECR) Receive Error Counter */
#define CAN_ECR_TEC_Pos 16
#define CAN_ECR_TEC_Msk (0xffu << CAN_ECR_TEC_Pos) /**< \brief (CAN_ECR) Transmit Error Counter */
/* -------- CAN_TCR : (CAN Offset: 0x0024) Transfer Command Register -------- */
#define CAN_TCR_MB0 (0x1u << 0) /**< \brief (CAN_TCR) Transfer Request for Mailbox 0 */
#define CAN_TCR_MB1 (0x1u << 1) /**< \brief (CAN_TCR) Transfer Request for Mailbox 1 */
#define CAN_TCR_MB2 (0x1u << 2) /**< \brief (CAN_TCR) Transfer Request for Mailbox 2 */
#define CAN_TCR_MB3 (0x1u << 3) /**< \brief (CAN_TCR) Transfer Request for Mailbox 3 */
#define CAN_TCR_MB4 (0x1u << 4) /**< \brief (CAN_TCR) Transfer Request for Mailbox 4 */
#define CAN_TCR_MB5 (0x1u << 5) /**< \brief (CAN_TCR) Transfer Request for Mailbox 5 */
#define CAN_TCR_MB6 (0x1u << 6) /**< \brief (CAN_TCR) Transfer Request for Mailbox 6 */
#define CAN_TCR_MB7 (0x1u << 7) /**< \brief (CAN_TCR) Transfer Request for Mailbox 7 */
#define CAN_TCR_TIMRST (0x1u << 31) /**< \brief (CAN_TCR) Timer Reset */
/* -------- CAN_ACR : (CAN Offset: 0x0028) Abort Command Register -------- */
#define CAN_ACR_MB0 (0x1u << 0) /**< \brief (CAN_ACR) Abort Request for Mailbox 0 */
#define CAN_ACR_MB1 (0x1u << 1) /**< \brief (CAN_ACR) Abort Request for Mailbox 1 */
#define CAN_ACR_MB2 (0x1u << 2) /**< \brief (CAN_ACR) Abort Request for Mailbox 2 */
#define CAN_ACR_MB3 (0x1u << 3) /**< \brief (CAN_ACR) Abort Request for Mailbox 3 */
#define CAN_ACR_MB4 (0x1u << 4) /**< \brief (CAN_ACR) Abort Request for Mailbox 4 */
#define CAN_ACR_MB5 (0x1u << 5) /**< \brief (CAN_ACR) Abort Request for Mailbox 5 */
#define CAN_ACR_MB6 (0x1u << 6) /**< \brief (CAN_ACR) Abort Request for Mailbox 6 */
#define CAN_ACR_MB7 (0x1u << 7) /**< \brief (CAN_ACR) Abort Request for Mailbox 7 */
/* -------- CAN_WPMR : (CAN Offset: 0x00E4) Write Protect Mode Register -------- */
#define CAN_WPMR_WPEN (0x1u << 0) /**< \brief (CAN_WPMR) Write Protection Enable */
#define CAN_WPMR_WPKEY_Pos 8
#define CAN_WPMR_WPKEY_Msk (0xffffffu << CAN_WPMR_WPKEY_Pos) /**< \brief (CAN_WPMR) SPI Write Protection Key Password */
#define CAN_WPMR_WPKEY(value) ((CAN_WPMR_WPKEY_Msk & ((value) << CAN_WPMR_WPKEY_Pos)))
/* -------- CAN_WPSR : (CAN Offset: 0x00E8) Write Protect Status Register -------- */
#define CAN_WPSR_WPVS (0x1u << 0) /**< \brief (CAN_WPSR) Write Protection Violation Status */
#define CAN_WPSR_WPVSRC_Pos 8
#define CAN_WPSR_WPVSRC_Msk (0xffu << CAN_WPSR_WPVSRC_Pos) /**< \brief (CAN_WPSR) Write Protection Violation Source */
/* -------- CAN_MMR : (CAN Offset: N/A) Mailbox Mode Register -------- */
#define CAN_MMR_MTIMEMARK_Pos 0
#define CAN_MMR_MTIMEMARK_Msk (0xffffu << CAN_MMR_MTIMEMARK_Pos) /**< \brief (CAN_MMR) Mailbox Timemark */
#define CAN_MMR_MTIMEMARK(value) ((CAN_MMR_MTIMEMARK_Msk & ((value) << CAN_MMR_MTIMEMARK_Pos)))
#define CAN_MMR_PRIOR_Pos 16
#define CAN_MMR_PRIOR_Msk (0xfu << CAN_MMR_PRIOR_Pos) /**< \brief (CAN_MMR) Mailbox Priority */
#define CAN_MMR_PRIOR(value) ((CAN_MMR_PRIOR_Msk & ((value) << CAN_MMR_PRIOR_Pos)))
#define CAN_MMR_MOT_Pos 24
#define CAN_MMR_MOT_Msk (0x7u << CAN_MMR_MOT_Pos) /**< \brief (CAN_MMR) Mailbox Object Type */
#define   CAN_MMR_MOT_MB_DISABLED (0x0u << 24) /**< \brief (CAN_MMR) Mailbox is disabled. This prevents receiving or transmitting any messages with this mailbox. */
#define   CAN_MMR_MOT_MB_RX (0x1u << 24) /**< \brief (CAN_MMR) Reception Mailbox. Mailbox is configured for reception. If a message is received while the mailbox data register is full, it is discarded. */
#define   CAN_MMR_MOT_MB_RX_OVERWRITE (0x2u << 24) /**< \brief (CAN_MMR) Reception mailbox with overwrite. Mailbox is configured for reception. If a message is received while the mailbox is full, it overwrites the previous message. */
#define   CAN_MMR_MOT_MB_TX (0x3u << 24) /**< \brief (CAN_MMR) Transmit mailbox. Mailbox is configured for transmission. */
#define   CAN_MMR_MOT_MB_CONSUMER (0x4u << 24) /**< \brief (CAN_MMR) Consumer Mailbox. Mailbox is configured in reception but behaves as a Transmit Mailbox, i.e., it sends a remote frame and waits for an answer. */
#define   CAN_MMR_MOT_MB_PRODUCER (0x5u << 24) /**< \brief (CAN_MMR) Producer Mailbox. Mailbox is configured in transmission but also behaves like a reception mailbox, i.e., it waits to receive a Remote Frame before sending its contents. */
/* -------- CAN_MAM : (CAN Offset: N/A) Mailbox Acceptance Mask Register -------- */
#define CAN_MAM_MIDvB_Pos 0
#define CAN_MAM_MIDvB_Msk (0x3ffffu << CAN_MAM_MIDvB_Pos) /**< \brief (CAN_MAM) Complementary bits for identifier in extended frame mode */
#define CAN_MAM_MIDvB(value) ((CAN_MAM_MIDvB_Msk & ((value) << CAN_MAM_MIDvB_Pos)))
#define CAN_MAM_MIDvA_Pos 18
#define CAN_MAM_MIDvA_Msk (0x7ffu << CAN_MAM_MIDvA_Pos) /**< \brief (CAN_MAM) Identifier for standard frame mode */
#define CAN_MAM_MIDvA(value) ((CAN_MAM_MIDvA_Msk & ((value) << CAN_MAM_MIDvA_Pos)))
#define CAN_MAM_MIDE (0x1u << 29) /**< \brief (CAN_MAM) Identifier Version */
/* -------- CAN_MID : (CAN Offset: N/A) Mailbox ID Register -------- */
#define CAN_MID_MIDvB_Pos 0
#define CAN_MID_MIDvB_Msk (0x3ffffu << CAN_MID_MIDvB_Pos) /**< \brief (CAN_MID) Complementary bits for identifier in extended frame mode */
#define CAN_MID_MIDvB(value) ((CAN_MID_MIDvB_Msk & ((value) << CAN_MID_MIDvB_Pos)))
#define CAN_MID_MIDvA_Pos 18
#define CAN_MID_MIDvA_Msk (0x7ffu << CAN_MID_MIDvA_Pos) /**< \brief (CAN_MID) Identifier for standard frame mode */
#define CAN_MID_MIDvA(value) ((CAN_MID_MIDvA_Msk & ((value) << CAN_MID_MIDvA_Pos)))
#define CAN_MID_MIDE (0x1u << 29) /**< \brief (CAN_MID) Identifier Version */
/* -------- CAN_MFID : (CAN Offset: N/A) Mailbox Family ID Register -------- */
#define CAN_MFID_MFID_Pos 0
#define CAN_MFID_MFID_Msk (0x1fffffffu << CAN_MFID_MFID_Pos) /**< \brief (CAN_MFID) Family ID */
/* -------- CAN_MSR : (CAN Offset: N/A) Mailbox Status Register -------- */
#define CAN_MSR_MTIMESTAMP_Pos 0
#define CAN_MSR_MTIMESTAMP_Msk (0xffffu << CAN_MSR_MTIMESTAMP_Pos) /**< \brief (CAN_MSR) Timer value */
#define CAN_MSR_MDLC_Pos 16
#define CAN_MSR_MDLC_Msk (0xfu << CAN_MSR_MDLC_Pos) /**< \brief (CAN_MSR) Mailbox Data Length Code */
#define CAN_MSR_MRTR (0x1u << 20) /**< \brief (CAN_MSR) Mailbox Remote Transmission Request */
#define CAN_MSR_MABT (0x1u << 22) /**< \brief (CAN_MSR) Mailbox Message Abort */
#define CAN_MSR_MRDY (0x1u << 23) /**< \brief (CAN_MSR) Mailbox Ready */
#define CAN_MSR_MMI (0x1u << 24) /**< \brief (CAN_MSR) Mailbox Message Ignored */
/* -------- CAN_MDL : (CAN Offset: N/A) Mailbox Data Low Register -------- */
#define CAN_MDL_MDL_Pos 0
#define CAN_MDL_MDL_Msk (0xffffffffu << CAN_MDL_MDL_Pos) /**< \brief (CAN_MDL) Message Data Low Value */
#define CAN_MDL_MDL(value) ((CAN_MDL_MDL_Msk & ((value) << CAN_MDL_MDL_Pos)))
/* -------- CAN_MDH : (CAN Offset: N/A) Mailbox Data High Register -------- */
#define CAN_MDH_MDH_Pos 0
#define CAN_MDH_MDH_Msk (0xffffffffu << CAN_MDH_MDH_Pos) /**< \brief (CAN_MDH) Message Data High Value */
#define CAN_MDH_MDH(value) ((CAN_MDH_MDH_Msk & ((value) << CAN_MDH_MDH_Pos)))
/* -------- CAN_MCR : (CAN Offset: N/A) Mailbox Control Register -------- */
#define CAN_MCR_MDLC_Pos 16
#define CAN_MCR_MDLC_Msk (0xfu << CAN_MCR_MDLC_Pos) /**< \brief (CAN_MCR) Mailbox Data Length Code */
#define CAN_MCR_MDLC(value) ((CAN_MCR_MDLC_Msk & ((value) << CAN_MCR_MDLC_Pos)))
#define CAN_MCR_MRTR (0x1u << 20) /**< \brief (CAN_MCR) Mailbox Remote Transmission Request */
#define CAN_MCR_MACR (0x1u << 22) /**< \brief (CAN_MCR) Abort Request for Mailbox x */
#define CAN_MCR_MTCR (0x1u << 23) /**< \brief (CAN_MCR) Mailbox Transfer Command */
/*@}*/ /* end of group SAM3XA_CAN */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Chip Identifier */
/* ============================================================================= */
/** \addtogroup SAM3XA_CHIPID Chip Identifier */
/*@{*/

/** \brief Chipid hardware registers */
typedef struct {
  RoReg CHIPID_CIDR; /**< \brief (Chipid Offset: 0x0) Chip ID Register */
  RoReg CHIPID_EXID; /**< \brief (Chipid Offset: 0x4) Chip ID Extension Register */
} Chipid;
/* -------- CHIPID_CIDR : (CHIPID Offset: 0x0) Chip ID Register -------- */
#define CHIPID_CIDR_VERSION_Pos 0
#define CHIPID_CIDR_VERSION_Msk (0x1fu << CHIPID_CIDR_VERSION_Pos) /**< \brief (CHIPID_CIDR) Version of the Device */
#define CHIPID_CIDR_EPROC_Pos 5
#define CHIPID_CIDR_EPROC_Msk (0x7u << CHIPID_CIDR_EPROC_Pos) /**< \brief (CHIPID_CIDR) Embedded Processor */
#define   CHIPID_CIDR_EPROC_ARM946ES (0x1u << 5) /**< \brief (CHIPID_CIDR) ARM946ES */
#define   CHIPID_CIDR_EPROC_ARM7TDMI (0x2u << 5) /**< \brief (CHIPID_CIDR) ARM7TDMI */
#define   CHIPID_CIDR_EPROC_CM3 (0x3u << 5) /**< \brief (CHIPID_CIDR) Cortex-M3 */
#define   CHIPID_CIDR_EPROC_ARM920T (0x4u << 5) /**< \brief (CHIPID_CIDR) ARM920T */
#define   CHIPID_CIDR_EPROC_ARM926EJS (0x5u << 5) /**< \brief (CHIPID_CIDR) ARM926EJS */
#define   CHIPID_CIDR_EPROC_CA5 (0x6u << 5) /**< \brief (CHIPID_CIDR) Cortex-A5 */
#define   CHIPID_CIDR_EPROC_CM4 (0x7u << 5) /**< \brief (CHIPID_CIDR) Cortex-M4 */
#define CHIPID_CIDR_NVPSIZ_Pos 8
#define CHIPID_CIDR_NVPSIZ_Msk (0xfu << CHIPID_CIDR_NVPSIZ_Pos) /**< \brief (CHIPID_CIDR) Nonvolatile Program Memory Size */
#define   CHIPID_CIDR_NVPSIZ_NONE (0x0u << 8) /**< \brief (CHIPID_CIDR) None */
#define   CHIPID_CIDR_NVPSIZ_8K (0x1u << 8) /**< \brief (CHIPID_CIDR) 8K bytes */
#define   CHIPID_CIDR_NVPSIZ_16K (0x2u << 8) /**< \brief (CHIPID_CIDR) 16K bytes */
#define   CHIPID_CIDR_NVPSIZ_32K (0x3u << 8) /**< \brief (CHIPID_CIDR) 32K bytes */
#define   CHIPID_CIDR_NVPSIZ_64K (0x5u << 8) /**< \brief (CHIPID_CIDR) 64K bytes */
#define   CHIPID_CIDR_NVPSIZ_128K (0x7u << 8) /**< \brief (CHIPID_CIDR) 128K bytes */
#define   CHIPID_CIDR_NVPSIZ_256K (0x9u << 8) /**< \brief (CHIPID_CIDR) 256K bytes */
#define   CHIPID_CIDR_NVPSIZ_512K (0xAu << 8) /**< \brief (CHIPID_CIDR) 512K bytes */
#define   CHIPID_CIDR_NVPSIZ_1024K (0xCu << 8) /**< \brief (CHIPID_CIDR) 1024K bytes */
#define   CHIPID_CIDR_NVPSIZ_2048K (0xEu << 8) /**< \brief (CHIPID_CIDR) 2048K bytes */
#define CHIPID_CIDR_NVPSIZ2_Pos 12
#define CHIPID_CIDR_NVPSIZ2_Msk (0xfu << CHIPID_CIDR_NVPSIZ2_Pos) /**< \brief (CHIPID_CIDR) Second Nonvolatile Program Memory Size */
#define   CHIPID_CIDR_NVPSIZ2_NONE (0x0u << 12) /**< \brief (CHIPID_CIDR) None */
#define   CHIPID_CIDR_NVPSIZ2_8K (0x1u << 12) /**< \brief (CHIPID_CIDR) 8K bytes */
#define   CHIPID_CIDR_NVPSIZ2_16K (0x2u << 12) /**< \brief (CHIPID_CIDR) 16K bytes */
#define   CHIPID_CIDR_NVPSIZ2_32K (0x3u << 12) /**< \brief (CHIPID_CIDR) 32K bytes */
#define   CHIPID_CIDR_NVPSIZ2_64K (0x5u << 12) /**< \brief (CHIPID_CIDR) 64K bytes */
#define   CHIPID_CIDR_NVPSIZ2_128K (0x7u << 12) /**< \brief (CHIPID_CIDR) 128K bytes */
#define   CHIPID_CIDR_NVPSIZ2_256K (0x9u << 12) /**< \brief (CHIPID_CIDR) 256K bytes */
#define   CHIPID_CIDR_NVPSIZ2_512K (0xAu << 12) /**< \brief (CHIPID_CIDR) 512K bytes */
#define   CHIPID_CIDR_NVPSIZ2_1024K (0xCu << 12) /**< \brief (CHIPID_CIDR) 1024K bytes */
#define   CHIPID_CIDR_NVPSIZ2_2048K (0xEu << 12) /**< \brief (CHIPID_CIDR) 2048K bytes */
#define CHIPID_CIDR_SRAMSIZ_Pos 16
#define CHIPID_CIDR_SRAMSIZ_Msk (0xfu << CHIPID_CIDR_SRAMSIZ_Pos) /**< \brief (CHIPID_CIDR) Internal SRAM Size */
#define   CHIPID_CIDR_SRAMSIZ_48K (0x0u << 16) /**< \brief (CHIPID_CIDR) 48K bytes */
#define   CHIPID_CIDR_SRAMSIZ_1K (0x1u << 16) /**< \brief (CHIPID_CIDR) 1K bytes */
#define   CHIPID_CIDR_SRAMSIZ_2K (0x2u << 16) /**< \brief (CHIPID_CIDR) 2K bytes */
#define   CHIPID_CIDR_SRAMSIZ_6K (0x3u << 16) /**< \brief (CHIPID_CIDR) 6K bytes */
#define   CHIPID_CIDR_SRAMSIZ_24K (0x4u << 16) /**< \brief (CHIPID_CIDR) 24K bytes */
#define   CHIPID_CIDR_SRAMSIZ_4K (0x5u << 16) /**< \brief (CHIPID_CIDR) 4K bytes */
#define   CHIPID_CIDR_SRAMSIZ_80K (0x6u << 16) /**< \brief (CHIPID_CIDR) 80K bytes */
#define   CHIPID_CIDR_SRAMSIZ_160K (0x7u << 16) /**< \brief (CHIPID_CIDR) 160K bytes */
#define   CHIPID_CIDR_SRAMSIZ_8K (0x8u << 16) /**< \brief (CHIPID_CIDR) 8K bytes */
#define   CHIPID_CIDR_SRAMSIZ_16K (0x9u << 16) /**< \brief (CHIPID_CIDR) 16K bytes */
#define   CHIPID_CIDR_SRAMSIZ_32K (0xAu << 16) /**< \brief (CHIPID_CIDR) 32K bytes */
#define   CHIPID_CIDR_SRAMSIZ_64K (0xBu << 16) /**< \brief (CHIPID_CIDR) 64K bytes */
#define   CHIPID_CIDR_SRAMSIZ_128K (0xCu << 16) /**< \brief (CHIPID_CIDR) 128K bytes */
#define   CHIPID_CIDR_SRAMSIZ_256K (0xDu << 16) /**< \brief (CHIPID_CIDR) 256K bytes */
#define   CHIPID_CIDR_SRAMSIZ_96K (0xEu << 16) /**< \brief (CHIPID_CIDR) 96K bytes */
#define   CHIPID_CIDR_SRAMSIZ_512K (0xFu << 16) /**< \brief (CHIPID_CIDR) 512K bytes */
#define CHIPID_CIDR_ARCH_Pos 20
#define CHIPID_CIDR_ARCH_Msk (0xffu << CHIPID_CIDR_ARCH_Pos) /**< \brief (CHIPID_CIDR) Architecture Identifier */
#define   CHIPID_CIDR_ARCH_AT91SAM9xx (0x19u << 20) /**< \brief (CHIPID_CIDR) AT91SAM9xx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM9XExx (0x29u << 20) /**< \brief (CHIPID_CIDR) AT91SAM9XExx Series */
#define   CHIPID_CIDR_ARCH_AT91x34 (0x34u << 20) /**< \brief (CHIPID_CIDR) AT91x34 Series */
#define   CHIPID_CIDR_ARCH_CAP7 (0x37u << 20) /**< \brief (CHIPID_CIDR) CAP7 Series */
#define   CHIPID_CIDR_ARCH_CAP9 (0x39u << 20) /**< \brief (CHIPID_CIDR) CAP9 Series */
#define   CHIPID_CIDR_ARCH_CAP11 (0x3Bu << 20) /**< \brief (CHIPID_CIDR) CAP11 Series */
#define   CHIPID_CIDR_ARCH_AT91x40 (0x40u << 20) /**< \brief (CHIPID_CIDR) AT91x40 Series */
#define   CHIPID_CIDR_ARCH_AT91x42 (0x42u << 20) /**< \brief (CHIPID_CIDR) AT91x42 Series */
#define   CHIPID_CIDR_ARCH_AT91x55 (0x55u << 20) /**< \brief (CHIPID_CIDR) AT91x55 Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7Axx (0x60u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7Axx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7AQxx (0x61u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7AQxx Series */
#define   CHIPID_CIDR_ARCH_AT91x63 (0x63u << 20) /**< \brief (CHIPID_CIDR) AT91x63 Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7Sxx (0x70u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7Sxx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7XCxx (0x71u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7XCxx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7SExx (0x72u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7SExx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7Lxx (0x73u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7Lxx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7Xxx (0x75u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7Xxx Series */
#define   CHIPID_CIDR_ARCH_AT91SAM7SLxx (0x76u << 20) /**< \brief (CHIPID_CIDR) AT91SAM7SLxx Series */
#define   CHIPID_CIDR_ARCH_SAM3UxC (0x80u << 20) /**< \brief (CHIPID_CIDR) SAM3UxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3UxE (0x81u << 20) /**< \brief (CHIPID_CIDR) SAM3UxE Series (144-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3AxC (0x83u << 20) /**< \brief (CHIPID_CIDR) SAM3AxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4AxC (0x83u << 20) /**< \brief (CHIPID_CIDR) SAM4AxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3XxC (0x84u << 20) /**< \brief (CHIPID_CIDR) SAM3XxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4XxC (0x84u << 20) /**< \brief (CHIPID_CIDR) SAM4XxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3XxE (0x85u << 20) /**< \brief (CHIPID_CIDR) SAM3XxE Series (144-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4XxE (0x85u << 20) /**< \brief (CHIPID_CIDR) SAM4XxE Series (144-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3XxG (0x86u << 20) /**< \brief (CHIPID_CIDR) SAM3XxG Series (208/217-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4XxG (0x86u << 20) /**< \brief (CHIPID_CIDR) SAM4XxG Series (208/217-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3SxA (0x88u << 20) /**< \brief (CHIPID_CIDR) SAM3SxASeries (48-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4SxA (0x88u << 20) /**< \brief (CHIPID_CIDR) SAM4SxA Series (48-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3SxB (0x89u << 20) /**< \brief (CHIPID_CIDR) SAM3SxB Series (64-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4SxB (0x89u << 20) /**< \brief (CHIPID_CIDR) SAM4SxB Series (64-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3SxC (0x8Au << 20) /**< \brief (CHIPID_CIDR) SAM3SxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM4SxC (0x8Au << 20) /**< \brief (CHIPID_CIDR) SAM4SxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_AT91x92 (0x92u << 20) /**< \brief (CHIPID_CIDR) AT91x92 Series */
#define   CHIPID_CIDR_ARCH_SAM3NxA (0x93u << 20) /**< \brief (CHIPID_CIDR) SAM3NxA Series (48-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3NxB (0x94u << 20) /**< \brief (CHIPID_CIDR) SAM3NxB Series (64-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3NxC (0x95u << 20) /**< \brief (CHIPID_CIDR) SAM3NxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3SDxB (0x99u << 20) /**< \brief (CHIPID_CIDR) SAM3SDxB Series (64-pin version) */
#define   CHIPID_CIDR_ARCH_SAM3SDxC (0x9Au << 20) /**< \brief (CHIPID_CIDR) SAM3SDxC Series (100-pin version) */
#define   CHIPID_CIDR_ARCH_SAM5A (0xA5u << 20) /**< \brief (CHIPID_CIDR) SAM5A */
#define   CHIPID_CIDR_ARCH_AT75Cxx (0xF0u << 20) /**< \brief (CHIPID_CIDR) AT75Cxx Series */
#define CHIPID_CIDR_NVPTYP_Pos 28
#define CHIPID_CIDR_NVPTYP_Msk (0x7u << CHIPID_CIDR_NVPTYP_Pos) /**< \brief (CHIPID_CIDR) Nonvolatile Program Memory Type */
#define   CHIPID_CIDR_NVPTYP_ROM (0x0u << 28) /**< \brief (CHIPID_CIDR) ROM */
#define   CHIPID_CIDR_NVPTYP_ROMLESS (0x1u << 28) /**< \brief (CHIPID_CIDR) ROMless or on-chip Flash */
#define   CHIPID_CIDR_NVPTYP_FLASH (0x2u << 28) /**< \brief (CHIPID_CIDR) Embedded Flash Memory */
#define   CHIPID_CIDR_NVPTYP_ROM_FLASH (0x3u << 28) /**< \brief (CHIPID_CIDR) ROM and Embedded Flash MemoryNVPSIZ is ROM size      NVPSIZ2 is Flash size */
#define   CHIPID_CIDR_NVPTYP_SRAM (0x4u << 28) /**< \brief (CHIPID_CIDR) SRAM emulating ROM */
#define CHIPID_CIDR_EXT (0x1u << 31) /**< \brief (CHIPID_CIDR) Extension Flag */
/* -------- CHIPID_EXID : (CHIPID Offset: 0x4) Chip ID Extension Register -------- */
#define CHIPID_EXID_EXID_Pos 0
#define CHIPID_EXID_EXID_Msk (0xffffffffu << CHIPID_EXID_EXID_Pos) /**< \brief (CHIPID_EXID) Chip ID Extension */
/*@}*/ /* end of group SAM3XA_CHIPID */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Digital-to-Analog Converter Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_DACC Digital-to-Analog Converter Controller */
/*@{*/

/** \brief Dacc hardware registers */
typedef struct {
  WoReg DACC_CR;       /**< \brief (Dacc Offset: 0x00) Control Register */
  RwReg DACC_MR;       /**< \brief (Dacc Offset: 0x04) Mode Register */
  RoReg Reserved1[2];
  WoReg DACC_CHER;     /**< \brief (Dacc Offset: 0x10) Channel Enable Register */
  WoReg DACC_CHDR;     /**< \brief (Dacc Offset: 0x14) Channel Disable Register */
  RoReg DACC_CHSR;     /**< \brief (Dacc Offset: 0x18) Channel Status Register */
  RoReg Reserved2[1];
  WoReg DACC_CDR;      /**< \brief (Dacc Offset: 0x20) Conversion Data Register */
  WoReg DACC_IER;      /**< \brief (Dacc Offset: 0x24) Interrupt Enable Register */
  WoReg DACC_IDR;      /**< \brief (Dacc Offset: 0x28) Interrupt Disable Register */
  RoReg DACC_IMR;      /**< \brief (Dacc Offset: 0x2C) Interrupt Mask Register */
  RoReg DACC_ISR;      /**< \brief (Dacc Offset: 0x30) Interrupt Status Register */
  RoReg Reserved3[24];
  RwReg DACC_ACR;      /**< \brief (Dacc Offset: 0x94) Analog Current Register */
  RoReg Reserved4[19];
  RwReg DACC_WPMR;     /**< \brief (Dacc Offset: 0xE4) Write Protect Mode register */
  RoReg DACC_WPSR;     /**< \brief (Dacc Offset: 0xE8) Write Protect Status register */
  RoReg Reserved5[7];
  RwReg DACC_TPR;      /**< \brief (Dacc Offset: 0x108) Transmit Pointer Register */
  RwReg DACC_TCR;      /**< \brief (Dacc Offset: 0x10C) Transmit Counter Register */
  RoReg Reserved6[2];
  RwReg DACC_TNPR;     /**< \brief (Dacc Offset: 0x118) Transmit Next Pointer Register */
  RwReg DACC_TNCR;     /**< \brief (Dacc Offset: 0x11C) Transmit Next Counter Register */
  WoReg DACC_PTCR;     /**< \brief (Dacc Offset: 0x120) Transfer Control Register */
  RoReg DACC_PTSR;     /**< \brief (Dacc Offset: 0x124) Transfer Status Register */
} Dacc;
/* -------- DACC_CR : (DACC Offset: 0x00) Control Register -------- */
#define DACC_CR_SWRST (0x1u << 0) /**< \brief (DACC_CR) Software Reset */
/* -------- DACC_MR : (DACC Offset: 0x04) Mode Register -------- */
#define DACC_MR_TRGEN (0x1u << 0) /**< \brief (DACC_MR) Trigger Enable */
#define   DACC_MR_TRGEN_DIS (0x0u << 0) /**< \brief (DACC_MR) External trigger mode disabled. DACC in free running mode. */
#define   DACC_MR_TRGEN_EN (0x1u << 0) /**< \brief (DACC_MR) External trigger mode enabled. */
#define DACC_MR_TRGSEL_Pos 1
#define DACC_MR_TRGSEL_Msk (0x7u << DACC_MR_TRGSEL_Pos) /**< \brief (DACC_MR) Trigger Selection */
#define DACC_MR_TRGSEL(value) ((DACC_MR_TRGSEL_Msk & ((value) << DACC_MR_TRGSEL_Pos)))
#define DACC_MR_WORD (0x1u << 4) /**< \brief (DACC_MR) Word Transfer */
#define   DACC_MR_WORD_HALF (0x0u << 4) /**< \brief (DACC_MR) Half-Word transfer */
#define   DACC_MR_WORD_WORD (0x1u << 4) /**< \brief (DACC_MR) Word Transfer */
#define DACC_MR_SLEEP (0x1u << 5) /**< \brief (DACC_MR) Sleep Mode */
#define DACC_MR_FASTWKUP (0x1u << 6) /**< \brief (DACC_MR) Fast Wake up Mode */
#define DACC_MR_REFRESH_Pos 8
#define DACC_MR_REFRESH_Msk (0xffu << DACC_MR_REFRESH_Pos) /**< \brief (DACC_MR) Refresh Period */
#define DACC_MR_REFRESH(value) ((DACC_MR_REFRESH_Msk & ((value) << DACC_MR_REFRESH_Pos)))
#define DACC_MR_USER_SEL_Pos 16
#define DACC_MR_USER_SEL_Msk (0x3u << DACC_MR_USER_SEL_Pos) /**< \brief (DACC_MR) User Channel Selection */
#define   DACC_MR_USER_SEL_CHANNEL0 (0x0u << 16) /**< \brief (DACC_MR) Channel 0 */
#define   DACC_MR_USER_SEL_CHANNEL1 (0x1u << 16) /**< \brief (DACC_MR) Channel 1 */
#define DACC_MR_TAG (0x1u << 20) /**< \brief (DACC_MR) Tag Selection Mode */
#define   DACC_MR_TAG_DIS (0x0u << 20) /**< \brief (DACC_MR) Tag selection mode disabled. Using USER_SEL to select the channel for the conversion. */
#define   DACC_MR_TAG_EN (0x1u << 20) /**< \brief (DACC_MR) Tag selection mode enabled */
#define DACC_MR_MAXS (0x1u << 21) /**< \brief (DACC_MR) Max Speed Mode */
#define   DACC_MR_MAXS_NORMAL (0x0u << 21) /**< \brief (DACC_MR) Normal Mode */
#define   DACC_MR_MAXS_MAXIMUM (0x1u << 21) /**< \brief (DACC_MR) Max Speed Mode enabled */
#define DACC_MR_STARTUP_Pos 24
#define DACC_MR_STARTUP_Msk (0x3fu << DACC_MR_STARTUP_Pos) /**< \brief (DACC_MR) Startup Time Selection */
#define   DACC_MR_STARTUP_0 (0x0u << 24) /**< \brief (DACC_MR) 0 periods of DACClock */
#define   DACC_MR_STARTUP_8 (0x1u << 24) /**< \brief (DACC_MR) 8 periods of DACClock */
#define   DACC_MR_STARTUP_16 (0x2u << 24) /**< \brief (DACC_MR) 16 periods of DACClock */
#define   DACC_MR_STARTUP_24 (0x3u << 24) /**< \brief (DACC_MR) 24 periods of DACClock */
#define   DACC_MR_STARTUP_64 (0x4u << 24) /**< \brief (DACC_MR) 64 periods of DACClock */
#define   DACC_MR_STARTUP_80 (0x5u << 24) /**< \brief (DACC_MR) 80 periods of DACClock */
#define   DACC_MR_STARTUP_96 (0x6u << 24) /**< \brief (DACC_MR) 96 periods of DACClock */
#define   DACC_MR_STARTUP_112 (0x7u << 24) /**< \brief (DACC_MR) 112 periods of DACClock */
#define   DACC_MR_STARTUP_512 (0x8u << 24) /**< \brief (DACC_MR) 512 periods of DACClock */
#define   DACC_MR_STARTUP_576 (0x9u << 24) /**< \brief (DACC_MR) 576 periods of DACClock */
#define   DACC_MR_STARTUP_640 (0xAu << 24) /**< \brief (DACC_MR) 640 periods of DACClock */
#define   DACC_MR_STARTUP_704 (0xBu << 24) /**< \brief (DACC_MR) 704 periods of DACClock */
#define   DACC_MR_STARTUP_768 (0xCu << 24) /**< \brief (DACC_MR) 768 periods of DACClock */
#define   DACC_MR_STARTUP_832 (0xDu << 24) /**< \brief (DACC_MR) 832 periods of DACClock */
#define   DACC_MR_STARTUP_896 (0xEu << 24) /**< \brief (DACC_MR) 896 periods of DACClock */
#define   DACC_MR_STARTUP_960 (0xFu << 24) /**< \brief (DACC_MR) 960 periods of DACClock */
#define   DACC_MR_STARTUP_1024 (0x10u << 24) /**< \brief (DACC_MR) 1024 periods of DACClock */
#define   DACC_MR_STARTUP_1088 (0x11u << 24) /**< \brief (DACC_MR) 1088 periods of DACClock */
#define   DACC_MR_STARTUP_1152 (0x12u << 24) /**< \brief (DACC_MR) 1152 periods of DACClock */
#define   DACC_MR_STARTUP_1216 (0x13u << 24) /**< \brief (DACC_MR) 1216 periods of DACClock */
#define   DACC_MR_STARTUP_1280 (0x14u << 24) /**< \brief (DACC_MR) 1280 periods of DACClock */
#define   DACC_MR_STARTUP_1344 (0x15u << 24) /**< \brief (DACC_MR) 1344 periods of DACClock */
#define   DACC_MR_STARTUP_1408 (0x16u << 24) /**< \brief (DACC_MR) 1408 periods of DACClock */
#define   DACC_MR_STARTUP_1472 (0x17u << 24) /**< \brief (DACC_MR) 1472 periods of DACClock */
#define   DACC_MR_STARTUP_1536 (0x18u << 24) /**< \brief (DACC_MR) 1536 periods of DACClock */
#define   DACC_MR_STARTUP_1600 (0x19u << 24) /**< \brief (DACC_MR) 1600 periods of DACClock */
#define   DACC_MR_STARTUP_1664 (0x1Au << 24) /**< \brief (DACC_MR) 1664 periods of DACClock */
#define   DACC_MR_STARTUP_1728 (0x1Bu << 24) /**< \brief (DACC_MR) 1728 periods of DACClock */
#define   DACC_MR_STARTUP_1792 (0x1Cu << 24) /**< \brief (DACC_MR) 1792 periods of DACClock */
#define   DACC_MR_STARTUP_1856 (0x1Du << 24) /**< \brief (DACC_MR) 1856 periods of DACClock */
#define   DACC_MR_STARTUP_1920 (0x1Eu << 24) /**< \brief (DACC_MR) 1920 periods of DACClock */
#define   DACC_MR_STARTUP_1984 (0x1Fu << 24) /**< \brief (DACC_MR) 1984 periods of DACClock */
/* -------- DACC_CHER : (DACC Offset: 0x10) Channel Enable Register -------- */
#define DACC_CHER_CH0 (0x1u << 0) /**< \brief (DACC_CHER) Channel 0 Enable */
#define DACC_CHER_CH1 (0x1u << 1) /**< \brief (DACC_CHER) Channel 1 Enable */
/* -------- DACC_CHDR : (DACC Offset: 0x14) Channel Disable Register -------- */
#define DACC_CHDR_CH0 (0x1u << 0) /**< \brief (DACC_CHDR) Channel 0 Disable */
#define DACC_CHDR_CH1 (0x1u << 1) /**< \brief (DACC_CHDR) Channel 1 Disable */
/* -------- DACC_CHSR : (DACC Offset: 0x18) Channel Status Register -------- */
#define DACC_CHSR_CH0 (0x1u << 0) /**< \brief (DACC_CHSR) Channel 0 Status */
#define DACC_CHSR_CH1 (0x1u << 1) /**< \brief (DACC_CHSR) Channel 1 Status */
/* -------- DACC_CDR : (DACC Offset: 0x20) Conversion Data Register -------- */
#define DACC_CDR_DATA_Pos 0
#define DACC_CDR_DATA_Msk (0xffffffffu << DACC_CDR_DATA_Pos) /**< \brief (DACC_CDR) Data to Convert */
#define DACC_CDR_DATA(value) ((DACC_CDR_DATA_Msk & ((value) << DACC_CDR_DATA_Pos)))
/* -------- DACC_IER : (DACC Offset: 0x24) Interrupt Enable Register -------- */
#define DACC_IER_TXRDY (0x1u << 0) /**< \brief (DACC_IER) Transmit Ready Interrupt Enable */
#define DACC_IER_EOC (0x1u << 1) /**< \brief (DACC_IER) End of Conversion Interrupt Enable */
#define DACC_IER_ENDTX (0x1u << 2) /**< \brief (DACC_IER) End of Transmit Buffer Interrupt Enable */
#define DACC_IER_TXBUFE (0x1u << 3) /**< \brief (DACC_IER) Transmit Buffer Empty Interrupt Enable */
/* -------- DACC_IDR : (DACC Offset: 0x28) Interrupt Disable Register -------- */
#define DACC_IDR_TXRDY (0x1u << 0) /**< \brief (DACC_IDR) Transmit Ready Interrupt Disable. */
#define DACC_IDR_EOC (0x1u << 1) /**< \brief (DACC_IDR) End of Conversion Interrupt Disable */
#define DACC_IDR_ENDTX (0x1u << 2) /**< \brief (DACC_IDR) End of Transmit Buffer Interrupt Disable */
#define DACC_IDR_TXBUFE (0x1u << 3) /**< \brief (DACC_IDR) Transmit Buffer Empty Interrupt Disable */
/* -------- DACC_IMR : (DACC Offset: 0x2C) Interrupt Mask Register -------- */
#define DACC_IMR_TXRDY (0x1u << 0) /**< \brief (DACC_IMR) Transmit Ready Interrupt Mask */
#define DACC_IMR_EOC (0x1u << 1) /**< \brief (DACC_IMR) End of Conversion Interrupt Mask */
#define DACC_IMR_ENDTX (0x1u << 2) /**< \brief (DACC_IMR) End of Transmit Buffer Interrupt Mask */
#define DACC_IMR_TXBUFE (0x1u << 3) /**< \brief (DACC_IMR) Transmit Buffer Empty Interrupt Mask */
/* -------- DACC_ISR : (DACC Offset: 0x30) Interrupt Status Register -------- */
#define DACC_ISR_TXRDY (0x1u << 0) /**< \brief (DACC_ISR) Transmit Ready Interrupt Flag */
#define DACC_ISR_EOC (0x1u << 1) /**< \brief (DACC_ISR) End of Conversion Interrupt Flag */
#define DACC_ISR_ENDTX (0x1u << 2) /**< \brief (DACC_ISR) End of DMA Interrupt Flag */
#define DACC_ISR_TXBUFE (0x1u << 3) /**< \brief (DACC_ISR) Transmit Buffer Empty */
/* -------- DACC_ACR : (DACC Offset: 0x94) Analog Current Register -------- */
#define DACC_ACR_IBCTLCH0_Pos 0
#define DACC_ACR_IBCTLCH0_Msk (0x3u << DACC_ACR_IBCTLCH0_Pos) /**< \brief (DACC_ACR) Analog Output Current Control */
#define DACC_ACR_IBCTLCH0(value) ((DACC_ACR_IBCTLCH0_Msk & ((value) << DACC_ACR_IBCTLCH0_Pos)))
#define DACC_ACR_IBCTLCH1_Pos 2
#define DACC_ACR_IBCTLCH1_Msk (0x3u << DACC_ACR_IBCTLCH1_Pos) /**< \brief (DACC_ACR) Analog Output Current Control */
#define DACC_ACR_IBCTLCH1(value) ((DACC_ACR_IBCTLCH1_Msk & ((value) << DACC_ACR_IBCTLCH1_Pos)))
#define DACC_ACR_IBCTLDACCORE_Pos 8
#define DACC_ACR_IBCTLDACCORE_Msk (0x3u << DACC_ACR_IBCTLDACCORE_Pos) /**< \brief (DACC_ACR) Bias Current Control for DAC Core */
#define DACC_ACR_IBCTLDACCORE(value) ((DACC_ACR_IBCTLDACCORE_Msk & ((value) << DACC_ACR_IBCTLDACCORE_Pos)))
/* -------- DACC_WPMR : (DACC Offset: 0xE4) Write Protect Mode register -------- */
#define DACC_WPMR_WPEN (0x1u << 0) /**< \brief (DACC_WPMR) Write Protect Enable */
#define DACC_WPMR_WPKEY_Pos 8
#define DACC_WPMR_WPKEY_Msk (0xffffffu << DACC_WPMR_WPKEY_Pos) /**< \brief (DACC_WPMR) Write Protect KEY */
#define DACC_WPMR_WPKEY(value) ((DACC_WPMR_WPKEY_Msk & ((value) << DACC_WPMR_WPKEY_Pos)))
/* -------- DACC_WPSR : (DACC Offset: 0xE8) Write Protect Status register -------- */
#define DACC_WPSR_WPROTERR (0x1u << 0) /**< \brief (DACC_WPSR) Write protection error */
#define DACC_WPSR_WPROTADDR_Pos 8
#define DACC_WPSR_WPROTADDR_Msk (0xffu << DACC_WPSR_WPROTADDR_Pos) /**< \brief (DACC_WPSR) Write protection error address */
/* -------- DACC_TPR : (DACC Offset: 0x108) Transmit Pointer Register -------- */
#define DACC_TPR_TXPTR_Pos 0
#define DACC_TPR_TXPTR_Msk (0xffffffffu << DACC_TPR_TXPTR_Pos) /**< \brief (DACC_TPR) Transmit Counter Register */
#define DACC_TPR_TXPTR(value) ((DACC_TPR_TXPTR_Msk & ((value) << DACC_TPR_TXPTR_Pos)))
/* -------- DACC_TCR : (DACC Offset: 0x10C) Transmit Counter Register -------- */
#define DACC_TCR_TXCTR_Pos 0
#define DACC_TCR_TXCTR_Msk (0xffffu << DACC_TCR_TXCTR_Pos) /**< \brief (DACC_TCR) Transmit Counter Register */
#define DACC_TCR_TXCTR(value) ((DACC_TCR_TXCTR_Msk & ((value) << DACC_TCR_TXCTR_Pos)))
/* -------- DACC_TNPR : (DACC Offset: 0x118) Transmit Next Pointer Register -------- */
#define DACC_TNPR_TXNPTR_Pos 0
#define DACC_TNPR_TXNPTR_Msk (0xffffffffu << DACC_TNPR_TXNPTR_Pos) /**< \brief (DACC_TNPR) Transmit Next Pointer */
#define DACC_TNPR_TXNPTR(value) ((DACC_TNPR_TXNPTR_Msk & ((value) << DACC_TNPR_TXNPTR_Pos)))
/* -------- DACC_TNCR : (DACC Offset: 0x11C) Transmit Next Counter Register -------- */
#define DACC_TNCR_TXNCTR_Pos 0
#define DACC_TNCR_TXNCTR_Msk (0xffffu << DACC_TNCR_TXNCTR_Pos) /**< \brief (DACC_TNCR) Transmit Counter Next */
#define DACC_TNCR_TXNCTR(value) ((DACC_TNCR_TXNCTR_Msk & ((value) << DACC_TNCR_TXNCTR_Pos)))
/* -------- DACC_PTCR : (DACC Offset: 0x120) Transfer Control Register -------- */
#define DACC_PTCR_RXTEN (0x1u << 0) /**< \brief (DACC_PTCR) Receiver Transfer Enable */
#define DACC_PTCR_RXTDIS (0x1u << 1) /**< \brief (DACC_PTCR) Receiver Transfer Disable */
#define DACC_PTCR_TXTEN (0x1u << 8) /**< \brief (DACC_PTCR) Transmitter Transfer Enable */
#define DACC_PTCR_TXTDIS (0x1u << 9) /**< \brief (DACC_PTCR) Transmitter Transfer Disable */
/* -------- DACC_PTSR : (DACC Offset: 0x124) Transfer Status Register -------- */
#define DACC_PTSR_RXTEN (0x1u << 0) /**< \brief (DACC_PTSR) Receiver Transfer Enable */
#define DACC_PTSR_TXTEN (0x1u << 8) /**< \brief (DACC_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_DACC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR DMA Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_DMAC DMA Controller */
/*@{*/

/** \brief DmacCh_num hardware registers */
typedef struct {
  RwReg       DMAC_SADDR;     /**< \brief (DmacCh_num Offset: 0x0) DMAC Channel Source Address Register */
  RwReg       DMAC_DADDR;     /**< \brief (DmacCh_num Offset: 0x4) DMAC Channel Destination Address Register */
  RwReg       DMAC_DSCR;      /**< \brief (DmacCh_num Offset: 0x8) DMAC Channel Descriptor Address Register */
  RwReg       DMAC_CTRLA;     /**< \brief (DmacCh_num Offset: 0xC) DMAC Channel Control A Register */
  RwReg       DMAC_CTRLB;     /**< \brief (DmacCh_num Offset: 0x10) DMAC Channel Control B Register */
  RwReg       DMAC_CFG;       /**< \brief (DmacCh_num Offset: 0x14) DMAC Channel Configuration Register */
  RoReg       Reserved1[4];
} DmacCh_num;
/** \brief Dmac hardware registers */
#define DMACCH_NUM_NUMBER 6
typedef struct {
  RwReg       DMAC_GCFG;      /**< \brief (Dmac Offset: 0x000) DMAC Global Configuration Register */
  RwReg       DMAC_EN;        /**< \brief (Dmac Offset: 0x004) DMAC Enable Register */
  RwReg       DMAC_SREQ;      /**< \brief (Dmac Offset: 0x008) DMAC Software Single Request Register */
  RwReg       DMAC_CREQ;      /**< \brief (Dmac Offset: 0x00C) DMAC Software Chunk Transfer Request Register */
  RwReg       DMAC_LAST;      /**< \brief (Dmac Offset: 0x010) DMAC Software Last Transfer Flag Register */
  RoReg       Reserved1[1];
  WoReg       DMAC_EBCIER;    /**< \brief (Dmac Offset: 0x018) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register. */
  WoReg       DMAC_EBCIDR;    /**< \brief (Dmac Offset: 0x01C) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register. */
  RoReg       DMAC_EBCIMR;    /**< \brief (Dmac Offset: 0x020) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register. */
  RoReg       DMAC_EBCISR;    /**< \brief (Dmac Offset: 0x024) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register. */
  WoReg       DMAC_CHER;      /**< \brief (Dmac Offset: 0x028) DMAC Channel Handler Enable Register */
  WoReg       DMAC_CHDR;      /**< \brief (Dmac Offset: 0x02C) DMAC Channel Handler Disable Register */
  RoReg       DMAC_CHSR;      /**< \brief (Dmac Offset: 0x030) DMAC Channel Handler Status Register */
  RoReg       Reserved2[2];
  DmacCh_num  DMAC_CH_NUM[DMACCH_NUM_NUMBER]; /**< \brief (Dmac Offset: 0x3C) ch_num = 0 .. 5 */
  RoReg       Reserved3[46];
  RwReg       DMAC_WPMR;      /**< \brief (Dmac Offset: 0x1E4) DMAC Write Protect Mode Register */
  RoReg       DMAC_WPSR;      /**< \brief (Dmac Offset: 0x1E8) DMAC Write Protect Status Register */
} Dmac;
/* -------- DMAC_GCFG : (DMAC Offset: 0x000) DMAC Global Configuration Register -------- */
#define DMAC_GCFG_ARB_CFG (0x1u << 4) /**< \brief (DMAC_GCFG) Arbiter Configuration */
#define   DMAC_GCFG_ARB_CFG_FIXED (0x0u << 4) /**< \brief (DMAC_GCFG) Fixed priority arbiter. */
#define   DMAC_GCFG_ARB_CFG_ROUND_ROBIN (0x1u << 4) /**< \brief (DMAC_GCFG) Modified round robin arbiter. */
/* -------- DMAC_EN : (DMAC Offset: 0x004) DMAC Enable Register -------- */
#define DMAC_EN_ENABLE (0x1u << 0) /**< \brief (DMAC_EN)  */
/* -------- DMAC_SREQ : (DMAC Offset: 0x008) DMAC Software Single Request Register -------- */
#define DMAC_SREQ_SSREQ0 (0x1u << 0) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ0 (0x1u << 1) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ1 (0x1u << 2) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ1 (0x1u << 3) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ2 (0x1u << 4) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ2 (0x1u << 5) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ3 (0x1u << 6) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ3 (0x1u << 7) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ4 (0x1u << 8) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ4 (0x1u << 9) /**< \brief (DMAC_SREQ) Destination Request */
#define DMAC_SREQ_SSREQ5 (0x1u << 10) /**< \brief (DMAC_SREQ) Source Request */
#define DMAC_SREQ_DSREQ5 (0x1u << 11) /**< \brief (DMAC_SREQ) Destination Request */
/* -------- DMAC_CREQ : (DMAC Offset: 0x00C) DMAC Software Chunk Transfer Request Register -------- */
#define DMAC_CREQ_SCREQ0 (0x1u << 0) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ0 (0x1u << 1) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ1 (0x1u << 2) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ1 (0x1u << 3) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ2 (0x1u << 4) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ2 (0x1u << 5) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ3 (0x1u << 6) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ3 (0x1u << 7) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ4 (0x1u << 8) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ4 (0x1u << 9) /**< \brief (DMAC_CREQ) Destination Chunk Request */
#define DMAC_CREQ_SCREQ5 (0x1u << 10) /**< \brief (DMAC_CREQ) Source Chunk Request */
#define DMAC_CREQ_DCREQ5 (0x1u << 11) /**< \brief (DMAC_CREQ) Destination Chunk Request */
/* -------- DMAC_LAST : (DMAC Offset: 0x010) DMAC Software Last Transfer Flag Register -------- */
#define DMAC_LAST_SLAST0 (0x1u << 0) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST0 (0x1u << 1) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST1 (0x1u << 2) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST1 (0x1u << 3) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST2 (0x1u << 4) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST2 (0x1u << 5) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST3 (0x1u << 6) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST3 (0x1u << 7) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST4 (0x1u << 8) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST4 (0x1u << 9) /**< \brief (DMAC_LAST) Destination Last */
#define DMAC_LAST_SLAST5 (0x1u << 10) /**< \brief (DMAC_LAST) Source Last */
#define DMAC_LAST_DLAST5 (0x1u << 11) /**< \brief (DMAC_LAST) Destination Last */
/* -------- DMAC_EBCIER : (DMAC Offset: 0x018) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register. -------- */
#define DMAC_EBCIER_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_BTC4 (0x1u << 4) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_BTC5 (0x1u << 5) /**< \brief (DMAC_EBCIER) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC4 (0x1u << 12) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_CBTC5 (0x1u << 13) /**< \brief (DMAC_EBCIER) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIER_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
#define DMAC_EBCIER_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
#define DMAC_EBCIER_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
#define DMAC_EBCIER_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
#define DMAC_EBCIER_ERR4 (0x1u << 20) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
#define DMAC_EBCIER_ERR5 (0x1u << 21) /**< \brief (DMAC_EBCIER) Access Error [5:0] */
/* -------- DMAC_EBCIDR : (DMAC Offset: 0x01C) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register. -------- */
#define DMAC_EBCIDR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_BTC4 (0x1u << 4) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_BTC5 (0x1u << 5) /**< \brief (DMAC_EBCIDR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC4 (0x1u << 12) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_CBTC5 (0x1u << 13) /**< \brief (DMAC_EBCIDR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIDR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
#define DMAC_EBCIDR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
#define DMAC_EBCIDR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
#define DMAC_EBCIDR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
#define DMAC_EBCIDR_ERR4 (0x1u << 20) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
#define DMAC_EBCIDR_ERR5 (0x1u << 21) /**< \brief (DMAC_EBCIDR) Access Error [5:0] */
/* -------- DMAC_EBCIMR : (DMAC Offset: 0x020) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register. -------- */
#define DMAC_EBCIMR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_BTC4 (0x1u << 4) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_BTC5 (0x1u << 5) /**< \brief (DMAC_EBCIMR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC4 (0x1u << 12) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_CBTC5 (0x1u << 13) /**< \brief (DMAC_EBCIMR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCIMR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
#define DMAC_EBCIMR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
#define DMAC_EBCIMR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
#define DMAC_EBCIMR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
#define DMAC_EBCIMR_ERR4 (0x1u << 20) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
#define DMAC_EBCIMR_ERR5 (0x1u << 21) /**< \brief (DMAC_EBCIMR) Access Error [5:0] */
/* -------- DMAC_EBCISR : (DMAC Offset: 0x024) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register. -------- */
#define DMAC_EBCISR_BTC0 (0x1u << 0) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_BTC1 (0x1u << 1) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_BTC2 (0x1u << 2) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_BTC3 (0x1u << 3) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_BTC4 (0x1u << 4) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_BTC5 (0x1u << 5) /**< \brief (DMAC_EBCISR) Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC0 (0x1u << 8) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC1 (0x1u << 9) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC2 (0x1u << 10) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC3 (0x1u << 11) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC4 (0x1u << 12) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_CBTC5 (0x1u << 13) /**< \brief (DMAC_EBCISR) Chained Buffer Transfer Completed [5:0] */
#define DMAC_EBCISR_ERR0 (0x1u << 16) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
#define DMAC_EBCISR_ERR1 (0x1u << 17) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
#define DMAC_EBCISR_ERR2 (0x1u << 18) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
#define DMAC_EBCISR_ERR3 (0x1u << 19) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
#define DMAC_EBCISR_ERR4 (0x1u << 20) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
#define DMAC_EBCISR_ERR5 (0x1u << 21) /**< \brief (DMAC_EBCISR) Access Error [5:0] */
/* -------- DMAC_CHER : (DMAC Offset: 0x028) DMAC Channel Handler Enable Register -------- */
#define DMAC_CHER_ENA0 (0x1u << 0) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_ENA1 (0x1u << 1) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_ENA2 (0x1u << 2) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_ENA3 (0x1u << 3) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_ENA4 (0x1u << 4) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_ENA5 (0x1u << 5) /**< \brief (DMAC_CHER) Enable [5:0] */
#define DMAC_CHER_SUSP0 (0x1u << 8) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_SUSP1 (0x1u << 9) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_SUSP2 (0x1u << 10) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_SUSP3 (0x1u << 11) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_SUSP4 (0x1u << 12) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_SUSP5 (0x1u << 13) /**< \brief (DMAC_CHER) Suspend [5:0] */
#define DMAC_CHER_KEEP0 (0x1u << 24) /**< \brief (DMAC_CHER) Keep on [5:0] */
#define DMAC_CHER_KEEP1 (0x1u << 25) /**< \brief (DMAC_CHER) Keep on [5:0] */
#define DMAC_CHER_KEEP2 (0x1u << 26) /**< \brief (DMAC_CHER) Keep on [5:0] */
#define DMAC_CHER_KEEP3 (0x1u << 27) /**< \brief (DMAC_CHER) Keep on [5:0] */
#define DMAC_CHER_KEEP4 (0x1u << 28) /**< \brief (DMAC_CHER) Keep on [5:0] */
#define DMAC_CHER_KEEP5 (0x1u << 29) /**< \brief (DMAC_CHER) Keep on [5:0] */
/* -------- DMAC_CHDR : (DMAC Offset: 0x02C) DMAC Channel Handler Disable Register -------- */
#define DMAC_CHDR_DIS0 (0x1u << 0) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_DIS1 (0x1u << 1) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_DIS2 (0x1u << 2) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_DIS3 (0x1u << 3) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_DIS4 (0x1u << 4) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_DIS5 (0x1u << 5) /**< \brief (DMAC_CHDR) Disable [5:0] */
#define DMAC_CHDR_RES0 (0x1u << 8) /**< \brief (DMAC_CHDR) Resume [5:0] */
#define DMAC_CHDR_RES1 (0x1u << 9) /**< \brief (DMAC_CHDR) Resume [5:0] */
#define DMAC_CHDR_RES2 (0x1u << 10) /**< \brief (DMAC_CHDR) Resume [5:0] */
#define DMAC_CHDR_RES3 (0x1u << 11) /**< \brief (DMAC_CHDR) Resume [5:0] */
#define DMAC_CHDR_RES4 (0x1u << 12) /**< \brief (DMAC_CHDR) Resume [5:0] */
#define DMAC_CHDR_RES5 (0x1u << 13) /**< \brief (DMAC_CHDR) Resume [5:0] */
/* -------- DMAC_CHSR : (DMAC Offset: 0x030) DMAC Channel Handler Status Register -------- */
#define DMAC_CHSR_ENA0 (0x1u << 0) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_ENA1 (0x1u << 1) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_ENA2 (0x1u << 2) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_ENA3 (0x1u << 3) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_ENA4 (0x1u << 4) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_ENA5 (0x1u << 5) /**< \brief (DMAC_CHSR) Enable [5:0] */
#define DMAC_CHSR_SUSP0 (0x1u << 8) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_SUSP1 (0x1u << 9) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_SUSP2 (0x1u << 10) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_SUSP3 (0x1u << 11) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_SUSP4 (0x1u << 12) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_SUSP5 (0x1u << 13) /**< \brief (DMAC_CHSR) Suspend [5:0] */
#define DMAC_CHSR_EMPT0 (0x1u << 16) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_EMPT1 (0x1u << 17) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_EMPT2 (0x1u << 18) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_EMPT3 (0x1u << 19) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_EMPT4 (0x1u << 20) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_EMPT5 (0x1u << 21) /**< \brief (DMAC_CHSR) Empty [5:0] */
#define DMAC_CHSR_STAL0 (0x1u << 24) /**< \brief (DMAC_CHSR) Stalled [5:0] */
#define DMAC_CHSR_STAL1 (0x1u << 25) /**< \brief (DMAC_CHSR) Stalled [5:0] */
#define DMAC_CHSR_STAL2 (0x1u << 26) /**< \brief (DMAC_CHSR) Stalled [5:0] */
#define DMAC_CHSR_STAL3 (0x1u << 27) /**< \brief (DMAC_CHSR) Stalled [5:0] */
#define DMAC_CHSR_STAL4 (0x1u << 28) /**< \brief (DMAC_CHSR) Stalled [5:0] */
#define DMAC_CHSR_STAL5 (0x1u << 29) /**< \brief (DMAC_CHSR) Stalled [5:0] */
/* -------- DMAC_SADDR : (DMAC Offset: N/A) DMAC Channel Source Address Register -------- */
#define DMAC_SADDR_SADDR_Pos 0
#define DMAC_SADDR_SADDR_Msk (0xffffffffu << DMAC_SADDR_SADDR_Pos) /**< \brief (DMAC_SADDR) Channel x Source Address */
#define DMAC_SADDR_SADDR(value) ((DMAC_SADDR_SADDR_Msk & ((value) << DMAC_SADDR_SADDR_Pos)))
/* -------- DMAC_DADDR : (DMAC Offset: N/A) DMAC Channel Destination Address Register -------- */
#define DMAC_DADDR_DADDR_Pos 0
#define DMAC_DADDR_DADDR_Msk (0xffffffffu << DMAC_DADDR_DADDR_Pos) /**< \brief (DMAC_DADDR) Channel x Destination Address */
#define DMAC_DADDR_DADDR(value) ((DMAC_DADDR_DADDR_Msk & ((value) << DMAC_DADDR_DADDR_Pos)))
/* -------- DMAC_DSCR : (DMAC Offset: N/A) DMAC Channel Descriptor Address Register -------- */
#define DMAC_DSCR_DSCR_Pos 2
#define DMAC_DSCR_DSCR_Msk (0x3fffffffu << DMAC_DSCR_DSCR_Pos) /**< \brief (DMAC_DSCR) Buffer Transfer Descriptor Address */
#define DMAC_DSCR_DSCR(value) ((DMAC_DSCR_DSCR_Msk & ((value) << DMAC_DSCR_DSCR_Pos)))
/* -------- DMAC_CTRLA : (DMAC Offset: N/A) DMAC Channel Control A Register -------- */
#define DMAC_CTRLA_BTSIZE_Pos 0
#define DMAC_CTRLA_BTSIZE_Msk (0xffffu << DMAC_CTRLA_BTSIZE_Pos) /**< \brief (DMAC_CTRLA) Buffer Transfer Size */
#define DMAC_CTRLA_BTSIZE(value) ((DMAC_CTRLA_BTSIZE_Msk & ((value) << DMAC_CTRLA_BTSIZE_Pos)))
#define DMAC_CTRLA_SCSIZE_Pos 16
#define DMAC_CTRLA_SCSIZE_Msk (0x7u << DMAC_CTRLA_SCSIZE_Pos) /**< \brief (DMAC_CTRLA) Source Chunk Transfer Size. */
#define   DMAC_CTRLA_SCSIZE_CHK_1 (0x0u << 16) /**< \brief (DMAC_CTRLA) 1 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_4 (0x1u << 16) /**< \brief (DMAC_CTRLA) 4 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_8 (0x2u << 16) /**< \brief (DMAC_CTRLA) 8 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_16 (0x3u << 16) /**< \brief (DMAC_CTRLA) 16 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_32 (0x4u << 16) /**< \brief (DMAC_CTRLA) 32 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_64 (0x5u << 16) /**< \brief (DMAC_CTRLA) 64 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_128 (0x6u << 16) /**< \brief (DMAC_CTRLA) 128 data transferred */
#define   DMAC_CTRLA_SCSIZE_CHK_256 (0x7u << 16) /**< \brief (DMAC_CTRLA) 256 data transferred */
#define DMAC_CTRLA_DCSIZE_Pos 20
#define DMAC_CTRLA_DCSIZE_Msk (0x7u << DMAC_CTRLA_DCSIZE_Pos) /**< \brief (DMAC_CTRLA) Destination Chunk Transfer Size */
#define   DMAC_CTRLA_DCSIZE_CHK_1 (0x0u << 20) /**< \brief (DMAC_CTRLA) 1 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_4 (0x1u << 20) /**< \brief (DMAC_CTRLA) 4 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_8 (0x2u << 20) /**< \brief (DMAC_CTRLA) 8 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_16 (0x3u << 20) /**< \brief (DMAC_CTRLA) 16 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_32 (0x4u << 20) /**< \brief (DMAC_CTRLA) 32 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_64 (0x5u << 20) /**< \brief (DMAC_CTRLA) 64 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_128 (0x6u << 20) /**< \brief (DMAC_CTRLA) 128 data transferred */
#define   DMAC_CTRLA_DCSIZE_CHK_256 (0x7u << 20) /**< \brief (DMAC_CTRLA) 256 data transferred */
#define DMAC_CTRLA_SRC_WIDTH_Pos 24
#define DMAC_CTRLA_SRC_WIDTH_Msk (0x3u << DMAC_CTRLA_SRC_WIDTH_Pos) /**< \brief (DMAC_CTRLA) Transfer Width for the Source */
#define   DMAC_CTRLA_SRC_WIDTH_BYTE (0x0u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 8-bit width */
#define   DMAC_CTRLA_SRC_WIDTH_HALF_WORD (0x1u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 16-bit width */
#define   DMAC_CTRLA_SRC_WIDTH_WORD (0x2u << 24) /**< \brief (DMAC_CTRLA) the transfer size is set to 32-bit width */
#define DMAC_CTRLA_DST_WIDTH_Pos 28
#define DMAC_CTRLA_DST_WIDTH_Msk (0x3u << DMAC_CTRLA_DST_WIDTH_Pos) /**< \brief (DMAC_CTRLA) Transfer Width for the Destination */
#define   DMAC_CTRLA_DST_WIDTH_BYTE (0x0u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 8-bit width */
#define   DMAC_CTRLA_DST_WIDTH_HALF_WORD (0x1u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 16-bit width */
#define   DMAC_CTRLA_DST_WIDTH_WORD (0x2u << 28) /**< \brief (DMAC_CTRLA) the transfer size is set to 32-bit width */
#define DMAC_CTRLA_DONE (0x1u << 31) /**< \brief (DMAC_CTRLA)  */
/* -------- DMAC_CTRLB : (DMAC Offset: N/A) DMAC Channel Control B Register -------- */
#define DMAC_CTRLB_SRC_DSCR (0x1u << 16) /**< \brief (DMAC_CTRLB) Source Address Descriptor */
#define   DMAC_CTRLB_SRC_DSCR_FETCH_FROM_MEM (0x0u << 16) /**< \brief (DMAC_CTRLB) Source address is updated when the descriptor is fetched from the memory. */
#define   DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE (0x1u << 16) /**< \brief (DMAC_CTRLB) Buffer Descriptor Fetch operation is disabled for the source. */
#define DMAC_CTRLB_DST_DSCR (0x1u << 20) /**< \brief (DMAC_CTRLB) Destination Address Descriptor */
#define   DMAC_CTRLB_DST_DSCR_FETCH_FROM_MEM (0x0u << 20) /**< \brief (DMAC_CTRLB) Destination address is updated when the descriptor is fetched from the memory. */
#define   DMAC_CTRLB_DST_DSCR_FETCH_DISABLE (0x1u << 20) /**< \brief (DMAC_CTRLB) Buffer Descriptor Fetch operation is disabled for the destination. */
#define DMAC_CTRLB_FC_Pos 21
#define DMAC_CTRLB_FC_Msk (0x7u << DMAC_CTRLB_FC_Pos) /**< \brief (DMAC_CTRLB) Flow Control */
#define   DMAC_CTRLB_FC_MEM2MEM_DMA_FC (0x0u << 21) /**< \brief (DMAC_CTRLB) Memory-to-Memory Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_MEM2PER_DMA_FC (0x1u << 21) /**< \brief (DMAC_CTRLB) Memory-to-Peripheral Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_PER2MEM_DMA_FC (0x2u << 21) /**< \brief (DMAC_CTRLB) Peripheral-to-Memory Transfer DMAC is flow controller */
#define   DMAC_CTRLB_FC_PER2PER_DMA_FC (0x3u << 21) /**< \brief (DMAC_CTRLB) Peripheral-to-Peripheral Transfer DMAC is flow controller */
#define DMAC_CTRLB_SRC_INCR_Pos 24
#define DMAC_CTRLB_SRC_INCR_Msk (0x3u << DMAC_CTRLB_SRC_INCR_Pos) /**< \brief (DMAC_CTRLB) Incrementing, Decrementing or Fixed Address for the Source */
#define   DMAC_CTRLB_SRC_INCR_INCREMENTING (0x0u << 24) /**< \brief (DMAC_CTRLB) The source address is incremented */
#define   DMAC_CTRLB_SRC_INCR_DECREMENTING (0x1u << 24) /**< \brief (DMAC_CTRLB) The source address is decremented */
#define   DMAC_CTRLB_SRC_INCR_FIXED (0x2u << 24) /**< \brief (DMAC_CTRLB) The source address remains unchanged */
#define DMAC_CTRLB_DST_INCR_Pos 28
#define DMAC_CTRLB_DST_INCR_Msk (0x3u << DMAC_CTRLB_DST_INCR_Pos) /**< \brief (DMAC_CTRLB) Incrementing, Decrementing or Fixed Address for the Destination */
#define   DMAC_CTRLB_DST_INCR_INCREMENTING (0x0u << 28) /**< \brief (DMAC_CTRLB) The destination address is incremented */
#define   DMAC_CTRLB_DST_INCR_DECREMENTING (0x1u << 28) /**< \brief (DMAC_CTRLB) The destination address is decremented */
#define   DMAC_CTRLB_DST_INCR_FIXED (0x2u << 28) /**< \brief (DMAC_CTRLB) The destination address remains unchanged */
#define DMAC_CTRLB_IEN (0x1u << 30) /**< \brief (DMAC_CTRLB)  */
/* -------- DMAC_CFG : (DMAC Offset: N/A) DMAC Channel Configuration Register -------- */
#define DMAC_CFG_SRC_PER_Pos 0
#define DMAC_CFG_SRC_PER_Msk (0xfu << DMAC_CFG_SRC_PER_Pos) /**< \brief (DMAC_CFG) Source with Peripheral identifier */
#define DMAC_CFG_SRC_PER(value) ((DMAC_CFG_SRC_PER_Msk & ((value) << DMAC_CFG_SRC_PER_Pos)))
#define DMAC_CFG_DST_PER_Pos 4
#define DMAC_CFG_DST_PER_Msk (0xfu << DMAC_CFG_DST_PER_Pos) /**< \brief (DMAC_CFG) Destination with Peripheral identifier */
#define DMAC_CFG_DST_PER(value) ((DMAC_CFG_DST_PER_Msk & ((value) << DMAC_CFG_DST_PER_Pos)))
#define DMAC_CFG_SRC_H2SEL (0x1u << 9) /**< \brief (DMAC_CFG) Software or Hardware Selection for the Source */
#define   DMAC_CFG_SRC_H2SEL_SW (0x0u << 9) /**< \brief (DMAC_CFG) Software handshaking interface is used to trigger a transfer request. */
#define   DMAC_CFG_SRC_H2SEL_HW (0x1u << 9) /**< \brief (DMAC_CFG) Hardware handshaking interface is used to trigger a transfer request. */
#define DMAC_CFG_DST_H2SEL (0x1u << 13) /**< \brief (DMAC_CFG) Software or Hardware Selection for the Destination */
#define   DMAC_CFG_DST_H2SEL_SW (0x0u << 13) /**< \brief (DMAC_CFG) Software handshaking interface is used to trigger a transfer request. */
#define   DMAC_CFG_DST_H2SEL_HW (0x1u << 13) /**< \brief (DMAC_CFG) Hardware handshaking interface is used to trigger a transfer request. */
#define DMAC_CFG_SOD (0x1u << 16) /**< \brief (DMAC_CFG) Stop On Done */
#define   DMAC_CFG_SOD_DISABLE (0x0u << 16) /**< \brief (DMAC_CFG) STOP ON DONE disabled, the descriptor fetch operation ignores DONE Field of CTRLA register. */
#define   DMAC_CFG_SOD_ENABLE (0x1u << 16) /**< \brief (DMAC_CFG) STOP ON DONE activated, the DMAC module is automatically disabled if DONE FIELD is set to 1. */
#define DMAC_CFG_LOCK_IF (0x1u << 20) /**< \brief (DMAC_CFG) Interface Lock */
#define   DMAC_CFG_LOCK_IF_DISABLE (0x0u << 20) /**< \brief (DMAC_CFG) Interface Lock capability is disabled */
#define   DMAC_CFG_LOCK_IF_ENABLE (0x1u << 20) /**< \brief (DMAC_CFG) Interface Lock capability is enabled */
#define DMAC_CFG_LOCK_B (0x1u << 21) /**< \brief (DMAC_CFG) Bus Lock */
#define   DMAC_CFG_LOCK_B_DISABLE (0x0u << 21) /**< \brief (DMAC_CFG) AHB Bus Locking capability is disabled. */
#define DMAC_CFG_LOCK_IF_L (0x1u << 22) /**< \brief (DMAC_CFG) Master Interface Arbiter Lock */
#define   DMAC_CFG_LOCK_IF_L_CHUNK (0x0u << 22) /**< \brief (DMAC_CFG) The Master Interface Arbiter is locked by the channel x for a chunk transfer. */
#define   DMAC_CFG_LOCK_IF_L_BUFFER (0x1u << 22) /**< \brief (DMAC_CFG) The Master Interface Arbiter is locked by the channel x for a buffer transfer. */
#define DMAC_CFG_AHB_PROT_Pos 24
#define DMAC_CFG_AHB_PROT_Msk (0x7u << DMAC_CFG_AHB_PROT_Pos) /**< \brief (DMAC_CFG) AHB Protection */
#define DMAC_CFG_AHB_PROT(value) ((DMAC_CFG_AHB_PROT_Msk & ((value) << DMAC_CFG_AHB_PROT_Pos)))
#define DMAC_CFG_FIFOCFG_Pos 28
#define DMAC_CFG_FIFOCFG_Msk (0x3u << DMAC_CFG_FIFOCFG_Pos) /**< \brief (DMAC_CFG) FIFO Configuration */
#define   DMAC_CFG_FIFOCFG_ALAP_CFG (0x0u << 28) /**< \brief (DMAC_CFG) The largest defined length AHB burst is performed on the destination AHB interface. */
#define   DMAC_CFG_FIFOCFG_HALF_CFG (0x1u << 28) /**< \brief (DMAC_CFG) When half FIFO size is available/filled, a source/destination request is serviced. */
#define   DMAC_CFG_FIFOCFG_ASAP_CFG (0x2u << 28) /**< \brief (DMAC_CFG) When there is enough space/data available to perform a single AHB access, then the request is serviced. */
/* -------- DMAC_WPMR : (DMAC Offset: 0x1E4) DMAC Write Protect Mode Register -------- */
#define DMAC_WPMR_WPEN (0x1u << 0) /**< \brief (DMAC_WPMR) Write Protect Enable */
#define DMAC_WPMR_WPKEY_Pos 8
#define DMAC_WPMR_WPKEY_Msk (0xffffffu << DMAC_WPMR_WPKEY_Pos) /**< \brief (DMAC_WPMR) Write Protect KEY */
#define DMAC_WPMR_WPKEY(value) ((DMAC_WPMR_WPKEY_Msk & ((value) << DMAC_WPMR_WPKEY_Pos)))
/* -------- DMAC_WPSR : (DMAC Offset: 0x1E8) DMAC Write Protect Status Register -------- */
#define DMAC_WPSR_WPVS (0x1u << 0) /**< \brief (DMAC_WPSR) Write Protect Violation Status */
#define DMAC_WPSR_WPVSRC_Pos 8
#define DMAC_WPSR_WPVSRC_Msk (0xffffu << DMAC_WPSR_WPVSRC_Pos) /**< \brief (DMAC_WPSR) Write Protect Violation Source */
/*@}*/ /* end of group SAM3XA_DMA */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Embedded Flash Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_EFC Embedded Flash Controller */
/*@{*/

/** \brief Efc hardware registers */
typedef struct {
  RwReg EEFC_FMR; /**< \brief (Efc Offset: 0x00) EEFC Flash Mode Register */
  WoReg EEFC_FCR; /**< \brief (Efc Offset: 0x04) EEFC Flash Command Register */
  RoReg EEFC_FSR; /**< \brief (Efc Offset: 0x08) EEFC Flash Status Register */
  RoReg EEFC_FRR; /**< \brief (Efc Offset: 0x0C) EEFC Flash Result Register */
} Efc;
/* -------- EEFC_FMR : (EFC Offset: 0x00) EEFC Flash Mode Register -------- */
#define EEFC_FMR_FRDY (0x1u << 0) /**< \brief (EEFC_FMR) Ready Interrupt Enable */
#define EEFC_FMR_FWS_Pos 8
#define EEFC_FMR_FWS_Msk (0xfu << EEFC_FMR_FWS_Pos) /**< \brief (EEFC_FMR) Flash Wait State */
#define EEFC_FMR_FWS(value) ((EEFC_FMR_FWS_Msk & ((value) << EEFC_FMR_FWS_Pos)))
#define EEFC_FMR_SCOD (0x1u << 16) /**< \brief (EEFC_FMR) Sequential Code Optimization Disable */
#define EEFC_FMR_FAM (0x1u << 24) /**< \brief (EEFC_FMR) Flash Access Mode */
/* -------- EEFC_FCR : (EFC Offset: 0x04) EEFC Flash Command Register -------- */
#define EEFC_FCR_FCMD_Pos 0
#define EEFC_FCR_FCMD_Msk (0xffu << EEFC_FCR_FCMD_Pos) /**< \brief (EEFC_FCR) Flash Command */
#define EEFC_FCR_FCMD(value) ((EEFC_FCR_FCMD_Msk & ((value) << EEFC_FCR_FCMD_Pos)))
#define EEFC_FCR_FARG_Pos 8
#define EEFC_FCR_FARG_Msk (0xffffu << EEFC_FCR_FARG_Pos) /**< \brief (EEFC_FCR) Flash Command Argument */
#define EEFC_FCR_FARG(value) ((EEFC_FCR_FARG_Msk & ((value) << EEFC_FCR_FARG_Pos)))
#define EEFC_FCR_FKEY_Pos 24
#define EEFC_FCR_FKEY_Msk (0xffu << EEFC_FCR_FKEY_Pos) /**< \brief (EEFC_FCR) Flash Writing Protection Key */
#define EEFC_FCR_FKEY(value) ((EEFC_FCR_FKEY_Msk & ((value) << EEFC_FCR_FKEY_Pos)))
/* -------- EEFC_FSR : (EFC Offset: 0x08) EEFC Flash Status Register -------- */
#define EEFC_FSR_FRDY (0x1u << 0) /**< \brief (EEFC_FSR) Flash Ready Status */
#define EEFC_FSR_FCMDE (0x1u << 1) /**< \brief (EEFC_FSR) Flash Command Error Status */
#define EEFC_FSR_FLOCKE (0x1u << 2) /**< \brief (EEFC_FSR) Flash Lock Error Status */
/* -------- EEFC_FRR : (EFC Offset: 0x0C) EEFC Flash Result Register -------- */
#define EEFC_FRR_FVALUE_Pos 0
#define EEFC_FRR_FVALUE_Msk (0xffffffffu << EEFC_FRR_FVALUE_Pos) /**< \brief (EEFC_FRR) Flash Result Value */
/*@}*/ /* end of group SAM3XA_EFC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Ethernet MAC 10/100 */
/* ============================================================================= */
/** \addtogroup SAM3XA_EMAC Ethernet MAC 10/100 */
/*@{*/

/** \brief EmacSa hardware registers */
typedef struct {
  RwReg   EMAC_SAxB;    /**< \brief (EmacSa Offset: 0x0) Specific Address 1 Bottom Register */
  RwReg   EMAC_SAxT;    /**< \brief (EmacSa Offset: 0x4) Specific Address 1 Top Register */
} EmacSa;
/** \brief Emac hardware registers */
#define EMACSA_NUMBER 4
typedef struct {
  RwReg   EMAC_NCR;     /**< \brief (Emac Offset: 0x00) Network Control Register */
  RwReg   EMAC_NCFGR;   /**< \brief (Emac Offset: 0x04) Network Configuration Register */
  RoReg   EMAC_NSR;     /**< \brief (Emac Offset: 0x08) Network Status Register */
  RoReg   Reserved1[2];
  RwReg   EMAC_TSR;     /**< \brief (Emac Offset: 0x14) Transmit Status Register */
  RwReg   EMAC_RBQP;    /**< \brief (Emac Offset: 0x18) Receive Buffer Queue Pointer Register */
  RwReg   EMAC_TBQP;    /**< \brief (Emac Offset: 0x1C) Transmit Buffer Queue Pointer Register */
  RwReg   EMAC_RSR;     /**< \brief (Emac Offset: 0x20) Receive Status Register */
  RwReg   EMAC_ISR;     /**< \brief (Emac Offset: 0x24) Interrupt Status Register */
  WoReg   EMAC_IER;     /**< \brief (Emac Offset: 0x28) Interrupt Enable Register */
  WoReg   EMAC_IDR;     /**< \brief (Emac Offset: 0x2C) Interrupt Disable Register */
  RoReg   EMAC_IMR;     /**< \brief (Emac Offset: 0x30) Interrupt Mask Register */
  RwReg   EMAC_MAN;     /**< \brief (Emac Offset: 0x34) Phy Maintenance Register */
  RwReg   EMAC_PTR;     /**< \brief (Emac Offset: 0x38) Pause Time Register */
  RwReg   EMAC_PFR;     /**< \brief (Emac Offset: 0x3C) Pause Frames Received Register */
  RwReg   EMAC_FTO;     /**< \brief (Emac Offset: 0x40) Frames Transmitted Ok Register */
  RwReg   EMAC_SCF;     /**< \brief (Emac Offset: 0x44) Single Collision Frames Register */
  RwReg   EMAC_MCF;     /**< \brief (Emac Offset: 0x48) Multiple Collision Frames Register */
  RwReg   EMAC_FRO;     /**< \brief (Emac Offset: 0x4C) Frames Received Ok Register */
  RwReg   EMAC_FCSE;    /**< \brief (Emac Offset: 0x50) Frame Check Sequence Errors Register */
  RwReg   EMAC_ALE;     /**< \brief (Emac Offset: 0x54) Alignment Errors Register */
  RwReg   EMAC_DTF;     /**< \brief (Emac Offset: 0x58) Deferred Transmission Frames Register */
  RwReg   EMAC_LCOL;    /**< \brief (Emac Offset: 0x5C) Late Collisions Register */
  RwReg   EMAC_ECOL;    /**< \brief (Emac Offset: 0x60) Excessive Collisions Register */
  RwReg   EMAC_TUND;    /**< \brief (Emac Offset: 0x64) Transmit Underrun Errors Register */
  RwReg   EMAC_CSE;     /**< \brief (Emac Offset: 0x68) Carrier Sense Errors Register */
  RwReg   EMAC_RRE;     /**< \brief (Emac Offset: 0x6C) Receive Resource Errors Register */
  RwReg   EMAC_ROV;     /**< \brief (Emac Offset: 0x70) Receive Overrun Errors Register */
  RwReg   EMAC_RSE;     /**< \brief (Emac Offset: 0x74) Receive Symbol Errors Register */
  RwReg   EMAC_ELE;     /**< \brief (Emac Offset: 0x78) Excessive Length Errors Register */
  RwReg   EMAC_RJA;     /**< \brief (Emac Offset: 0x7C) Receive Jabbers Register */
  RwReg   EMAC_USF;     /**< \brief (Emac Offset: 0x80) Undersize Frames Register */
  RwReg   EMAC_STE;     /**< \brief (Emac Offset: 0x84) SQE Test Errors Register */
  RwReg   EMAC_RLE;     /**< \brief (Emac Offset: 0x88) Received Length Field Mismatch Register */
  RoReg   Reserved2[1];
  RwReg   EMAC_HRB;     /**< \brief (Emac Offset: 0x90) Hash Register Bottom [31:0] Register */
  RwReg   EMAC_HRT;     /**< \brief (Emac Offset: 0x94) Hash Register Top [63:32] Register */
  EmacSa  EMAC_SA[EMACSA_NUMBER]; /**< \brief (Emac Offset: 0x98) sa = 1 .. 4 */
  RwReg   EMAC_TID;     /**< \brief (Emac Offset: 0xB8) Type ID Checking Register */
  RoReg   Reserved3[1];
  RwReg   EMAC_USRIO;   /**< \brief (Emac Offset: 0xC0) User Input/Output Register */
} Emac;
/* -------- EMAC_NCR : (EMAC Offset: 0x00) Network Control Register -------- */
#define EMAC_NCR_LB (0x1u << 0) /**< \brief (EMAC_NCR) LoopBack */
#define EMAC_NCR_LLB (0x1u << 1) /**< \brief (EMAC_NCR) Loopback local */
#define EMAC_NCR_RE (0x1u << 2) /**< \brief (EMAC_NCR) Receive enable */
#define EMAC_NCR_TE (0x1u << 3) /**< \brief (EMAC_NCR) Transmit enable */
#define EMAC_NCR_MPE (0x1u << 4) /**< \brief (EMAC_NCR) Management port enable */
#define EMAC_NCR_CLRSTAT (0x1u << 5) /**< \brief (EMAC_NCR) Clear statistics registers */
#define EMAC_NCR_INCSTAT (0x1u << 6) /**< \brief (EMAC_NCR) Increment statistics registers */
#define EMAC_NCR_WESTAT (0x1u << 7) /**< \brief (EMAC_NCR) Write enable for statistics registers */
#define EMAC_NCR_BP (0x1u << 8) /**< \brief (EMAC_NCR) Back pressure */
#define EMAC_NCR_TSTART (0x1u << 9) /**< \brief (EMAC_NCR) Start transmission */
#define EMAC_NCR_THALT (0x1u << 10) /**< \brief (EMAC_NCR) Transmit halt */
/* -------- EMAC_NCFGR : (EMAC Offset: 0x04) Network Configuration Register -------- */
#define EMAC_NCFGR_SPD (0x1u << 0) /**< \brief (EMAC_NCFGR) Speed */
#define EMAC_NCFGR_FD (0x1u << 1) /**< \brief (EMAC_NCFGR) Full Duplex */
#define EMAC_NCFGR_JFRAME (0x1u << 3) /**< \brief (EMAC_NCFGR) Jumbo Frames */
#define EMAC_NCFGR_CAF (0x1u << 4) /**< \brief (EMAC_NCFGR) Copy All Frames */
#define EMAC_NCFGR_NBC (0x1u << 5) /**< \brief (EMAC_NCFGR) No Broadcast */
#define EMAC_NCFGR_MTI (0x1u << 6) /**< \brief (EMAC_NCFGR) Multicast Hash Enable */
#define EMAC_NCFGR_UNI (0x1u << 7) /**< \brief (EMAC_NCFGR) Unicast Hash Enable */
#define EMAC_NCFGR_BIG (0x1u << 8) /**< \brief (EMAC_NCFGR) Receive 1536 bytes frames */
#define EMAC_NCFGR_CLK_Pos 10
#define EMAC_NCFGR_CLK_Msk (0x3u << EMAC_NCFGR_CLK_Pos) /**< \brief (EMAC_NCFGR) MDC clock divider */
#define   EMAC_NCFGR_CLK_MCK_8 (0x0u << 10) /**< \brief (EMAC_NCFGR) MCK divided by 8 (MCK up to 20 MHz). */
#define   EMAC_NCFGR_CLK_MCK_16 (0x1u << 10) /**< \brief (EMAC_NCFGR) MCK divided by 16 (MCK up to 40 MHz). */
#define   EMAC_NCFGR_CLK_MCK_32 (0x2u << 10) /**< \brief (EMAC_NCFGR) MCK divided by 32 (MCK up to 80 MHz). */
#define   EMAC_NCFGR_CLK_MCK_64 (0x3u << 10) /**< \brief (EMAC_NCFGR) MCK divided by 64 (MCK up to 160 MHz). */
#define EMAC_NCFGR_RTY (0x1u << 12) /**< \brief (EMAC_NCFGR) Retry test */
#define EMAC_NCFGR_PAE (0x1u << 13) /**< \brief (EMAC_NCFGR) Pause Enable */
#define EMAC_NCFGR_RBOF_Pos 14
#define EMAC_NCFGR_RBOF_Msk (0x3u << EMAC_NCFGR_RBOF_Pos) /**< \brief (EMAC_NCFGR) Receive Buffer Offset */
#define   EMAC_NCFGR_RBOF_OFFSET_0 (0x0u << 14) /**< \brief (EMAC_NCFGR) No offset from start of receive buffer. */
#define   EMAC_NCFGR_RBOF_OFFSET_1 (0x1u << 14) /**< \brief (EMAC_NCFGR) One-byte offset from start of receive buffer. */
#define   EMAC_NCFGR_RBOF_OFFSET_2 (0x2u << 14) /**< \brief (EMAC_NCFGR) Two-byte offset from start of receive buffer. */
#define   EMAC_NCFGR_RBOF_OFFSET_3 (0x3u << 14) /**< \brief (EMAC_NCFGR) Three-byte offset from start of receive buffer. */
#define EMAC_NCFGR_RLCE (0x1u << 16) /**< \brief (EMAC_NCFGR) Receive Length field Checking Enable */
#define EMAC_NCFGR_DRFCS (0x1u << 17) /**< \brief (EMAC_NCFGR) Discard Receive FCS */
#define EMAC_NCFGR_EFRHD (0x1u << 18) /**< \brief (EMAC_NCFGR)  */
#define EMAC_NCFGR_IRXFCS (0x1u << 19) /**< \brief (EMAC_NCFGR) Ignore RX FCS */
/* -------- EMAC_NSR : (EMAC Offset: 0x08) Network Status Register -------- */
#define EMAC_NSR_MDIO (0x1u << 1) /**< \brief (EMAC_NSR)  */
#define EMAC_NSR_IDLE (0x1u << 2) /**< \brief (EMAC_NSR)  */
/* -------- EMAC_TSR : (EMAC Offset: 0x14) Transmit Status Register -------- */
#define EMAC_TSR_UBR (0x1u << 0) /**< \brief (EMAC_TSR) Used Bit Read */
#define EMAC_TSR_COL (0x1u << 1) /**< \brief (EMAC_TSR) Collision Occurred */
#define EMAC_TSR_RLES (0x1u << 2) /**< \brief (EMAC_TSR) Retry Limit exceeded */
#define EMAC_TSR_TGO (0x1u << 3) /**< \brief (EMAC_TSR) Transmit Go */
#define EMAC_TSR_BEX (0x1u << 4) /**< \brief (EMAC_TSR) Buffers exhausted mid frame */
#define EMAC_TSR_COMP (0x1u << 5) /**< \brief (EMAC_TSR) Transmit Complete */
#define EMAC_TSR_UND (0x1u << 6) /**< \brief (EMAC_TSR) Transmit Underrun */
/* -------- EMAC_RBQP : (EMAC Offset: 0x18) Receive Buffer Queue Pointer Register -------- */
#define EMAC_RBQP_ADDR_Pos 2
#define EMAC_RBQP_ADDR_Msk (0x3fffffffu << EMAC_RBQP_ADDR_Pos) /**< \brief (EMAC_RBQP) Receive buffer queue pointer address */
#define EMAC_RBQP_ADDR(value) ((EMAC_RBQP_ADDR_Msk & ((value) << EMAC_RBQP_ADDR_Pos)))
/* -------- EMAC_TBQP : (EMAC Offset: 0x1C) Transmit Buffer Queue Pointer Register -------- */
#define EMAC_TBQP_ADDR_Pos 2
#define EMAC_TBQP_ADDR_Msk (0x3fffffffu << EMAC_TBQP_ADDR_Pos) /**< \brief (EMAC_TBQP) Transmit buffer queue pointer address */
#define EMAC_TBQP_ADDR(value) ((EMAC_TBQP_ADDR_Msk & ((value) << EMAC_TBQP_ADDR_Pos)))
/* -------- EMAC_RSR : (EMAC Offset: 0x20) Receive Status Register -------- */
#define EMAC_RSR_BNA (0x1u << 0) /**< \brief (EMAC_RSR) Buffer Not Available */
#define EMAC_RSR_REC (0x1u << 1) /**< \brief (EMAC_RSR) Frame Received */
#define EMAC_RSR_OVR (0x1u << 2) /**< \brief (EMAC_RSR) Receive Overrun */
/* -------- EMAC_ISR : (EMAC Offset: 0x24) Interrupt Status Register -------- */
#define EMAC_ISR_MFD (0x1u << 0) /**< \brief (EMAC_ISR) Management Frame Done */
#define EMAC_ISR_RCOMP (0x1u << 1) /**< \brief (EMAC_ISR) Receive Complete */
#define EMAC_ISR_RXUBR (0x1u << 2) /**< \brief (EMAC_ISR) Receive Used Bit Read */
#define EMAC_ISR_TXUBR (0x1u << 3) /**< \brief (EMAC_ISR) Transmit Used Bit Read */
#define EMAC_ISR_TUND (0x1u << 4) /**< \brief (EMAC_ISR) Ethernet Transmit Buffer Underrun */
#define EMAC_ISR_RLEX (0x1u << 5) /**< \brief (EMAC_ISR) Retry Limit Exceeded */
#define EMAC_ISR_TXERR (0x1u << 6) /**< \brief (EMAC_ISR) Transmit Error */
#define EMAC_ISR_TCOMP (0x1u << 7) /**< \brief (EMAC_ISR) Transmit Complete */
#define EMAC_ISR_ROVR (0x1u << 10) /**< \brief (EMAC_ISR) Receive Overrun */
#define EMAC_ISR_HRESP (0x1u << 11) /**< \brief (EMAC_ISR) Hresp not OK */
#define EMAC_ISR_PFRE (0x1u << 12) /**< \brief (EMAC_ISR) Pause Frame Received */
#define EMAC_ISR_PTZ (0x1u << 13) /**< \brief (EMAC_ISR) Pause Time Zero */
/* -------- EMAC_IER : (EMAC Offset: 0x28) Interrupt Enable Register -------- */
#define EMAC_IER_MFD (0x1u << 0) /**< \brief (EMAC_IER) Management Frame sent */
#define EMAC_IER_RCOMP (0x1u << 1) /**< \brief (EMAC_IER) Receive Complete */
#define EMAC_IER_RXUBR (0x1u << 2) /**< \brief (EMAC_IER) Receive Used Bit Read */
#define EMAC_IER_TXUBR (0x1u << 3) /**< \brief (EMAC_IER) Transmit Used Bit Read */
#define EMAC_IER_TUND (0x1u << 4) /**< \brief (EMAC_IER) Ethernet Transmit Buffer Underrun */
#define EMAC_IER_RLE (0x1u << 5) /**< \brief (EMAC_IER) Retry Limit Exceeded */
#define EMAC_IER_TXERR (0x1u << 6) /**< \brief (EMAC_IER)  */
#define EMAC_IER_TCOMP (0x1u << 7) /**< \brief (EMAC_IER) Transmit Complete */
#define EMAC_IER_ROVR (0x1u << 10) /**< \brief (EMAC_IER) Receive Overrun */
#define EMAC_IER_HRESP (0x1u << 11) /**< \brief (EMAC_IER) Hresp not OK */
#define EMAC_IER_PFR (0x1u << 12) /**< \brief (EMAC_IER) Pause Frame Received */
#define EMAC_IER_PTZ (0x1u << 13) /**< \brief (EMAC_IER) Pause Time Zero */
/* -------- EMAC_IDR : (EMAC Offset: 0x2C) Interrupt Disable Register -------- */
#define EMAC_IDR_MFD (0x1u << 0) /**< \brief (EMAC_IDR) Management Frame sent */
#define EMAC_IDR_RCOMP (0x1u << 1) /**< \brief (EMAC_IDR) Receive Complete */
#define EMAC_IDR_RXUBR (0x1u << 2) /**< \brief (EMAC_IDR) Receive Used Bit Read */
#define EMAC_IDR_TXUBR (0x1u << 3) /**< \brief (EMAC_IDR) Transmit Used Bit Read */
#define EMAC_IDR_TUND (0x1u << 4) /**< \brief (EMAC_IDR) Ethernet Transmit Buffer Underrun */
#define EMAC_IDR_RLE (0x1u << 5) /**< \brief (EMAC_IDR) Retry Limit Exceeded */
#define EMAC_IDR_TXERR (0x1u << 6) /**< \brief (EMAC_IDR)  */
#define EMAC_IDR_TCOMP (0x1u << 7) /**< \brief (EMAC_IDR) Transmit Complete */
#define EMAC_IDR_ROVR (0x1u << 10) /**< \brief (EMAC_IDR) Receive Overrun */
#define EMAC_IDR_HRESP (0x1u << 11) /**< \brief (EMAC_IDR) Hresp not OK */
#define EMAC_IDR_PFR (0x1u << 12) /**< \brief (EMAC_IDR) Pause Frame Received */
#define EMAC_IDR_PTZ (0x1u << 13) /**< \brief (EMAC_IDR) Pause Time Zero */
/* -------- EMAC_IMR : (EMAC Offset: 0x30) Interrupt Mask Register -------- */
#define EMAC_IMR_MFD (0x1u << 0) /**< \brief (EMAC_IMR) Management Frame sent */
#define EMAC_IMR_RCOMP (0x1u << 1) /**< \brief (EMAC_IMR) Receive Complete */
#define EMAC_IMR_RXUBR (0x1u << 2) /**< \brief (EMAC_IMR) Receive Used Bit Read */
#define EMAC_IMR_TXUBR (0x1u << 3) /**< \brief (EMAC_IMR) Transmit Used Bit Read */
#define EMAC_IMR_TUND (0x1u << 4) /**< \brief (EMAC_IMR) Ethernet Transmit Buffer Underrun */
#define EMAC_IMR_RLE (0x1u << 5) /**< \brief (EMAC_IMR) Retry Limit Exceeded */
#define EMAC_IMR_TXERR (0x1u << 6) /**< \brief (EMAC_IMR)  */
#define EMAC_IMR_TCOMP (0x1u << 7) /**< \brief (EMAC_IMR) Transmit Complete */
#define EMAC_IMR_ROVR (0x1u << 10) /**< \brief (EMAC_IMR) Receive Overrun */
#define EMAC_IMR_HRESP (0x1u << 11) /**< \brief (EMAC_IMR) Hresp not OK */
#define EMAC_IMR_PFR (0x1u << 12) /**< \brief (EMAC_IMR) Pause Frame Received */
#define EMAC_IMR_PTZ (0x1u << 13) /**< \brief (EMAC_IMR) Pause Time Zero */
/* -------- EMAC_MAN : (EMAC Offset: 0x34) Phy Maintenance Register -------- */
#define EMAC_MAN_DATA_Pos 0
#define EMAC_MAN_DATA_Msk (0xffffu << EMAC_MAN_DATA_Pos) /**< \brief (EMAC_MAN)  */
#define EMAC_MAN_DATA(value) ((EMAC_MAN_DATA_Msk & ((value) << EMAC_MAN_DATA_Pos)))
#define EMAC_MAN_CODE_Pos 16
#define EMAC_MAN_CODE_Msk (0x3u << EMAC_MAN_CODE_Pos) /**< \brief (EMAC_MAN)  */
#define EMAC_MAN_CODE(value) ((EMAC_MAN_CODE_Msk & ((value) << EMAC_MAN_CODE_Pos)))
#define EMAC_MAN_REGA_Pos 18
#define EMAC_MAN_REGA_Msk (0x1fu << EMAC_MAN_REGA_Pos) /**< \brief (EMAC_MAN) Register Address */
#define EMAC_MAN_REGA(value) ((EMAC_MAN_REGA_Msk & ((value) << EMAC_MAN_REGA_Pos)))
#define EMAC_MAN_PHYA_Pos 23
#define EMAC_MAN_PHYA_Msk (0x1fu << EMAC_MAN_PHYA_Pos) /**< \brief (EMAC_MAN) PHY Address */
#define EMAC_MAN_PHYA(value) ((EMAC_MAN_PHYA_Msk & ((value) << EMAC_MAN_PHYA_Pos)))
#define EMAC_MAN_RW_Pos 28
#define EMAC_MAN_RW_Msk (0x3u << EMAC_MAN_RW_Pos) /**< \brief (EMAC_MAN) Read-write */
#define EMAC_MAN_RW(value) ((EMAC_MAN_RW_Msk & ((value) << EMAC_MAN_RW_Pos)))
#define EMAC_MAN_SOF_Pos 30
#define EMAC_MAN_SOF_Msk (0x3u << EMAC_MAN_SOF_Pos) /**< \brief (EMAC_MAN) Start of frame */
#define EMAC_MAN_SOF(value) ((EMAC_MAN_SOF_Msk & ((value) << EMAC_MAN_SOF_Pos)))
/* -------- EMAC_PTR : (EMAC Offset: 0x38) Pause Time Register -------- */
#define EMAC_PTR_PTIME_Pos 0
#define EMAC_PTR_PTIME_Msk (0xffffu << EMAC_PTR_PTIME_Pos) /**< \brief (EMAC_PTR) Pause Time */
#define EMAC_PTR_PTIME(value) ((EMAC_PTR_PTIME_Msk & ((value) << EMAC_PTR_PTIME_Pos)))
/* -------- EMAC_PFR : (EMAC Offset: 0x3C) Pause Frames Received Register -------- */
#define EMAC_PFR_FROK_Pos 0
#define EMAC_PFR_FROK_Msk (0xffffu << EMAC_PFR_FROK_Pos) /**< \brief (EMAC_PFR) Pause Frames received OK */
#define EMAC_PFR_FROK(value) ((EMAC_PFR_FROK_Msk & ((value) << EMAC_PFR_FROK_Pos)))
/* -------- EMAC_FTO : (EMAC Offset: 0x40) Frames Transmitted Ok Register -------- */
#define EMAC_FTO_FTOK_Pos 0
#define EMAC_FTO_FTOK_Msk (0xffffffu << EMAC_FTO_FTOK_Pos) /**< \brief (EMAC_FTO) Frames Transmitted OK */
#define EMAC_FTO_FTOK(value) ((EMAC_FTO_FTOK_Msk & ((value) << EMAC_FTO_FTOK_Pos)))
/* -------- EMAC_SCF : (EMAC Offset: 0x44) Single Collision Frames Register -------- */
#define EMAC_SCF_SCF_Pos 0
#define EMAC_SCF_SCF_Msk (0xffffu << EMAC_SCF_SCF_Pos) /**< \brief (EMAC_SCF) Single Collision Frames */
#define EMAC_SCF_SCF(value) ((EMAC_SCF_SCF_Msk & ((value) << EMAC_SCF_SCF_Pos)))
/* -------- EMAC_MCF : (EMAC Offset: 0x48) Multiple Collision Frames Register -------- */
#define EMAC_MCF_MCF_Pos 0
#define EMAC_MCF_MCF_Msk (0xffffu << EMAC_MCF_MCF_Pos) /**< \brief (EMAC_MCF) Multicollision Frames */
#define EMAC_MCF_MCF(value) ((EMAC_MCF_MCF_Msk & ((value) << EMAC_MCF_MCF_Pos)))
/* -------- EMAC_FRO : (EMAC Offset: 0x4C) Frames Received Ok Register -------- */
#define EMAC_FRO_FROK_Pos 0
#define EMAC_FRO_FROK_Msk (0xffffffu << EMAC_FRO_FROK_Pos) /**< \brief (EMAC_FRO) Frames Received OK */
#define EMAC_FRO_FROK(value) ((EMAC_FRO_FROK_Msk & ((value) << EMAC_FRO_FROK_Pos)))
/* -------- EMAC_FCSE : (EMAC Offset: 0x50) Frame Check Sequence Errors Register -------- */
#define EMAC_FCSE_FCSE_Pos 0
#define EMAC_FCSE_FCSE_Msk (0xffu << EMAC_FCSE_FCSE_Pos) /**< \brief (EMAC_FCSE) Frame Check Sequence Errors */
#define EMAC_FCSE_FCSE(value) ((EMAC_FCSE_FCSE_Msk & ((value) << EMAC_FCSE_FCSE_Pos)))
/* -------- EMAC_ALE : (EMAC Offset: 0x54) Alignment Errors Register -------- */
#define EMAC_ALE_ALE_Pos 0
#define EMAC_ALE_ALE_Msk (0xffu << EMAC_ALE_ALE_Pos) /**< \brief (EMAC_ALE) Alignment Errors */
#define EMAC_ALE_ALE(value) ((EMAC_ALE_ALE_Msk & ((value) << EMAC_ALE_ALE_Pos)))
/* -------- EMAC_DTF : (EMAC Offset: 0x58) Deferred Transmission Frames Register -------- */
#define EMAC_DTF_DTF_Pos 0
#define EMAC_DTF_DTF_Msk (0xffffu << EMAC_DTF_DTF_Pos) /**< \brief (EMAC_DTF) Deferred Transmission Frames */
#define EMAC_DTF_DTF(value) ((EMAC_DTF_DTF_Msk & ((value) << EMAC_DTF_DTF_Pos)))
/* -------- EMAC_LCOL : (EMAC Offset: 0x5C) Late Collisions Register -------- */
#define EMAC_LCOL_LCOL_Pos 0
#define EMAC_LCOL_LCOL_Msk (0xffu << EMAC_LCOL_LCOL_Pos) /**< \brief (EMAC_LCOL) Late Collisions */
#define EMAC_LCOL_LCOL(value) ((EMAC_LCOL_LCOL_Msk & ((value) << EMAC_LCOL_LCOL_Pos)))
/* -------- EMAC_ECOL : (EMAC Offset: 0x60) Excessive Collisions Register -------- */
#define EMAC_ECOL_EXCOL_Pos 0
#define EMAC_ECOL_EXCOL_Msk (0xffu << EMAC_ECOL_EXCOL_Pos) /**< \brief (EMAC_ECOL) Excessive Collisions */
#define EMAC_ECOL_EXCOL(value) ((EMAC_ECOL_EXCOL_Msk & ((value) << EMAC_ECOL_EXCOL_Pos)))
/* -------- EMAC_TUND : (EMAC Offset: 0x64) Transmit Underrun Errors Register -------- */
#define EMAC_TUND_TUND_Pos 0
#define EMAC_TUND_TUND_Msk (0xffu << EMAC_TUND_TUND_Pos) /**< \brief (EMAC_TUND) Transmit Underruns */
#define EMAC_TUND_TUND(value) ((EMAC_TUND_TUND_Msk & ((value) << EMAC_TUND_TUND_Pos)))
/* -------- EMAC_CSE : (EMAC Offset: 0x68) Carrier Sense Errors Register -------- */
#define EMAC_CSE_CSE_Pos 0
#define EMAC_CSE_CSE_Msk (0xffu << EMAC_CSE_CSE_Pos) /**< \brief (EMAC_CSE) Carrier Sense Errors */
#define EMAC_CSE_CSE(value) ((EMAC_CSE_CSE_Msk & ((value) << EMAC_CSE_CSE_Pos)))
/* -------- EMAC_RRE : (EMAC Offset: 0x6C) Receive Resource Errors Register -------- */
#define EMAC_RRE_RRE_Pos 0
#define EMAC_RRE_RRE_Msk (0xffffu << EMAC_RRE_RRE_Pos) /**< \brief (EMAC_RRE) Receive Resource Errors */
#define EMAC_RRE_RRE(value) ((EMAC_RRE_RRE_Msk & ((value) << EMAC_RRE_RRE_Pos)))
/* -------- EMAC_ROV : (EMAC Offset: 0x70) Receive Overrun Errors Register -------- */
#define EMAC_ROV_ROVR_Pos 0
#define EMAC_ROV_ROVR_Msk (0xffu << EMAC_ROV_ROVR_Pos) /**< \brief (EMAC_ROV) Receive Overrun */
#define EMAC_ROV_ROVR(value) ((EMAC_ROV_ROVR_Msk & ((value) << EMAC_ROV_ROVR_Pos)))
/* -------- EMAC_RSE : (EMAC Offset: 0x74) Receive Symbol Errors Register -------- */
#define EMAC_RSE_RSE_Pos 0
#define EMAC_RSE_RSE_Msk (0xffu << EMAC_RSE_RSE_Pos) /**< \brief (EMAC_RSE) Receive Symbol Errors */
#define EMAC_RSE_RSE(value) ((EMAC_RSE_RSE_Msk & ((value) << EMAC_RSE_RSE_Pos)))
/* -------- EMAC_ELE : (EMAC Offset: 0x78) Excessive Length Errors Register -------- */
#define EMAC_ELE_EXL_Pos 0
#define EMAC_ELE_EXL_Msk (0xffu << EMAC_ELE_EXL_Pos) /**< \brief (EMAC_ELE) Excessive Length Errors */
#define EMAC_ELE_EXL(value) ((EMAC_ELE_EXL_Msk & ((value) << EMAC_ELE_EXL_Pos)))
/* -------- EMAC_RJA : (EMAC Offset: 0x7C) Receive Jabbers Register -------- */
#define EMAC_RJA_RJB_Pos 0
#define EMAC_RJA_RJB_Msk (0xffu << EMAC_RJA_RJB_Pos) /**< \brief (EMAC_RJA) Receive Jabbers */
#define EMAC_RJA_RJB(value) ((EMAC_RJA_RJB_Msk & ((value) << EMAC_RJA_RJB_Pos)))
/* -------- EMAC_USF : (EMAC Offset: 0x80) Undersize Frames Register -------- */
#define EMAC_USF_USF_Pos 0
#define EMAC_USF_USF_Msk (0xffu << EMAC_USF_USF_Pos) /**< \brief (EMAC_USF) Undersize frames */
#define EMAC_USF_USF(value) ((EMAC_USF_USF_Msk & ((value) << EMAC_USF_USF_Pos)))
/* -------- EMAC_STE : (EMAC Offset: 0x84) SQE Test Errors Register -------- */
#define EMAC_STE_SQER_Pos 0
#define EMAC_STE_SQER_Msk (0xffu << EMAC_STE_SQER_Pos) /**< \brief (EMAC_STE) SQE test errors */
#define EMAC_STE_SQER(value) ((EMAC_STE_SQER_Msk & ((value) << EMAC_STE_SQER_Pos)))
/* -------- EMAC_RLE : (EMAC Offset: 0x88) Received Length Field Mismatch Register -------- */
#define EMAC_RLE_RLFM_Pos 0
#define EMAC_RLE_RLFM_Msk (0xffu << EMAC_RLE_RLFM_Pos) /**< \brief (EMAC_RLE) Receive Length Field Mismatch */
#define EMAC_RLE_RLFM(value) ((EMAC_RLE_RLFM_Msk & ((value) << EMAC_RLE_RLFM_Pos)))
/* -------- EMAC_HRB : (EMAC Offset: 0x90) Hash Register Bottom [31:0] Register -------- */
#define EMAC_HRB_ADDR_Pos 0
#define EMAC_HRB_ADDR_Msk (0xffffffffu << EMAC_HRB_ADDR_Pos) /**< \brief (EMAC_HRB)  */
#define EMAC_HRB_ADDR(value) ((EMAC_HRB_ADDR_Msk & ((value) << EMAC_HRB_ADDR_Pos)))
/* -------- EMAC_HRT : (EMAC Offset: 0x94) Hash Register Top [63:32] Register -------- */
#define EMAC_HRT_ADDR_Pos 0
#define EMAC_HRT_ADDR_Msk (0xffffffffu << EMAC_HRT_ADDR_Pos) /**< \brief (EMAC_HRT)  */
#define EMAC_HRT_ADDR(value) ((EMAC_HRT_ADDR_Msk & ((value) << EMAC_HRT_ADDR_Pos)))
/* -------- EMAC_SAxB : (EMAC Offset: N/A) Specific Address 1 Bottom Register -------- */
#define EMAC_SAxB_ADDR_Pos 0
#define EMAC_SAxB_ADDR_Msk (0xffffffffu << EMAC_SAxB_ADDR_Pos) /**< \brief (EMAC_SAxB)  */
#define EMAC_SAxB_ADDR(value) ((EMAC_SAxB_ADDR_Msk & ((value) << EMAC_SAxB_ADDR_Pos)))
/* -------- EMAC_SAxT : (EMAC Offset: N/A) Specific Address 1 Top Register -------- */
#define EMAC_SAxT_ADDR_Pos 0
#define EMAC_SAxT_ADDR_Msk (0xffffu << EMAC_SAxT_ADDR_Pos) /**< \brief (EMAC_SAxT)  */
#define EMAC_SAxT_ADDR(value) ((EMAC_SAxT_ADDR_Msk & ((value) << EMAC_SAxT_ADDR_Pos)))
/* -------- EMAC_TID : (EMAC Offset: 0xB8) Type ID Checking Register -------- */
#define EMAC_TID_TID_Pos 0
#define EMAC_TID_TID_Msk (0xffffu << EMAC_TID_TID_Pos) /**< \brief (EMAC_TID) Type ID checking */
#define EMAC_TID_TID(value) ((EMAC_TID_TID_Msk & ((value) << EMAC_TID_TID_Pos)))
/* -------- EMAC_USRIO : (EMAC Offset: 0xC0) User Input/Output Register -------- */
#define EMAC_USRIO_RMII (0x1u << 0) /**< \brief (EMAC_USRIO) Reduce MII */
#define EMAC_USRIO_CLKEN (0x1u << 1) /**< \brief (EMAC_USRIO) Clock Enable */
/*@}*/ /* end of group SAM3XA_EMAC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR General Purpose Backup Register */
/* ============================================================================= */
/** \addtogroup SAM3XA_GPBR General Purpose Backup Register */
/*@{*/

/** \brief Gpbr hardware registers */
typedef struct {
  RwReg SYS_GPBR[8]; /**< \brief (Gpbr Offset: 0x0) General Purpose Backup Register */
} Gpbr;
/* -------- SYS_GPBR[8] : (GPBR Offset: 0x0) General Purpose Backup Register -------- */
#define SYS_GPBR_GPBR_VALUE_Pos 0
#define SYS_GPBR_GPBR_VALUE_Msk (0xffffffffu << SYS_GPBR_GPBR_VALUE_Pos) /**< \brief (SYS_GPBR[8]) Value of GPBR x */
#define SYS_GPBR_GPBR_VALUE(value) ((SYS_GPBR_GPBR_VALUE_Msk & ((value) << SYS_GPBR_GPBR_VALUE_Pos)))
/*@}*/ /* end of group SAM3XA_GPBR */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR High Speed MultiMedia Card Interface */
/* ============================================================================= */
/** \addtogroup SAM3XA_HSMCI High Speed MultiMedia Card Interface */
/*@{*/

/** \brief Hsmci hardware registers */
typedef struct {
  WoReg HSMCI_CR;        /**< \brief (Hsmci Offset: 0x00) Control Register */
  RwReg HSMCI_MR;        /**< \brief (Hsmci Offset: 0x04) Mode Register */
  RwReg HSMCI_DTOR;      /**< \brief (Hsmci Offset: 0x08) Data Timeout Register */
  RwReg HSMCI_SDCR;      /**< \brief (Hsmci Offset: 0x0C) SD/SDIO Card Register */
  RwReg HSMCI_ARGR;      /**< \brief (Hsmci Offset: 0x10) Argument Register */
  WoReg HSMCI_CMDR;      /**< \brief (Hsmci Offset: 0x14) Command Register */
  RwReg HSMCI_BLKR;      /**< \brief (Hsmci Offset: 0x18) Block Register */
  RwReg HSMCI_CSTOR;     /**< \brief (Hsmci Offset: 0x1C) Completion Signal Timeout Register */
  RoReg HSMCI_RSPR[4];   /**< \brief (Hsmci Offset: 0x20) Response Register */
  RoReg HSMCI_RDR;       /**< \brief (Hsmci Offset: 0x30) Receive Data Register */
  WoReg HSMCI_TDR;       /**< \brief (Hsmci Offset: 0x34) Transmit Data Register */
  RoReg Reserved1[2];
  RoReg HSMCI_SR;        /**< \brief (Hsmci Offset: 0x40) Status Register */
  WoReg HSMCI_IER;       /**< \brief (Hsmci Offset: 0x44) Interrupt Enable Register */
  WoReg HSMCI_IDR;       /**< \brief (Hsmci Offset: 0x48) Interrupt Disable Register */
  RoReg HSMCI_IMR;       /**< \brief (Hsmci Offset: 0x4C) Interrupt Mask Register */
  RwReg HSMCI_DMA;       /**< \brief (Hsmci Offset: 0x50) DMA Configuration Register */
  RwReg HSMCI_CFG;       /**< \brief (Hsmci Offset: 0x54) Configuration Register */
  RoReg Reserved2[35];
  RwReg HSMCI_WPMR;      /**< \brief (Hsmci Offset: 0xE4) Write Protection Mode Register */
  RoReg HSMCI_WPSR;      /**< \brief (Hsmci Offset: 0xE8) Write Protection Status Register */
  RoReg Reserved3[69];
  RwReg HSMCI_FIFO[256]; /**< \brief (Hsmci Offset: 0x200) FIFO Memory Aperture0 */
} Hsmci;
/* -------- HSMCI_CR : (HSMCI Offset: 0x00) Control Register -------- */
#define HSMCI_CR_MCIEN (0x1u << 0) /**< \brief (HSMCI_CR) Multi-Media Interface Enable */
#define HSMCI_CR_MCIDIS (0x1u << 1) /**< \brief (HSMCI_CR) Multi-Media Interface Disable */
#define HSMCI_CR_PWSEN (0x1u << 2) /**< \brief (HSMCI_CR) Power Save Mode Enable */
#define HSMCI_CR_PWSDIS (0x1u << 3) /**< \brief (HSMCI_CR) Power Save Mode Disable */
#define HSMCI_CR_SWRST (0x1u << 7) /**< \brief (HSMCI_CR) Software Reset */
/* -------- HSMCI_MR : (HSMCI Offset: 0x04) Mode Register -------- */
#define HSMCI_MR_CLKDIV_Pos 0
#define HSMCI_MR_CLKDIV_Msk (0xffu << HSMCI_MR_CLKDIV_Pos) /**< \brief (HSMCI_MR) Clock Divider */
#define HSMCI_MR_CLKDIV(value) ((HSMCI_MR_CLKDIV_Msk & ((value) << HSMCI_MR_CLKDIV_Pos)))
#define HSMCI_MR_PWSDIV_Pos 8
#define HSMCI_MR_PWSDIV_Msk (0x7u << HSMCI_MR_PWSDIV_Pos) /**< \brief (HSMCI_MR) Power Saving Divider */
#define HSMCI_MR_PWSDIV(value) ((HSMCI_MR_PWSDIV_Msk & ((value) << HSMCI_MR_PWSDIV_Pos)))
#define HSMCI_MR_RDPROOF (0x1u << 11) /**< \brief (HSMCI_MR)  */
#define HSMCI_MR_WRPROOF (0x1u << 12) /**< \brief (HSMCI_MR)  */
#define HSMCI_MR_FBYTE (0x1u << 13) /**< \brief (HSMCI_MR) Force Byte Transfer */
#define HSMCI_MR_PADV (0x1u << 14) /**< \brief (HSMCI_MR) Padding Value */
/* -------- HSMCI_DTOR : (HSMCI Offset: 0x08) Data Timeout Register -------- */
#define HSMCI_DTOR_DTOCYC_Pos 0
#define HSMCI_DTOR_DTOCYC_Msk (0xfu << HSMCI_DTOR_DTOCYC_Pos) /**< \brief (HSMCI_DTOR) Data Timeout Cycle Number */
#define HSMCI_DTOR_DTOCYC(value) ((HSMCI_DTOR_DTOCYC_Msk & ((value) << HSMCI_DTOR_DTOCYC_Pos)))
#define HSMCI_DTOR_DTOMUL_Pos 4
#define HSMCI_DTOR_DTOMUL_Msk (0x7u << HSMCI_DTOR_DTOMUL_Pos) /**< \brief (HSMCI_DTOR) Data Timeout Multiplier */
#define   HSMCI_DTOR_DTOMUL_1 (0x0u << 4) /**< \brief (HSMCI_DTOR) DTOCYC */
#define   HSMCI_DTOR_DTOMUL_16 (0x1u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 16 */
#define   HSMCI_DTOR_DTOMUL_128 (0x2u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 128 */
#define   HSMCI_DTOR_DTOMUL_256 (0x3u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 256 */
#define   HSMCI_DTOR_DTOMUL_1024 (0x4u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 1024 */
#define   HSMCI_DTOR_DTOMUL_4096 (0x5u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 4096 */
#define   HSMCI_DTOR_DTOMUL_65536 (0x6u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 65536 */
#define   HSMCI_DTOR_DTOMUL_1048576 (0x7u << 4) /**< \brief (HSMCI_DTOR) DTOCYC x 1048576 */
/* -------- HSMCI_SDCR : (HSMCI Offset: 0x0C) SD/SDIO Card Register -------- */
#define HSMCI_SDCR_SDCSEL_Pos 0
#define HSMCI_SDCR_SDCSEL_Msk (0x3u << HSMCI_SDCR_SDCSEL_Pos) /**< \brief (HSMCI_SDCR) SDCard/SDIO Slot */
#define   HSMCI_SDCR_SDCSEL_SLOTA (0x0u << 0) /**< \brief (HSMCI_SDCR) Slot A is selected. */
#define   HSMCI_SDCR_SDCSEL_SLOTB (0x1u << 0) /**< \brief (HSMCI_SDCR) SDCARD/SDIO Slot B selected */
#define   HSMCI_SDCR_SDCSEL_SLOTC (0x2u << 0) /**< \brief (HSMCI_SDCR) - */
#define   HSMCI_SDCR_SDCSEL_SLOTD (0x3u << 0) /**< \brief (HSMCI_SDCR) - */
#define HSMCI_SDCR_SDCBUS_Pos 6
#define HSMCI_SDCR_SDCBUS_Msk (0x3u << HSMCI_SDCR_SDCBUS_Pos) /**< \brief (HSMCI_SDCR) SDCard/SDIO Bus Width */
#define   HSMCI_SDCR_SDCBUS_1 (0x0u << 6) /**< \brief (HSMCI_SDCR) 1 bit */
#define   HSMCI_SDCR_SDCBUS_4 (0x2u << 6) /**< \brief (HSMCI_SDCR) 4 bit */
#define   HSMCI_SDCR_SDCBUS_8 (0x3u << 6) /**< \brief (HSMCI_SDCR) 8 bit */
/* -------- HSMCI_ARGR : (HSMCI Offset: 0x10) Argument Register -------- */
#define HSMCI_ARGR_ARG_Pos 0
#define HSMCI_ARGR_ARG_Msk (0xffffffffu << HSMCI_ARGR_ARG_Pos) /**< \brief (HSMCI_ARGR) Command Argument */
#define HSMCI_ARGR_ARG(value) ((HSMCI_ARGR_ARG_Msk & ((value) << HSMCI_ARGR_ARG_Pos)))
/* -------- HSMCI_CMDR : (HSMCI Offset: 0x14) Command Register -------- */
#define HSMCI_CMDR_CMDNB_Pos 0
#define HSMCI_CMDR_CMDNB_Msk (0x3fu << HSMCI_CMDR_CMDNB_Pos) /**< \brief (HSMCI_CMDR) Command Number */
#define HSMCI_CMDR_CMDNB(value) ((HSMCI_CMDR_CMDNB_Msk & ((value) << HSMCI_CMDR_CMDNB_Pos)))
#define HSMCI_CMDR_RSPTYP_Pos 6
#define HSMCI_CMDR_RSPTYP_Msk (0x3u << HSMCI_CMDR_RSPTYP_Pos) /**< \brief (HSMCI_CMDR) Response Type */
#define   HSMCI_CMDR_RSPTYP_NORESP (0x0u << 6) /**< \brief (HSMCI_CMDR) No response. */
#define   HSMCI_CMDR_RSPTYP_48_BIT (0x1u << 6) /**< \brief (HSMCI_CMDR) 48-bit response. */
#define   HSMCI_CMDR_RSPTYP_136_BIT (0x2u << 6) /**< \brief (HSMCI_CMDR) 136-bit response. */
#define   HSMCI_CMDR_RSPTYP_R1B (0x3u << 6) /**< \brief (HSMCI_CMDR) R1b response type */
#define HSMCI_CMDR_SPCMD_Pos 8
#define HSMCI_CMDR_SPCMD_Msk (0x7u << HSMCI_CMDR_SPCMD_Pos) /**< \brief (HSMCI_CMDR) Special Command */
#define   HSMCI_CMDR_SPCMD_STD (0x0u << 8) /**< \brief (HSMCI_CMDR) Not a special CMD. */
#define   HSMCI_CMDR_SPCMD_INIT (0x1u << 8) /**< \brief (HSMCI_CMDR) Initialization CMD: 74 clock cycles for initialization sequence. */
#define   HSMCI_CMDR_SPCMD_SYNC (0x2u << 8) /**< \brief (HSMCI_CMDR) Synchronized CMD: Wait for the end of the current data block transfer before sending the pending command. */
#define   HSMCI_CMDR_SPCMD_CE_ATA (0x3u << 8) /**< \brief (HSMCI_CMDR) CE-ATA Completion Signal disable Command. The host cancels the ability for the device to return a command completion signal on the command line. */
#define   HSMCI_CMDR_SPCMD_IT_CMD (0x4u << 8) /**< \brief (HSMCI_CMDR) Interrupt command: Corresponds to the Interrupt Mode (CMD40). */
#define   HSMCI_CMDR_SPCMD_IT_RESP (0x5u << 8) /**< \brief (HSMCI_CMDR) Interrupt response: Corresponds to the Interrupt Mode (CMD40). */
#define   HSMCI_CMDR_SPCMD_BOR (0x6u << 8) /**< \brief (HSMCI_CMDR) Boot Operation Request. Start a boot operation mode, the host processor can read boot data from the MMC device directly. */
#define   HSMCI_CMDR_SPCMD_EBO (0x7u << 8) /**< \brief (HSMCI_CMDR) End Boot Operation. This command allows the host processor to terminate the boot operation mode. */
#define HSMCI_CMDR_OPDCMD (0x1u << 11) /**< \brief (HSMCI_CMDR) Open Drain Command */
#define   HSMCI_CMDR_OPDCMD_PUSHPULL (0x0u << 11) /**< \brief (HSMCI_CMDR) Push pull command. */
#define   HSMCI_CMDR_OPDCMD_OPENDRAIN (0x1u << 11) /**< \brief (HSMCI_CMDR) Open drain command. */
#define HSMCI_CMDR_MAXLAT (0x1u << 12) /**< \brief (HSMCI_CMDR) Max Latency for Command to Response */
#define   HSMCI_CMDR_MAXLAT_5 (0x0u << 12) /**< \brief (HSMCI_CMDR) 5-cycle max latency. */
#define   HSMCI_CMDR_MAXLAT_64 (0x1u << 12) /**< \brief (HSMCI_CMDR) 64-cycle max latency. */
#define HSMCI_CMDR_TRCMD_Pos 16
#define HSMCI_CMDR_TRCMD_Msk (0x3u << HSMCI_CMDR_TRCMD_Pos) /**< \brief (HSMCI_CMDR) Transfer Command */
#define   HSMCI_CMDR_TRCMD_NO_DATA (0x0u << 16) /**< \brief (HSMCI_CMDR) No data transfer */
#define   HSMCI_CMDR_TRCMD_START_DATA (0x1u << 16) /**< \brief (HSMCI_CMDR) Start data transfer */
#define   HSMCI_CMDR_TRCMD_STOP_DATA (0x2u << 16) /**< \brief (HSMCI_CMDR) Stop data transfer */
#define HSMCI_CMDR_TRDIR (0x1u << 18) /**< \brief (HSMCI_CMDR) Transfer Direction */
#define   HSMCI_CMDR_TRDIR_WRITE (0x0u << 18) /**< \brief (HSMCI_CMDR) Write. */
#define   HSMCI_CMDR_TRDIR_READ (0x1u << 18) /**< \brief (HSMCI_CMDR) Read. */
#define HSMCI_CMDR_TRTYP_Pos 19
#define HSMCI_CMDR_TRTYP_Msk (0x7u << HSMCI_CMDR_TRTYP_Pos) /**< \brief (HSMCI_CMDR) Transfer Type */
#define   HSMCI_CMDR_TRTYP_SINGLE (0x0u << 19) /**< \brief (HSMCI_CMDR) MMC/SDCard Single Block */
#define   HSMCI_CMDR_TRTYP_MULTIPLE (0x1u << 19) /**< \brief (HSMCI_CMDR) MMC/SDCard Multiple Block */
#define   HSMCI_CMDR_TRTYP_STREAM (0x2u << 19) /**< \brief (HSMCI_CMDR) MMC Stream */
#define   HSMCI_CMDR_TRTYP_BYTE (0x4u << 19) /**< \brief (HSMCI_CMDR) SDIO Byte */
#define   HSMCI_CMDR_TRTYP_BLOCK (0x5u << 19) /**< \brief (HSMCI_CMDR) SDIO Block */
#define HSMCI_CMDR_IOSPCMD_Pos 24
#define HSMCI_CMDR_IOSPCMD_Msk (0x3u << HSMCI_CMDR_IOSPCMD_Pos) /**< \brief (HSMCI_CMDR) SDIO Special Command */
#define   HSMCI_CMDR_IOSPCMD_STD (0x0u << 24) /**< \brief (HSMCI_CMDR) Not an SDIO Special Command */
#define   HSMCI_CMDR_IOSPCMD_SUSPEND (0x1u << 24) /**< \brief (HSMCI_CMDR) SDIO Suspend Command */
#define   HSMCI_CMDR_IOSPCMD_RESUME (0x2u << 24) /**< \brief (HSMCI_CMDR) SDIO Resume Command */
#define HSMCI_CMDR_ATACS (0x1u << 26) /**< \brief (HSMCI_CMDR) ATA with Command Completion Signal */
#define   HSMCI_CMDR_ATACS_NORMAL (0x0u << 26) /**< \brief (HSMCI_CMDR) Normal operation mode. */
#define   HSMCI_CMDR_ATACS_COMPLETION (0x1u << 26) /**< \brief (HSMCI_CMDR) This bit indicates that a completion signal is expected within a programmed amount of time (HSMCI_CSTOR). */
#define HSMCI_CMDR_BOOT_ACK (0x1u << 27) /**< \brief (HSMCI_CMDR) Boot Operation Acknowledge. */
/* -------- HSMCI_BLKR : (HSMCI Offset: 0x18) Block Register -------- */
#define HSMCI_BLKR_BCNT_Pos 0
#define HSMCI_BLKR_BCNT_Msk (0xffffu << HSMCI_BLKR_BCNT_Pos) /**< \brief (HSMCI_BLKR) MMC/SDIO Block Count - SDIO Byte Count */
#define   HSMCI_BLKR_BCNT_MULTIPLE (0x0u << 0) /**< \brief (HSMCI_BLKR) MMC/SDCARD Multiple BlockFrom 1 to 65635: Value 0 corresponds to an infinite block transfer. */
#define   HSMCI_BLKR_BCNT_BYTE (0x4u << 0) /**< \brief (HSMCI_BLKR) SDIO ByteFrom 1 to 512 bytes: Value 0 corresponds to a 512-byte transfer.Values from 0x200 to 0xFFFF are forbidden. */
#define   HSMCI_BLKR_BCNT_BLOCK (0x5u << 0) /**< \brief (HSMCI_BLKR) SDIO BlockFrom 1 to 511 blocks: Value 0 corresponds to an infinite block transfer.Values from 0x200 to 0xFFFF are forbidden. */
#define HSMCI_BLKR_BLKLEN_Pos 16
#define HSMCI_BLKR_BLKLEN_Msk (0xffffu << HSMCI_BLKR_BLKLEN_Pos) /**< \brief (HSMCI_BLKR) Data Block Length */
#define HSMCI_BLKR_BLKLEN(value) ((HSMCI_BLKR_BLKLEN_Msk & ((value) << HSMCI_BLKR_BLKLEN_Pos)))
/* -------- HSMCI_CSTOR : (HSMCI Offset: 0x1C) Completion Signal Timeout Register -------- */
#define HSMCI_CSTOR_CSTOCYC_Pos 0
#define HSMCI_CSTOR_CSTOCYC_Msk (0xfu << HSMCI_CSTOR_CSTOCYC_Pos) /**< \brief (HSMCI_CSTOR) Completion Signal Timeout Cycle Number */
#define HSMCI_CSTOR_CSTOCYC(value) ((HSMCI_CSTOR_CSTOCYC_Msk & ((value) << HSMCI_CSTOR_CSTOCYC_Pos)))
#define HSMCI_CSTOR_CSTOMUL_Pos 4
#define HSMCI_CSTOR_CSTOMUL_Msk (0x7u << HSMCI_CSTOR_CSTOMUL_Pos) /**< \brief (HSMCI_CSTOR) Completion Signal Timeout Multiplier */
#define   HSMCI_CSTOR_CSTOMUL_1 (0x0u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 1 */
#define   HSMCI_CSTOR_CSTOMUL_16 (0x1u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 16 */
#define   HSMCI_CSTOR_CSTOMUL_128 (0x2u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 128 */
#define   HSMCI_CSTOR_CSTOMUL_256 (0x3u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 256 */
#define   HSMCI_CSTOR_CSTOMUL_1024 (0x4u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 1024 */
#define   HSMCI_CSTOR_CSTOMUL_4096 (0x5u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 4096 */
#define   HSMCI_CSTOR_CSTOMUL_65536 (0x6u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 65536 */
#define   HSMCI_CSTOR_CSTOMUL_1048576 (0x7u << 4) /**< \brief (HSMCI_CSTOR) CSTOCYC x 1048576 */
/* -------- HSMCI_RSPR[4] : (HSMCI Offset: 0x20) Response Register -------- */
#define HSMCI_RSPR_RSP_Pos 0
#define HSMCI_RSPR_RSP_Msk (0xffffffffu << HSMCI_RSPR_RSP_Pos) /**< \brief (HSMCI_RSPR[4]) Response */
/* -------- HSMCI_RDR : (HSMCI Offset: 0x30) Receive Data Register -------- */
#define HSMCI_RDR_DATA_Pos 0
#define HSMCI_RDR_DATA_Msk (0xffffffffu << HSMCI_RDR_DATA_Pos) /**< \brief (HSMCI_RDR) Data to Read */
/* -------- HSMCI_TDR : (HSMCI Offset: 0x34) Transmit Data Register -------- */
#define HSMCI_TDR_DATA_Pos 0
#define HSMCI_TDR_DATA_Msk (0xffffffffu << HSMCI_TDR_DATA_Pos) /**< \brief (HSMCI_TDR) Data to Write */
#define HSMCI_TDR_DATA(value) ((HSMCI_TDR_DATA_Msk & ((value) << HSMCI_TDR_DATA_Pos)))
/* -------- HSMCI_SR : (HSMCI Offset: 0x40) Status Register -------- */
#define HSMCI_SR_CMDRDY (0x1u << 0) /**< \brief (HSMCI_SR) Command Ready */
#define HSMCI_SR_RXRDY (0x1u << 1) /**< \brief (HSMCI_SR) Receiver Ready */
#define HSMCI_SR_TXRDY (0x1u << 2) /**< \brief (HSMCI_SR) Transmit Ready */
#define HSMCI_SR_BLKE (0x1u << 3) /**< \brief (HSMCI_SR) Data Block Ended */
#define HSMCI_SR_DTIP (0x1u << 4) /**< \brief (HSMCI_SR) Data Transfer in Progress */
#define HSMCI_SR_NOTBUSY (0x1u << 5) /**< \brief (HSMCI_SR) HSMCI Not Busy */
#define HSMCI_SR_SDIOIRQforSlotA (0x1u << 8) /**< \brief (HSMCI_SR)  */
#define HSMCI_SR_SDIOIRQforSlotB (0x1u << 9) /**< \brief (HSMCI_SR)  */
#define HSMCI_SR_SDIOWAIT (0x1u << 12) /**< \brief (HSMCI_SR) SDIO Read Wait Operation Status */
#define HSMCI_SR_CSRCV (0x1u << 13) /**< \brief (HSMCI_SR) CE-ATA Completion Signal Received */
#define HSMCI_SR_RINDE (0x1u << 16) /**< \brief (HSMCI_SR) Response Index Error */
#define HSMCI_SR_RDIRE (0x1u << 17) /**< \brief (HSMCI_SR) Response Direction Error */
#define HSMCI_SR_RCRCE (0x1u << 18) /**< \brief (HSMCI_SR) Response CRC Error */
#define HSMCI_SR_RENDE (0x1u << 19) /**< \brief (HSMCI_SR) Response End Bit Error */
#define HSMCI_SR_RTOE (0x1u << 20) /**< \brief (HSMCI_SR) Response Time-out Error */
#define HSMCI_SR_DCRCE (0x1u << 21) /**< \brief (HSMCI_SR) Data CRC Error */
#define HSMCI_SR_DTOE (0x1u << 22) /**< \brief (HSMCI_SR) Data Time-out Error */
#define HSMCI_SR_CSTOE (0x1u << 23) /**< \brief (HSMCI_SR) Completion Signal Time-out Error */
#define HSMCI_SR_BLKOVRE (0x1u << 24) /**< \brief (HSMCI_SR) DMA Block Overrun Error */
#define HSMCI_SR_DMADONE (0x1u << 25) /**< \brief (HSMCI_SR) DMA Transfer done */
#define HSMCI_SR_FIFOEMPTY (0x1u << 26) /**< \brief (HSMCI_SR) FIFO empty flag */
#define HSMCI_SR_XFRDONE (0x1u << 27) /**< \brief (HSMCI_SR) Transfer Done flag */
#define HSMCI_SR_ACKRCV (0x1u << 28) /**< \brief (HSMCI_SR) Boot Operation Acknowledge Received */
#define HSMCI_SR_ACKRCVE (0x1u << 29) /**< \brief (HSMCI_SR) Boot Operation Acknowledge Error */
#define HSMCI_SR_OVRE (0x1u << 30) /**< \brief (HSMCI_SR) Overrun */
#define HSMCI_SR_UNRE (0x1u << 31) /**< \brief (HSMCI_SR) Underrun */
/* -------- HSMCI_IER : (HSMCI Offset: 0x44) Interrupt Enable Register -------- */
#define HSMCI_IER_CMDRDY (0x1u << 0) /**< \brief (HSMCI_IER) Command Ready Interrupt Enable */
#define HSMCI_IER_RXRDY (0x1u << 1) /**< \brief (HSMCI_IER) Receiver Ready Interrupt Enable */
#define HSMCI_IER_TXRDY (0x1u << 2) /**< \brief (HSMCI_IER) Transmit Ready Interrupt Enable */
#define HSMCI_IER_BLKE (0x1u << 3) /**< \brief (HSMCI_IER) Data Block Ended Interrupt Enable */
#define HSMCI_IER_DTIP (0x1u << 4) /**< \brief (HSMCI_IER) Data Transfer in Progress Interrupt Enable */
#define HSMCI_IER_NOTBUSY (0x1u << 5) /**< \brief (HSMCI_IER) Data Not Busy Interrupt Enable */
#define HSMCI_IER_SDIOIRQforSlotA (0x1u << 8) /**< \brief (HSMCI_IER)  */
#define HSMCI_IER_SDIOIRQforSlotB (0x1u << 9) /**< \brief (HSMCI_IER)  */
#define HSMCI_IER_SDIOWAIT (0x1u << 12) /**< \brief (HSMCI_IER) SDIO Read Wait Operation Status Interrupt Enable */
#define HSMCI_IER_CSRCV (0x1u << 13) /**< \brief (HSMCI_IER) Completion Signal Received Interrupt Enable */
#define HSMCI_IER_RINDE (0x1u << 16) /**< \brief (HSMCI_IER) Response Index Error Interrupt Enable */
#define HSMCI_IER_RDIRE (0x1u << 17) /**< \brief (HSMCI_IER) Response Direction Error Interrupt Enable */
#define HSMCI_IER_RCRCE (0x1u << 18) /**< \brief (HSMCI_IER) Response CRC Error Interrupt Enable */
#define HSMCI_IER_RENDE (0x1u << 19) /**< \brief (HSMCI_IER) Response End Bit Error Interrupt Enable */
#define HSMCI_IER_RTOE (0x1u << 20) /**< \brief (HSMCI_IER) Response Time-out Error Interrupt Enable */
#define HSMCI_IER_DCRCE (0x1u << 21) /**< \brief (HSMCI_IER) Data CRC Error Interrupt Enable */
#define HSMCI_IER_DTOE (0x1u << 22) /**< \brief (HSMCI_IER) Data Time-out Error Interrupt Enable */
#define HSMCI_IER_CSTOE (0x1u << 23) /**< \brief (HSMCI_IER) Completion Signal Timeout Error Interrupt Enable */
#define HSMCI_IER_BLKOVRE (0x1u << 24) /**< \brief (HSMCI_IER) DMA Block Overrun Error Interrupt Enable */
#define HSMCI_IER_DMADONE (0x1u << 25) /**< \brief (HSMCI_IER) DMA Transfer completed Interrupt Enable */
#define HSMCI_IER_FIFOEMPTY (0x1u << 26) /**< \brief (HSMCI_IER) FIFO empty Interrupt enable */
#define HSMCI_IER_XFRDONE (0x1u << 27) /**< \brief (HSMCI_IER) Transfer Done Interrupt enable */
#define HSMCI_IER_ACKRCV (0x1u << 28) /**< \brief (HSMCI_IER) Boot Acknowledge Interrupt Enable */
#define HSMCI_IER_ACKRCVE (0x1u << 29) /**< \brief (HSMCI_IER) Boot Acknowledge Error Interrupt Enable */
#define HSMCI_IER_OVRE (0x1u << 30) /**< \brief (HSMCI_IER) Overrun Interrupt Enable */
#define HSMCI_IER_UNRE (0x1u << 31) /**< \brief (HSMCI_IER) Underrun Interrupt Enable */
/* -------- HSMCI_IDR : (HSMCI Offset: 0x48) Interrupt Disable Register -------- */
#define HSMCI_IDR_CMDRDY (0x1u << 0) /**< \brief (HSMCI_IDR) Command Ready Interrupt Disable */
#define HSMCI_IDR_RXRDY (0x1u << 1) /**< \brief (HSMCI_IDR) Receiver Ready Interrupt Disable */
#define HSMCI_IDR_TXRDY (0x1u << 2) /**< \brief (HSMCI_IDR) Transmit Ready Interrupt Disable */
#define HSMCI_IDR_BLKE (0x1u << 3) /**< \brief (HSMCI_IDR) Data Block Ended Interrupt Disable */
#define HSMCI_IDR_DTIP (0x1u << 4) /**< \brief (HSMCI_IDR) Data Transfer in Progress Interrupt Disable */
#define HSMCI_IDR_NOTBUSY (0x1u << 5) /**< \brief (HSMCI_IDR) Data Not Busy Interrupt Disable */
#define HSMCI_IDR_SDIOIRQforSlotA (0x1u << 8) /**< \brief (HSMCI_IDR)  */
#define HSMCI_IDR_SDIOIRQforSlotB (0x1u << 9) /**< \brief (HSMCI_IDR)  */
#define HSMCI_IDR_SDIOWAIT (0x1u << 12) /**< \brief (HSMCI_IDR) SDIO Read Wait Operation Status Interrupt Disable */
#define HSMCI_IDR_CSRCV (0x1u << 13) /**< \brief (HSMCI_IDR) Completion Signal received interrupt Disable */
#define HSMCI_IDR_RINDE (0x1u << 16) /**< \brief (HSMCI_IDR) Response Index Error Interrupt Disable */
#define HSMCI_IDR_RDIRE (0x1u << 17) /**< \brief (HSMCI_IDR) Response Direction Error Interrupt Disable */
#define HSMCI_IDR_RCRCE (0x1u << 18) /**< \brief (HSMCI_IDR) Response CRC Error Interrupt Disable */
#define HSMCI_IDR_RENDE (0x1u << 19) /**< \brief (HSMCI_IDR) Response End Bit Error Interrupt Disable */
#define HSMCI_IDR_RTOE (0x1u << 20) /**< \brief (HSMCI_IDR) Response Time-out Error Interrupt Disable */
#define HSMCI_IDR_DCRCE (0x1u << 21) /**< \brief (HSMCI_IDR) Data CRC Error Interrupt Disable */
#define HSMCI_IDR_DTOE (0x1u << 22) /**< \brief (HSMCI_IDR) Data Time-out Error Interrupt Disable */
#define HSMCI_IDR_CSTOE (0x1u << 23) /**< \brief (HSMCI_IDR) Completion Signal Time out Error Interrupt Disable */
#define HSMCI_IDR_BLKOVRE (0x1u << 24) /**< \brief (HSMCI_IDR) DMA Block Overrun Error Interrupt Disable */
#define HSMCI_IDR_DMADONE (0x1u << 25) /**< \brief (HSMCI_IDR) DMA Transfer completed Interrupt Disable */
#define HSMCI_IDR_FIFOEMPTY (0x1u << 26) /**< \brief (HSMCI_IDR) FIFO empty Interrupt Disable */
#define HSMCI_IDR_XFRDONE (0x1u << 27) /**< \brief (HSMCI_IDR) Transfer Done Interrupt Disable */
#define HSMCI_IDR_ACKRCV (0x1u << 28) /**< \brief (HSMCI_IDR) Boot Acknowledge Interrupt Disable */
#define HSMCI_IDR_ACKRCVE (0x1u << 29) /**< \brief (HSMCI_IDR) Boot Acknowledge Error Interrupt Disable */
#define HSMCI_IDR_OVRE (0x1u << 30) /**< \brief (HSMCI_IDR) Overrun Interrupt Disable */
#define HSMCI_IDR_UNRE (0x1u << 31) /**< \brief (HSMCI_IDR) Underrun Interrupt Disable */
/* -------- HSMCI_IMR : (HSMCI Offset: 0x4C) Interrupt Mask Register -------- */
#define HSMCI_IMR_CMDRDY (0x1u << 0) /**< \brief (HSMCI_IMR) Command Ready Interrupt Mask */
#define HSMCI_IMR_RXRDY (0x1u << 1) /**< \brief (HSMCI_IMR) Receiver Ready Interrupt Mask */
#define HSMCI_IMR_TXRDY (0x1u << 2) /**< \brief (HSMCI_IMR) Transmit Ready Interrupt Mask */
#define HSMCI_IMR_BLKE (0x1u << 3) /**< \brief (HSMCI_IMR) Data Block Ended Interrupt Mask */
#define HSMCI_IMR_DTIP (0x1u << 4) /**< \brief (HSMCI_IMR) Data Transfer in Progress Interrupt Mask */
#define HSMCI_IMR_NOTBUSY (0x1u << 5) /**< \brief (HSMCI_IMR) Data Not Busy Interrupt Mask */
#define HSMCI_IMR_SDIOIRQforSlotA (0x1u << 8) /**< \brief (HSMCI_IMR)  */
#define HSMCI_IMR_SDIOIRQforSlotB (0x1u << 9) /**< \brief (HSMCI_IMR)  */
#define HSMCI_IMR_SDIOWAIT (0x1u << 12) /**< \brief (HSMCI_IMR) SDIO Read Wait Operation Status Interrupt Mask */
#define HSMCI_IMR_CSRCV (0x1u << 13) /**< \brief (HSMCI_IMR) Completion Signal Received Interrupt Mask */
#define HSMCI_IMR_RINDE (0x1u << 16) /**< \brief (HSMCI_IMR) Response Index Error Interrupt Mask */
#define HSMCI_IMR_RDIRE (0x1u << 17) /**< \brief (HSMCI_IMR) Response Direction Error Interrupt Mask */
#define HSMCI_IMR_RCRCE (0x1u << 18) /**< \brief (HSMCI_IMR) Response CRC Error Interrupt Mask */
#define HSMCI_IMR_RENDE (0x1u << 19) /**< \brief (HSMCI_IMR) Response End Bit Error Interrupt Mask */
#define HSMCI_IMR_RTOE (0x1u << 20) /**< \brief (HSMCI_IMR) Response Time-out Error Interrupt Mask */
#define HSMCI_IMR_DCRCE (0x1u << 21) /**< \brief (HSMCI_IMR) Data CRC Error Interrupt Mask */
#define HSMCI_IMR_DTOE (0x1u << 22) /**< \brief (HSMCI_IMR) Data Time-out Error Interrupt Mask */
#define HSMCI_IMR_CSTOE (0x1u << 23) /**< \brief (HSMCI_IMR) Completion Signal Time-out Error Interrupt Mask */
#define HSMCI_IMR_BLKOVRE (0x1u << 24) /**< \brief (HSMCI_IMR) DMA Block Overrun Error Interrupt Mask */
#define HSMCI_IMR_DMADONE (0x1u << 25) /**< \brief (HSMCI_IMR) DMA Transfer Completed Interrupt Mask */
#define HSMCI_IMR_FIFOEMPTY (0x1u << 26) /**< \brief (HSMCI_IMR) FIFO Empty Interrupt Mask */
#define HSMCI_IMR_XFRDONE (0x1u << 27) /**< \brief (HSMCI_IMR) Transfer Done Interrupt Mask */
#define HSMCI_IMR_ACKRCV (0x1u << 28) /**< \brief (HSMCI_IMR) Boot Operation Acknowledge Received Interrupt Mask */
#define HSMCI_IMR_ACKRCVE (0x1u << 29) /**< \brief (HSMCI_IMR) Boot Operation Acknowledge Error Interrupt Mask */
#define HSMCI_IMR_OVRE (0x1u << 30) /**< \brief (HSMCI_IMR) Overrun Interrupt Mask */
#define HSMCI_IMR_UNRE (0x1u << 31) /**< \brief (HSMCI_IMR) Underrun Interrupt Mask */
/* -------- HSMCI_DMA : (HSMCI Offset: 0x50) DMA Configuration Register -------- */
#define HSMCI_DMA_OFFSET_Pos 0
#define HSMCI_DMA_OFFSET_Msk (0x3u << HSMCI_DMA_OFFSET_Pos) /**< \brief (HSMCI_DMA) DMA Write Buffer Offset */
#define HSMCI_DMA_OFFSET(value) ((HSMCI_DMA_OFFSET_Msk & ((value) << HSMCI_DMA_OFFSET_Pos)))
#define HSMCI_DMA_CHKSIZE (0x1u << 4) /**< \brief (HSMCI_DMA) DMA Channel Read and Write Chunk Size */
#define   HSMCI_DMA_CHKSIZE_1 (0x0u << 4) /**< \brief (HSMCI_DMA) 1 data available */
#define   HSMCI_DMA_CHKSIZE_4 (0x1u << 4) /**< \brief (HSMCI_DMA) 4 data available */
#define HSMCI_DMA_DMAEN (0x1u << 8) /**< \brief (HSMCI_DMA) DMA Hardware Handshaking Enable */
#define HSMCI_DMA_ROPT (0x1u << 12) /**< \brief (HSMCI_DMA) Read Optimization with padding */
/* -------- HSMCI_CFG : (HSMCI Offset: 0x54) Configuration Register -------- */
#define HSMCI_CFG_FIFOMODE (0x1u << 0) /**< \brief (HSMCI_CFG) HSMCI Internal FIFO control mode */
#define HSMCI_CFG_FERRCTRL (0x1u << 4) /**< \brief (HSMCI_CFG) Flow Error flag reset control mode */
#define HSMCI_CFG_HSMODE (0x1u << 8) /**< \brief (HSMCI_CFG) High Speed Mode */
#define HSMCI_CFG_LSYNC (0x1u << 12) /**< \brief (HSMCI_CFG) Synchronize on the last block */
/* -------- HSMCI_WPMR : (HSMCI Offset: 0xE4) Write Protection Mode Register -------- */
#define HSMCI_WPMR_WP_EN (0x1u << 0) /**< \brief (HSMCI_WPMR) Write Protection Enable */
#define HSMCI_WPMR_WP_KEY_Pos 8
#define HSMCI_WPMR_WP_KEY_Msk (0xffffffu << HSMCI_WPMR_WP_KEY_Pos) /**< \brief (HSMCI_WPMR) Write Protection Key password */
#define HSMCI_WPMR_WP_KEY(value) ((HSMCI_WPMR_WP_KEY_Msk & ((value) << HSMCI_WPMR_WP_KEY_Pos)))
/* -------- HSMCI_WPSR : (HSMCI Offset: 0xE8) Write Protection Status Register -------- */
#define HSMCI_WPSR_WP_VS_Pos 0
#define HSMCI_WPSR_WP_VS_Msk (0xfu << HSMCI_WPSR_WP_VS_Pos) /**< \brief (HSMCI_WPSR) Write Protection Violation Status */
#define   HSMCI_WPSR_WP_VS_NONE (0x0u << 0) /**< \brief (HSMCI_WPSR) No Write Protection Violation occurred since the last read of this register (WP_SR) */
#define   HSMCI_WPSR_WP_VS_WRITE (0x1u << 0) /**< \brief (HSMCI_WPSR) Write Protection detected unauthorized attempt to write a control register had occurred (since the last read.) */
#define   HSMCI_WPSR_WP_VS_RESET (0x2u << 0) /**< \brief (HSMCI_WPSR) Software reset had been performed while Write Protection was enabled (since the last read). */
#define   HSMCI_WPSR_WP_VS_BOTH (0x3u << 0) /**< \brief (HSMCI_WPSR) Both Write Protection violation and software reset with Write Protection enabled have occurred since the last read. */
#define HSMCI_WPSR_WP_VSRC_Pos 8
#define HSMCI_WPSR_WP_VSRC_Msk (0xffffu << HSMCI_WPSR_WP_VSRC_Pos) /**< \brief (HSMCI_WPSR) Write Protection Violation SouRCe */
/* -------- HSMCI_FIFO[256] : (HSMCI Offset: 0x200) FIFO Memory Aperture0 -------- */
#define HSMCI_FIFO_DATA_Pos 0
#define HSMCI_FIFO_DATA_Msk (0xffffffffu << HSMCI_FIFO_DATA_Pos) /**< \brief (HSMCI_FIFO[256]) Data to Read or Data to Write */
#define HSMCI_FIFO_DATA(value) ((HSMCI_FIFO_DATA_Msk & ((value) << HSMCI_FIFO_DATA_Pos)))
/*@}*/ /* end of group SAM3XA_HSMCI */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR AHB Bus Matrix */
/* ============================================================================= */
/** \addtogroup SAM3XA_MATRIX AHB Bus Matrix */
/*@{*/

/** \brief Matrix hardware registers */
typedef struct {
  RwReg MATRIX_MCFG[6]; /**< \brief (Matrix Offset: 0x0000) Master Configuration Register */
  RoReg Reserved1[10];
  RwReg MATRIX_SCFG[9]; /**< \brief (Matrix Offset: 0x0040) Slave Configuration Register */
  RoReg Reserved2[7];
  RwReg MATRIX_PRAS0;   /**< \brief (Matrix Offset: 0x0080) Priority Register A for Slave 0 */
  RoReg Reserved3[1];
  RwReg MATRIX_PRAS1;   /**< \brief (Matrix Offset: 0x0088) Priority Register A for Slave 1 */
  RoReg Reserved4[1];
  RwReg MATRIX_PRAS2;   /**< \brief (Matrix Offset: 0x0090) Priority Register A for Slave 2 */
  RoReg Reserved5[1];
  RwReg MATRIX_PRAS3;   /**< \brief (Matrix Offset: 0x0098) Priority Register A for Slave 3 */
  RoReg Reserved6[1];
  RwReg MATRIX_PRAS4;   /**< \brief (Matrix Offset: 0x00A0) Priority Register A for Slave 4 */
  RoReg Reserved7[1];
  RwReg MATRIX_PRAS5;   /**< \brief (Matrix Offset: 0x00A8) Priority Register A for Slave 5 */
  RoReg Reserved8[1];
  RwReg MATRIX_PRAS6;   /**< \brief (Matrix Offset: 0x00B0) Priority Register A for Slave 6 */
  RoReg Reserved9[1];
  RwReg MATRIX_PRAS7;   /**< \brief (Matrix Offset: 0x00B8) Priority Register A for Slave 7 */
  RoReg Reserved10[1];
  RwReg MATRIX_PRAS8;   /**< \brief (Matrix Offset: 0x00C0) Priority Register A for Slave 8 */
  RoReg Reserved11[1];
  RoReg Reserved12[14];
  RwReg MATRIX_MRCR;    /**< \brief (Matrix Offset: 0x0100) Master Remap Control Register */
  RoReg Reserved13[4];
  RwReg CCFG_SYSIO;     /**< \brief (Matrix Offset: 0x0114) System I/O Configuration register */
  RoReg Reserved14[51];
  RwReg MATRIX_WPMR;    /**< \brief (Matrix Offset: 0x1E4) Write Protect Mode Register */
  RoReg MATRIX_WPSR;    /**< \brief (Matrix Offset: 0x1E8) Write Protect Status Register */
} Matrix;
/* -------- MATRIX_MCFG[6] : (MATRIX Offset: 0x0000) Master Configuration Register -------- */
#define MATRIX_MCFG_ULBT_Pos 0
#define MATRIX_MCFG_ULBT_Msk (0x7u << MATRIX_MCFG_ULBT_Pos) /**< \brief (MATRIX_MCFG[6]) Undefined Length Burst Type */
#define MATRIX_MCFG_ULBT(value) ((MATRIX_MCFG_ULBT_Msk & ((value) << MATRIX_MCFG_ULBT_Pos)))
/* -------- MATRIX_SCFG[9] : (MATRIX Offset: 0x0040) Slave Configuration Register -------- */
#define MATRIX_SCFG_SLOT_CYCLE_Pos 0
#define MATRIX_SCFG_SLOT_CYCLE_Msk (0xffu << MATRIX_SCFG_SLOT_CYCLE_Pos) /**< \brief (MATRIX_SCFG[9]) Maximum Number of Allowed Cycles for a Burst */
#define MATRIX_SCFG_SLOT_CYCLE(value) ((MATRIX_SCFG_SLOT_CYCLE_Msk & ((value) << MATRIX_SCFG_SLOT_CYCLE_Pos)))
#define MATRIX_SCFG_DEFMSTR_TYPE_Pos 16
#define MATRIX_SCFG_DEFMSTR_TYPE_Msk (0x3u << MATRIX_SCFG_DEFMSTR_TYPE_Pos) /**< \brief (MATRIX_SCFG[9]) Default Master Type */
#define MATRIX_SCFG_DEFMSTR_TYPE(value) ((MATRIX_SCFG_DEFMSTR_TYPE_Msk & ((value) << MATRIX_SCFG_DEFMSTR_TYPE_Pos)))
#define MATRIX_SCFG_FIXED_DEFMSTR_Pos 18
#define MATRIX_SCFG_FIXED_DEFMSTR_Msk (0x7u << MATRIX_SCFG_FIXED_DEFMSTR_Pos) /**< \brief (MATRIX_SCFG[9]) Fixed Default Master */
#define MATRIX_SCFG_FIXED_DEFMSTR(value) ((MATRIX_SCFG_FIXED_DEFMSTR_Msk & ((value) << MATRIX_SCFG_FIXED_DEFMSTR_Pos)))
#define MATRIX_SCFG_ARBT_Pos 24
#define MATRIX_SCFG_ARBT_Msk (0x3u << MATRIX_SCFG_ARBT_Pos) /**< \brief (MATRIX_SCFG[9]) Arbitration Type */
#define MATRIX_SCFG_ARBT(value) ((MATRIX_SCFG_ARBT_Msk & ((value) << MATRIX_SCFG_ARBT_Pos)))
/* -------- MATRIX_PRAS0 : (MATRIX Offset: 0x0080) Priority Register A for Slave 0 -------- */
#define MATRIX_PRAS0_M0PR_Pos 0
#define MATRIX_PRAS0_M0PR_Msk (0x3u << MATRIX_PRAS0_M0PR_Pos) /**< \brief (MATRIX_PRAS0) Master 0 Priority */
#define MATRIX_PRAS0_M0PR(value) ((MATRIX_PRAS0_M0PR_Msk & ((value) << MATRIX_PRAS0_M0PR_Pos)))
#define MATRIX_PRAS0_M1PR_Pos 4
#define MATRIX_PRAS0_M1PR_Msk (0x3u << MATRIX_PRAS0_M1PR_Pos) /**< \brief (MATRIX_PRAS0) Master 1 Priority */
#define MATRIX_PRAS0_M1PR(value) ((MATRIX_PRAS0_M1PR_Msk & ((value) << MATRIX_PRAS0_M1PR_Pos)))
#define MATRIX_PRAS0_M2PR_Pos 8
#define MATRIX_PRAS0_M2PR_Msk (0x3u << MATRIX_PRAS0_M2PR_Pos) /**< \brief (MATRIX_PRAS0) Master 2 Priority */
#define MATRIX_PRAS0_M2PR(value) ((MATRIX_PRAS0_M2PR_Msk & ((value) << MATRIX_PRAS0_M2PR_Pos)))
#define MATRIX_PRAS0_M3PR_Pos 12
#define MATRIX_PRAS0_M3PR_Msk (0x3u << MATRIX_PRAS0_M3PR_Pos) /**< \brief (MATRIX_PRAS0) Master 3 Priority */
#define MATRIX_PRAS0_M3PR(value) ((MATRIX_PRAS0_M3PR_Msk & ((value) << MATRIX_PRAS0_M3PR_Pos)))
#define MATRIX_PRAS0_M4PR_Pos 16
#define MATRIX_PRAS0_M4PR_Msk (0x3u << MATRIX_PRAS0_M4PR_Pos) /**< \brief (MATRIX_PRAS0) Master 4 Priority */
#define MATRIX_PRAS0_M4PR(value) ((MATRIX_PRAS0_M4PR_Msk & ((value) << MATRIX_PRAS0_M4PR_Pos)))
#define MATRIX_PRAS0_M5PR_Pos 20
#define MATRIX_PRAS0_M5PR_Msk (0x3u << MATRIX_PRAS0_M5PR_Pos) /**< \brief (MATRIX_PRAS0) Master 5 Priority */
#define MATRIX_PRAS0_M5PR(value) ((MATRIX_PRAS0_M5PR_Msk & ((value) << MATRIX_PRAS0_M5PR_Pos)))
/* -------- MATRIX_PRAS1 : (MATRIX Offset: 0x0088) Priority Register A for Slave 1 -------- */
#define MATRIX_PRAS1_M0PR_Pos 0
#define MATRIX_PRAS1_M0PR_Msk (0x3u << MATRIX_PRAS1_M0PR_Pos) /**< \brief (MATRIX_PRAS1) Master 0 Priority */
#define MATRIX_PRAS1_M0PR(value) ((MATRIX_PRAS1_M0PR_Msk & ((value) << MATRIX_PRAS1_M0PR_Pos)))
#define MATRIX_PRAS1_M1PR_Pos 4
#define MATRIX_PRAS1_M1PR_Msk (0x3u << MATRIX_PRAS1_M1PR_Pos) /**< \brief (MATRIX_PRAS1) Master 1 Priority */
#define MATRIX_PRAS1_M1PR(value) ((MATRIX_PRAS1_M1PR_Msk & ((value) << MATRIX_PRAS1_M1PR_Pos)))
#define MATRIX_PRAS1_M2PR_Pos 8
#define MATRIX_PRAS1_M2PR_Msk (0x3u << MATRIX_PRAS1_M2PR_Pos) /**< \brief (MATRIX_PRAS1) Master 2 Priority */
#define MATRIX_PRAS1_M2PR(value) ((MATRIX_PRAS1_M2PR_Msk & ((value) << MATRIX_PRAS1_M2PR_Pos)))
#define MATRIX_PRAS1_M3PR_Pos 12
#define MATRIX_PRAS1_M3PR_Msk (0x3u << MATRIX_PRAS1_M3PR_Pos) /**< \brief (MATRIX_PRAS1) Master 3 Priority */
#define MATRIX_PRAS1_M3PR(value) ((MATRIX_PRAS1_M3PR_Msk & ((value) << MATRIX_PRAS1_M3PR_Pos)))
#define MATRIX_PRAS1_M4PR_Pos 16
#define MATRIX_PRAS1_M4PR_Msk (0x3u << MATRIX_PRAS1_M4PR_Pos) /**< \brief (MATRIX_PRAS1) Master 4 Priority */
#define MATRIX_PRAS1_M4PR(value) ((MATRIX_PRAS1_M4PR_Msk & ((value) << MATRIX_PRAS1_M4PR_Pos)))
#define MATRIX_PRAS1_M5PR_Pos 20
#define MATRIX_PRAS1_M5PR_Msk (0x3u << MATRIX_PRAS1_M5PR_Pos) /**< \brief (MATRIX_PRAS1) Master 5 Priority */
#define MATRIX_PRAS1_M5PR(value) ((MATRIX_PRAS1_M5PR_Msk & ((value) << MATRIX_PRAS1_M5PR_Pos)))
/* -------- MATRIX_PRAS2 : (MATRIX Offset: 0x0090) Priority Register A for Slave 2 -------- */
#define MATRIX_PRAS2_M0PR_Pos 0
#define MATRIX_PRAS2_M0PR_Msk (0x3u << MATRIX_PRAS2_M0PR_Pos) /**< \brief (MATRIX_PRAS2) Master 0 Priority */
#define MATRIX_PRAS2_M0PR(value) ((MATRIX_PRAS2_M0PR_Msk & ((value) << MATRIX_PRAS2_M0PR_Pos)))
#define MATRIX_PRAS2_M1PR_Pos 4
#define MATRIX_PRAS2_M1PR_Msk (0x3u << MATRIX_PRAS2_M1PR_Pos) /**< \brief (MATRIX_PRAS2) Master 1 Priority */
#define MATRIX_PRAS2_M1PR(value) ((MATRIX_PRAS2_M1PR_Msk & ((value) << MATRIX_PRAS2_M1PR_Pos)))
#define MATRIX_PRAS2_M2PR_Pos 8
#define MATRIX_PRAS2_M2PR_Msk (0x3u << MATRIX_PRAS2_M2PR_Pos) /**< \brief (MATRIX_PRAS2) Master 2 Priority */
#define MATRIX_PRAS2_M2PR(value) ((MATRIX_PRAS2_M2PR_Msk & ((value) << MATRIX_PRAS2_M2PR_Pos)))
#define MATRIX_PRAS2_M3PR_Pos 12
#define MATRIX_PRAS2_M3PR_Msk (0x3u << MATRIX_PRAS2_M3PR_Pos) /**< \brief (MATRIX_PRAS2) Master 3 Priority */
#define MATRIX_PRAS2_M3PR(value) ((MATRIX_PRAS2_M3PR_Msk & ((value) << MATRIX_PRAS2_M3PR_Pos)))
#define MATRIX_PRAS2_M4PR_Pos 16
#define MATRIX_PRAS2_M4PR_Msk (0x3u << MATRIX_PRAS2_M4PR_Pos) /**< \brief (MATRIX_PRAS2) Master 4 Priority */
#define MATRIX_PRAS2_M4PR(value) ((MATRIX_PRAS2_M4PR_Msk & ((value) << MATRIX_PRAS2_M4PR_Pos)))
#define MATRIX_PRAS2_M5PR_Pos 20
#define MATRIX_PRAS2_M5PR_Msk (0x3u << MATRIX_PRAS2_M5PR_Pos) /**< \brief (MATRIX_PRAS2) Master 5 Priority */
#define MATRIX_PRAS2_M5PR(value) ((MATRIX_PRAS2_M5PR_Msk & ((value) << MATRIX_PRAS2_M5PR_Pos)))
/* -------- MATRIX_PRAS3 : (MATRIX Offset: 0x0098) Priority Register A for Slave 3 -------- */
#define MATRIX_PRAS3_M0PR_Pos 0
#define MATRIX_PRAS3_M0PR_Msk (0x3u << MATRIX_PRAS3_M0PR_Pos) /**< \brief (MATRIX_PRAS3) Master 0 Priority */
#define MATRIX_PRAS3_M0PR(value) ((MATRIX_PRAS3_M0PR_Msk & ((value) << MATRIX_PRAS3_M0PR_Pos)))
#define MATRIX_PRAS3_M1PR_Pos 4
#define MATRIX_PRAS3_M1PR_Msk (0x3u << MATRIX_PRAS3_M1PR_Pos) /**< \brief (MATRIX_PRAS3) Master 1 Priority */
#define MATRIX_PRAS3_M1PR(value) ((MATRIX_PRAS3_M1PR_Msk & ((value) << MATRIX_PRAS3_M1PR_Pos)))
#define MATRIX_PRAS3_M2PR_Pos 8
#define MATRIX_PRAS3_M2PR_Msk (0x3u << MATRIX_PRAS3_M2PR_Pos) /**< \brief (MATRIX_PRAS3) Master 2 Priority */
#define MATRIX_PRAS3_M2PR(value) ((MATRIX_PRAS3_M2PR_Msk & ((value) << MATRIX_PRAS3_M2PR_Pos)))
#define MATRIX_PRAS3_M3PR_Pos 12
#define MATRIX_PRAS3_M3PR_Msk (0x3u << MATRIX_PRAS3_M3PR_Pos) /**< \brief (MATRIX_PRAS3) Master 3 Priority */
#define MATRIX_PRAS3_M3PR(value) ((MATRIX_PRAS3_M3PR_Msk & ((value) << MATRIX_PRAS3_M3PR_Pos)))
#define MATRIX_PRAS3_M4PR_Pos 16
#define MATRIX_PRAS3_M4PR_Msk (0x3u << MATRIX_PRAS3_M4PR_Pos) /**< \brief (MATRIX_PRAS3) Master 4 Priority */
#define MATRIX_PRAS3_M4PR(value) ((MATRIX_PRAS3_M4PR_Msk & ((value) << MATRIX_PRAS3_M4PR_Pos)))
#define MATRIX_PRAS3_M5PR_Pos 20
#define MATRIX_PRAS3_M5PR_Msk (0x3u << MATRIX_PRAS3_M5PR_Pos) /**< \brief (MATRIX_PRAS3) Master 5 Priority */
#define MATRIX_PRAS3_M5PR(value) ((MATRIX_PRAS3_M5PR_Msk & ((value) << MATRIX_PRAS3_M5PR_Pos)))
/* -------- MATRIX_PRAS4 : (MATRIX Offset: 0x00A0) Priority Register A for Slave 4 -------- */
#define MATRIX_PRAS4_M0PR_Pos 0
#define MATRIX_PRAS4_M0PR_Msk (0x3u << MATRIX_PRAS4_M0PR_Pos) /**< \brief (MATRIX_PRAS4) Master 0 Priority */
#define MATRIX_PRAS4_M0PR(value) ((MATRIX_PRAS4_M0PR_Msk & ((value) << MATRIX_PRAS4_M0PR_Pos)))
#define MATRIX_PRAS4_M1PR_Pos 4
#define MATRIX_PRAS4_M1PR_Msk (0x3u << MATRIX_PRAS4_M1PR_Pos) /**< \brief (MATRIX_PRAS4) Master 1 Priority */
#define MATRIX_PRAS4_M1PR(value) ((MATRIX_PRAS4_M1PR_Msk & ((value) << MATRIX_PRAS4_M1PR_Pos)))
#define MATRIX_PRAS4_M2PR_Pos 8
#define MATRIX_PRAS4_M2PR_Msk (0x3u << MATRIX_PRAS4_M2PR_Pos) /**< \brief (MATRIX_PRAS4) Master 2 Priority */
#define MATRIX_PRAS4_M2PR(value) ((MATRIX_PRAS4_M2PR_Msk & ((value) << MATRIX_PRAS4_M2PR_Pos)))
#define MATRIX_PRAS4_M3PR_Pos 12
#define MATRIX_PRAS4_M3PR_Msk (0x3u << MATRIX_PRAS4_M3PR_Pos) /**< \brief (MATRIX_PRAS4) Master 3 Priority */
#define MATRIX_PRAS4_M3PR(value) ((MATRIX_PRAS4_M3PR_Msk & ((value) << MATRIX_PRAS4_M3PR_Pos)))
#define MATRIX_PRAS4_M4PR_Pos 16
#define MATRIX_PRAS4_M4PR_Msk (0x3u << MATRIX_PRAS4_M4PR_Pos) /**< \brief (MATRIX_PRAS4) Master 4 Priority */
#define MATRIX_PRAS4_M4PR(value) ((MATRIX_PRAS4_M4PR_Msk & ((value) << MATRIX_PRAS4_M4PR_Pos)))
#define MATRIX_PRAS4_M5PR_Pos 20
#define MATRIX_PRAS4_M5PR_Msk (0x3u << MATRIX_PRAS4_M5PR_Pos) /**< \brief (MATRIX_PRAS4) Master 5 Priority */
#define MATRIX_PRAS4_M5PR(value) ((MATRIX_PRAS4_M5PR_Msk & ((value) << MATRIX_PRAS4_M5PR_Pos)))
/* -------- MATRIX_PRAS5 : (MATRIX Offset: 0x00A8) Priority Register A for Slave 5 -------- */
#define MATRIX_PRAS5_M0PR_Pos 0
#define MATRIX_PRAS5_M0PR_Msk (0x3u << MATRIX_PRAS5_M0PR_Pos) /**< \brief (MATRIX_PRAS5) Master 0 Priority */
#define MATRIX_PRAS5_M0PR(value) ((MATRIX_PRAS5_M0PR_Msk & ((value) << MATRIX_PRAS5_M0PR_Pos)))
#define MATRIX_PRAS5_M1PR_Pos 4
#define MATRIX_PRAS5_M1PR_Msk (0x3u << MATRIX_PRAS5_M1PR_Pos) /**< \brief (MATRIX_PRAS5) Master 1 Priority */
#define MATRIX_PRAS5_M1PR(value) ((MATRIX_PRAS5_M1PR_Msk & ((value) << MATRIX_PRAS5_M1PR_Pos)))
#define MATRIX_PRAS5_M2PR_Pos 8
#define MATRIX_PRAS5_M2PR_Msk (0x3u << MATRIX_PRAS5_M2PR_Pos) /**< \brief (MATRIX_PRAS5) Master 2 Priority */
#define MATRIX_PRAS5_M2PR(value) ((MATRIX_PRAS5_M2PR_Msk & ((value) << MATRIX_PRAS5_M2PR_Pos)))
#define MATRIX_PRAS5_M3PR_Pos 12
#define MATRIX_PRAS5_M3PR_Msk (0x3u << MATRIX_PRAS5_M3PR_Pos) /**< \brief (MATRIX_PRAS5) Master 3 Priority */
#define MATRIX_PRAS5_M3PR(value) ((MATRIX_PRAS5_M3PR_Msk & ((value) << MATRIX_PRAS5_M3PR_Pos)))
#define MATRIX_PRAS5_M4PR_Pos 16
#define MATRIX_PRAS5_M4PR_Msk (0x3u << MATRIX_PRAS5_M4PR_Pos) /**< \brief (MATRIX_PRAS5) Master 4 Priority */
#define MATRIX_PRAS5_M4PR(value) ((MATRIX_PRAS5_M4PR_Msk & ((value) << MATRIX_PRAS5_M4PR_Pos)))
#define MATRIX_PRAS5_M5PR_Pos 20
#define MATRIX_PRAS5_M5PR_Msk (0x3u << MATRIX_PRAS5_M5PR_Pos) /**< \brief (MATRIX_PRAS5) Master 5 Priority */
#define MATRIX_PRAS5_M5PR(value) ((MATRIX_PRAS5_M5PR_Msk & ((value) << MATRIX_PRAS5_M5PR_Pos)))
/* -------- MATRIX_PRAS6 : (MATRIX Offset: 0x00B0) Priority Register A for Slave 6 -------- */
#define MATRIX_PRAS6_M0PR_Pos 0
#define MATRIX_PRAS6_M0PR_Msk (0x3u << MATRIX_PRAS6_M0PR_Pos) /**< \brief (MATRIX_PRAS6) Master 0 Priority */
#define MATRIX_PRAS6_M0PR(value) ((MATRIX_PRAS6_M0PR_Msk & ((value) << MATRIX_PRAS6_M0PR_Pos)))
#define MATRIX_PRAS6_M1PR_Pos 4
#define MATRIX_PRAS6_M1PR_Msk (0x3u << MATRIX_PRAS6_M1PR_Pos) /**< \brief (MATRIX_PRAS6) Master 1 Priority */
#define MATRIX_PRAS6_M1PR(value) ((MATRIX_PRAS6_M1PR_Msk & ((value) << MATRIX_PRAS6_M1PR_Pos)))
#define MATRIX_PRAS6_M2PR_Pos 8
#define MATRIX_PRAS6_M2PR_Msk (0x3u << MATRIX_PRAS6_M2PR_Pos) /**< \brief (MATRIX_PRAS6) Master 2 Priority */
#define MATRIX_PRAS6_M2PR(value) ((MATRIX_PRAS6_M2PR_Msk & ((value) << MATRIX_PRAS6_M2PR_Pos)))
#define MATRIX_PRAS6_M3PR_Pos 12
#define MATRIX_PRAS6_M3PR_Msk (0x3u << MATRIX_PRAS6_M3PR_Pos) /**< \brief (MATRIX_PRAS6) Master 3 Priority */
#define MATRIX_PRAS6_M3PR(value) ((MATRIX_PRAS6_M3PR_Msk & ((value) << MATRIX_PRAS6_M3PR_Pos)))
#define MATRIX_PRAS6_M4PR_Pos 16
#define MATRIX_PRAS6_M4PR_Msk (0x3u << MATRIX_PRAS6_M4PR_Pos) /**< \brief (MATRIX_PRAS6) Master 4 Priority */
#define MATRIX_PRAS6_M4PR(value) ((MATRIX_PRAS6_M4PR_Msk & ((value) << MATRIX_PRAS6_M4PR_Pos)))
#define MATRIX_PRAS6_M5PR_Pos 20
#define MATRIX_PRAS6_M5PR_Msk (0x3u << MATRIX_PRAS6_M5PR_Pos) /**< \brief (MATRIX_PRAS6) Master 5 Priority */
#define MATRIX_PRAS6_M5PR(value) ((MATRIX_PRAS6_M5PR_Msk & ((value) << MATRIX_PRAS6_M5PR_Pos)))
/* -------- MATRIX_PRAS7 : (MATRIX Offset: 0x00B8) Priority Register A for Slave 7 -------- */
#define MATRIX_PRAS7_M0PR_Pos 0
#define MATRIX_PRAS7_M0PR_Msk (0x3u << MATRIX_PRAS7_M0PR_Pos) /**< \brief (MATRIX_PRAS7) Master 0 Priority */
#define MATRIX_PRAS7_M0PR(value) ((MATRIX_PRAS7_M0PR_Msk & ((value) << MATRIX_PRAS7_M0PR_Pos)))
#define MATRIX_PRAS7_M1PR_Pos 4
#define MATRIX_PRAS7_M1PR_Msk (0x3u << MATRIX_PRAS7_M1PR_Pos) /**< \brief (MATRIX_PRAS7) Master 1 Priority */
#define MATRIX_PRAS7_M1PR(value) ((MATRIX_PRAS7_M1PR_Msk & ((value) << MATRIX_PRAS7_M1PR_Pos)))
#define MATRIX_PRAS7_M2PR_Pos 8
#define MATRIX_PRAS7_M2PR_Msk (0x3u << MATRIX_PRAS7_M2PR_Pos) /**< \brief (MATRIX_PRAS7) Master 2 Priority */
#define MATRIX_PRAS7_M2PR(value) ((MATRIX_PRAS7_M2PR_Msk & ((value) << MATRIX_PRAS7_M2PR_Pos)))
#define MATRIX_PRAS7_M3PR_Pos 12
#define MATRIX_PRAS7_M3PR_Msk (0x3u << MATRIX_PRAS7_M3PR_Pos) /**< \brief (MATRIX_PRAS7) Master 3 Priority */
#define MATRIX_PRAS7_M3PR(value) ((MATRIX_PRAS7_M3PR_Msk & ((value) << MATRIX_PRAS7_M3PR_Pos)))
#define MATRIX_PRAS7_M4PR_Pos 16
#define MATRIX_PRAS7_M4PR_Msk (0x3u << MATRIX_PRAS7_M4PR_Pos) /**< \brief (MATRIX_PRAS7) Master 4 Priority */
#define MATRIX_PRAS7_M4PR(value) ((MATRIX_PRAS7_M4PR_Msk & ((value) << MATRIX_PRAS7_M4PR_Pos)))
#define MATRIX_PRAS7_M5PR_Pos 20
#define MATRIX_PRAS7_M5PR_Msk (0x3u << MATRIX_PRAS7_M5PR_Pos) /**< \brief (MATRIX_PRAS7) Master 5 Priority */
#define MATRIX_PRAS7_M5PR(value) ((MATRIX_PRAS7_M5PR_Msk & ((value) << MATRIX_PRAS7_M5PR_Pos)))
/* -------- MATRIX_PRAS8 : (MATRIX Offset: 0x00C0) Priority Register A for Slave 8 -------- */
#define MATRIX_PRAS8_M0PR_Pos 0
#define MATRIX_PRAS8_M0PR_Msk (0x3u << MATRIX_PRAS8_M0PR_Pos) /**< \brief (MATRIX_PRAS8) Master 0 Priority */
#define MATRIX_PRAS8_M0PR(value) ((MATRIX_PRAS8_M0PR_Msk & ((value) << MATRIX_PRAS8_M0PR_Pos)))
#define MATRIX_PRAS8_M1PR_Pos 4
#define MATRIX_PRAS8_M1PR_Msk (0x3u << MATRIX_PRAS8_M1PR_Pos) /**< \brief (MATRIX_PRAS8) Master 1 Priority */
#define MATRIX_PRAS8_M1PR(value) ((MATRIX_PRAS8_M1PR_Msk & ((value) << MATRIX_PRAS8_M1PR_Pos)))
#define MATRIX_PRAS8_M2PR_Pos 8
#define MATRIX_PRAS8_M2PR_Msk (0x3u << MATRIX_PRAS8_M2PR_Pos) /**< \brief (MATRIX_PRAS8) Master 2 Priority */
#define MATRIX_PRAS8_M2PR(value) ((MATRIX_PRAS8_M2PR_Msk & ((value) << MATRIX_PRAS8_M2PR_Pos)))
#define MATRIX_PRAS8_M3PR_Pos 12
#define MATRIX_PRAS8_M3PR_Msk (0x3u << MATRIX_PRAS8_M3PR_Pos) /**< \brief (MATRIX_PRAS8) Master 3 Priority */
#define MATRIX_PRAS8_M3PR(value) ((MATRIX_PRAS8_M3PR_Msk & ((value) << MATRIX_PRAS8_M3PR_Pos)))
#define MATRIX_PRAS8_M4PR_Pos 16
#define MATRIX_PRAS8_M4PR_Msk (0x3u << MATRIX_PRAS8_M4PR_Pos) /**< \brief (MATRIX_PRAS8) Master 4 Priority */
#define MATRIX_PRAS8_M4PR(value) ((MATRIX_PRAS8_M4PR_Msk & ((value) << MATRIX_PRAS8_M4PR_Pos)))
#define MATRIX_PRAS8_M5PR_Pos 20
#define MATRIX_PRAS8_M5PR_Msk (0x3u << MATRIX_PRAS8_M5PR_Pos) /**< \brief (MATRIX_PRAS8) Master 5 Priority */
#define MATRIX_PRAS8_M5PR(value) ((MATRIX_PRAS8_M5PR_Msk & ((value) << MATRIX_PRAS8_M5PR_Pos)))
/* -------- MATRIX_MRCR : (MATRIX Offset: 0x0100) Master Remap Control Register -------- */
#define MATRIX_MRCR_RCB0 (0x1u << 0) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 0 */
#define MATRIX_MRCR_RCB1 (0x1u << 1) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 1 */
#define MATRIX_MRCR_RCB2 (0x1u << 2) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 2 */
#define MATRIX_MRCR_RCB3 (0x1u << 3) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 3 */
#define MATRIX_MRCR_RCB4_Pos 4
#define MATRIX_MRCR_RCB4_Msk (0x3u << MATRIX_MRCR_RCB4_Pos) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 4 */
#define MATRIX_MRCR_RCB4(value) ((MATRIX_MRCR_RCB4_Msk & ((value) << MATRIX_MRCR_RCB4_Pos)))
#define MATRIX_MRCR_RCB5 (0x1u << 6) /**< \brief (MATRIX_MRCR) Remap Command Bit for AHB Master 5 */
/* -------- CCFG_SYSIO : (MATRIX Offset: 0x0114) System I/O Configuration register -------- */
#define CCFG_SYSIO_SYSIO12 (0x1u << 12) /**< \brief (CCFG_SYSIO) PC0 or ERASE Assignment */
/* -------- MATRIX_WPMR : (MATRIX Offset: 0x1E4) Write Protect Mode Register -------- */
#define MATRIX_WPMR_WPEN (0x1u << 0) /**< \brief (MATRIX_WPMR) Write Protect ENable */
#define MATRIX_WPMR_WPKEY_Pos 8
#define MATRIX_WPMR_WPKEY_Msk (0xffffffu << MATRIX_WPMR_WPKEY_Pos) /**< \brief (MATRIX_WPMR) Write Protect KEY (Write-only) */
#define MATRIX_WPMR_WPKEY(value) ((MATRIX_WPMR_WPKEY_Msk & ((value) << MATRIX_WPMR_WPKEY_Pos)))
/* -------- MATRIX_WPSR : (MATRIX Offset: 0x1E8) Write Protect Status Register -------- */
#define MATRIX_WPSR_WPVS (0x1u << 0) /**< \brief (MATRIX_WPSR) Write Protect Violation Status */
#define MATRIX_WPSR_WPVSRC_Pos 8
#define MATRIX_WPSR_WPVSRC_Msk (0xffffu << MATRIX_WPSR_WPVSRC_Pos) /**< \brief (MATRIX_WPSR) Write Protect Violation Source */
/*@}*/ /* end of group SAM3XA_AHB */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Peripheral DMA Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_PDC Peripheral DMA Controller */
/*@{*/

/** \brief Pdc hardware registers */
typedef struct {
  RwReg PERIPH_RPR;  /**< \brief (Pdc Offset: 0x0) Receive Pointer Register */
  RwReg PERIPH_RCR;  /**< \brief (Pdc Offset: 0x4) Receive Counter Register */
  RwReg PERIPH_TPR;  /**< \brief (Pdc Offset: 0x8) Transmit Pointer Register */
  RwReg PERIPH_TCR;  /**< \brief (Pdc Offset: 0xC) Transmit Counter Register */
  RwReg PERIPH_RNPR; /**< \brief (Pdc Offset: 0x10) Receive Next Pointer Register */
  RwReg PERIPH_RNCR; /**< \brief (Pdc Offset: 0x14) Receive Next Counter Register */
  RwReg PERIPH_TNPR; /**< \brief (Pdc Offset: 0x18) Transmit Next Pointer Register */
  RwReg PERIPH_TNCR; /**< \brief (Pdc Offset: 0x1C) Transmit Next Counter Register */
  WoReg PERIPH_PTCR; /**< \brief (Pdc Offset: 0x20) Transfer Control Register */
  RoReg PERIPH_PTSR; /**< \brief (Pdc Offset: 0x24) Transfer Status Register */
} Pdc;
/* -------- PERIPH_RPR : (PDC Offset: 0x0) Receive Pointer Register -------- */
#define PERIPH_RPR_RXPTR_Pos 0
#define PERIPH_RPR_RXPTR_Msk (0xffffffffu << PERIPH_RPR_RXPTR_Pos) /**< \brief (PERIPH_RPR) Receive Pointer Register */
#define PERIPH_RPR_RXPTR(value) ((PERIPH_RPR_RXPTR_Msk & ((value) << PERIPH_RPR_RXPTR_Pos)))
/* -------- PERIPH_RCR : (PDC Offset: 0x4) Receive Counter Register -------- */
#define PERIPH_RCR_RXCTR_Pos 0
#define PERIPH_RCR_RXCTR_Msk (0xffffu << PERIPH_RCR_RXCTR_Pos) /**< \brief (PERIPH_RCR) Receive Counter Register */
#define PERIPH_RCR_RXCTR(value) ((PERIPH_RCR_RXCTR_Msk & ((value) << PERIPH_RCR_RXCTR_Pos)))
/* -------- PERIPH_TPR : (PDC Offset: 0x8) Transmit Pointer Register -------- */
#define PERIPH_TPR_TXPTR_Pos 0
#define PERIPH_TPR_TXPTR_Msk (0xffffffffu << PERIPH_TPR_TXPTR_Pos) /**< \brief (PERIPH_TPR) Transmit Counter Register */
#define PERIPH_TPR_TXPTR(value) ((PERIPH_TPR_TXPTR_Msk & ((value) << PERIPH_TPR_TXPTR_Pos)))
/* -------- PERIPH_TCR : (PDC Offset: 0xC) Transmit Counter Register -------- */
#define PERIPH_TCR_TXCTR_Pos 0
#define PERIPH_TCR_TXCTR_Msk (0xffffu << PERIPH_TCR_TXCTR_Pos) /**< \brief (PERIPH_TCR) Transmit Counter Register */
#define PERIPH_TCR_TXCTR(value) ((PERIPH_TCR_TXCTR_Msk & ((value) << PERIPH_TCR_TXCTR_Pos)))
/* -------- PERIPH_RNPR : (PDC Offset: 0x10) Receive Next Pointer Register -------- */
#define PERIPH_RNPR_RXNPTR_Pos 0
#define PERIPH_RNPR_RXNPTR_Msk (0xffffffffu << PERIPH_RNPR_RXNPTR_Pos) /**< \brief (PERIPH_RNPR) Receive Next Pointer */
#define PERIPH_RNPR_RXNPTR(value) ((PERIPH_RNPR_RXNPTR_Msk & ((value) << PERIPH_RNPR_RXNPTR_Pos)))
/* -------- PERIPH_RNCR : (PDC Offset: 0x14) Receive Next Counter Register -------- */
#define PERIPH_RNCR_RXNCTR_Pos 0
#define PERIPH_RNCR_RXNCTR_Msk (0xffffu << PERIPH_RNCR_RXNCTR_Pos) /**< \brief (PERIPH_RNCR) Receive Next Counter */
#define PERIPH_RNCR_RXNCTR(value) ((PERIPH_RNCR_RXNCTR_Msk & ((value) << PERIPH_RNCR_RXNCTR_Pos)))
/* -------- PERIPH_TNPR : (PDC Offset: 0x18) Transmit Next Pointer Register -------- */
#define PERIPH_TNPR_TXNPTR_Pos 0
#define PERIPH_TNPR_TXNPTR_Msk (0xffffffffu << PERIPH_TNPR_TXNPTR_Pos) /**< \brief (PERIPH_TNPR) Transmit Next Pointer */
#define PERIPH_TNPR_TXNPTR(value) ((PERIPH_TNPR_TXNPTR_Msk & ((value) << PERIPH_TNPR_TXNPTR_Pos)))
/* -------- PERIPH_TNCR : (PDC Offset: 0x1C) Transmit Next Counter Register -------- */
#define PERIPH_TNCR_TXNCTR_Pos 0
#define PERIPH_TNCR_TXNCTR_Msk (0xffffu << PERIPH_TNCR_TXNCTR_Pos) /**< \brief (PERIPH_TNCR) Transmit Counter Next */
#define PERIPH_TNCR_TXNCTR(value) ((PERIPH_TNCR_TXNCTR_Msk & ((value) << PERIPH_TNCR_TXNCTR_Pos)))
/* -------- PERIPH_PTCR : (PDC Offset: 0x20) Transfer Control Register -------- */
#define PERIPH_PTCR_RXTEN (0x1u << 0) /**< \brief (PERIPH_PTCR) Receiver Transfer Enable */
#define PERIPH_PTCR_RXTDIS (0x1u << 1) /**< \brief (PERIPH_PTCR) Receiver Transfer Disable */
#define PERIPH_PTCR_TXTEN (0x1u << 8) /**< \brief (PERIPH_PTCR) Transmitter Transfer Enable */
#define PERIPH_PTCR_TXTDIS (0x1u << 9) /**< \brief (PERIPH_PTCR) Transmitter Transfer Disable */
/* -------- PERIPH_PTSR : (PDC Offset: 0x24) Transfer Status Register -------- */
#define PERIPH_PTSR_RXTEN (0x1u << 0) /**< \brief (PERIPH_PTSR) Receiver Transfer Enable */
#define PERIPH_PTSR_TXTEN (0x1u << 8) /**< \brief (PERIPH_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_PDC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Parallel Input/Output Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_PIO Parallel Input/Output Controller */
/*@{*/

/** \brief Pio hardware registers */
typedef struct {
  WoReg PIO_PER;       /**< \brief (Pio Offset: 0x0000) PIO Enable Register */
  WoReg PIO_PDR;       /**< \brief (Pio Offset: 0x0004) PIO Disable Register */
  RoReg PIO_PSR;       /**< \brief (Pio Offset: 0x0008) PIO Status Register */
  RoReg Reserved1[1];
  WoReg PIO_OER;       /**< \brief (Pio Offset: 0x0010) Output Enable Register */
  WoReg PIO_ODR;       /**< \brief (Pio Offset: 0x0014) Output Disable Register */
  RoReg PIO_OSR;       /**< \brief (Pio Offset: 0x0018) Output Status Register */
  RoReg Reserved2[1];
  WoReg PIO_IFER;      /**< \brief (Pio Offset: 0x0020) Glitch Input Filter Enable Register */
  WoReg PIO_IFDR;      /**< \brief (Pio Offset: 0x0024) Glitch Input Filter Disable Register */
  RoReg PIO_IFSR;      /**< \brief (Pio Offset: 0x0028) Glitch Input Filter Status Register */
  RoReg Reserved3[1];
  WoReg PIO_SODR;      /**< \brief (Pio Offset: 0x0030) Set Output Data Register */
  WoReg PIO_CODR;      /**< \brief (Pio Offset: 0x0034) Clear Output Data Register */
  RwReg PIO_ODSR;      /**< \brief (Pio Offset: 0x0038) Output Data Status Register */
  RoReg PIO_PDSR;      /**< \brief (Pio Offset: 0x003C) Pin Data Status Register */
  WoReg PIO_IER;       /**< \brief (Pio Offset: 0x0040) Interrupt Enable Register */
  WoReg PIO_IDR;       /**< \brief (Pio Offset: 0x0044) Interrupt Disable Register */
  RoReg PIO_IMR;       /**< \brief (Pio Offset: 0x0048) Interrupt Mask Register */
  RoReg PIO_ISR;       /**< \brief (Pio Offset: 0x004C) Interrupt Status Register */
  WoReg PIO_MDER;      /**< \brief (Pio Offset: 0x0050) Multi-driver Enable Register */
  WoReg PIO_MDDR;      /**< \brief (Pio Offset: 0x0054) Multi-driver Disable Register */
  RoReg PIO_MDSR;      /**< \brief (Pio Offset: 0x0058) Multi-driver Status Register */
  RoReg Reserved4[1];
  WoReg PIO_PUDR;      /**< \brief (Pio Offset: 0x0060) Pull-up Disable Register */
  WoReg PIO_PUER;      /**< \brief (Pio Offset: 0x0064) Pull-up Enable Register */
  RoReg PIO_PUSR;      /**< \brief (Pio Offset: 0x0068) Pad Pull-up Status Register */
  RoReg Reserved5[1];
  RwReg PIO_ABSR;      /**< \brief (Pio Offset: 0x0070) Peripheral AB Select Register */
  RoReg Reserved6[3];
  WoReg PIO_SCIFSR;    /**< \brief (Pio Offset: 0x0080) System Clock Glitch Input Filter Select Register */
  WoReg PIO_DIFSR;     /**< \brief (Pio Offset: 0x0084) Debouncing Input Filter Select Register */
  RoReg PIO_IFDGSR;    /**< \brief (Pio Offset: 0x0088) Glitch or Debouncing Input Filter Clock Selection Status Register */
  RwReg PIO_SCDR;      /**< \brief (Pio Offset: 0x008C) Slow Clock Divider Debouncing Register */
  RoReg Reserved7[4];
  WoReg PIO_OWER;      /**< \brief (Pio Offset: 0x00A0) Output Write Enable */
  WoReg PIO_OWDR;      /**< \brief (Pio Offset: 0x00A4) Output Write Disable */
  RoReg PIO_OWSR;      /**< \brief (Pio Offset: 0x00A8) Output Write Status Register */
  RoReg Reserved8[1];
  WoReg PIO_AIMER;     /**< \brief (Pio Offset: 0x00B0) Additional Interrupt Modes Enable Register */
  WoReg PIO_AIMDR;     /**< \brief (Pio Offset: 0x00B4) Additional Interrupt Modes Disables Register */
  RoReg PIO_AIMMR;     /**< \brief (Pio Offset: 0x00B8) Additional Interrupt Modes Mask Register */
  RoReg Reserved9[1];
  WoReg PIO_ESR;       /**< \brief (Pio Offset: 0x00C0) Edge Select Register */
  WoReg PIO_LSR;       /**< \brief (Pio Offset: 0x00C4) Level Select Register */
  RoReg PIO_ELSR;      /**< \brief (Pio Offset: 0x00C8) Edge/Level Status Register */
  RoReg Reserved10[1];
  WoReg PIO_FELLSR;    /**< \brief (Pio Offset: 0x00D0) Falling Edge/Low Level Select Register */
  WoReg PIO_REHLSR;    /**< \brief (Pio Offset: 0x00D4) Rising Edge/ High Level Select Register */
  RoReg PIO_FRLHSR;    /**< \brief (Pio Offset: 0x00D8) Fall/Rise - Low/High Status Register */
  RoReg Reserved11[1];
  RoReg PIO_LOCKSR;    /**< \brief (Pio Offset: 0x00E0) Lock Status */
  RwReg PIO_WPMR;      /**< \brief (Pio Offset: 0x00E4) Write Protect Mode Register */
  RoReg PIO_WPSR;      /**< \brief (Pio Offset: 0x00E8) Write Protect Status Register */
} Pio;
/* -------- PIO_PER : (PIO Offset: 0x0000) PIO Enable Register -------- */
#define PIO_PER_P0 (0x1u << 0) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P1 (0x1u << 1) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P2 (0x1u << 2) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P3 (0x1u << 3) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P4 (0x1u << 4) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P5 (0x1u << 5) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P6 (0x1u << 6) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P7 (0x1u << 7) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P8 (0x1u << 8) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P9 (0x1u << 9) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P10 (0x1u << 10) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P11 (0x1u << 11) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P12 (0x1u << 12) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P13 (0x1u << 13) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P14 (0x1u << 14) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P15 (0x1u << 15) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P16 (0x1u << 16) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P17 (0x1u << 17) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P18 (0x1u << 18) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P19 (0x1u << 19) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P20 (0x1u << 20) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P21 (0x1u << 21) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P22 (0x1u << 22) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P23 (0x1u << 23) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P24 (0x1u << 24) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P25 (0x1u << 25) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P26 (0x1u << 26) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P27 (0x1u << 27) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P28 (0x1u << 28) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P29 (0x1u << 29) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P30 (0x1u << 30) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P31 (0x1u << 31) /**< \brief (PIO_PER) PIO Enable */
/* -------- PIO_PDR : (PIO Offset: 0x0004) PIO Disable Register -------- */
#define PIO_PDR_P0 (0x1u << 0) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P1 (0x1u << 1) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P2 (0x1u << 2) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P3 (0x1u << 3) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P4 (0x1u << 4) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P5 (0x1u << 5) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P6 (0x1u << 6) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P7 (0x1u << 7) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P8 (0x1u << 8) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P9 (0x1u << 9) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P10 (0x1u << 10) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P11 (0x1u << 11) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P12 (0x1u << 12) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P13 (0x1u << 13) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P14 (0x1u << 14) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P15 (0x1u << 15) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P16 (0x1u << 16) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P17 (0x1u << 17) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P18 (0x1u << 18) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P19 (0x1u << 19) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P20 (0x1u << 20) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P21 (0x1u << 21) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P22 (0x1u << 22) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P23 (0x1u << 23) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P24 (0x1u << 24) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P25 (0x1u << 25) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P26 (0x1u << 26) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P27 (0x1u << 27) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P28 (0x1u << 28) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P29 (0x1u << 29) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P30 (0x1u << 30) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P31 (0x1u << 31) /**< \brief (PIO_PDR) PIO Disable */
/* -------- PIO_PSR : (PIO Offset: 0x0008) PIO Status Register -------- */
#define PIO_PSR_P0 (0x1u << 0) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P1 (0x1u << 1) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P2 (0x1u << 2) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P3 (0x1u << 3) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P4 (0x1u << 4) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P5 (0x1u << 5) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P6 (0x1u << 6) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P7 (0x1u << 7) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P8 (0x1u << 8) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P9 (0x1u << 9) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P10 (0x1u << 10) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P11 (0x1u << 11) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P12 (0x1u << 12) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P13 (0x1u << 13) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P14 (0x1u << 14) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P15 (0x1u << 15) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P16 (0x1u << 16) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P17 (0x1u << 17) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P18 (0x1u << 18) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P19 (0x1u << 19) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P20 (0x1u << 20) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P21 (0x1u << 21) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P22 (0x1u << 22) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P23 (0x1u << 23) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P24 (0x1u << 24) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P25 (0x1u << 25) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P26 (0x1u << 26) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P27 (0x1u << 27) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P28 (0x1u << 28) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P29 (0x1u << 29) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P30 (0x1u << 30) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P31 (0x1u << 31) /**< \brief (PIO_PSR) PIO Status */
/* -------- PIO_OER : (PIO Offset: 0x0010) Output Enable Register -------- */
#define PIO_OER_P0 (0x1u << 0) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P1 (0x1u << 1) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P2 (0x1u << 2) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P3 (0x1u << 3) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P4 (0x1u << 4) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P5 (0x1u << 5) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P6 (0x1u << 6) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P7 (0x1u << 7) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P8 (0x1u << 8) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P9 (0x1u << 9) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P10 (0x1u << 10) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P11 (0x1u << 11) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P12 (0x1u << 12) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P13 (0x1u << 13) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P14 (0x1u << 14) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P15 (0x1u << 15) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P16 (0x1u << 16) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P17 (0x1u << 17) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P18 (0x1u << 18) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P19 (0x1u << 19) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P20 (0x1u << 20) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P21 (0x1u << 21) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P22 (0x1u << 22) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P23 (0x1u << 23) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P24 (0x1u << 24) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P25 (0x1u << 25) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P26 (0x1u << 26) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P27 (0x1u << 27) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P28 (0x1u << 28) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P29 (0x1u << 29) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P30 (0x1u << 30) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P31 (0x1u << 31) /**< \brief (PIO_OER) Output Enable */
/* -------- PIO_ODR : (PIO Offset: 0x0014) Output Disable Register -------- */
#define PIO_ODR_P0 (0x1u << 0) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P1 (0x1u << 1) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P2 (0x1u << 2) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P3 (0x1u << 3) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P4 (0x1u << 4) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P5 (0x1u << 5) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P6 (0x1u << 6) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P7 (0x1u << 7) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P8 (0x1u << 8) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P9 (0x1u << 9) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P10 (0x1u << 10) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P11 (0x1u << 11) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P12 (0x1u << 12) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P13 (0x1u << 13) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P14 (0x1u << 14) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P15 (0x1u << 15) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P16 (0x1u << 16) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P17 (0x1u << 17) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P18 (0x1u << 18) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P19 (0x1u << 19) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P20 (0x1u << 20) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P21 (0x1u << 21) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P22 (0x1u << 22) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P23 (0x1u << 23) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P24 (0x1u << 24) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P25 (0x1u << 25) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P26 (0x1u << 26) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P27 (0x1u << 27) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P28 (0x1u << 28) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P29 (0x1u << 29) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P30 (0x1u << 30) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P31 (0x1u << 31) /**< \brief (PIO_ODR) Output Disable */
/* -------- PIO_OSR : (PIO Offset: 0x0018) Output Status Register -------- */
#define PIO_OSR_P0 (0x1u << 0) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P1 (0x1u << 1) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P2 (0x1u << 2) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P3 (0x1u << 3) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P4 (0x1u << 4) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P5 (0x1u << 5) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P6 (0x1u << 6) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P7 (0x1u << 7) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P8 (0x1u << 8) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P9 (0x1u << 9) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P10 (0x1u << 10) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P11 (0x1u << 11) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P12 (0x1u << 12) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P13 (0x1u << 13) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P14 (0x1u << 14) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P15 (0x1u << 15) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P16 (0x1u << 16) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P17 (0x1u << 17) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P18 (0x1u << 18) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P19 (0x1u << 19) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P20 (0x1u << 20) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P21 (0x1u << 21) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P22 (0x1u << 22) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P23 (0x1u << 23) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P24 (0x1u << 24) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P25 (0x1u << 25) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P26 (0x1u << 26) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P27 (0x1u << 27) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P28 (0x1u << 28) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P29 (0x1u << 29) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P30 (0x1u << 30) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P31 (0x1u << 31) /**< \brief (PIO_OSR) Output Status */
/* -------- PIO_IFER : (PIO Offset: 0x0020) Glitch Input Filter Enable Register -------- */
#define PIO_IFER_P0 (0x1u << 0) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P1 (0x1u << 1) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P2 (0x1u << 2) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P3 (0x1u << 3) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P4 (0x1u << 4) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P5 (0x1u << 5) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P6 (0x1u << 6) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P7 (0x1u << 7) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P8 (0x1u << 8) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P9 (0x1u << 9) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P10 (0x1u << 10) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P11 (0x1u << 11) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P12 (0x1u << 12) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P13 (0x1u << 13) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P14 (0x1u << 14) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P15 (0x1u << 15) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P16 (0x1u << 16) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P17 (0x1u << 17) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P18 (0x1u << 18) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P19 (0x1u << 19) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P20 (0x1u << 20) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P21 (0x1u << 21) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P22 (0x1u << 22) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P23 (0x1u << 23) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P24 (0x1u << 24) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P25 (0x1u << 25) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P26 (0x1u << 26) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P27 (0x1u << 27) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P28 (0x1u << 28) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P29 (0x1u << 29) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P30 (0x1u << 30) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P31 (0x1u << 31) /**< \brief (PIO_IFER) Input Filter Enable */
/* -------- PIO_IFDR : (PIO Offset: 0x0024) Glitch Input Filter Disable Register -------- */
#define PIO_IFDR_P0 (0x1u << 0) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P1 (0x1u << 1) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P2 (0x1u << 2) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P3 (0x1u << 3) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P4 (0x1u << 4) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P5 (0x1u << 5) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P6 (0x1u << 6) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P7 (0x1u << 7) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P8 (0x1u << 8) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P9 (0x1u << 9) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P10 (0x1u << 10) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P11 (0x1u << 11) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P12 (0x1u << 12) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P13 (0x1u << 13) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P14 (0x1u << 14) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P15 (0x1u << 15) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P16 (0x1u << 16) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P17 (0x1u << 17) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P18 (0x1u << 18) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P19 (0x1u << 19) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P20 (0x1u << 20) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P21 (0x1u << 21) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P22 (0x1u << 22) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P23 (0x1u << 23) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P24 (0x1u << 24) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P25 (0x1u << 25) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P26 (0x1u << 26) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P27 (0x1u << 27) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P28 (0x1u << 28) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P29 (0x1u << 29) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P30 (0x1u << 30) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P31 (0x1u << 31) /**< \brief (PIO_IFDR) Input Filter Disable */
/* -------- PIO_IFSR : (PIO Offset: 0x0028) Glitch Input Filter Status Register -------- */
#define PIO_IFSR_P0 (0x1u << 0) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P1 (0x1u << 1) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P2 (0x1u << 2) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P3 (0x1u << 3) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P4 (0x1u << 4) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P5 (0x1u << 5) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P6 (0x1u << 6) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P7 (0x1u << 7) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P8 (0x1u << 8) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P9 (0x1u << 9) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P10 (0x1u << 10) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P11 (0x1u << 11) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P12 (0x1u << 12) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P13 (0x1u << 13) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P14 (0x1u << 14) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P15 (0x1u << 15) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P16 (0x1u << 16) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P17 (0x1u << 17) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P18 (0x1u << 18) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P19 (0x1u << 19) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P20 (0x1u << 20) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P21 (0x1u << 21) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P22 (0x1u << 22) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P23 (0x1u << 23) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P24 (0x1u << 24) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P25 (0x1u << 25) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P26 (0x1u << 26) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P27 (0x1u << 27) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P28 (0x1u << 28) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P29 (0x1u << 29) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P30 (0x1u << 30) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P31 (0x1u << 31) /**< \brief (PIO_IFSR) Input Filer Status */
/* -------- PIO_SODR : (PIO Offset: 0x0030) Set Output Data Register -------- */
#define PIO_SODR_P0 (0x1u << 0) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P1 (0x1u << 1) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P2 (0x1u << 2) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P3 (0x1u << 3) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P4 (0x1u << 4) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P5 (0x1u << 5) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P6 (0x1u << 6) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P7 (0x1u << 7) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P8 (0x1u << 8) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P9 (0x1u << 9) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P10 (0x1u << 10) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P11 (0x1u << 11) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P12 (0x1u << 12) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P13 (0x1u << 13) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P14 (0x1u << 14) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P15 (0x1u << 15) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P16 (0x1u << 16) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P17 (0x1u << 17) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P18 (0x1u << 18) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P19 (0x1u << 19) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P20 (0x1u << 20) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P21 (0x1u << 21) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P22 (0x1u << 22) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P23 (0x1u << 23) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P24 (0x1u << 24) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P25 (0x1u << 25) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P26 (0x1u << 26) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P27 (0x1u << 27) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P28 (0x1u << 28) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P29 (0x1u << 29) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P30 (0x1u << 30) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P31 (0x1u << 31) /**< \brief (PIO_SODR) Set Output Data */
/* -------- PIO_CODR : (PIO Offset: 0x0034) Clear Output Data Register -------- */
#define PIO_CODR_P0 (0x1u << 0) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P1 (0x1u << 1) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P2 (0x1u << 2) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P3 (0x1u << 3) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P4 (0x1u << 4) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P5 (0x1u << 5) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P6 (0x1u << 6) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P7 (0x1u << 7) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P8 (0x1u << 8) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P9 (0x1u << 9) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P10 (0x1u << 10) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P11 (0x1u << 11) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P12 (0x1u << 12) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P13 (0x1u << 13) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P14 (0x1u << 14) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P15 (0x1u << 15) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P16 (0x1u << 16) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P17 (0x1u << 17) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P18 (0x1u << 18) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P19 (0x1u << 19) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P20 (0x1u << 20) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P21 (0x1u << 21) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P22 (0x1u << 22) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P23 (0x1u << 23) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P24 (0x1u << 24) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P25 (0x1u << 25) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P26 (0x1u << 26) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P27 (0x1u << 27) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P28 (0x1u << 28) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P29 (0x1u << 29) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P30 (0x1u << 30) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P31 (0x1u << 31) /**< \brief (PIO_CODR) Clear Output Data */
/* -------- PIO_ODSR : (PIO Offset: 0x0038) Output Data Status Register -------- */
#define PIO_ODSR_P0 (0x1u << 0) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P1 (0x1u << 1) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P2 (0x1u << 2) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P3 (0x1u << 3) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P4 (0x1u << 4) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P5 (0x1u << 5) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P6 (0x1u << 6) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P7 (0x1u << 7) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P8 (0x1u << 8) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P9 (0x1u << 9) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P10 (0x1u << 10) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P11 (0x1u << 11) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P12 (0x1u << 12) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P13 (0x1u << 13) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P14 (0x1u << 14) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P15 (0x1u << 15) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P16 (0x1u << 16) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P17 (0x1u << 17) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P18 (0x1u << 18) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P19 (0x1u << 19) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P20 (0x1u << 20) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P21 (0x1u << 21) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P22 (0x1u << 22) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P23 (0x1u << 23) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P24 (0x1u << 24) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P25 (0x1u << 25) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P26 (0x1u << 26) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P27 (0x1u << 27) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P28 (0x1u << 28) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P29 (0x1u << 29) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P30 (0x1u << 30) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P31 (0x1u << 31) /**< \brief (PIO_ODSR) Output Data Status */
/* -------- PIO_PDSR : (PIO Offset: 0x003C) Pin Data Status Register -------- */
#define PIO_PDSR_P0 (0x1u << 0) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P1 (0x1u << 1) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P2 (0x1u << 2) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P3 (0x1u << 3) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P4 (0x1u << 4) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P5 (0x1u << 5) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P6 (0x1u << 6) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P7 (0x1u << 7) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P8 (0x1u << 8) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P9 (0x1u << 9) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P10 (0x1u << 10) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P11 (0x1u << 11) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P12 (0x1u << 12) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P13 (0x1u << 13) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P14 (0x1u << 14) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P15 (0x1u << 15) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P16 (0x1u << 16) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P17 (0x1u << 17) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P18 (0x1u << 18) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P19 (0x1u << 19) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P20 (0x1u << 20) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P21 (0x1u << 21) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P22 (0x1u << 22) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P23 (0x1u << 23) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P24 (0x1u << 24) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P25 (0x1u << 25) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P26 (0x1u << 26) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P27 (0x1u << 27) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P28 (0x1u << 28) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P29 (0x1u << 29) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P30 (0x1u << 30) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P31 (0x1u << 31) /**< \brief (PIO_PDSR) Output Data Status */
/* -------- PIO_IER : (PIO Offset: 0x0040) Interrupt Enable Register -------- */
#define PIO_IER_P0 (0x1u << 0) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P1 (0x1u << 1) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P2 (0x1u << 2) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P3 (0x1u << 3) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P4 (0x1u << 4) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P5 (0x1u << 5) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P6 (0x1u << 6) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P7 (0x1u << 7) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P8 (0x1u << 8) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P9 (0x1u << 9) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P10 (0x1u << 10) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P11 (0x1u << 11) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P12 (0x1u << 12) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P13 (0x1u << 13) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P14 (0x1u << 14) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P15 (0x1u << 15) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P16 (0x1u << 16) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P17 (0x1u << 17) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P18 (0x1u << 18) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P19 (0x1u << 19) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P20 (0x1u << 20) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P21 (0x1u << 21) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P22 (0x1u << 22) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P23 (0x1u << 23) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P24 (0x1u << 24) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P25 (0x1u << 25) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P26 (0x1u << 26) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P27 (0x1u << 27) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P28 (0x1u << 28) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P29 (0x1u << 29) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P30 (0x1u << 30) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P31 (0x1u << 31) /**< \brief (PIO_IER) Input Change Interrupt Enable */
/* -------- PIO_IDR : (PIO Offset: 0x0044) Interrupt Disable Register -------- */
#define PIO_IDR_P0 (0x1u << 0) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P1 (0x1u << 1) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P2 (0x1u << 2) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P3 (0x1u << 3) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P4 (0x1u << 4) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P5 (0x1u << 5) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P6 (0x1u << 6) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P7 (0x1u << 7) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P8 (0x1u << 8) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P9 (0x1u << 9) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P10 (0x1u << 10) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P11 (0x1u << 11) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P12 (0x1u << 12) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P13 (0x1u << 13) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P14 (0x1u << 14) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P15 (0x1u << 15) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P16 (0x1u << 16) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P17 (0x1u << 17) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P18 (0x1u << 18) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P19 (0x1u << 19) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P20 (0x1u << 20) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P21 (0x1u << 21) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P22 (0x1u << 22) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P23 (0x1u << 23) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P24 (0x1u << 24) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P25 (0x1u << 25) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P26 (0x1u << 26) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P27 (0x1u << 27) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P28 (0x1u << 28) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P29 (0x1u << 29) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P30 (0x1u << 30) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P31 (0x1u << 31) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
/* -------- PIO_IMR : (PIO Offset: 0x0048) Interrupt Mask Register -------- */
#define PIO_IMR_P0 (0x1u << 0) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P1 (0x1u << 1) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P2 (0x1u << 2) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P3 (0x1u << 3) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P4 (0x1u << 4) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P5 (0x1u << 5) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P6 (0x1u << 6) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P7 (0x1u << 7) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P8 (0x1u << 8) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P9 (0x1u << 9) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P10 (0x1u << 10) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P11 (0x1u << 11) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P12 (0x1u << 12) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P13 (0x1u << 13) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P14 (0x1u << 14) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P15 (0x1u << 15) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P16 (0x1u << 16) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P17 (0x1u << 17) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P18 (0x1u << 18) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P19 (0x1u << 19) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P20 (0x1u << 20) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P21 (0x1u << 21) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P22 (0x1u << 22) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P23 (0x1u << 23) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P24 (0x1u << 24) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P25 (0x1u << 25) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P26 (0x1u << 26) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P27 (0x1u << 27) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P28 (0x1u << 28) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P29 (0x1u << 29) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P30 (0x1u << 30) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P31 (0x1u << 31) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
/* -------- PIO_ISR : (PIO Offset: 0x004C) Interrupt Status Register -------- */
#define PIO_ISR_P0 (0x1u << 0) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P1 (0x1u << 1) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P2 (0x1u << 2) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P3 (0x1u << 3) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P4 (0x1u << 4) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P5 (0x1u << 5) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P6 (0x1u << 6) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P7 (0x1u << 7) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P8 (0x1u << 8) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P9 (0x1u << 9) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P10 (0x1u << 10) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P11 (0x1u << 11) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P12 (0x1u << 12) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P13 (0x1u << 13) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P14 (0x1u << 14) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P15 (0x1u << 15) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P16 (0x1u << 16) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P17 (0x1u << 17) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P18 (0x1u << 18) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P19 (0x1u << 19) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P20 (0x1u << 20) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P21 (0x1u << 21) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P22 (0x1u << 22) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P23 (0x1u << 23) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P24 (0x1u << 24) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P25 (0x1u << 25) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P26 (0x1u << 26) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P27 (0x1u << 27) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P28 (0x1u << 28) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P29 (0x1u << 29) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P30 (0x1u << 30) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P31 (0x1u << 31) /**< \brief (PIO_ISR) Input Change Interrupt Status */
/* -------- PIO_MDER : (PIO Offset: 0x0050) Multi-driver Enable Register -------- */
#define PIO_MDER_P0 (0x1u << 0) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P1 (0x1u << 1) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P2 (0x1u << 2) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P3 (0x1u << 3) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P4 (0x1u << 4) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P5 (0x1u << 5) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P6 (0x1u << 6) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P7 (0x1u << 7) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P8 (0x1u << 8) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P9 (0x1u << 9) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P10 (0x1u << 10) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P11 (0x1u << 11) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P12 (0x1u << 12) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P13 (0x1u << 13) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P14 (0x1u << 14) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P15 (0x1u << 15) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P16 (0x1u << 16) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P17 (0x1u << 17) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P18 (0x1u << 18) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P19 (0x1u << 19) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P20 (0x1u << 20) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P21 (0x1u << 21) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P22 (0x1u << 22) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P23 (0x1u << 23) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P24 (0x1u << 24) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P25 (0x1u << 25) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P26 (0x1u << 26) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P27 (0x1u << 27) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P28 (0x1u << 28) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P29 (0x1u << 29) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P30 (0x1u << 30) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P31 (0x1u << 31) /**< \brief (PIO_MDER) Multi Drive Enable. */
/* -------- PIO_MDDR : (PIO Offset: 0x0054) Multi-driver Disable Register -------- */
#define PIO_MDDR_P0 (0x1u << 0) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P1 (0x1u << 1) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P2 (0x1u << 2) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P3 (0x1u << 3) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P4 (0x1u << 4) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P5 (0x1u << 5) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P6 (0x1u << 6) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P7 (0x1u << 7) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P8 (0x1u << 8) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P9 (0x1u << 9) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P10 (0x1u << 10) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P11 (0x1u << 11) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P12 (0x1u << 12) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P13 (0x1u << 13) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P14 (0x1u << 14) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P15 (0x1u << 15) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P16 (0x1u << 16) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P17 (0x1u << 17) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P18 (0x1u << 18) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P19 (0x1u << 19) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P20 (0x1u << 20) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P21 (0x1u << 21) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P22 (0x1u << 22) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P23 (0x1u << 23) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P24 (0x1u << 24) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P25 (0x1u << 25) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P26 (0x1u << 26) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P27 (0x1u << 27) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P28 (0x1u << 28) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P29 (0x1u << 29) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P30 (0x1u << 30) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P31 (0x1u << 31) /**< \brief (PIO_MDDR) Multi Drive Disable. */
/* -------- PIO_MDSR : (PIO Offset: 0x0058) Multi-driver Status Register -------- */
#define PIO_MDSR_P0 (0x1u << 0) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P1 (0x1u << 1) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P2 (0x1u << 2) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P3 (0x1u << 3) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P4 (0x1u << 4) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P5 (0x1u << 5) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P6 (0x1u << 6) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P7 (0x1u << 7) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P8 (0x1u << 8) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P9 (0x1u << 9) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P10 (0x1u << 10) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P11 (0x1u << 11) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P12 (0x1u << 12) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P13 (0x1u << 13) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P14 (0x1u << 14) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P15 (0x1u << 15) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P16 (0x1u << 16) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P17 (0x1u << 17) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P18 (0x1u << 18) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P19 (0x1u << 19) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P20 (0x1u << 20) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P21 (0x1u << 21) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P22 (0x1u << 22) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P23 (0x1u << 23) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P24 (0x1u << 24) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P25 (0x1u << 25) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P26 (0x1u << 26) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P27 (0x1u << 27) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P28 (0x1u << 28) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P29 (0x1u << 29) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P30 (0x1u << 30) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P31 (0x1u << 31) /**< \brief (PIO_MDSR) Multi Drive Status. */
/* -------- PIO_PUDR : (PIO Offset: 0x0060) Pull-up Disable Register -------- */
#define PIO_PUDR_P0 (0x1u << 0) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P1 (0x1u << 1) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P2 (0x1u << 2) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P3 (0x1u << 3) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P4 (0x1u << 4) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P5 (0x1u << 5) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P6 (0x1u << 6) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P7 (0x1u << 7) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P8 (0x1u << 8) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P9 (0x1u << 9) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P10 (0x1u << 10) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P11 (0x1u << 11) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P12 (0x1u << 12) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P13 (0x1u << 13) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P14 (0x1u << 14) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P15 (0x1u << 15) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P16 (0x1u << 16) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P17 (0x1u << 17) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P18 (0x1u << 18) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P19 (0x1u << 19) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P20 (0x1u << 20) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P21 (0x1u << 21) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P22 (0x1u << 22) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P23 (0x1u << 23) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P24 (0x1u << 24) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P25 (0x1u << 25) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P26 (0x1u << 26) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P27 (0x1u << 27) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P28 (0x1u << 28) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P29 (0x1u << 29) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P30 (0x1u << 30) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P31 (0x1u << 31) /**< \brief (PIO_PUDR) Pull Up Disable. */
/* -------- PIO_PUER : (PIO Offset: 0x0064) Pull-up Enable Register -------- */
#define PIO_PUER_P0 (0x1u << 0) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P1 (0x1u << 1) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P2 (0x1u << 2) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P3 (0x1u << 3) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P4 (0x1u << 4) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P5 (0x1u << 5) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P6 (0x1u << 6) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P7 (0x1u << 7) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P8 (0x1u << 8) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P9 (0x1u << 9) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P10 (0x1u << 10) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P11 (0x1u << 11) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P12 (0x1u << 12) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P13 (0x1u << 13) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P14 (0x1u << 14) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P15 (0x1u << 15) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P16 (0x1u << 16) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P17 (0x1u << 17) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P18 (0x1u << 18) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P19 (0x1u << 19) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P20 (0x1u << 20) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P21 (0x1u << 21) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P22 (0x1u << 22) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P23 (0x1u << 23) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P24 (0x1u << 24) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P25 (0x1u << 25) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P26 (0x1u << 26) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P27 (0x1u << 27) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P28 (0x1u << 28) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P29 (0x1u << 29) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P30 (0x1u << 30) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P31 (0x1u << 31) /**< \brief (PIO_PUER) Pull Up Enable. */
/* -------- PIO_PUSR : (PIO Offset: 0x0068) Pad Pull-up Status Register -------- */
#define PIO_PUSR_P0 (0x1u << 0) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P1 (0x1u << 1) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P2 (0x1u << 2) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P3 (0x1u << 3) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P4 (0x1u << 4) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P5 (0x1u << 5) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P6 (0x1u << 6) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P7 (0x1u << 7) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P8 (0x1u << 8) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P9 (0x1u << 9) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P10 (0x1u << 10) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P11 (0x1u << 11) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P12 (0x1u << 12) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P13 (0x1u << 13) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P14 (0x1u << 14) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P15 (0x1u << 15) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P16 (0x1u << 16) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P17 (0x1u << 17) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P18 (0x1u << 18) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P19 (0x1u << 19) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P20 (0x1u << 20) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P21 (0x1u << 21) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P22 (0x1u << 22) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P23 (0x1u << 23) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P24 (0x1u << 24) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P25 (0x1u << 25) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P26 (0x1u << 26) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P27 (0x1u << 27) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P28 (0x1u << 28) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P29 (0x1u << 29) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P30 (0x1u << 30) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P31 (0x1u << 31) /**< \brief (PIO_PUSR) Pull Up Status. */
/* -------- PIO_ABSR : (PIO Offset: 0x0070) Peripheral AB Select Register -------- */
#define PIO_ABSR_P0 (0x1u << 0) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P1 (0x1u << 1) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P2 (0x1u << 2) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P3 (0x1u << 3) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P4 (0x1u << 4) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P5 (0x1u << 5) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P6 (0x1u << 6) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P7 (0x1u << 7) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P8 (0x1u << 8) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P9 (0x1u << 9) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P10 (0x1u << 10) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P11 (0x1u << 11) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P12 (0x1u << 12) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P13 (0x1u << 13) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P14 (0x1u << 14) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P15 (0x1u << 15) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P16 (0x1u << 16) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P17 (0x1u << 17) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P18 (0x1u << 18) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P19 (0x1u << 19) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P20 (0x1u << 20) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P21 (0x1u << 21) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P22 (0x1u << 22) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P23 (0x1u << 23) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P24 (0x1u << 24) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P25 (0x1u << 25) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P26 (0x1u << 26) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P27 (0x1u << 27) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P28 (0x1u << 28) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P29 (0x1u << 29) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P30 (0x1u << 30) /**< \brief (PIO_ABSR) Peripheral A Select. */
#define PIO_ABSR_P31 (0x1u << 31) /**< \brief (PIO_ABSR) Peripheral A Select. */
/* -------- PIO_SCIFSR : (PIO Offset: 0x0080) System Clock Glitch Input Filter Select Register -------- */
#define PIO_SCIFSR_P0 (0x1u << 0) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P1 (0x1u << 1) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P2 (0x1u << 2) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P3 (0x1u << 3) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P4 (0x1u << 4) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P5 (0x1u << 5) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P6 (0x1u << 6) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P7 (0x1u << 7) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P8 (0x1u << 8) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P9 (0x1u << 9) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P10 (0x1u << 10) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P11 (0x1u << 11) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P12 (0x1u << 12) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P13 (0x1u << 13) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P14 (0x1u << 14) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P15 (0x1u << 15) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P16 (0x1u << 16) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P17 (0x1u << 17) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P18 (0x1u << 18) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P19 (0x1u << 19) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P20 (0x1u << 20) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P21 (0x1u << 21) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P22 (0x1u << 22) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P23 (0x1u << 23) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P24 (0x1u << 24) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P25 (0x1u << 25) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P26 (0x1u << 26) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P27 (0x1u << 27) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P28 (0x1u << 28) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P29 (0x1u << 29) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P30 (0x1u << 30) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
#define PIO_SCIFSR_P31 (0x1u << 31) /**< \brief (PIO_SCIFSR) System Clock Glitch Filtering Select. */
/* -------- PIO_DIFSR : (PIO Offset: 0x0084) Debouncing Input Filter Select Register -------- */
#define PIO_DIFSR_P0 (0x1u << 0) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P1 (0x1u << 1) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P2 (0x1u << 2) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P3 (0x1u << 3) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P4 (0x1u << 4) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P5 (0x1u << 5) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P6 (0x1u << 6) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P7 (0x1u << 7) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P8 (0x1u << 8) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P9 (0x1u << 9) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P10 (0x1u << 10) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P11 (0x1u << 11) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P12 (0x1u << 12) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P13 (0x1u << 13) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P14 (0x1u << 14) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P15 (0x1u << 15) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P16 (0x1u << 16) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P17 (0x1u << 17) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P18 (0x1u << 18) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P19 (0x1u << 19) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P20 (0x1u << 20) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P21 (0x1u << 21) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P22 (0x1u << 22) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P23 (0x1u << 23) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P24 (0x1u << 24) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P25 (0x1u << 25) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P26 (0x1u << 26) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P27 (0x1u << 27) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P28 (0x1u << 28) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P29 (0x1u << 29) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P30 (0x1u << 30) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
#define PIO_DIFSR_P31 (0x1u << 31) /**< \brief (PIO_DIFSR) Debouncing Filtering Select. */
/* -------- PIO_IFDGSR : (PIO Offset: 0x0088) Glitch or Debouncing Input Filter Clock Selection Status Register -------- */
#define PIO_IFDGSR_P0 (0x1u << 0) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P1 (0x1u << 1) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P2 (0x1u << 2) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P3 (0x1u << 3) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P4 (0x1u << 4) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P5 (0x1u << 5) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P6 (0x1u << 6) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P7 (0x1u << 7) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P8 (0x1u << 8) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P9 (0x1u << 9) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P10 (0x1u << 10) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P11 (0x1u << 11) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P12 (0x1u << 12) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P13 (0x1u << 13) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P14 (0x1u << 14) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P15 (0x1u << 15) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P16 (0x1u << 16) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P17 (0x1u << 17) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P18 (0x1u << 18) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P19 (0x1u << 19) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P20 (0x1u << 20) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P21 (0x1u << 21) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P22 (0x1u << 22) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P23 (0x1u << 23) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P24 (0x1u << 24) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P25 (0x1u << 25) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P26 (0x1u << 26) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P27 (0x1u << 27) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P28 (0x1u << 28) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P29 (0x1u << 29) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P30 (0x1u << 30) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFDGSR_P31 (0x1u << 31) /**< \brief (PIO_IFDGSR) Glitch or Debouncing Filter Selection Status */
/* -------- PIO_SCDR : (PIO Offset: 0x008C) Slow Clock Divider Debouncing Register -------- */
#define PIO_SCDR_DIV_Pos 0
#define PIO_SCDR_DIV_Msk (0x3fffu << PIO_SCDR_DIV_Pos) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV(value) ((PIO_SCDR_DIV_Msk & ((value) << PIO_SCDR_DIV_Pos)))
/* -------- PIO_OWER : (PIO Offset: 0x00A0) Output Write Enable -------- */
#define PIO_OWER_P0 (0x1u << 0) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P1 (0x1u << 1) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P2 (0x1u << 2) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P3 (0x1u << 3) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P4 (0x1u << 4) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P5 (0x1u << 5) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P6 (0x1u << 6) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P7 (0x1u << 7) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P8 (0x1u << 8) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P9 (0x1u << 9) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P10 (0x1u << 10) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P11 (0x1u << 11) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P12 (0x1u << 12) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P13 (0x1u << 13) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P14 (0x1u << 14) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P15 (0x1u << 15) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P16 (0x1u << 16) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P17 (0x1u << 17) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P18 (0x1u << 18) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P19 (0x1u << 19) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P20 (0x1u << 20) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P21 (0x1u << 21) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P22 (0x1u << 22) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P23 (0x1u << 23) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P24 (0x1u << 24) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P25 (0x1u << 25) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P26 (0x1u << 26) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P27 (0x1u << 27) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P28 (0x1u << 28) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P29 (0x1u << 29) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P30 (0x1u << 30) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P31 (0x1u << 31) /**< \brief (PIO_OWER) Output Write Enable. */
/* -------- PIO_OWDR : (PIO Offset: 0x00A4) Output Write Disable -------- */
#define PIO_OWDR_P0 (0x1u << 0) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P1 (0x1u << 1) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P2 (0x1u << 2) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P3 (0x1u << 3) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P4 (0x1u << 4) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P5 (0x1u << 5) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P6 (0x1u << 6) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P7 (0x1u << 7) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P8 (0x1u << 8) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P9 (0x1u << 9) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P10 (0x1u << 10) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P11 (0x1u << 11) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P12 (0x1u << 12) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P13 (0x1u << 13) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P14 (0x1u << 14) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P15 (0x1u << 15) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P16 (0x1u << 16) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P17 (0x1u << 17) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P18 (0x1u << 18) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P19 (0x1u << 19) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P20 (0x1u << 20) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P21 (0x1u << 21) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P22 (0x1u << 22) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P23 (0x1u << 23) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P24 (0x1u << 24) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P25 (0x1u << 25) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P26 (0x1u << 26) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P27 (0x1u << 27) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P28 (0x1u << 28) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P29 (0x1u << 29) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P30 (0x1u << 30) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P31 (0x1u << 31) /**< \brief (PIO_OWDR) Output Write Disable. */
/* -------- PIO_OWSR : (PIO Offset: 0x00A8) Output Write Status Register -------- */
#define PIO_OWSR_P0 (0x1u << 0) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P1 (0x1u << 1) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P2 (0x1u << 2) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P3 (0x1u << 3) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P4 (0x1u << 4) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P5 (0x1u << 5) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P6 (0x1u << 6) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P7 (0x1u << 7) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P8 (0x1u << 8) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P9 (0x1u << 9) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P10 (0x1u << 10) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P11 (0x1u << 11) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P12 (0x1u << 12) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P13 (0x1u << 13) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P14 (0x1u << 14) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P15 (0x1u << 15) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P16 (0x1u << 16) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P17 (0x1u << 17) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P18 (0x1u << 18) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P19 (0x1u << 19) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P20 (0x1u << 20) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P21 (0x1u << 21) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P22 (0x1u << 22) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P23 (0x1u << 23) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P24 (0x1u << 24) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P25 (0x1u << 25) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P26 (0x1u << 26) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P27 (0x1u << 27) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P28 (0x1u << 28) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P29 (0x1u << 29) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P30 (0x1u << 30) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P31 (0x1u << 31) /**< \brief (PIO_OWSR) Output Write Status. */
/* -------- PIO_AIMER : (PIO Offset: 0x00B0) Additional Interrupt Modes Enable Register -------- */
#define PIO_AIMER_P0 (0x1u << 0) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P1 (0x1u << 1) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P2 (0x1u << 2) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P3 (0x1u << 3) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P4 (0x1u << 4) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P5 (0x1u << 5) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P6 (0x1u << 6) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P7 (0x1u << 7) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P8 (0x1u << 8) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P9 (0x1u << 9) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P10 (0x1u << 10) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P11 (0x1u << 11) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P12 (0x1u << 12) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P13 (0x1u << 13) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P14 (0x1u << 14) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P15 (0x1u << 15) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P16 (0x1u << 16) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P17 (0x1u << 17) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P18 (0x1u << 18) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P19 (0x1u << 19) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P20 (0x1u << 20) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P21 (0x1u << 21) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P22 (0x1u << 22) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P23 (0x1u << 23) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P24 (0x1u << 24) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P25 (0x1u << 25) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P26 (0x1u << 26) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P27 (0x1u << 27) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P28 (0x1u << 28) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P29 (0x1u << 29) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P30 (0x1u << 30) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P31 (0x1u << 31) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
/* -------- PIO_AIMDR : (PIO Offset: 0x00B4) Additional Interrupt Modes Disables Register -------- */
#define PIO_AIMDR_P0 (0x1u << 0) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P1 (0x1u << 1) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P2 (0x1u << 2) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P3 (0x1u << 3) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P4 (0x1u << 4) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P5 (0x1u << 5) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P6 (0x1u << 6) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P7 (0x1u << 7) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P8 (0x1u << 8) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P9 (0x1u << 9) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P10 (0x1u << 10) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P11 (0x1u << 11) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P12 (0x1u << 12) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P13 (0x1u << 13) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P14 (0x1u << 14) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P15 (0x1u << 15) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P16 (0x1u << 16) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P17 (0x1u << 17) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P18 (0x1u << 18) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P19 (0x1u << 19) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P20 (0x1u << 20) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P21 (0x1u << 21) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P22 (0x1u << 22) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P23 (0x1u << 23) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P24 (0x1u << 24) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P25 (0x1u << 25) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P26 (0x1u << 26) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P27 (0x1u << 27) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P28 (0x1u << 28) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P29 (0x1u << 29) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P30 (0x1u << 30) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P31 (0x1u << 31) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
/* -------- PIO_AIMMR : (PIO Offset: 0x00B8) Additional Interrupt Modes Mask Register -------- */
#define PIO_AIMMR_P0 (0x1u << 0) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P1 (0x1u << 1) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P2 (0x1u << 2) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P3 (0x1u << 3) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P4 (0x1u << 4) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P5 (0x1u << 5) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P6 (0x1u << 6) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P7 (0x1u << 7) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P8 (0x1u << 8) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P9 (0x1u << 9) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P10 (0x1u << 10) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P11 (0x1u << 11) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P12 (0x1u << 12) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P13 (0x1u << 13) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P14 (0x1u << 14) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P15 (0x1u << 15) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P16 (0x1u << 16) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P17 (0x1u << 17) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P18 (0x1u << 18) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P19 (0x1u << 19) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P20 (0x1u << 20) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P21 (0x1u << 21) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P22 (0x1u << 22) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P23 (0x1u << 23) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P24 (0x1u << 24) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P25 (0x1u << 25) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P26 (0x1u << 26) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P27 (0x1u << 27) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P28 (0x1u << 28) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P29 (0x1u << 29) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P30 (0x1u << 30) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P31 (0x1u << 31) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
/* -------- PIO_ESR : (PIO Offset: 0x00C0) Edge Select Register -------- */
#define PIO_ESR_P0 (0x1u << 0) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P1 (0x1u << 1) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P2 (0x1u << 2) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P3 (0x1u << 3) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P4 (0x1u << 4) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P5 (0x1u << 5) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P6 (0x1u << 6) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P7 (0x1u << 7) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P8 (0x1u << 8) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P9 (0x1u << 9) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P10 (0x1u << 10) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P11 (0x1u << 11) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P12 (0x1u << 12) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P13 (0x1u << 13) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P14 (0x1u << 14) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P15 (0x1u << 15) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P16 (0x1u << 16) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P17 (0x1u << 17) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P18 (0x1u << 18) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P19 (0x1u << 19) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P20 (0x1u << 20) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P21 (0x1u << 21) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P22 (0x1u << 22) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P23 (0x1u << 23) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P24 (0x1u << 24) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P25 (0x1u << 25) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P26 (0x1u << 26) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P27 (0x1u << 27) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P28 (0x1u << 28) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P29 (0x1u << 29) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P30 (0x1u << 30) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P31 (0x1u << 31) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
/* -------- PIO_LSR : (PIO Offset: 0x00C4) Level Select Register -------- */
#define PIO_LSR_P0 (0x1u << 0) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P1 (0x1u << 1) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P2 (0x1u << 2) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P3 (0x1u << 3) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P4 (0x1u << 4) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P5 (0x1u << 5) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P6 (0x1u << 6) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P7 (0x1u << 7) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P8 (0x1u << 8) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P9 (0x1u << 9) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P10 (0x1u << 10) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P11 (0x1u << 11) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P12 (0x1u << 12) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P13 (0x1u << 13) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P14 (0x1u << 14) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P15 (0x1u << 15) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P16 (0x1u << 16) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P17 (0x1u << 17) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P18 (0x1u << 18) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P19 (0x1u << 19) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P20 (0x1u << 20) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P21 (0x1u << 21) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P22 (0x1u << 22) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P23 (0x1u << 23) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P24 (0x1u << 24) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P25 (0x1u << 25) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P26 (0x1u << 26) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P27 (0x1u << 27) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P28 (0x1u << 28) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P29 (0x1u << 29) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P30 (0x1u << 30) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P31 (0x1u << 31) /**< \brief (PIO_LSR) Level Interrupt Selection. */
/* -------- PIO_ELSR : (PIO Offset: 0x00C8) Edge/Level Status Register -------- */
#define PIO_ELSR_P0 (0x1u << 0) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P1 (0x1u << 1) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P2 (0x1u << 2) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P3 (0x1u << 3) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P4 (0x1u << 4) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P5 (0x1u << 5) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P6 (0x1u << 6) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P7 (0x1u << 7) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P8 (0x1u << 8) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P9 (0x1u << 9) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P10 (0x1u << 10) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P11 (0x1u << 11) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P12 (0x1u << 12) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P13 (0x1u << 13) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P14 (0x1u << 14) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P15 (0x1u << 15) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P16 (0x1u << 16) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P17 (0x1u << 17) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P18 (0x1u << 18) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P19 (0x1u << 19) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P20 (0x1u << 20) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P21 (0x1u << 21) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P22 (0x1u << 22) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P23 (0x1u << 23) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P24 (0x1u << 24) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P25 (0x1u << 25) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P26 (0x1u << 26) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P27 (0x1u << 27) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P28 (0x1u << 28) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P29 (0x1u << 29) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P30 (0x1u << 30) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P31 (0x1u << 31) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
/* -------- PIO_FELLSR : (PIO Offset: 0x00D0) Falling Edge/Low Level Select Register -------- */
#define PIO_FELLSR_P0 (0x1u << 0) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P1 (0x1u << 1) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P2 (0x1u << 2) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P3 (0x1u << 3) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P4 (0x1u << 4) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P5 (0x1u << 5) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P6 (0x1u << 6) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P7 (0x1u << 7) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P8 (0x1u << 8) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P9 (0x1u << 9) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P10 (0x1u << 10) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P11 (0x1u << 11) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P12 (0x1u << 12) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P13 (0x1u << 13) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P14 (0x1u << 14) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P15 (0x1u << 15) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P16 (0x1u << 16) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P17 (0x1u << 17) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P18 (0x1u << 18) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P19 (0x1u << 19) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P20 (0x1u << 20) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P21 (0x1u << 21) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P22 (0x1u << 22) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P23 (0x1u << 23) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P24 (0x1u << 24) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P25 (0x1u << 25) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P26 (0x1u << 26) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P27 (0x1u << 27) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P28 (0x1u << 28) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P29 (0x1u << 29) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P30 (0x1u << 30) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P31 (0x1u << 31) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
/* -------- PIO_REHLSR : (PIO Offset: 0x00D4) Rising Edge/ High Level Select Register -------- */
#define PIO_REHLSR_P0 (0x1u << 0) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P1 (0x1u << 1) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P2 (0x1u << 2) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P3 (0x1u << 3) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P4 (0x1u << 4) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P5 (0x1u << 5) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P6 (0x1u << 6) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P7 (0x1u << 7) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P8 (0x1u << 8) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P9 (0x1u << 9) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P10 (0x1u << 10) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P11 (0x1u << 11) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P12 (0x1u << 12) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P13 (0x1u << 13) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P14 (0x1u << 14) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P15 (0x1u << 15) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P16 (0x1u << 16) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P17 (0x1u << 17) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P18 (0x1u << 18) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P19 (0x1u << 19) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P20 (0x1u << 20) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P21 (0x1u << 21) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P22 (0x1u << 22) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P23 (0x1u << 23) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P24 (0x1u << 24) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P25 (0x1u << 25) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P26 (0x1u << 26) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P27 (0x1u << 27) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P28 (0x1u << 28) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P29 (0x1u << 29) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P30 (0x1u << 30) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P31 (0x1u << 31) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
/* -------- PIO_FRLHSR : (PIO Offset: 0x00D8) Fall/Rise - Low/High Status Register -------- */
#define PIO_FRLHSR_P0 (0x1u << 0) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P1 (0x1u << 1) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P2 (0x1u << 2) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P3 (0x1u << 3) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P4 (0x1u << 4) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P5 (0x1u << 5) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P6 (0x1u << 6) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P7 (0x1u << 7) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P8 (0x1u << 8) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P9 (0x1u << 9) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P10 (0x1u << 10) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P11 (0x1u << 11) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P12 (0x1u << 12) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P13 (0x1u << 13) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P14 (0x1u << 14) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P15 (0x1u << 15) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P16 (0x1u << 16) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P17 (0x1u << 17) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P18 (0x1u << 18) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P19 (0x1u << 19) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P20 (0x1u << 20) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P21 (0x1u << 21) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P22 (0x1u << 22) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P23 (0x1u << 23) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P24 (0x1u << 24) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P25 (0x1u << 25) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P26 (0x1u << 26) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P27 (0x1u << 27) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P28 (0x1u << 28) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P29 (0x1u << 29) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P30 (0x1u << 30) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P31 (0x1u << 31) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
/* -------- PIO_LOCKSR : (PIO Offset: 0x00E0) Lock Status -------- */
#define PIO_LOCKSR_P0 (0x1u << 0) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P1 (0x1u << 1) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P2 (0x1u << 2) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P3 (0x1u << 3) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P4 (0x1u << 4) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P5 (0x1u << 5) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P6 (0x1u << 6) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P7 (0x1u << 7) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P8 (0x1u << 8) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P9 (0x1u << 9) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P10 (0x1u << 10) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P11 (0x1u << 11) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P12 (0x1u << 12) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P13 (0x1u << 13) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P14 (0x1u << 14) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P15 (0x1u << 15) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P16 (0x1u << 16) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P17 (0x1u << 17) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P18 (0x1u << 18) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P19 (0x1u << 19) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P20 (0x1u << 20) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P21 (0x1u << 21) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P22 (0x1u << 22) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P23 (0x1u << 23) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P24 (0x1u << 24) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P25 (0x1u << 25) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P26 (0x1u << 26) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P27 (0x1u << 27) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P28 (0x1u << 28) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P29 (0x1u << 29) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P30 (0x1u << 30) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P31 (0x1u << 31) /**< \brief (PIO_LOCKSR) Lock Status. */
/* -------- PIO_WPMR : (PIO Offset: 0x00E4) Write Protect Mode Register -------- */
#define PIO_WPMR_WPEN (0x1u << 0) /**< \brief (PIO_WPMR) Write Protect Enable */
#define PIO_WPMR_WPKEY_Pos 8
#define PIO_WPMR_WPKEY_Msk (0xffffffu << PIO_WPMR_WPKEY_Pos) /**< \brief (PIO_WPMR) Write Protect KEY */
#define PIO_WPMR_WPKEY(value) ((PIO_WPMR_WPKEY_Msk & ((value) << PIO_WPMR_WPKEY_Pos)))
/* -------- PIO_WPSR : (PIO Offset: 0x00E8) Write Protect Status Register -------- */
#define PIO_WPSR_WPVS (0x1u << 0) /**< \brief (PIO_WPSR) Write Protect Violation Status */
#define PIO_WPSR_WPVSRC_Pos 8
#define PIO_WPSR_WPVSRC_Msk (0xffffu << PIO_WPSR_WPVSRC_Pos) /**< \brief (PIO_WPSR) Write Protect Violation Source */
/*@}*/ /* end of group SAM3XA_PIO */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Power Management Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_PMC Power Management Controller */
/*@{*/

/** \brief Pmc hardware registers */
typedef struct {
  WoReg PMC_SCER;      /**< \brief (Pmc Offset: 0x0000) System Clock Enable Register */
  WoReg PMC_SCDR;      /**< \brief (Pmc Offset: 0x0004) System Clock Disable Register */
  RoReg PMC_SCSR;      /**< \brief (Pmc Offset: 0x0008) System Clock Status Register */
  RoReg Reserved1[1];
  WoReg PMC_PCER0;     /**< \brief (Pmc Offset: 0x0010) Peripheral Clock Enable Register 0 */
  WoReg PMC_PCDR0;     /**< \brief (Pmc Offset: 0x0014) Peripheral Clock Disable Register 0 */
  RoReg PMC_PCSR0;     /**< \brief (Pmc Offset: 0x0018) Peripheral Clock Status Register 0 */
  RwReg CKGR_UCKR;     /**< \brief (Pmc Offset: 0x001C) UTMI Clock Register */
  RwReg CKGR_MOR;      /**< \brief (Pmc Offset: 0x0020) Main Oscillator Register */
  RoReg CKGR_MCFR;     /**< \brief (Pmc Offset: 0x0024) Main Clock Frequency Register */
  RwReg CKGR_PLLAR;    /**< \brief (Pmc Offset: 0x0028) PLLA Register */
  RoReg Reserved2[1];
  RwReg PMC_MCKR;      /**< \brief (Pmc Offset: 0x0030) Master Clock Register */
  RoReg Reserved3[1];
  RwReg PMC_USB;       /**< \brief (Pmc Offset: 0x0038) USB Clock Register */
  RoReg Reserved4[1];
  RwReg PMC_PCK[3];    /**< \brief (Pmc Offset: 0x0040) Programmable Clock 0 Register */
  RoReg Reserved5[5];
  WoReg PMC_IER;       /**< \brief (Pmc Offset: 0x0060) Interrupt Enable Register */
  WoReg PMC_IDR;       /**< \brief (Pmc Offset: 0x0064) Interrupt Disable Register */
  RoReg PMC_SR;        /**< \brief (Pmc Offset: 0x0068) Status Register */
  RoReg PMC_IMR;       /**< \brief (Pmc Offset: 0x006C) Interrupt Mask Register */
  RwReg PMC_FSMR;      /**< \brief (Pmc Offset: 0x0070) Fast Startup Mode Register */
  RwReg PMC_FSPR;      /**< \brief (Pmc Offset: 0x0074) Fast Startup Polarity Register */
  WoReg PMC_FOCR;      /**< \brief (Pmc Offset: 0x0078) Fault Output Clear Register */
  RoReg Reserved6[26];
  RwReg PMC_WPMR;      /**< \brief (Pmc Offset: 0x00E4) Write Protect Mode Register */
  RoReg PMC_WPSR;      /**< \brief (Pmc Offset: 0x00E8) Write Protect Status Register */
  RoReg Reserved7[5];
  WoReg PMC_PCER1;     /**< \brief (Pmc Offset: 0x0100) Peripheral Clock Enable Register 1 */
  WoReg PMC_PCDR1;     /**< \brief (Pmc Offset: 0x0104) Peripheral Clock Disable Register 1 */
  RoReg PMC_PCSR1;     /**< \brief (Pmc Offset: 0x0108) Peripheral Clock Status Register 1 */
  RwReg PMC_PCR;       /**< \brief (Pmc Offset: 0x010C) Peripheral Control Register */
} Pmc;
/* -------- PMC_SCER : (PMC Offset: 0x0000) System Clock Enable Register -------- */
#define PMC_SCER_UOTGCLK (0x1u << 5) /**< \brief (PMC_SCER) Enable USB OTG Clock (48 MHz, USB_48M) for UTMI */
#define PMC_SCER_PCK0 (0x1u << 8) /**< \brief (PMC_SCER) Programmable Clock 0 Output Enable */
#define PMC_SCER_PCK1 (0x1u << 9) /**< \brief (PMC_SCER) Programmable Clock 1 Output Enable */
#define PMC_SCER_PCK2 (0x1u << 10) /**< \brief (PMC_SCER) Programmable Clock 2 Output Enable */
/* -------- PMC_SCDR : (PMC Offset: 0x0004) System Clock Disable Register -------- */
#define PMC_SCDR_UOTGCLK (0x1u << 5) /**< \brief (PMC_SCDR) Disable USB OTG Clock (48 MHz, USB_48M) for UTMI */
#define PMC_SCDR_PCK0 (0x1u << 8) /**< \brief (PMC_SCDR) Programmable Clock 0 Output Disable */
#define PMC_SCDR_PCK1 (0x1u << 9) /**< \brief (PMC_SCDR) Programmable Clock 1 Output Disable */
#define PMC_SCDR_PCK2 (0x1u << 10) /**< \brief (PMC_SCDR) Programmable Clock 2 Output Disable */
/* -------- PMC_SCSR : (PMC Offset: 0x0008) System Clock Status Register -------- */
#define PMC_SCSR_UOTGCLK (0x1u << 5) /**< \brief (PMC_SCSR) USB OTG Clock (48 MHz, USB_48M) Clock Status */
#define PMC_SCSR_PCK0 (0x1u << 8) /**< \brief (PMC_SCSR) Programmable Clock 0 Output Status */
#define PMC_SCSR_PCK1 (0x1u << 9) /**< \brief (PMC_SCSR) Programmable Clock 1 Output Status */
#define PMC_SCSR_PCK2 (0x1u << 10) /**< \brief (PMC_SCSR) Programmable Clock 2 Output Status */
/* -------- PMC_PCER0 : (PMC Offset: 0x0010) Peripheral Clock Enable Register 0 -------- */
#define PMC_PCER0_PID2 (0x1u << 2) /**< \brief (PMC_PCER0) Peripheral Clock 2 Enable */
#define PMC_PCER0_PID3 (0x1u << 3) /**< \brief (PMC_PCER0) Peripheral Clock 3 Enable */
#define PMC_PCER0_PID4 (0x1u << 4) /**< \brief (PMC_PCER0) Peripheral Clock 4 Enable */
#define PMC_PCER0_PID5 (0x1u << 5) /**< \brief (PMC_PCER0) Peripheral Clock 5 Enable */
#define PMC_PCER0_PID6 (0x1u << 6) /**< \brief (PMC_PCER0) Peripheral Clock 6 Enable */
#define PMC_PCER0_PID7 (0x1u << 7) /**< \brief (PMC_PCER0) Peripheral Clock 7 Enable */
#define PMC_PCER0_PID8 (0x1u << 8) /**< \brief (PMC_PCER0) Peripheral Clock 8 Enable */
#define PMC_PCER0_PID9 (0x1u << 9) /**< \brief (PMC_PCER0) Peripheral Clock 9 Enable */
#define PMC_PCER0_PID10 (0x1u << 10) /**< \brief (PMC_PCER0) Peripheral Clock 10 Enable */
#define PMC_PCER0_PID11 (0x1u << 11) /**< \brief (PMC_PCER0) Peripheral Clock 11 Enable */
#define PMC_PCER0_PID12 (0x1u << 12) /**< \brief (PMC_PCER0) Peripheral Clock 12 Enable */
#define PMC_PCER0_PID13 (0x1u << 13) /**< \brief (PMC_PCER0) Peripheral Clock 13 Enable */
#define PMC_PCER0_PID14 (0x1u << 14) /**< \brief (PMC_PCER0) Peripheral Clock 14 Enable */
#define PMC_PCER0_PID15 (0x1u << 15) /**< \brief (PMC_PCER0) Peripheral Clock 15 Enable */
#define PMC_PCER0_PID16 (0x1u << 16) /**< \brief (PMC_PCER0) Peripheral Clock 16 Enable */
#define PMC_PCER0_PID17 (0x1u << 17) /**< \brief (PMC_PCER0) Peripheral Clock 17 Enable */
#define PMC_PCER0_PID18 (0x1u << 18) /**< \brief (PMC_PCER0) Peripheral Clock 18 Enable */
#define PMC_PCER0_PID19 (0x1u << 19) /**< \brief (PMC_PCER0) Peripheral Clock 19 Enable */
#define PMC_PCER0_PID20 (0x1u << 20) /**< \brief (PMC_PCER0) Peripheral Clock 20 Enable */
#define PMC_PCER0_PID21 (0x1u << 21) /**< \brief (PMC_PCER0) Peripheral Clock 21 Enable */
#define PMC_PCER0_PID22 (0x1u << 22) /**< \brief (PMC_PCER0) Peripheral Clock 22 Enable */
#define PMC_PCER0_PID23 (0x1u << 23) /**< \brief (PMC_PCER0) Peripheral Clock 23 Enable */
#define PMC_PCER0_PID24 (0x1u << 24) /**< \brief (PMC_PCER0) Peripheral Clock 24 Enable */
#define PMC_PCER0_PID25 (0x1u << 25) /**< \brief (PMC_PCER0) Peripheral Clock 25 Enable */
#define PMC_PCER0_PID26 (0x1u << 26) /**< \brief (PMC_PCER0) Peripheral Clock 26 Enable */
#define PMC_PCER0_PID27 (0x1u << 27) /**< \brief (PMC_PCER0) Peripheral Clock 27 Enable */
#define PMC_PCER0_PID28 (0x1u << 28) /**< \brief (PMC_PCER0) Peripheral Clock 28 Enable */
#define PMC_PCER0_PID29 (0x1u << 29) /**< \brief (PMC_PCER0) Peripheral Clock 29 Enable */
#define PMC_PCER0_PID30 (0x1u << 30) /**< \brief (PMC_PCER0) Peripheral Clock 30 Enable */
#define PMC_PCER0_PID31 (0x1u << 31) /**< \brief (PMC_PCER0) Peripheral Clock 31 Enable */
/* -------- PMC_PCDR0 : (PMC Offset: 0x0014) Peripheral Clock Disable Register 0 -------- */
#define PMC_PCDR0_PID2 (0x1u << 2) /**< \brief (PMC_PCDR0) Peripheral Clock 2 Disable */
#define PMC_PCDR0_PID3 (0x1u << 3) /**< \brief (PMC_PCDR0) Peripheral Clock 3 Disable */
#define PMC_PCDR0_PID4 (0x1u << 4) /**< \brief (PMC_PCDR0) Peripheral Clock 4 Disable */
#define PMC_PCDR0_PID5 (0x1u << 5) /**< \brief (PMC_PCDR0) Peripheral Clock 5 Disable */
#define PMC_PCDR0_PID6 (0x1u << 6) /**< \brief (PMC_PCDR0) Peripheral Clock 6 Disable */
#define PMC_PCDR0_PID7 (0x1u << 7) /**< \brief (PMC_PCDR0) Peripheral Clock 7 Disable */
#define PMC_PCDR0_PID8 (0x1u << 8) /**< \brief (PMC_PCDR0) Peripheral Clock 8 Disable */
#define PMC_PCDR0_PID9 (0x1u << 9) /**< \brief (PMC_PCDR0) Peripheral Clock 9 Disable */
#define PMC_PCDR0_PID10 (0x1u << 10) /**< \brief (PMC_PCDR0) Peripheral Clock 10 Disable */
#define PMC_PCDR0_PID11 (0x1u << 11) /**< \brief (PMC_PCDR0) Peripheral Clock 11 Disable */
#define PMC_PCDR0_PID12 (0x1u << 12) /**< \brief (PMC_PCDR0) Peripheral Clock 12 Disable */
#define PMC_PCDR0_PID13 (0x1u << 13) /**< \brief (PMC_PCDR0) Peripheral Clock 13 Disable */
#define PMC_PCDR0_PID14 (0x1u << 14) /**< \brief (PMC_PCDR0) Peripheral Clock 14 Disable */
#define PMC_PCDR0_PID15 (0x1u << 15) /**< \brief (PMC_PCDR0) Peripheral Clock 15 Disable */
#define PMC_PCDR0_PID16 (0x1u << 16) /**< \brief (PMC_PCDR0) Peripheral Clock 16 Disable */
#define PMC_PCDR0_PID17 (0x1u << 17) /**< \brief (PMC_PCDR0) Peripheral Clock 17 Disable */
#define PMC_PCDR0_PID18 (0x1u << 18) /**< \brief (PMC_PCDR0) Peripheral Clock 18 Disable */
#define PMC_PCDR0_PID19 (0x1u << 19) /**< \brief (PMC_PCDR0) Peripheral Clock 19 Disable */
#define PMC_PCDR0_PID20 (0x1u << 20) /**< \brief (PMC_PCDR0) Peripheral Clock 20 Disable */
#define PMC_PCDR0_PID21 (0x1u << 21) /**< \brief (PMC_PCDR0) Peripheral Clock 21 Disable */
#define PMC_PCDR0_PID22 (0x1u << 22) /**< \brief (PMC_PCDR0) Peripheral Clock 22 Disable */
#define PMC_PCDR0_PID23 (0x1u << 23) /**< \brief (PMC_PCDR0) Peripheral Clock 23 Disable */
#define PMC_PCDR0_PID24 (0x1u << 24) /**< \brief (PMC_PCDR0) Peripheral Clock 24 Disable */
#define PMC_PCDR0_PID25 (0x1u << 25) /**< \brief (PMC_PCDR0) Peripheral Clock 25 Disable */
#define PMC_PCDR0_PID26 (0x1u << 26) /**< \brief (PMC_PCDR0) Peripheral Clock 26 Disable */
#define PMC_PCDR0_PID27 (0x1u << 27) /**< \brief (PMC_PCDR0) Peripheral Clock 27 Disable */
#define PMC_PCDR0_PID28 (0x1u << 28) /**< \brief (PMC_PCDR0) Peripheral Clock 28 Disable */
#define PMC_PCDR0_PID29 (0x1u << 29) /**< \brief (PMC_PCDR0) Peripheral Clock 29 Disable */
#define PMC_PCDR0_PID30 (0x1u << 30) /**< \brief (PMC_PCDR0) Peripheral Clock 30 Disable */
#define PMC_PCDR0_PID31 (0x1u << 31) /**< \brief (PMC_PCDR0) Peripheral Clock 31 Disable */
/* -------- PMC_PCSR0 : (PMC Offset: 0x0018) Peripheral Clock Status Register 0 -------- */
#define PMC_PCSR0_PID2 (0x1u << 2) /**< \brief (PMC_PCSR0) Peripheral Clock 2 Status */
#define PMC_PCSR0_PID3 (0x1u << 3) /**< \brief (PMC_PCSR0) Peripheral Clock 3 Status */
#define PMC_PCSR0_PID4 (0x1u << 4) /**< \brief (PMC_PCSR0) Peripheral Clock 4 Status */
#define PMC_PCSR0_PID5 (0x1u << 5) /**< \brief (PMC_PCSR0) Peripheral Clock 5 Status */
#define PMC_PCSR0_PID6 (0x1u << 6) /**< \brief (PMC_PCSR0) Peripheral Clock 6 Status */
#define PMC_PCSR0_PID7 (0x1u << 7) /**< \brief (PMC_PCSR0) Peripheral Clock 7 Status */
#define PMC_PCSR0_PID8 (0x1u << 8) /**< \brief (PMC_PCSR0) Peripheral Clock 8 Status */
#define PMC_PCSR0_PID9 (0x1u << 9) /**< \brief (PMC_PCSR0) Peripheral Clock 9 Status */
#define PMC_PCSR0_PID10 (0x1u << 10) /**< \brief (PMC_PCSR0) Peripheral Clock 10 Status */
#define PMC_PCSR0_PID11 (0x1u << 11) /**< \brief (PMC_PCSR0) Peripheral Clock 11 Status */
#define PMC_PCSR0_PID12 (0x1u << 12) /**< \brief (PMC_PCSR0) Peripheral Clock 12 Status */
#define PMC_PCSR0_PID13 (0x1u << 13) /**< \brief (PMC_PCSR0) Peripheral Clock 13 Status */
#define PMC_PCSR0_PID14 (0x1u << 14) /**< \brief (PMC_PCSR0) Peripheral Clock 14 Status */
#define PMC_PCSR0_PID15 (0x1u << 15) /**< \brief (PMC_PCSR0) Peripheral Clock 15 Status */
#define PMC_PCSR0_PID16 (0x1u << 16) /**< \brief (PMC_PCSR0) Peripheral Clock 16 Status */
#define PMC_PCSR0_PID17 (0x1u << 17) /**< \brief (PMC_PCSR0) Peripheral Clock 17 Status */
#define PMC_PCSR0_PID18 (0x1u << 18) /**< \brief (PMC_PCSR0) Peripheral Clock 18 Status */
#define PMC_PCSR0_PID19 (0x1u << 19) /**< \brief (PMC_PCSR0) Peripheral Clock 19 Status */
#define PMC_PCSR0_PID20 (0x1u << 20) /**< \brief (PMC_PCSR0) Peripheral Clock 20 Status */
#define PMC_PCSR0_PID21 (0x1u << 21) /**< \brief (PMC_PCSR0) Peripheral Clock 21 Status */
#define PMC_PCSR0_PID22 (0x1u << 22) /**< \brief (PMC_PCSR0) Peripheral Clock 22 Status */
#define PMC_PCSR0_PID23 (0x1u << 23) /**< \brief (PMC_PCSR0) Peripheral Clock 23 Status */
#define PMC_PCSR0_PID24 (0x1u << 24) /**< \brief (PMC_PCSR0) Peripheral Clock 24 Status */
#define PMC_PCSR0_PID25 (0x1u << 25) /**< \brief (PMC_PCSR0) Peripheral Clock 25 Status */
#define PMC_PCSR0_PID26 (0x1u << 26) /**< \brief (PMC_PCSR0) Peripheral Clock 26 Status */
#define PMC_PCSR0_PID27 (0x1u << 27) /**< \brief (PMC_PCSR0) Peripheral Clock 27 Status */
#define PMC_PCSR0_PID28 (0x1u << 28) /**< \brief (PMC_PCSR0) Peripheral Clock 28 Status */
#define PMC_PCSR0_PID29 (0x1u << 29) /**< \brief (PMC_PCSR0) Peripheral Clock 29 Status */
#define PMC_PCSR0_PID30 (0x1u << 30) /**< \brief (PMC_PCSR0) Peripheral Clock 30 Status */
#define PMC_PCSR0_PID31 (0x1u << 31) /**< \brief (PMC_PCSR0) Peripheral Clock 31 Status */
/* -------- CKGR_UCKR : (PMC Offset: 0x001C) UTMI Clock Register -------- */
#define CKGR_UCKR_UPLLEN (0x1u << 16) /**< \brief (CKGR_UCKR) UTMI PLL Enable */
#define CKGR_UCKR_UPLLCOUNT_Pos 20
#define CKGR_UCKR_UPLLCOUNT_Msk (0xfu << CKGR_UCKR_UPLLCOUNT_Pos) /**< \brief (CKGR_UCKR) UTMI PLL Start-up Time */
#define CKGR_UCKR_UPLLCOUNT(value) ((CKGR_UCKR_UPLLCOUNT_Msk & ((value) << CKGR_UCKR_UPLLCOUNT_Pos)))
/* -------- CKGR_MOR : (PMC Offset: 0x0020) Main Oscillator Register -------- */
#define CKGR_MOR_MOSCXTEN (0x1u << 0) /**< \brief (CKGR_MOR) Main Crystal Oscillator Enable */
#define CKGR_MOR_MOSCXTBY (0x1u << 1) /**< \brief (CKGR_MOR) Main Crystal Oscillator Bypass */
#define CKGR_MOR_MOSCRCEN (0x1u << 3) /**< \brief (CKGR_MOR) Main On-Chip RC Oscillator Enable */
#define CKGR_MOR_MOSCRCF_Pos 4
#define CKGR_MOR_MOSCRCF_Msk (0x7u << CKGR_MOR_MOSCRCF_Pos) /**< \brief (CKGR_MOR) Main On-Chip RC Oscillator Frequency Selection */
#define   CKGR_MOR_MOSCRCF_4_MHz (0x0u << 4) /**< \brief (CKGR_MOR) The Fast RC Oscillator Frequency is at 4 MHz (default) */
#define   CKGR_MOR_MOSCRCF_8_MHz (0x1u << 4) /**< \brief (CKGR_MOR) The Fast RC Oscillator Frequency is at 8 MHz */
#define   CKGR_MOR_MOSCRCF_12_MHz (0x2u << 4) /**< \brief (CKGR_MOR) The Fast RC Oscillator Frequency is at 12 MHz */
#define CKGR_MOR_MOSCXTST_Pos 8
#define CKGR_MOR_MOSCXTST_Msk (0xffu << CKGR_MOR_MOSCXTST_Pos) /**< \brief (CKGR_MOR) Main Crystal Oscillator Start-up Time */
#define CKGR_MOR_MOSCXTST(value) ((CKGR_MOR_MOSCXTST_Msk & ((value) << CKGR_MOR_MOSCXTST_Pos)))
#define CKGR_MOR_KEY_Pos 16
#define CKGR_MOR_KEY_Msk (0xffu << CKGR_MOR_KEY_Pos) /**< \brief (CKGR_MOR) Password */
#define CKGR_MOR_KEY(value) ((CKGR_MOR_KEY_Msk & ((value) << CKGR_MOR_KEY_Pos)))
#define CKGR_MOR_MOSCSEL (0x1u << 24) /**< \brief (CKGR_MOR) Main Oscillator Selection */
#define CKGR_MOR_CFDEN (0x1u << 25) /**< \brief (CKGR_MOR) Clock Failure Detector Enable */
/* -------- CKGR_MCFR : (PMC Offset: 0x0024) Main Clock Frequency Register -------- */
#define CKGR_MCFR_MAINF_Pos 0
#define CKGR_MCFR_MAINF_Msk (0xffffu << CKGR_MCFR_MAINF_Pos) /**< \brief (CKGR_MCFR) Main Clock Frequency */
#define CKGR_MCFR_MAINFRDY (0x1u << 16) /**< \brief (CKGR_MCFR) Main Clock Ready */
/* -------- CKGR_PLLAR : (PMC Offset: 0x0028) PLLA Register -------- */
#define CKGR_PLLAR_DIVA_Pos 0
#define CKGR_PLLAR_DIVA_Msk (0xffu << CKGR_PLLAR_DIVA_Pos) /**< \brief (CKGR_PLLAR) Divider */
#define CKGR_PLLAR_DIVA(value) ((CKGR_PLLAR_DIVA_Msk & ((value) << CKGR_PLLAR_DIVA_Pos)))
#define CKGR_PLLAR_PLLACOUNT_Pos 8
#define CKGR_PLLAR_PLLACOUNT_Msk (0x3fu << CKGR_PLLAR_PLLACOUNT_Pos) /**< \brief (CKGR_PLLAR) PLLA Counter */
#define CKGR_PLLAR_PLLACOUNT(value) ((CKGR_PLLAR_PLLACOUNT_Msk & ((value) << CKGR_PLLAR_PLLACOUNT_Pos)))
#define CKGR_PLLAR_MULA_Pos 16
#define CKGR_PLLAR_MULA_Msk (0x7ffu << CKGR_PLLAR_MULA_Pos) /**< \brief (CKGR_PLLAR) PLLA Multiplier */
#define CKGR_PLLAR_MULA(value) ((CKGR_PLLAR_MULA_Msk & ((value) << CKGR_PLLAR_MULA_Pos)))
#define CKGR_PLLAR_ONE (0x1u << 29) /**< \brief (CKGR_PLLAR) Must Be Set to 1 */
/* -------- PMC_MCKR : (PMC Offset: 0x0030) Master Clock Register -------- */
#define PMC_MCKR_CSS_Pos 0
#define PMC_MCKR_CSS_Msk (0x3u << PMC_MCKR_CSS_Pos) /**< \brief (PMC_MCKR) Master Clock Source Selection */
#define   PMC_MCKR_CSS_SLOW_CLK (0x0u << 0) /**< \brief (PMC_MCKR) Slow Clock is selected */
#define   PMC_MCKR_CSS_MAIN_CLK (0x1u << 0) /**< \brief (PMC_MCKR) Main Clock is selected */
#define   PMC_MCKR_CSS_PLLA_CLK (0x2u << 0) /**< \brief (PMC_MCKR) PLLA Clock is selected */
#define   PMC_MCKR_CSS_UPLL_CLK (0x3u << 0) /**< \brief (PMC_MCKR) UPLL Clock is selected */
#define PMC_MCKR_PRES_Pos 4
#define PMC_MCKR_PRES_Msk (0x7u << PMC_MCKR_PRES_Pos) /**< \brief (PMC_MCKR) Processor Clock Prescaler */
#define   PMC_MCKR_PRES_CLK_1 (0x0u << 4) /**< \brief (PMC_MCKR) Selected clock */
#define   PMC_MCKR_PRES_CLK_2 (0x1u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 2 */
#define   PMC_MCKR_PRES_CLK_4 (0x2u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 4 */
#define   PMC_MCKR_PRES_CLK_8 (0x3u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 8 */
#define   PMC_MCKR_PRES_CLK_16 (0x4u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 16 */
#define   PMC_MCKR_PRES_CLK_32 (0x5u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 32 */
#define   PMC_MCKR_PRES_CLK_64 (0x6u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 64 */
#define   PMC_MCKR_PRES_CLK_3 (0x7u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 3 */
#define PMC_MCKR_PLLADIV2 (0x1u << 12) /**< \brief (PMC_MCKR) PLLA Divisor by 2 */
#define PMC_MCKR_UPLLDIV2 (0x1u << 13) /**< \brief (PMC_MCKR)  */
/* -------- PMC_USB : (PMC Offset: 0x0038) USB Clock Register -------- */
#define PMC_USB_USBS (0x1u << 0) /**< \brief (PMC_USB) USB Input Clock Selection */
#define PMC_USB_USBDIV_Pos 8
#define PMC_USB_USBDIV_Msk (0xfu << PMC_USB_USBDIV_Pos) /**< \brief (PMC_USB) Divider for USB Clock. */
#define PMC_USB_USBDIV(value) ((PMC_USB_USBDIV_Msk & ((value) << PMC_USB_USBDIV_Pos)))
/* -------- PMC_PCK[3] : (PMC Offset: 0x0040) Programmable Clock 0 Register -------- */
#define PMC_PCK_CSS_Pos 0
#define PMC_PCK_CSS_Msk (0x7u << PMC_PCK_CSS_Pos) /**< \brief (PMC_PCK[3]) Master Clock Source Selection */
#define   PMC_PCK_CSS_SLOW_CLK (0x0u << 0) /**< \brief (PMC_PCK[3]) Slow Clock is selected */
#define   PMC_PCK_CSS_MAIN_CLK (0x1u << 0) /**< \brief (PMC_PCK[3]) Main Clock is selected */
#define   PMC_PCK_CSS_PLLA_CLK (0x2u << 0) /**< \brief (PMC_PCK[3]) PLLA Clock is selected */
#define   PMC_PCK_CSS_UPLL_CLK (0x3u << 0) /**< \brief (PMC_PCK[3]) UPLL Clock is selected */
#define   PMC_PCK_CSS_MCK (0x4u << 0) /**< \brief (PMC_PCK[3]) Master Clock is selected */
#define PMC_PCK_PRES_Pos 4
#define PMC_PCK_PRES_Msk (0x7u << PMC_PCK_PRES_Pos) /**< \brief (PMC_PCK[3]) Programmable Clock Prescaler */
#define   PMC_PCK_PRES_CLK_1 (0x0u << 4) /**< \brief (PMC_PCK[3]) Selected clock */
#define   PMC_PCK_PRES_CLK_2 (0x1u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 2 */
#define   PMC_PCK_PRES_CLK_4 (0x2u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 4 */
#define   PMC_PCK_PRES_CLK_8 (0x3u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 8 */
#define   PMC_PCK_PRES_CLK_16 (0x4u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 16 */
#define   PMC_PCK_PRES_CLK_32 (0x5u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 32 */
#define   PMC_PCK_PRES_CLK_64 (0x6u << 4) /**< \brief (PMC_PCK[3]) Selected clock divided by 64 */
/* -------- PMC_IER : (PMC Offset: 0x0060) Interrupt Enable Register -------- */
#define PMC_IER_MOSCXTS (0x1u << 0) /**< \brief (PMC_IER) Main Crystal Oscillator Status Interrupt Enable */
#define PMC_IER_LOCKA (0x1u << 1) /**< \brief (PMC_IER) PLLA Lock Interrupt Enable */
#define PMC_IER_MCKRDY (0x1u << 3) /**< \brief (PMC_IER) Master Clock Ready Interrupt Enable */
#define PMC_IER_LOCKU (0x1u << 6) /**< \brief (PMC_IER) UTMI PLL Lock Interrupt Enable */
#define PMC_IER_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IER) Programmable Clock Ready 0 Interrupt Enable */
#define PMC_IER_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IER) Programmable Clock Ready 1 Interrupt Enable */
#define PMC_IER_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IER) Programmable Clock Ready 2 Interrupt Enable */
#define PMC_IER_MOSCSELS (0x1u << 16) /**< \brief (PMC_IER) Main Oscillator Selection Status Interrupt Enable */
#define PMC_IER_MOSCRCS (0x1u << 17) /**< \brief (PMC_IER) Main On-Chip RC Status Interrupt Enable */
#define PMC_IER_CFDEV (0x1u << 18) /**< \brief (PMC_IER) Clock Failure Detector Event Interrupt Enable */
/* -------- PMC_IDR : (PMC Offset: 0x0064) Interrupt Disable Register -------- */
#define PMC_IDR_MOSCXTS (0x1u << 0) /**< \brief (PMC_IDR) Main Crystal Oscillator Status Interrupt Disable */
#define PMC_IDR_LOCKA (0x1u << 1) /**< \brief (PMC_IDR) PLLA Lock Interrupt Disable */
#define PMC_IDR_MCKRDY (0x1u << 3) /**< \brief (PMC_IDR) Master Clock Ready Interrupt Disable */
#define PMC_IDR_LOCKU (0x1u << 6) /**< \brief (PMC_IDR) UTMI PLL Lock Interrupt Disable */
#define PMC_IDR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IDR) Programmable Clock Ready 0 Interrupt Disable */
#define PMC_IDR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IDR) Programmable Clock Ready 1 Interrupt Disable */
#define PMC_IDR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IDR) Programmable Clock Ready 2 Interrupt Disable */
#define PMC_IDR_MOSCSELS (0x1u << 16) /**< \brief (PMC_IDR) Main Oscillator Selection Status Interrupt Disable */
#define PMC_IDR_MOSCRCS (0x1u << 17) /**< \brief (PMC_IDR) Main On-Chip RC Status Interrupt Disable */
#define PMC_IDR_CFDEV (0x1u << 18) /**< \brief (PMC_IDR) Clock Failure Detector Event Interrupt Disable */
/* -------- PMC_SR : (PMC Offset: 0x0068) Status Register -------- */
#define PMC_SR_MOSCXTS (0x1u << 0) /**< \brief (PMC_SR) Main XTAL Oscillator Status */
#define PMC_SR_LOCKA (0x1u << 1) /**< \brief (PMC_SR) PLLA Lock Status */
#define PMC_SR_MCKRDY (0x1u << 3) /**< \brief (PMC_SR) Master Clock Status */
#define PMC_SR_LOCKU (0x1u << 6) /**< \brief (PMC_SR) UTMI PLL Lock Status */
#define PMC_SR_OSCSELS (0x1u << 7) /**< \brief (PMC_SR) Slow Clock Oscillator Selection */
#define PMC_SR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_MOSCSELS (0x1u << 16) /**< \brief (PMC_SR) Main Oscillator Selection Status */
#define PMC_SR_MOSCRCS (0x1u << 17) /**< \brief (PMC_SR) Main On-Chip RC Oscillator Status */
#define PMC_SR_CFDEV (0x1u << 18) /**< \brief (PMC_SR) Clock Failure Detector Event */
#define PMC_SR_CFDS (0x1u << 19) /**< \brief (PMC_SR) Clock Failure Detector Status */
#define PMC_SR_FOS (0x1u << 20) /**< \brief (PMC_SR) Clock Failure Detector Fault Output Status */
/* -------- PMC_IMR : (PMC Offset: 0x006C) Interrupt Mask Register -------- */
#define PMC_IMR_MOSCXTS (0x1u << 0) /**< \brief (PMC_IMR) Main Crystal Oscillator Status Interrupt Mask */
#define PMC_IMR_LOCKA (0x1u << 1) /**< \brief (PMC_IMR) PLLA Lock Interrupt Mask */
#define PMC_IMR_MCKRDY (0x1u << 3) /**< \brief (PMC_IMR) Master Clock Ready Interrupt Mask */
#define PMC_IMR_LOCKU (0x1u << 6) /**< \brief (PMC_IMR) UTMI PLL Lock Interrupt Mask */
#define PMC_IMR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IMR) Programmable Clock Ready 0 Interrupt Mask */
#define PMC_IMR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IMR) Programmable Clock Ready 1 Interrupt Mask */
#define PMC_IMR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IMR) Programmable Clock Ready 2 Interrupt Mask */
#define PMC_IMR_MOSCSELS (0x1u << 16) /**< \brief (PMC_IMR) Main Oscillator Selection Status Interrupt Mask */
#define PMC_IMR_MOSCRCS (0x1u << 17) /**< \brief (PMC_IMR) Main On-Chip RC Status Interrupt Mask */
#define PMC_IMR_CFDEV (0x1u << 18) /**< \brief (PMC_IMR) Clock Failure Detector Event Interrupt Mask */
/* -------- PMC_FSMR : (PMC Offset: 0x0070) Fast Startup Mode Register -------- */
#define PMC_FSMR_FSTT0 (0x1u << 0) /**< \brief (PMC_FSMR) Fast Startup Input Enable 0 */
#define PMC_FSMR_FSTT1 (0x1u << 1) /**< \brief (PMC_FSMR) Fast Startup Input Enable 1 */
#define PMC_FSMR_FSTT2 (0x1u << 2) /**< \brief (PMC_FSMR) Fast Startup Input Enable 2 */
#define PMC_FSMR_FSTT3 (0x1u << 3) /**< \brief (PMC_FSMR) Fast Startup Input Enable 3 */
#define PMC_FSMR_FSTT4 (0x1u << 4) /**< \brief (PMC_FSMR) Fast Startup Input Enable 4 */
#define PMC_FSMR_FSTT5 (0x1u << 5) /**< \brief (PMC_FSMR) Fast Startup Input Enable 5 */
#define PMC_FSMR_FSTT6 (0x1u << 6) /**< \brief (PMC_FSMR) Fast Startup Input Enable 6 */
#define PMC_FSMR_FSTT7 (0x1u << 7) /**< \brief (PMC_FSMR) Fast Startup Input Enable 7 */
#define PMC_FSMR_FSTT8 (0x1u << 8) /**< \brief (PMC_FSMR) Fast Startup Input Enable 8 */
#define PMC_FSMR_FSTT9 (0x1u << 9) /**< \brief (PMC_FSMR) Fast Startup Input Enable 9 */
#define PMC_FSMR_FSTT10 (0x1u << 10) /**< \brief (PMC_FSMR) Fast Startup Input Enable 10 */
#define PMC_FSMR_FSTT11 (0x1u << 11) /**< \brief (PMC_FSMR) Fast Startup Input Enable 11 */
#define PMC_FSMR_FSTT12 (0x1u << 12) /**< \brief (PMC_FSMR) Fast Startup Input Enable 12 */
#define PMC_FSMR_FSTT13 (0x1u << 13) /**< \brief (PMC_FSMR) Fast Startup Input Enable 13 */
#define PMC_FSMR_FSTT14 (0x1u << 14) /**< \brief (PMC_FSMR) Fast Startup Input Enable 14 */
#define PMC_FSMR_FSTT15 (0x1u << 15) /**< \brief (PMC_FSMR) Fast Startup Input Enable 15 */
#define PMC_FSMR_RTTAL (0x1u << 16) /**< \brief (PMC_FSMR) RTT Alarm Enable */
#define PMC_FSMR_RTCAL (0x1u << 17) /**< \brief (PMC_FSMR) RTC Alarm Enable */
#define PMC_FSMR_USBAL (0x1u << 18) /**< \brief (PMC_FSMR) USB Alarm Enable */
#define PMC_FSMR_LPM (0x1u << 20) /**< \brief (PMC_FSMR) Low Power Mode */
/* -------- PMC_FSPR : (PMC Offset: 0x0074) Fast Startup Polarity Register -------- */
#define PMC_FSPR_FSTP0 (0x1u << 0) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP1 (0x1u << 1) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP2 (0x1u << 2) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP3 (0x1u << 3) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP4 (0x1u << 4) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP5 (0x1u << 5) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP6 (0x1u << 6) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP7 (0x1u << 7) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP8 (0x1u << 8) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP9 (0x1u << 9) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP10 (0x1u << 10) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP11 (0x1u << 11) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP12 (0x1u << 12) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP13 (0x1u << 13) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP14 (0x1u << 14) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP15 (0x1u << 15) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
/* -------- PMC_FOCR : (PMC Offset: 0x0078) Fault Output Clear Register -------- */
#define PMC_FOCR_FOCLR (0x1u << 0) /**< \brief (PMC_FOCR) Fault Output Clear */
/* -------- PMC_WPMR : (PMC Offset: 0x00E4) Write Protect Mode Register -------- */
#define PMC_WPMR_WPEN (0x1u << 0) /**< \brief (PMC_WPMR) Write Protect Enable */
#define PMC_WPMR_WPKEY_Pos 8
#define PMC_WPMR_WPKEY_Msk (0xffffffu << PMC_WPMR_WPKEY_Pos) /**< \brief (PMC_WPMR) Write Protect KEY */
#define PMC_WPMR_WPKEY(value) ((PMC_WPMR_WPKEY_Msk & ((value) << PMC_WPMR_WPKEY_Pos)))
/* -------- PMC_WPSR : (PMC Offset: 0x00E8) Write Protect Status Register -------- */
#define PMC_WPSR_WPVS (0x1u << 0) /**< \brief (PMC_WPSR) Write Protect Violation Status */
#define PMC_WPSR_WPVSRC_Pos 8
#define PMC_WPSR_WPVSRC_Msk (0xffffu << PMC_WPSR_WPVSRC_Pos) /**< \brief (PMC_WPSR) Write Protect Violation Source */
/* -------- PMC_PCER1 : (PMC Offset: 0x0100) Peripheral Clock Enable Register 1 -------- */
#define PMC_PCER1_PID32 (0x1u << 0) /**< \brief (PMC_PCER1) Peripheral Clock 32 Enable */
#define PMC_PCER1_PID33 (0x1u << 1) /**< \brief (PMC_PCER1) Peripheral Clock 33 Enable */
#define PMC_PCER1_PID34 (0x1u << 2) /**< \brief (PMC_PCER1) Peripheral Clock 34 Enable */
#define PMC_PCER1_PID35 (0x1u << 3) /**< \brief (PMC_PCER1) Peripheral Clock 35 Enable */
#define PMC_PCER1_PID36 (0x1u << 4) /**< \brief (PMC_PCER1) Peripheral Clock 36 Enable */
#define PMC_PCER1_PID37 (0x1u << 5) /**< \brief (PMC_PCER1) Peripheral Clock 37 Enable */
#define PMC_PCER1_PID38 (0x1u << 6) /**< \brief (PMC_PCER1) Peripheral Clock 38 Enable */
#define PMC_PCER1_PID39 (0x1u << 7) /**< \brief (PMC_PCER1) Peripheral Clock 39 Enable */
#define PMC_PCER1_PID40 (0x1u << 8) /**< \brief (PMC_PCER1) Peripheral Clock 40 Enable */
#define PMC_PCER1_PID41 (0x1u << 9) /**< \brief (PMC_PCER1) Peripheral Clock 41 Enable */
#define PMC_PCER1_PID42 (0x1u << 10) /**< \brief (PMC_PCER1) Peripheral Clock 42 Enable */
#define PMC_PCER1_PID43 (0x1u << 11) /**< \brief (PMC_PCER1) Peripheral Clock 43 Enable */
#define PMC_PCER1_PID44 (0x1u << 12) /**< \brief (PMC_PCER1) Peripheral Clock 44 Enable */
/* -------- PMC_PCDR1 : (PMC Offset: 0x0104) Peripheral Clock Disable Register 1 -------- */
#define PMC_PCDR1_PID32 (0x1u << 0) /**< \brief (PMC_PCDR1) Peripheral Clock 32 Disable */
#define PMC_PCDR1_PID33 (0x1u << 1) /**< \brief (PMC_PCDR1) Peripheral Clock 33 Disable */
#define PMC_PCDR1_PID34 (0x1u << 2) /**< \brief (PMC_PCDR1) Peripheral Clock 34 Disable */
#define PMC_PCDR1_PID35 (0x1u << 3) /**< \brief (PMC_PCDR1) Peripheral Clock 35 Disable */
#define PMC_PCDR1_PID36 (0x1u << 4) /**< \brief (PMC_PCDR1) Peripheral Clock 36 Disable */
#define PMC_PCDR1_PID37 (0x1u << 5) /**< \brief (PMC_PCDR1) Peripheral Clock 37 Disable */
#define PMC_PCDR1_PID38 (0x1u << 6) /**< \brief (PMC_PCDR1) Peripheral Clock 38 Disable */
#define PMC_PCDR1_PID39 (0x1u << 7) /**< \brief (PMC_PCDR1) Peripheral Clock 39 Disable */
#define PMC_PCDR1_PID40 (0x1u << 8) /**< \brief (PMC_PCDR1) Peripheral Clock 40 Disable */
#define PMC_PCDR1_PID41 (0x1u << 9) /**< \brief (PMC_PCDR1) Peripheral Clock 41 Disable */
#define PMC_PCDR1_PID42 (0x1u << 10) /**< \brief (PMC_PCDR1) Peripheral Clock 42 Disable */
#define PMC_PCDR1_PID43 (0x1u << 11) /**< \brief (PMC_PCDR1) Peripheral Clock 43 Disable */
#define PMC_PCDR1_PID44 (0x1u << 12) /**< \brief (PMC_PCDR1) Peripheral Clock 44 Disable */
/* -------- PMC_PCSR1 : (PMC Offset: 0x0108) Peripheral Clock Status Register 1 -------- */
#define PMC_PCSR1_PID32 (0x1u << 0) /**< \brief (PMC_PCSR1) Peripheral Clock 32 Status */
#define PMC_PCSR1_PID33 (0x1u << 1) /**< \brief (PMC_PCSR1) Peripheral Clock 33 Status */
#define PMC_PCSR1_PID34 (0x1u << 2) /**< \brief (PMC_PCSR1) Peripheral Clock 34 Status */
#define PMC_PCSR1_PID35 (0x1u << 3) /**< \brief (PMC_PCSR1) Peripheral Clock 35 Status */
#define PMC_PCSR1_PID36 (0x1u << 4) /**< \brief (PMC_PCSR1) Peripheral Clock 36 Status */
#define PMC_PCSR1_PID37 (0x1u << 5) /**< \brief (PMC_PCSR1) Peripheral Clock 37 Status */
#define PMC_PCSR1_PID38 (0x1u << 6) /**< \brief (PMC_PCSR1) Peripheral Clock 38 Status */
#define PMC_PCSR1_PID39 (0x1u << 7) /**< \brief (PMC_PCSR1) Peripheral Clock 39 Status */
#define PMC_PCSR1_PID40 (0x1u << 8) /**< \brief (PMC_PCSR1) Peripheral Clock 40 Status */
#define PMC_PCSR1_PID41 (0x1u << 9) /**< \brief (PMC_PCSR1) Peripheral Clock 41 Status */
#define PMC_PCSR1_PID42 (0x1u << 10) /**< \brief (PMC_PCSR1) Peripheral Clock 42 Status */
#define PMC_PCSR1_PID43 (0x1u << 11) /**< \brief (PMC_PCSR1) Peripheral Clock 43 Status */
#define PMC_PCSR1_PID44 (0x1u << 12) /**< \brief (PMC_PCSR1) Peripheral Clock 44 Status */
/* -------- PMC_PCR : (PMC Offset: 0x010C) Peripheral Control Register -------- */
#define PMC_PCR_PID_Pos 0
#define PMC_PCR_PID_Msk (0x3fu << PMC_PCR_PID_Pos) /**< \brief (PMC_PCR) Peripheral ID */
#define PMC_PCR_PID(value) ((PMC_PCR_PID_Msk & ((value) << PMC_PCR_PID_Pos)))
#define PMC_PCR_CMD (0x1u << 12) /**< \brief (PMC_PCR) Command */
#define PMC_PCR_DIV_Pos 16
#define PMC_PCR_DIV_Msk (0x3u << PMC_PCR_DIV_Pos) /**< \brief (PMC_PCR) Divisor Value */
#define   PMC_PCR_DIV_PERIPH_DIV_MCK (0x0u << 16) /**< \brief (PMC_PCR) Peripheral clock is MCK */
#define   PMC_PCR_DIV_PERIPH_DIV2_MCK (0x1u << 16) /**< \brief (PMC_PCR) Peripheral clock is MCK/2 */
#define   PMC_PCR_DIV_PERIPH_DIV4_MCK (0x2u << 16) /**< \brief (PMC_PCR) Peripheral clock is MCK/4 */
#define PMC_PCR_EN (0x1u << 28) /**< \brief (PMC_PCR) Enable */
/*@}*/ /* end of group SAM3XA_PMC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Pulse Width Modulation Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_PWM Pulse Width Modulation Controller */
/*@{*/

/** \brief PwmCh_num hardware registers */
typedef struct {
  RwReg      PWM_CMR;       /**< \brief (PwmCh_num Offset: 0x0) PWM Channel Mode Register */
  RwReg      PWM_CDTY;      /**< \brief (PwmCh_num Offset: 0x4) PWM Channel Duty Cycle Register */
  RwReg      PWM_CDTYUPD;   /**< \brief (PwmCh_num Offset: 0x8) PWM Channel Duty Cycle Update Register */
  RwReg      PWM_CPRD;      /**< \brief (PwmCh_num Offset: 0xC) PWM Channel Period Register */
  RwReg      PWM_CPRDUPD;   /**< \brief (PwmCh_num Offset: 0x10) PWM Channel Period Update Register */
  RwReg      PWM_CCNT;      /**< \brief (PwmCh_num Offset: 0x14) PWM Channel Counter Register */
  RwReg      PWM_DT;        /**< \brief (PwmCh_num Offset: 0x18) PWM Channel Dead Time Register */
  RwReg      PWM_DTUPD;     /**< \brief (PwmCh_num Offset: 0x1C) PWM Channel Dead Time Update Register */
} PwmCh_num;
/** \brief PwmCmp hardware registers */
typedef struct {
  RwReg      PWM_CMPV;      /**< \brief (PwmCmp Offset: 0x0) PWM Comparison 0 Value Register */
  RwReg      PWM_CMPVUPD;   /**< \brief (PwmCmp Offset: 0x4) PWM Comparison 0 Value Update Register */
  RwReg      PWM_CMPM;      /**< \brief (PwmCmp Offset: 0x8) PWM Comparison 0 Mode Register */
  RwReg      PWM_CMPMUPD;   /**< \brief (PwmCmp Offset: 0xC) PWM Comparison 0 Mode Update Register */
} PwmCmp;
/** \brief Pwm hardware registers */
#define PWMCMP_NUMBER 8
#define PWMCH_NUM_NUMBER 8
typedef struct {
  RwReg      PWM_CLK;       /**< \brief (Pwm Offset: 0x00) PWM Clock Register */
  WoReg      PWM_ENA;       /**< \brief (Pwm Offset: 0x04) PWM Enable Register */
  WoReg      PWM_DIS;       /**< \brief (Pwm Offset: 0x08) PWM Disable Register */
  RoReg      PWM_SR;        /**< \brief (Pwm Offset: 0x0C) PWM Status Register */
  WoReg      PWM_IER1;      /**< \brief (Pwm Offset: 0x10) PWM Interrupt Enable Register 1 */
  WoReg      PWM_IDR1;      /**< \brief (Pwm Offset: 0x14) PWM Interrupt Disable Register 1 */
  RoReg      PWM_IMR1;      /**< \brief (Pwm Offset: 0x18) PWM Interrupt Mask Register 1 */
  RoReg      PWM_ISR1;      /**< \brief (Pwm Offset: 0x1C) PWM Interrupt Status Register 1 */
  RwReg      PWM_SCM;       /**< \brief (Pwm Offset: 0x20) PWM Sync Channels Mode Register */
  RoReg      Reserved1[1];
  RwReg      PWM_SCUC;      /**< \brief (Pwm Offset: 0x28) PWM Sync Channels Update Control Register */
  RwReg      PWM_SCUP;      /**< \brief (Pwm Offset: 0x2C) PWM Sync Channels Update Period Register */
  WoReg      PWM_SCUPUPD;   /**< \brief (Pwm Offset: 0x30) PWM Sync Channels Update Period Update Register */
  WoReg      PWM_IER2;      /**< \brief (Pwm Offset: 0x34) PWM Interrupt Enable Register 2 */
  WoReg      PWM_IDR2;      /**< \brief (Pwm Offset: 0x38) PWM Interrupt Disable Register 2 */
  RoReg      PWM_IMR2;      /**< \brief (Pwm Offset: 0x3C) PWM Interrupt Mask Register 2 */
  RoReg      PWM_ISR2;      /**< \brief (Pwm Offset: 0x40) PWM Interrupt Status Register 2 */
  RwReg      PWM_OOV;       /**< \brief (Pwm Offset: 0x44) PWM Output Override Value Register */
  RwReg      PWM_OS;        /**< \brief (Pwm Offset: 0x48) PWM Output Selection Register */
  WoReg      PWM_OSS;       /**< \brief (Pwm Offset: 0x4C) PWM Output Selection Set Register */
  WoReg      PWM_OSC;       /**< \brief (Pwm Offset: 0x50) PWM Output Selection Clear Register */
  WoReg      PWM_OSSUPD;    /**< \brief (Pwm Offset: 0x54) PWM Output Selection Set Update Register */
  WoReg      PWM_OSCUPD;    /**< \brief (Pwm Offset: 0x58) PWM Output Selection Clear Update Register */
  RwReg      PWM_FMR;       /**< \brief (Pwm Offset: 0x5C) PWM Fault Mode Register */
  RoReg      PWM_FSR;       /**< \brief (Pwm Offset: 0x60) PWM Fault Status Register */
  WoReg      PWM_FCR;       /**< \brief (Pwm Offset: 0x64) PWM Fault Clear Register */
  RwReg      PWM_FPV;       /**< \brief (Pwm Offset: 0x68) PWM Fault Protection Value Register */
  RwReg      PWM_FPE1;      /**< \brief (Pwm Offset: 0x6C) PWM Fault Protection Enable Register 1 */
  RwReg      PWM_FPE2;      /**< \brief (Pwm Offset: 0x70) PWM Fault Protection Enable Register 2 */
  RoReg      Reserved2[2];
  RwReg      PWM_ELMR[2];   /**< \brief (Pwm Offset: 0x7C) PWM Event Line 0 Mode Register */
  RoReg      Reserved3[11];
  RwReg      PWM_SMMR;      /**< \brief (Pwm Offset: 0xB0) PWM Stepper Motor Mode Register */
  RoReg      Reserved4[12];
  WoReg      PWM_WPCR;      /**< \brief (Pwm Offset: 0xE4) PWM Write Protect Control Register */
  RoReg      PWM_WPSR;      /**< \brief (Pwm Offset: 0xE8) PWM Write Protect Status Register */
  RoReg      Reserved5[7];
  RwReg      PWM_TPR;       /**< \brief (Pwm Offset: 0x108) Transmit Pointer Register */
  RwReg      PWM_TCR;       /**< \brief (Pwm Offset: 0x10C) Transmit Counter Register */
  RoReg      Reserved6[2];
  RwReg      PWM_TNPR;      /**< \brief (Pwm Offset: 0x118) Transmit Next Pointer Register */
  RwReg      PWM_TNCR;      /**< \brief (Pwm Offset: 0x11C) Transmit Next Counter Register */
  WoReg      PWM_PTCR;      /**< \brief (Pwm Offset: 0x120) Transfer Control Register */
  RoReg      PWM_PTSR;      /**< \brief (Pwm Offset: 0x124) Transfer Status Register */
  RoReg      Reserved7[2];
  PwmCmp     PWM_CMP[PWMCMP_NUMBER]; /**< \brief (Pwm Offset: 0x130) 0 .. 7 */
  RoReg      Reserved8[20];
  PwmCh_num  PWM_CH_NUM[PWMCH_NUM_NUMBER]; /**< \brief (Pwm Offset: 0x200) ch_num = 0 .. 7 */
} Pwm;
/* -------- PWM_CLK : (PWM Offset: 0x00) PWM Clock Register -------- */
#define PWM_CLK_DIVA_Pos 0
#define PWM_CLK_DIVA_Msk (0xffu << PWM_CLK_DIVA_Pos) /**< \brief (PWM_CLK) CLKA, CLKB Divide Factor */
#define PWM_CLK_DIVA(value) ((PWM_CLK_DIVA_Msk & ((value) << PWM_CLK_DIVA_Pos)))
#define PWM_CLK_PREA_Pos 8
#define PWM_CLK_PREA_Msk (0xfu << PWM_CLK_PREA_Pos) /**< \brief (PWM_CLK) CLKA, CLKB Source Clock Selection */
#define PWM_CLK_PREA(value) ((PWM_CLK_PREA_Msk & ((value) << PWM_CLK_PREA_Pos)))
#define PWM_CLK_DIVB_Pos 16
#define PWM_CLK_DIVB_Msk (0xffu << PWM_CLK_DIVB_Pos) /**< \brief (PWM_CLK) CLKA, CLKB Divide Factor */
#define PWM_CLK_DIVB(value) ((PWM_CLK_DIVB_Msk & ((value) << PWM_CLK_DIVB_Pos)))
#define PWM_CLK_PREB_Pos 24
#define PWM_CLK_PREB_Msk (0xfu << PWM_CLK_PREB_Pos) /**< \brief (PWM_CLK) CLKA, CLKB Source Clock Selection */
#define PWM_CLK_PREB(value) ((PWM_CLK_PREB_Msk & ((value) << PWM_CLK_PREB_Pos)))
/* -------- PWM_ENA : (PWM Offset: 0x04) PWM Enable Register -------- */
#define PWM_ENA_CHID0 (0x1u << 0) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID1 (0x1u << 1) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID2 (0x1u << 2) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID3 (0x1u << 3) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID4 (0x1u << 4) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID5 (0x1u << 5) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID6 (0x1u << 6) /**< \brief (PWM_ENA) Channel ID */
#define PWM_ENA_CHID7 (0x1u << 7) /**< \brief (PWM_ENA) Channel ID */
/* -------- PWM_DIS : (PWM Offset: 0x08) PWM Disable Register -------- */
#define PWM_DIS_CHID0 (0x1u << 0) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID1 (0x1u << 1) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID2 (0x1u << 2) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID3 (0x1u << 3) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID4 (0x1u << 4) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID5 (0x1u << 5) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID6 (0x1u << 6) /**< \brief (PWM_DIS) Channel ID */
#define PWM_DIS_CHID7 (0x1u << 7) /**< \brief (PWM_DIS) Channel ID */
/* -------- PWM_SR : (PWM Offset: 0x0C) PWM Status Register -------- */
#define PWM_SR_CHID0 (0x1u << 0) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID1 (0x1u << 1) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID2 (0x1u << 2) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID3 (0x1u << 3) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID4 (0x1u << 4) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID5 (0x1u << 5) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID6 (0x1u << 6) /**< \brief (PWM_SR) Channel ID */
#define PWM_SR_CHID7 (0x1u << 7) /**< \brief (PWM_SR) Channel ID */
/* -------- PWM_IER1 : (PWM Offset: 0x10) PWM Interrupt Enable Register 1 -------- */
#define PWM_IER1_CHID0 (0x1u << 0) /**< \brief (PWM_IER1) Counter Event on Channel 0 Interrupt Enable */
#define PWM_IER1_CHID1 (0x1u << 1) /**< \brief (PWM_IER1) Counter Event on Channel 1 Interrupt Enable */
#define PWM_IER1_CHID2 (0x1u << 2) /**< \brief (PWM_IER1) Counter Event on Channel 2 Interrupt Enable */
#define PWM_IER1_CHID3 (0x1u << 3) /**< \brief (PWM_IER1) Counter Event on Channel 3 Interrupt Enable */
#define PWM_IER1_CHID4 (0x1u << 4) /**< \brief (PWM_IER1) Counter Event on Channel 4 Interrupt Enable */
#define PWM_IER1_CHID5 (0x1u << 5) /**< \brief (PWM_IER1) Counter Event on Channel 5 Interrupt Enable */
#define PWM_IER1_CHID6 (0x1u << 6) /**< \brief (PWM_IER1) Counter Event on Channel 6 Interrupt Enable */
#define PWM_IER1_CHID7 (0x1u << 7) /**< \brief (PWM_IER1) Counter Event on Channel 7 Interrupt Enable */
#define PWM_IER1_FCHID0 (0x1u << 16) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 0 Interrupt Enable */
#define PWM_IER1_FCHID1 (0x1u << 17) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 1 Interrupt Enable */
#define PWM_IER1_FCHID2 (0x1u << 18) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 2 Interrupt Enable */
#define PWM_IER1_FCHID3 (0x1u << 19) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 3 Interrupt Enable */
#define PWM_IER1_FCHID4 (0x1u << 20) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 4 Interrupt Enable */
#define PWM_IER1_FCHID5 (0x1u << 21) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 5 Interrupt Enable */
#define PWM_IER1_FCHID6 (0x1u << 22) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 6 Interrupt Enable */
#define PWM_IER1_FCHID7 (0x1u << 23) /**< \brief (PWM_IER1) Fault Protection Trigger on Channel 7 Interrupt Enable */
/* -------- PWM_IDR1 : (PWM Offset: 0x14) PWM Interrupt Disable Register 1 -------- */
#define PWM_IDR1_CHID0 (0x1u << 0) /**< \brief (PWM_IDR1) Counter Event on Channel 0 Interrupt Disable */
#define PWM_IDR1_CHID1 (0x1u << 1) /**< \brief (PWM_IDR1) Counter Event on Channel 1 Interrupt Disable */
#define PWM_IDR1_CHID2 (0x1u << 2) /**< \brief (PWM_IDR1) Counter Event on Channel 2 Interrupt Disable */
#define PWM_IDR1_CHID3 (0x1u << 3) /**< \brief (PWM_IDR1) Counter Event on Channel 3 Interrupt Disable */
#define PWM_IDR1_CHID4 (0x1u << 4) /**< \brief (PWM_IDR1) Counter Event on Channel 4 Interrupt Disable */
#define PWM_IDR1_CHID5 (0x1u << 5) /**< \brief (PWM_IDR1) Counter Event on Channel 5 Interrupt Disable */
#define PWM_IDR1_CHID6 (0x1u << 6) /**< \brief (PWM_IDR1) Counter Event on Channel 6 Interrupt Disable */
#define PWM_IDR1_CHID7 (0x1u << 7) /**< \brief (PWM_IDR1) Counter Event on Channel 7 Interrupt Disable */
#define PWM_IDR1_FCHID0 (0x1u << 16) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 0 Interrupt Disable */
#define PWM_IDR1_FCHID1 (0x1u << 17) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 1 Interrupt Disable */
#define PWM_IDR1_FCHID2 (0x1u << 18) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 2 Interrupt Disable */
#define PWM_IDR1_FCHID3 (0x1u << 19) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 3 Interrupt Disable */
#define PWM_IDR1_FCHID4 (0x1u << 20) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 4 Interrupt Disable */
#define PWM_IDR1_FCHID5 (0x1u << 21) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 5 Interrupt Disable */
#define PWM_IDR1_FCHID6 (0x1u << 22) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 6 Interrupt Disable */
#define PWM_IDR1_FCHID7 (0x1u << 23) /**< \brief (PWM_IDR1) Fault Protection Trigger on Channel 7 Interrupt Disable */
/* -------- PWM_IMR1 : (PWM Offset: 0x18) PWM Interrupt Mask Register 1 -------- */
#define PWM_IMR1_CHID0 (0x1u << 0) /**< \brief (PWM_IMR1) Counter Event on Channel 0 Interrupt Mask */
#define PWM_IMR1_CHID1 (0x1u << 1) /**< \brief (PWM_IMR1) Counter Event on Channel 1 Interrupt Mask */
#define PWM_IMR1_CHID2 (0x1u << 2) /**< \brief (PWM_IMR1) Counter Event on Channel 2 Interrupt Mask */
#define PWM_IMR1_CHID3 (0x1u << 3) /**< \brief (PWM_IMR1) Counter Event on Channel 3 Interrupt Mask */
#define PWM_IMR1_CHID4 (0x1u << 4) /**< \brief (PWM_IMR1) Counter Event on Channel 4 Interrupt Mask */
#define PWM_IMR1_CHID5 (0x1u << 5) /**< \brief (PWM_IMR1) Counter Event on Channel 5 Interrupt Mask */
#define PWM_IMR1_CHID6 (0x1u << 6) /**< \brief (PWM_IMR1) Counter Event on Channel 6 Interrupt Mask */
#define PWM_IMR1_CHID7 (0x1u << 7) /**< \brief (PWM_IMR1) Counter Event on Channel 7 Interrupt Mask */
#define PWM_IMR1_FCHID0 (0x1u << 16) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 0 Interrupt Mask */
#define PWM_IMR1_FCHID1 (0x1u << 17) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 1 Interrupt Mask */
#define PWM_IMR1_FCHID2 (0x1u << 18) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 2 Interrupt Mask */
#define PWM_IMR1_FCHID3 (0x1u << 19) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 3 Interrupt Mask */
#define PWM_IMR1_FCHID4 (0x1u << 20) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 4 Interrupt Mask */
#define PWM_IMR1_FCHID5 (0x1u << 21) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 5 Interrupt Mask */
#define PWM_IMR1_FCHID6 (0x1u << 22) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 6 Interrupt Mask */
#define PWM_IMR1_FCHID7 (0x1u << 23) /**< \brief (PWM_IMR1) Fault Protection Trigger on Channel 7 Interrupt Mask */
/* -------- PWM_ISR1 : (PWM Offset: 0x1C) PWM Interrupt Status Register 1 -------- */
#define PWM_ISR1_CHID0 (0x1u << 0) /**< \brief (PWM_ISR1) Counter Event on Channel 0 */
#define PWM_ISR1_CHID1 (0x1u << 1) /**< \brief (PWM_ISR1) Counter Event on Channel 1 */
#define PWM_ISR1_CHID2 (0x1u << 2) /**< \brief (PWM_ISR1) Counter Event on Channel 2 */
#define PWM_ISR1_CHID3 (0x1u << 3) /**< \brief (PWM_ISR1) Counter Event on Channel 3 */
#define PWM_ISR1_CHID4 (0x1u << 4) /**< \brief (PWM_ISR1) Counter Event on Channel 4 */
#define PWM_ISR1_CHID5 (0x1u << 5) /**< \brief (PWM_ISR1) Counter Event on Channel 5 */
#define PWM_ISR1_CHID6 (0x1u << 6) /**< \brief (PWM_ISR1) Counter Event on Channel 6 */
#define PWM_ISR1_CHID7 (0x1u << 7) /**< \brief (PWM_ISR1) Counter Event on Channel 7 */
#define PWM_ISR1_FCHID0 (0x1u << 16) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 0 */
#define PWM_ISR1_FCHID1 (0x1u << 17) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 1 */
#define PWM_ISR1_FCHID2 (0x1u << 18) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 2 */
#define PWM_ISR1_FCHID3 (0x1u << 19) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 3 */
#define PWM_ISR1_FCHID4 (0x1u << 20) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 4 */
#define PWM_ISR1_FCHID5 (0x1u << 21) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 5 */
#define PWM_ISR1_FCHID6 (0x1u << 22) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 6 */
#define PWM_ISR1_FCHID7 (0x1u << 23) /**< \brief (PWM_ISR1) Fault Protection Trigger on Channel 7 */
/* -------- PWM_SCM : (PWM Offset: 0x20) PWM Sync Channels Mode Register -------- */
#define PWM_SCM_SYNC0 (0x1u << 0) /**< \brief (PWM_SCM) Synchronous Channel 0 */
#define PWM_SCM_SYNC1 (0x1u << 1) /**< \brief (PWM_SCM) Synchronous Channel 1 */
#define PWM_SCM_SYNC2 (0x1u << 2) /**< \brief (PWM_SCM) Synchronous Channel 2 */
#define PWM_SCM_SYNC3 (0x1u << 3) /**< \brief (PWM_SCM) Synchronous Channel 3 */
#define PWM_SCM_SYNC4 (0x1u << 4) /**< \brief (PWM_SCM) Synchronous Channel 4 */
#define PWM_SCM_SYNC5 (0x1u << 5) /**< \brief (PWM_SCM) Synchronous Channel 5 */
#define PWM_SCM_SYNC6 (0x1u << 6) /**< \brief (PWM_SCM) Synchronous Channel 6 */
#define PWM_SCM_SYNC7 (0x1u << 7) /**< \brief (PWM_SCM) Synchronous Channel 7 */
#define PWM_SCM_UPDM_Pos 16
#define PWM_SCM_UPDM_Msk (0x3u << PWM_SCM_UPDM_Pos) /**< \brief (PWM_SCM) Synchronous Channels Update Mode */
#define   PWM_SCM_UPDM_MODE0 (0x0u << 16) /**< \brief (PWM_SCM) Manual write of double buffer registers and manual update of synchronous channels */
#define   PWM_SCM_UPDM_MODE1 (0x1u << 16) /**< \brief (PWM_SCM) Manual write of double buffer registers and automatic update of synchronous channels */
#define   PWM_SCM_UPDM_MODE2 (0x2u << 16) /**< \brief (PWM_SCM) Automatic write of duty-cycle update registers by the PDC and automatic update of synchronous channels */
#define PWM_SCM_PTRM (0x1u << 20) /**< \brief (PWM_SCM) PDC Transfer Request Mode */
#define PWM_SCM_PTRCS_Pos 21
#define PWM_SCM_PTRCS_Msk (0x7u << PWM_SCM_PTRCS_Pos) /**< \brief (PWM_SCM) PDC Transfer Request Comparison Selection */
#define PWM_SCM_PTRCS(value) ((PWM_SCM_PTRCS_Msk & ((value) << PWM_SCM_PTRCS_Pos)))
/* -------- PWM_SCUC : (PWM Offset: 0x28) PWM Sync Channels Update Control Register -------- */
#define PWM_SCUC_UPDULOCK (0x1u << 0) /**< \brief (PWM_SCUC) Synchronous Channels Update Unlock */
/* -------- PWM_SCUP : (PWM Offset: 0x2C) PWM Sync Channels Update Period Register -------- */
#define PWM_SCUP_UPR_Pos 0
#define PWM_SCUP_UPR_Msk (0xfu << PWM_SCUP_UPR_Pos) /**< \brief (PWM_SCUP) Update Period */
#define PWM_SCUP_UPR(value) ((PWM_SCUP_UPR_Msk & ((value) << PWM_SCUP_UPR_Pos)))
#define PWM_SCUP_UPRCNT_Pos 4
#define PWM_SCUP_UPRCNT_Msk (0xfu << PWM_SCUP_UPRCNT_Pos) /**< \brief (PWM_SCUP) Update Period Counter */
#define PWM_SCUP_UPRCNT(value) ((PWM_SCUP_UPRCNT_Msk & ((value) << PWM_SCUP_UPRCNT_Pos)))
/* -------- PWM_SCUPUPD : (PWM Offset: 0x30) PWM Sync Channels Update Period Update Register -------- */
#define PWM_SCUPUPD_UPRUPD_Pos 0
#define PWM_SCUPUPD_UPRUPD_Msk (0xfu << PWM_SCUPUPD_UPRUPD_Pos) /**< \brief (PWM_SCUPUPD) Update Period Update */
#define PWM_SCUPUPD_UPRUPD(value) ((PWM_SCUPUPD_UPRUPD_Msk & ((value) << PWM_SCUPUPD_UPRUPD_Pos)))
/* -------- PWM_IER2 : (PWM Offset: 0x34) PWM Interrupt Enable Register 2 -------- */
#define PWM_IER2_WRDY (0x1u << 0) /**< \brief (PWM_IER2) Write Ready for Synchronous Channels Update Interrupt Enable */
#define PWM_IER2_ENDTX (0x1u << 1) /**< \brief (PWM_IER2) PDC End of TX Buffer Interrupt Enable */
#define PWM_IER2_TXBUFE (0x1u << 2) /**< \brief (PWM_IER2) PDC TX Buffer Empty Interrupt Enable */
#define PWM_IER2_UNRE (0x1u << 3) /**< \brief (PWM_IER2) Synchronous Channels Update Underrun Error Interrupt Enable */
#define PWM_IER2_CMPM0 (0x1u << 8) /**< \brief (PWM_IER2) Comparison 0 Match Interrupt Enable */
#define PWM_IER2_CMPM1 (0x1u << 9) /**< \brief (PWM_IER2) Comparison 1 Match Interrupt Enable */
#define PWM_IER2_CMPM2 (0x1u << 10) /**< \brief (PWM_IER2) Comparison 2 Match Interrupt Enable */
#define PWM_IER2_CMPM3 (0x1u << 11) /**< \brief (PWM_IER2) Comparison 3 Match Interrupt Enable */
#define PWM_IER2_CMPM4 (0x1u << 12) /**< \brief (PWM_IER2) Comparison 4 Match Interrupt Enable */
#define PWM_IER2_CMPM5 (0x1u << 13) /**< \brief (PWM_IER2) Comparison 5 Match Interrupt Enable */
#define PWM_IER2_CMPM6 (0x1u << 14) /**< \brief (PWM_IER2) Comparison 6 Match Interrupt Enable */
#define PWM_IER2_CMPM7 (0x1u << 15) /**< \brief (PWM_IER2) Comparison 7 Match Interrupt Enable */
#define PWM_IER2_CMPU0 (0x1u << 16) /**< \brief (PWM_IER2) Comparison 0 Update Interrupt Enable */
#define PWM_IER2_CMPU1 (0x1u << 17) /**< \brief (PWM_IER2) Comparison 1 Update Interrupt Enable */
#define PWM_IER2_CMPU2 (0x1u << 18) /**< \brief (PWM_IER2) Comparison 2 Update Interrupt Enable */
#define PWM_IER2_CMPU3 (0x1u << 19) /**< \brief (PWM_IER2) Comparison 3 Update Interrupt Enable */
#define PWM_IER2_CMPU4 (0x1u << 20) /**< \brief (PWM_IER2) Comparison 4 Update Interrupt Enable */
#define PWM_IER2_CMPU5 (0x1u << 21) /**< \brief (PWM_IER2) Comparison 5 Update Interrupt Enable */
#define PWM_IER2_CMPU6 (0x1u << 22) /**< \brief (PWM_IER2) Comparison 6 Update Interrupt Enable */
#define PWM_IER2_CMPU7 (0x1u << 23) /**< \brief (PWM_IER2) Comparison 7 Update Interrupt Enable */
/* -------- PWM_IDR2 : (PWM Offset: 0x38) PWM Interrupt Disable Register 2 -------- */
#define PWM_IDR2_WRDY (0x1u << 0) /**< \brief (PWM_IDR2) Write Ready for Synchronous Channels Update Interrupt Disable */
#define PWM_IDR2_ENDTX (0x1u << 1) /**< \brief (PWM_IDR2) PDC End of TX Buffer Interrupt Disable */
#define PWM_IDR2_TXBUFE (0x1u << 2) /**< \brief (PWM_IDR2) PDC TX Buffer Empty Interrupt Disable */
#define PWM_IDR2_UNRE (0x1u << 3) /**< \brief (PWM_IDR2) Synchronous Channels Update Underrun Error Interrupt Disable */
#define PWM_IDR2_CMPM0 (0x1u << 8) /**< \brief (PWM_IDR2) Comparison 0 Match Interrupt Disable */
#define PWM_IDR2_CMPM1 (0x1u << 9) /**< \brief (PWM_IDR2) Comparison 1 Match Interrupt Disable */
#define PWM_IDR2_CMPM2 (0x1u << 10) /**< \brief (PWM_IDR2) Comparison 2 Match Interrupt Disable */
#define PWM_IDR2_CMPM3 (0x1u << 11) /**< \brief (PWM_IDR2) Comparison 3 Match Interrupt Disable */
#define PWM_IDR2_CMPM4 (0x1u << 12) /**< \brief (PWM_IDR2) Comparison 4 Match Interrupt Disable */
#define PWM_IDR2_CMPM5 (0x1u << 13) /**< \brief (PWM_IDR2) Comparison 5 Match Interrupt Disable */
#define PWM_IDR2_CMPM6 (0x1u << 14) /**< \brief (PWM_IDR2) Comparison 6 Match Interrupt Disable */
#define PWM_IDR2_CMPM7 (0x1u << 15) /**< \brief (PWM_IDR2) Comparison 7 Match Interrupt Disable */
#define PWM_IDR2_CMPU0 (0x1u << 16) /**< \brief (PWM_IDR2) Comparison 0 Update Interrupt Disable */
#define PWM_IDR2_CMPU1 (0x1u << 17) /**< \brief (PWM_IDR2) Comparison 1 Update Interrupt Disable */
#define PWM_IDR2_CMPU2 (0x1u << 18) /**< \brief (PWM_IDR2) Comparison 2 Update Interrupt Disable */
#define PWM_IDR2_CMPU3 (0x1u << 19) /**< \brief (PWM_IDR2) Comparison 3 Update Interrupt Disable */
#define PWM_IDR2_CMPU4 (0x1u << 20) /**< \brief (PWM_IDR2) Comparison 4 Update Interrupt Disable */
#define PWM_IDR2_CMPU5 (0x1u << 21) /**< \brief (PWM_IDR2) Comparison 5 Update Interrupt Disable */
#define PWM_IDR2_CMPU6 (0x1u << 22) /**< \brief (PWM_IDR2) Comparison 6 Update Interrupt Disable */
#define PWM_IDR2_CMPU7 (0x1u << 23) /**< \brief (PWM_IDR2) Comparison 7 Update Interrupt Disable */
/* -------- PWM_IMR2 : (PWM Offset: 0x3C) PWM Interrupt Mask Register 2 -------- */
#define PWM_IMR2_WRDY (0x1u << 0) /**< \brief (PWM_IMR2) Write Ready for Synchronous Channels Update Interrupt Mask */
#define PWM_IMR2_ENDTX (0x1u << 1) /**< \brief (PWM_IMR2) PDC End of TX Buffer Interrupt Mask */
#define PWM_IMR2_TXBUFE (0x1u << 2) /**< \brief (PWM_IMR2) PDC TX Buffer Empty Interrupt Mask */
#define PWM_IMR2_UNRE (0x1u << 3) /**< \brief (PWM_IMR2) Synchronous Channels Update Underrun Error Interrupt Mask */
#define PWM_IMR2_CMPM0 (0x1u << 8) /**< \brief (PWM_IMR2) Comparison 0 Match Interrupt Mask */
#define PWM_IMR2_CMPM1 (0x1u << 9) /**< \brief (PWM_IMR2) Comparison 1 Match Interrupt Mask */
#define PWM_IMR2_CMPM2 (0x1u << 10) /**< \brief (PWM_IMR2) Comparison 2 Match Interrupt Mask */
#define PWM_IMR2_CMPM3 (0x1u << 11) /**< \brief (PWM_IMR2) Comparison 3 Match Interrupt Mask */
#define PWM_IMR2_CMPM4 (0x1u << 12) /**< \brief (PWM_IMR2) Comparison 4 Match Interrupt Mask */
#define PWM_IMR2_CMPM5 (0x1u << 13) /**< \brief (PWM_IMR2) Comparison 5 Match Interrupt Mask */
#define PWM_IMR2_CMPM6 (0x1u << 14) /**< \brief (PWM_IMR2) Comparison 6 Match Interrupt Mask */
#define PWM_IMR2_CMPM7 (0x1u << 15) /**< \brief (PWM_IMR2) Comparison 7 Match Interrupt Mask */
#define PWM_IMR2_CMPU0 (0x1u << 16) /**< \brief (PWM_IMR2) Comparison 0 Update Interrupt Mask */
#define PWM_IMR2_CMPU1 (0x1u << 17) /**< \brief (PWM_IMR2) Comparison 1 Update Interrupt Mask */
#define PWM_IMR2_CMPU2 (0x1u << 18) /**< \brief (PWM_IMR2) Comparison 2 Update Interrupt Mask */
#define PWM_IMR2_CMPU3 (0x1u << 19) /**< \brief (PWM_IMR2) Comparison 3 Update Interrupt Mask */
#define PWM_IMR2_CMPU4 (0x1u << 20) /**< \brief (PWM_IMR2) Comparison 4 Update Interrupt Mask */
#define PWM_IMR2_CMPU5 (0x1u << 21) /**< \brief (PWM_IMR2) Comparison 5 Update Interrupt Mask */
#define PWM_IMR2_CMPU6 (0x1u << 22) /**< \brief (PWM_IMR2) Comparison 6 Update Interrupt Mask */
#define PWM_IMR2_CMPU7 (0x1u << 23) /**< \brief (PWM_IMR2) Comparison 7 Update Interrupt Mask */
/* -------- PWM_ISR2 : (PWM Offset: 0x40) PWM Interrupt Status Register 2 -------- */
#define PWM_ISR2_WRDY (0x1u << 0) /**< \brief (PWM_ISR2) Write Ready for Synchronous Channels Update */
#define PWM_ISR2_ENDTX (0x1u << 1) /**< \brief (PWM_ISR2) PDC End of TX Buffer */
#define PWM_ISR2_TXBUFE (0x1u << 2) /**< \brief (PWM_ISR2) PDC TX Buffer Empty */
#define PWM_ISR2_UNRE (0x1u << 3) /**< \brief (PWM_ISR2) Synchronous Channels Update Underrun Error */
#define PWM_ISR2_CMPM0 (0x1u << 8) /**< \brief (PWM_ISR2) Comparison 0 Match */
#define PWM_ISR2_CMPM1 (0x1u << 9) /**< \brief (PWM_ISR2) Comparison 1 Match */
#define PWM_ISR2_CMPM2 (0x1u << 10) /**< \brief (PWM_ISR2) Comparison 2 Match */
#define PWM_ISR2_CMPM3 (0x1u << 11) /**< \brief (PWM_ISR2) Comparison 3 Match */
#define PWM_ISR2_CMPM4 (0x1u << 12) /**< \brief (PWM_ISR2) Comparison 4 Match */
#define PWM_ISR2_CMPM5 (0x1u << 13) /**< \brief (PWM_ISR2) Comparison 5 Match */
#define PWM_ISR2_CMPM6 (0x1u << 14) /**< \brief (PWM_ISR2) Comparison 6 Match */
#define PWM_ISR2_CMPM7 (0x1u << 15) /**< \brief (PWM_ISR2) Comparison 7 Match */
#define PWM_ISR2_CMPU0 (0x1u << 16) /**< \brief (PWM_ISR2) Comparison 0 Update */
#define PWM_ISR2_CMPU1 (0x1u << 17) /**< \brief (PWM_ISR2) Comparison 1 Update */
#define PWM_ISR2_CMPU2 (0x1u << 18) /**< \brief (PWM_ISR2) Comparison 2 Update */
#define PWM_ISR2_CMPU3 (0x1u << 19) /**< \brief (PWM_ISR2) Comparison 3 Update */
#define PWM_ISR2_CMPU4 (0x1u << 20) /**< \brief (PWM_ISR2) Comparison 4 Update */
#define PWM_ISR2_CMPU5 (0x1u << 21) /**< \brief (PWM_ISR2) Comparison 5 Update */
#define PWM_ISR2_CMPU6 (0x1u << 22) /**< \brief (PWM_ISR2) Comparison 6 Update */
#define PWM_ISR2_CMPU7 (0x1u << 23) /**< \brief (PWM_ISR2) Comparison 7 Update */
/* -------- PWM_OOV : (PWM Offset: 0x44) PWM Output Override Value Register -------- */
#define PWM_OOV_OOVH0 (0x1u << 0) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 0 */
#define PWM_OOV_OOVH1 (0x1u << 1) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 1 */
#define PWM_OOV_OOVH2 (0x1u << 2) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 2 */
#define PWM_OOV_OOVH3 (0x1u << 3) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 3 */
#define PWM_OOV_OOVH4 (0x1u << 4) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 4 */
#define PWM_OOV_OOVH5 (0x1u << 5) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 5 */
#define PWM_OOV_OOVH6 (0x1u << 6) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 6 */
#define PWM_OOV_OOVH7 (0x1u << 7) /**< \brief (PWM_OOV) Output Override Value for PWMH output of the channel 7 */
#define PWM_OOV_OOVL0 (0x1u << 16) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 0 */
#define PWM_OOV_OOVL1 (0x1u << 17) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 1 */
#define PWM_OOV_OOVL2 (0x1u << 18) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 2 */
#define PWM_OOV_OOVL3 (0x1u << 19) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 3 */
#define PWM_OOV_OOVL4 (0x1u << 20) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 4 */
#define PWM_OOV_OOVL5 (0x1u << 21) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 5 */
#define PWM_OOV_OOVL6 (0x1u << 22) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 6 */
#define PWM_OOV_OOVL7 (0x1u << 23) /**< \brief (PWM_OOV) Output Override Value for PWML output of the channel 7 */
/* -------- PWM_OS : (PWM Offset: 0x48) PWM Output Selection Register -------- */
#define PWM_OS_OSH0 (0x1u << 0) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 0 */
#define PWM_OS_OSH1 (0x1u << 1) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 1 */
#define PWM_OS_OSH2 (0x1u << 2) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 2 */
#define PWM_OS_OSH3 (0x1u << 3) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 3 */
#define PWM_OS_OSH4 (0x1u << 4) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 4 */
#define PWM_OS_OSH5 (0x1u << 5) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 5 */
#define PWM_OS_OSH6 (0x1u << 6) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 6 */
#define PWM_OS_OSH7 (0x1u << 7) /**< \brief (PWM_OS) Output Selection for PWMH output of the channel 7 */
#define PWM_OS_OSL0 (0x1u << 16) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 0 */
#define PWM_OS_OSL1 (0x1u << 17) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 1 */
#define PWM_OS_OSL2 (0x1u << 18) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 2 */
#define PWM_OS_OSL3 (0x1u << 19) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 3 */
#define PWM_OS_OSL4 (0x1u << 20) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 4 */
#define PWM_OS_OSL5 (0x1u << 21) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 5 */
#define PWM_OS_OSL6 (0x1u << 22) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 6 */
#define PWM_OS_OSL7 (0x1u << 23) /**< \brief (PWM_OS) Output Selection for PWML output of the channel 7 */
/* -------- PWM_OSS : (PWM Offset: 0x4C) PWM Output Selection Set Register -------- */
#define PWM_OSS_OSSH0 (0x1u << 0) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 0 */
#define PWM_OSS_OSSH1 (0x1u << 1) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 1 */
#define PWM_OSS_OSSH2 (0x1u << 2) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 2 */
#define PWM_OSS_OSSH3 (0x1u << 3) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 3 */
#define PWM_OSS_OSSH4 (0x1u << 4) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 4 */
#define PWM_OSS_OSSH5 (0x1u << 5) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 5 */
#define PWM_OSS_OSSH6 (0x1u << 6) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 6 */
#define PWM_OSS_OSSH7 (0x1u << 7) /**< \brief (PWM_OSS) Output Selection Set for PWMH output of the channel 7 */
#define PWM_OSS_OSSL0 (0x1u << 16) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 0 */
#define PWM_OSS_OSSL1 (0x1u << 17) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 1 */
#define PWM_OSS_OSSL2 (0x1u << 18) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 2 */
#define PWM_OSS_OSSL3 (0x1u << 19) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 3 */
#define PWM_OSS_OSSL4 (0x1u << 20) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 4 */
#define PWM_OSS_OSSL5 (0x1u << 21) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 5 */
#define PWM_OSS_OSSL6 (0x1u << 22) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 6 */
#define PWM_OSS_OSSL7 (0x1u << 23) /**< \brief (PWM_OSS) Output Selection Set for PWML output of the channel 7 */
/* -------- PWM_OSC : (PWM Offset: 0x50) PWM Output Selection Clear Register -------- */
#define PWM_OSC_OSCH0 (0x1u << 0) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 0 */
#define PWM_OSC_OSCH1 (0x1u << 1) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 1 */
#define PWM_OSC_OSCH2 (0x1u << 2) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 2 */
#define PWM_OSC_OSCH3 (0x1u << 3) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 3 */
#define PWM_OSC_OSCH4 (0x1u << 4) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 4 */
#define PWM_OSC_OSCH5 (0x1u << 5) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 5 */
#define PWM_OSC_OSCH6 (0x1u << 6) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 6 */
#define PWM_OSC_OSCH7 (0x1u << 7) /**< \brief (PWM_OSC) Output Selection Clear for PWMH output of the channel 7 */
#define PWM_OSC_OSCL0 (0x1u << 16) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 0 */
#define PWM_OSC_OSCL1 (0x1u << 17) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 1 */
#define PWM_OSC_OSCL2 (0x1u << 18) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 2 */
#define PWM_OSC_OSCL3 (0x1u << 19) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 3 */
#define PWM_OSC_OSCL4 (0x1u << 20) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 4 */
#define PWM_OSC_OSCL5 (0x1u << 21) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 5 */
#define PWM_OSC_OSCL6 (0x1u << 22) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 6 */
#define PWM_OSC_OSCL7 (0x1u << 23) /**< \brief (PWM_OSC) Output Selection Clear for PWML output of the channel 7 */
/* -------- PWM_OSSUPD : (PWM Offset: 0x54) PWM Output Selection Set Update Register -------- */
#define PWM_OSSUPD_OSSUPH0 (0x1u << 0) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 0 */
#define PWM_OSSUPD_OSSUPH1 (0x1u << 1) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 1 */
#define PWM_OSSUPD_OSSUPH2 (0x1u << 2) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 2 */
#define PWM_OSSUPD_OSSUPH3 (0x1u << 3) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 3 */
#define PWM_OSSUPD_OSSUPH4 (0x1u << 4) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 4 */
#define PWM_OSSUPD_OSSUPH5 (0x1u << 5) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 5 */
#define PWM_OSSUPD_OSSUPH6 (0x1u << 6) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 6 */
#define PWM_OSSUPD_OSSUPH7 (0x1u << 7) /**< \brief (PWM_OSSUPD) Output Selection Set for PWMH output of the channel 7 */
#define PWM_OSSUPD_OSSUPL0 (0x1u << 16) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 0 */
#define PWM_OSSUPD_OSSUPL1 (0x1u << 17) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 1 */
#define PWM_OSSUPD_OSSUPL2 (0x1u << 18) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 2 */
#define PWM_OSSUPD_OSSUPL3 (0x1u << 19) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 3 */
#define PWM_OSSUPD_OSSUPL4 (0x1u << 20) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 4 */
#define PWM_OSSUPD_OSSUPL5 (0x1u << 21) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 5 */
#define PWM_OSSUPD_OSSUPL6 (0x1u << 22) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 6 */
#define PWM_OSSUPD_OSSUPL7 (0x1u << 23) /**< \brief (PWM_OSSUPD) Output Selection Set for PWML output of the channel 7 */
/* -------- PWM_OSCUPD : (PWM Offset: 0x58) PWM Output Selection Clear Update Register -------- */
#define PWM_OSCUPD_OSCUPH0 (0x1u << 0) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 0 */
#define PWM_OSCUPD_OSCUPH1 (0x1u << 1) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 1 */
#define PWM_OSCUPD_OSCUPH2 (0x1u << 2) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 2 */
#define PWM_OSCUPD_OSCUPH3 (0x1u << 3) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 3 */
#define PWM_OSCUPD_OSCUPH4 (0x1u << 4) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 4 */
#define PWM_OSCUPD_OSCUPH5 (0x1u << 5) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 5 */
#define PWM_OSCUPD_OSCUPH6 (0x1u << 6) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 6 */
#define PWM_OSCUPD_OSCUPH7 (0x1u << 7) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWMH output of the channel 7 */
#define PWM_OSCUPD_OSCUPL0 (0x1u << 16) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 0 */
#define PWM_OSCUPD_OSCUPL1 (0x1u << 17) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 1 */
#define PWM_OSCUPD_OSCUPL2 (0x1u << 18) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 2 */
#define PWM_OSCUPD_OSCUPL3 (0x1u << 19) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 3 */
#define PWM_OSCUPD_OSCUPL4 (0x1u << 20) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 4 */
#define PWM_OSCUPD_OSCUPL5 (0x1u << 21) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 5 */
#define PWM_OSCUPD_OSCUPDL6 (0x1u << 22) /**< \brief (PWM_OSCUPD)  */
#define PWM_OSCUPD_OSCUPL7 (0x1u << 23) /**< \brief (PWM_OSCUPD) Output Selection Clear for PWML output of the channel 7 */
/* -------- PWM_FMR : (PWM Offset: 0x5C) PWM Fault Mode Register -------- */
#define PWM_FMR_FPOL_Pos 0
#define PWM_FMR_FPOL_Msk (0xffu << PWM_FMR_FPOL_Pos) /**< \brief (PWM_FMR) Fault Polarity (fault input bit varies from 0 to 5) */
#define PWM_FMR_FPOL(value) ((PWM_FMR_FPOL_Msk & ((value) << PWM_FMR_FPOL_Pos)))
#define PWM_FMR_FMOD_Pos 8
#define PWM_FMR_FMOD_Msk (0xffu << PWM_FMR_FMOD_Pos) /**< \brief (PWM_FMR) Fault Activation Mode (fault input bit varies from 0 to 5) */
#define PWM_FMR_FMOD(value) ((PWM_FMR_FMOD_Msk & ((value) << PWM_FMR_FMOD_Pos)))
#define PWM_FMR_FFIL_Pos 16
#define PWM_FMR_FFIL_Msk (0xffu << PWM_FMR_FFIL_Pos) /**< \brief (PWM_FMR) Fault Filtering (fault input bit varies from 0 to 5) */
#define PWM_FMR_FFIL(value) ((PWM_FMR_FFIL_Msk & ((value) << PWM_FMR_FFIL_Pos)))
/* -------- PWM_FSR : (PWM Offset: 0x60) PWM Fault Status Register -------- */
#define PWM_FSR_FIV_Pos 0
#define PWM_FSR_FIV_Msk (0xffu << PWM_FSR_FIV_Pos) /**< \brief (PWM_FSR) Fault Input Value (fault input bit varies from 0 to 5) */
#define PWM_FSR_FS_Pos 8
#define PWM_FSR_FS_Msk (0xffu << PWM_FSR_FS_Pos) /**< \brief (PWM_FSR) Fault Status (fault input bit varies from 0 to 5) */
/* -------- PWM_FCR : (PWM Offset: 0x64) PWM Fault Clear Register -------- */
#define PWM_FCR_FCLR_Pos 0
#define PWM_FCR_FCLR_Msk (0xffu << PWM_FCR_FCLR_Pos) /**< \brief (PWM_FCR) Fault Clear (fault input bit varies from 0 to 5) */
#define PWM_FCR_FCLR(value) ((PWM_FCR_FCLR_Msk & ((value) << PWM_FCR_FCLR_Pos)))
/* -------- PWM_FPV : (PWM Offset: 0x68) PWM Fault Protection Value Register -------- */
#define PWM_FPV_FPVH0 (0x1u << 0) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 0 */
#define PWM_FPV_FPVH1 (0x1u << 1) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 1 */
#define PWM_FPV_FPVH2 (0x1u << 2) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 2 */
#define PWM_FPV_FPVH3 (0x1u << 3) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 3 */
#define PWM_FPV_FPVH4 (0x1u << 4) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 4 */
#define PWM_FPV_FPVH5 (0x1u << 5) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 5 */
#define PWM_FPV_FPVH6 (0x1u << 6) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 6 */
#define PWM_FPV_FPVH7 (0x1u << 7) /**< \brief (PWM_FPV) Fault Protection Value for PWMH output on channel 7 */
#define PWM_FPV_FPVL0 (0x1u << 16) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 0 */
#define PWM_FPV_FPVL1 (0x1u << 17) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 1 */
#define PWM_FPV_FPVL2 (0x1u << 18) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 2 */
#define PWM_FPV_FPVL3 (0x1u << 19) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 3 */
#define PWM_FPV_FPVL4 (0x1u << 20) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 4 */
#define PWM_FPV_FPVL5 (0x1u << 21) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 5 */
#define PWM_FPV_FPVL6 (0x1u << 22) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 6 */
#define PWM_FPV_FPVL7 (0x1u << 23) /**< \brief (PWM_FPV) Fault Protection Value for PWML output on channel 7 */
/* -------- PWM_FPE1 : (PWM Offset: 0x6C) PWM Fault Protection Enable Register 1 -------- */
#define PWM_FPE1_FPE0_Pos 0
#define PWM_FPE1_FPE0_Msk (0xffu << PWM_FPE1_FPE0_Pos) /**< \brief (PWM_FPE1) Fault Protection Enable for channel 0 (fault input bit varies from 0 to 5) */
#define PWM_FPE1_FPE0(value) ((PWM_FPE1_FPE0_Msk & ((value) << PWM_FPE1_FPE0_Pos)))
#define PWM_FPE1_FPE1_Pos 8
#define PWM_FPE1_FPE1_Msk (0xffu << PWM_FPE1_FPE1_Pos) /**< \brief (PWM_FPE1) Fault Protection Enable for channel 1 (fault input bit varies from 0 to 5) */
#define PWM_FPE1_FPE1(value) ((PWM_FPE1_FPE1_Msk & ((value) << PWM_FPE1_FPE1_Pos)))
#define PWM_FPE1_FPE2_Pos 16
#define PWM_FPE1_FPE2_Msk (0xffu << PWM_FPE1_FPE2_Pos) /**< \brief (PWM_FPE1) Fault Protection Enable for channel 2 (fault input bit varies from 0 to 5) */
#define PWM_FPE1_FPE2(value) ((PWM_FPE1_FPE2_Msk & ((value) << PWM_FPE1_FPE2_Pos)))
#define PWM_FPE1_FPE3_Pos 24
#define PWM_FPE1_FPE3_Msk (0xffu << PWM_FPE1_FPE3_Pos) /**< \brief (PWM_FPE1) Fault Protection Enable for channel 3 (fault input bit varies from 0 to 5) */
#define PWM_FPE1_FPE3(value) ((PWM_FPE1_FPE3_Msk & ((value) << PWM_FPE1_FPE3_Pos)))
/* -------- PWM_FPE2 : (PWM Offset: 0x70) PWM Fault Protection Enable Register 2 -------- */
#define PWM_FPE2_FPE4_Pos 0
#define PWM_FPE2_FPE4_Msk (0xffu << PWM_FPE2_FPE4_Pos) /**< \brief (PWM_FPE2) Fault Protection Enable for channel 4 (fault input bit varies from 0 to 5) */
#define PWM_FPE2_FPE4(value) ((PWM_FPE2_FPE4_Msk & ((value) << PWM_FPE2_FPE4_Pos)))
#define PWM_FPE2_FPE5_Pos 8
#define PWM_FPE2_FPE5_Msk (0xffu << PWM_FPE2_FPE5_Pos) /**< \brief (PWM_FPE2) Fault Protection Enable for channel 5 (fault input bit varies from 0 to 5) */
#define PWM_FPE2_FPE5(value) ((PWM_FPE2_FPE5_Msk & ((value) << PWM_FPE2_FPE5_Pos)))
#define PWM_FPE2_FPE6_Pos 16
#define PWM_FPE2_FPE6_Msk (0xffu << PWM_FPE2_FPE6_Pos) /**< \brief (PWM_FPE2) Fault Protection Enable for channel 6 (fault input bit varies from 0 to 5) */
#define PWM_FPE2_FPE6(value) ((PWM_FPE2_FPE6_Msk & ((value) << PWM_FPE2_FPE6_Pos)))
#define PWM_FPE2_FPE7_Pos 24
#define PWM_FPE2_FPE7_Msk (0xffu << PWM_FPE2_FPE7_Pos) /**< \brief (PWM_FPE2) Fault Protection Enable for channel 7 (fault input bit varies from 0 to 5) */
#define PWM_FPE2_FPE7(value) ((PWM_FPE2_FPE7_Msk & ((value) << PWM_FPE2_FPE7_Pos)))
/* -------- PWM_ELMR[2] : (PWM Offset: 0x7C) PWM Event Line 0 Mode Register -------- */
#define PWM_ELMR_CSEL0 (0x1u << 0) /**< \brief (PWM_ELMR[2]) Comparison 0 Selection */
#define PWM_ELMR_CSEL1 (0x1u << 1) /**< \brief (PWM_ELMR[2]) Comparison 1 Selection */
#define PWM_ELMR_CSEL2 (0x1u << 2) /**< \brief (PWM_ELMR[2]) Comparison 2 Selection */
#define PWM_ELMR_CSEL3 (0x1u << 3) /**< \brief (PWM_ELMR[2]) Comparison 3 Selection */
#define PWM_ELMR_CSEL4 (0x1u << 4) /**< \brief (PWM_ELMR[2]) Comparison 4 Selection */
#define PWM_ELMR_CSEL5 (0x1u << 5) /**< \brief (PWM_ELMR[2]) Comparison 5 Selection */
#define PWM_ELMR_CSEL6 (0x1u << 6) /**< \brief (PWM_ELMR[2]) Comparison 6 Selection */
#define PWM_ELMR_CSEL7 (0x1u << 7) /**< \brief (PWM_ELMR[2]) Comparison 7 Selection */
/* -------- PWM_SMMR : (PWM Offset: 0xB0) PWM Stepper Motor Mode Register -------- */
#define PWM_SMMR_GCEN0 (0x1u << 0) /**< \brief (PWM_SMMR) Gray Count ENable */
#define PWM_SMMR_GCEN1 (0x1u << 1) /**< \brief (PWM_SMMR) Gray Count ENable */
#define PWM_SMMR_GCEN2 (0x1u << 2) /**< \brief (PWM_SMMR) Gray Count ENable */
#define PWM_SMMR_GCEN3 (0x1u << 3) /**< \brief (PWM_SMMR) Gray Count ENable */
#define PWM_SMMR_DOWN0 (0x1u << 16) /**< \brief (PWM_SMMR) DOWN Count */
#define PWM_SMMR_DOWN1 (0x1u << 17) /**< \brief (PWM_SMMR) DOWN Count */
#define PWM_SMMR_DOWN2 (0x1u << 18) /**< \brief (PWM_SMMR) DOWN Count */
#define PWM_SMMR_DOWN3 (0x1u << 19) /**< \brief (PWM_SMMR) DOWN Count */
/* -------- PWM_WPCR : (PWM Offset: 0xE4) PWM Write Protect Control Register -------- */
#define PWM_WPCR_WPCMD_Pos 0
#define PWM_WPCR_WPCMD_Msk (0x3u << PWM_WPCR_WPCMD_Pos) /**< \brief (PWM_WPCR) Write Protect Command */
#define PWM_WPCR_WPCMD(value) ((PWM_WPCR_WPCMD_Msk & ((value) << PWM_WPCR_WPCMD_Pos)))
#define PWM_WPCR_WPRG0 (0x1u << 2) /**< \brief (PWM_WPCR) Write Protect Register Group 0 */
#define PWM_WPCR_WPRG1 (0x1u << 3) /**< \brief (PWM_WPCR) Write Protect Register Group 1 */
#define PWM_WPCR_WPRG2 (0x1u << 4) /**< \brief (PWM_WPCR) Write Protect Register Group 2 */
#define PWM_WPCR_WPRG3 (0x1u << 5) /**< \brief (PWM_WPCR) Write Protect Register Group 3 */
#define PWM_WPCR_WPRG4 (0x1u << 6) /**< \brief (PWM_WPCR) Write Protect Register Group 4 */
#define PWM_WPCR_WPRG5 (0x1u << 7) /**< \brief (PWM_WPCR) Write Protect Register Group 5 */
#define PWM_WPCR_WPKEY_Pos 8
#define PWM_WPCR_WPKEY_Msk (0xffffffu << PWM_WPCR_WPKEY_Pos) /**< \brief (PWM_WPCR) Write Protect Key */
#define PWM_WPCR_WPKEY(value) ((PWM_WPCR_WPKEY_Msk & ((value) << PWM_WPCR_WPKEY_Pos)))
/* -------- PWM_WPSR : (PWM Offset: 0xE8) PWM Write Protect Status Register -------- */
#define PWM_WPSR_WPSWS0 (0x1u << 0) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPSWS1 (0x1u << 1) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPSWS2 (0x1u << 2) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPSWS3 (0x1u << 3) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPSWS4 (0x1u << 4) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPSWS5 (0x1u << 5) /**< \brief (PWM_WPSR) Write Protect SW Status */
#define PWM_WPSR_WPVS (0x1u << 7) /**< \brief (PWM_WPSR) Write Protect Violation Status */
#define PWM_WPSR_WPHWS0 (0x1u << 8) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPHWS1 (0x1u << 9) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPHWS2 (0x1u << 10) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPHWS3 (0x1u << 11) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPHWS4 (0x1u << 12) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPHWS5 (0x1u << 13) /**< \brief (PWM_WPSR) Write Protect HW Status */
#define PWM_WPSR_WPVSRC_Pos 16
#define PWM_WPSR_WPVSRC_Msk (0xffffu << PWM_WPSR_WPVSRC_Pos) /**< \brief (PWM_WPSR) Write Protect Violation Source */
/* -------- PWM_TPR : (PWM Offset: 0x108) Transmit Pointer Register -------- */
#define PWM_TPR_TXPTR_Pos 0
#define PWM_TPR_TXPTR_Msk (0xffffffffu << PWM_TPR_TXPTR_Pos) /**< \brief (PWM_TPR) Transmit Counter Register */
#define PWM_TPR_TXPTR(value) ((PWM_TPR_TXPTR_Msk & ((value) << PWM_TPR_TXPTR_Pos)))
/* -------- PWM_TCR : (PWM Offset: 0x10C) Transmit Counter Register -------- */
#define PWM_TCR_TXCTR_Pos 0
#define PWM_TCR_TXCTR_Msk (0xffffu << PWM_TCR_TXCTR_Pos) /**< \brief (PWM_TCR) Transmit Counter Register */
#define PWM_TCR_TXCTR(value) ((PWM_TCR_TXCTR_Msk & ((value) << PWM_TCR_TXCTR_Pos)))
/* -------- PWM_TNPR : (PWM Offset: 0x118) Transmit Next Pointer Register -------- */
#define PWM_TNPR_TXNPTR_Pos 0
#define PWM_TNPR_TXNPTR_Msk (0xffffffffu << PWM_TNPR_TXNPTR_Pos) /**< \brief (PWM_TNPR) Transmit Next Pointer */
#define PWM_TNPR_TXNPTR(value) ((PWM_TNPR_TXNPTR_Msk & ((value) << PWM_TNPR_TXNPTR_Pos)))
/* -------- PWM_TNCR : (PWM Offset: 0x11C) Transmit Next Counter Register -------- */
#define PWM_TNCR_TXNCTR_Pos 0
#define PWM_TNCR_TXNCTR_Msk (0xffffu << PWM_TNCR_TXNCTR_Pos) /**< \brief (PWM_TNCR) Transmit Counter Next */
#define PWM_TNCR_TXNCTR(value) ((PWM_TNCR_TXNCTR_Msk & ((value) << PWM_TNCR_TXNCTR_Pos)))
/* -------- PWM_PTCR : (PWM Offset: 0x120) Transfer Control Register -------- */
#define PWM_PTCR_RXTEN (0x1u << 0) /**< \brief (PWM_PTCR) Receiver Transfer Enable */
#define PWM_PTCR_RXTDIS (0x1u << 1) /**< \brief (PWM_PTCR) Receiver Transfer Disable */
#define PWM_PTCR_TXTEN (0x1u << 8) /**< \brief (PWM_PTCR) Transmitter Transfer Enable */
#define PWM_PTCR_TXTDIS (0x1u << 9) /**< \brief (PWM_PTCR) Transmitter Transfer Disable */
/* -------- PWM_PTSR : (PWM Offset: 0x124) Transfer Status Register -------- */
#define PWM_PTSR_RXTEN (0x1u << 0) /**< \brief (PWM_PTSR) Receiver Transfer Enable */
#define PWM_PTSR_TXTEN (0x1u << 8) /**< \brief (PWM_PTSR) Transmitter Transfer Enable */
/* -------- PWM_CMPV : (PWM Offset: N/A) PWM Comparison 0 Value Register -------- */
#define PWM_CMPV_CV_Pos 0
#define PWM_CMPV_CV_Msk (0xffffffu << PWM_CMPV_CV_Pos) /**< \brief (PWM_CMPV) Comparison x Value */
#define PWM_CMPV_CV(value) ((PWM_CMPV_CV_Msk & ((value) << PWM_CMPV_CV_Pos)))
#define PWM_CMPV_CVM (0x1u << 24) /**< \brief (PWM_CMPV) Comparison x Value Mode */
/* -------- PWM_CMPVUPD : (PWM Offset: N/A) PWM Comparison 0 Value Update Register -------- */
#define PWM_CMPVUPD_CVUPD_Pos 0
#define PWM_CMPVUPD_CVUPD_Msk (0xffffffu << PWM_CMPVUPD_CVUPD_Pos) /**< \brief (PWM_CMPVUPD) Comparison x Value Update */
#define PWM_CMPVUPD_CVUPD(value) ((PWM_CMPVUPD_CVUPD_Msk & ((value) << PWM_CMPVUPD_CVUPD_Pos)))
#define PWM_CMPVUPD_CVMUPD (0x1u << 24) /**< \brief (PWM_CMPVUPD) Comparison x Value Mode Update */
/* -------- PWM_CMPM : (PWM Offset: N/A) PWM Comparison 0 Mode Register -------- */
#define PWM_CMPM_CEN (0x1u << 0) /**< \brief (PWM_CMPM) Comparison x Enable */
#define PWM_CMPM_CTR_Pos 4
#define PWM_CMPM_CTR_Msk (0xfu << PWM_CMPM_CTR_Pos) /**< \brief (PWM_CMPM) Comparison x Trigger */
#define PWM_CMPM_CTR(value) ((PWM_CMPM_CTR_Msk & ((value) << PWM_CMPM_CTR_Pos)))
#define PWM_CMPM_CPR_Pos 8
#define PWM_CMPM_CPR_Msk (0xfu << PWM_CMPM_CPR_Pos) /**< \brief (PWM_CMPM) Comparison x Period */
#define PWM_CMPM_CPR(value) ((PWM_CMPM_CPR_Msk & ((value) << PWM_CMPM_CPR_Pos)))
#define PWM_CMPM_CPRCNT_Pos 12
#define PWM_CMPM_CPRCNT_Msk (0xfu << PWM_CMPM_CPRCNT_Pos) /**< \brief (PWM_CMPM) Comparison x Period Counter */
#define PWM_CMPM_CPRCNT(value) ((PWM_CMPM_CPRCNT_Msk & ((value) << PWM_CMPM_CPRCNT_Pos)))
#define PWM_CMPM_CUPR_Pos 16
#define PWM_CMPM_CUPR_Msk (0xfu << PWM_CMPM_CUPR_Pos) /**< \brief (PWM_CMPM) Comparison x Update Period */
#define PWM_CMPM_CUPR(value) ((PWM_CMPM_CUPR_Msk & ((value) << PWM_CMPM_CUPR_Pos)))
#define PWM_CMPM_CUPRCNT_Pos 20
#define PWM_CMPM_CUPRCNT_Msk (0xfu << PWM_CMPM_CUPRCNT_Pos) /**< \brief (PWM_CMPM) Comparison x Update Period Counter */
#define PWM_CMPM_CUPRCNT(value) ((PWM_CMPM_CUPRCNT_Msk & ((value) << PWM_CMPM_CUPRCNT_Pos)))
/* -------- PWM_CMPMUPD : (PWM Offset: N/A) PWM Comparison 0 Mode Update Register -------- */
#define PWM_CMPMUPD_CENUPD (0x1u << 0) /**< \brief (PWM_CMPMUPD) Comparison x Enable Update */
#define PWM_CMPMUPD_CTRUPD_Pos 4
#define PWM_CMPMUPD_CTRUPD_Msk (0xfu << PWM_CMPMUPD_CTRUPD_Pos) /**< \brief (PWM_CMPMUPD) Comparison x Trigger Update */
#define PWM_CMPMUPD_CTRUPD(value) ((PWM_CMPMUPD_CTRUPD_Msk & ((value) << PWM_CMPMUPD_CTRUPD_Pos)))
#define PWM_CMPMUPD_CPRUPD_Pos 8
#define PWM_CMPMUPD_CPRUPD_Msk (0xfu << PWM_CMPMUPD_CPRUPD_Pos) /**< \brief (PWM_CMPMUPD) Comparison x Period Update */
#define PWM_CMPMUPD_CPRUPD(value) ((PWM_CMPMUPD_CPRUPD_Msk & ((value) << PWM_CMPMUPD_CPRUPD_Pos)))
#define PWM_CMPMUPD_CUPRUPD_Pos 16
#define PWM_CMPMUPD_CUPRUPD_Msk (0xfu << PWM_CMPMUPD_CUPRUPD_Pos) /**< \brief (PWM_CMPMUPD) Comparison x Update Period Update */
#define PWM_CMPMUPD_CUPRUPD(value) ((PWM_CMPMUPD_CUPRUPD_Msk & ((value) << PWM_CMPMUPD_CUPRUPD_Pos)))
/* -------- PWM_CMR : (PWM Offset: N/A) PWM Channel Mode Register -------- */
#define PWM_CMR_CPRE_Pos 0
#define PWM_CMR_CPRE_Msk (0xfu << PWM_CMR_CPRE_Pos) /**< \brief (PWM_CMR) Channel Pre-scaler */
#define   PWM_CMR_CPRE_MCK (0x0u << 0) /**< \brief (PWM_CMR) Master clock */
#define   PWM_CMR_CPRE_MCK_DIV_2 (0x1u << 0) /**< \brief (PWM_CMR) Master clock/2 */
#define   PWM_CMR_CPRE_MCK_DIV_4 (0x2u << 0) /**< \brief (PWM_CMR) Master clock/4 */
#define   PWM_CMR_CPRE_MCK_DIV_8 (0x3u << 0) /**< \brief (PWM_CMR) Master clock/8 */
#define   PWM_CMR_CPRE_MCK_DIV_16 (0x4u << 0) /**< \brief (PWM_CMR) Master clock/16 */
#define   PWM_CMR_CPRE_MCK_DIV_32 (0x5u << 0) /**< \brief (PWM_CMR) Master clock/32 */
#define   PWM_CMR_CPRE_MCK_DIV_64 (0x6u << 0) /**< \brief (PWM_CMR) Master clock/64 */
#define   PWM_CMR_CPRE_MCK_DIV_128 (0x7u << 0) /**< \brief (PWM_CMR) Master clock/128 */
#define   PWM_CMR_CPRE_MCK_DIV_256 (0x8u << 0) /**< \brief (PWM_CMR) Master clock/256 */
#define   PWM_CMR_CPRE_MCK_DIV_512 (0x9u << 0) /**< \brief (PWM_CMR) Master clock/512 */
#define   PWM_CMR_CPRE_MCK_DIV_1024 (0xAu << 0) /**< \brief (PWM_CMR) Master clock/1024 */
#define   PWM_CMR_CPRE_CLKA (0xBu << 0) /**< \brief (PWM_CMR) Clock A */
#define   PWM_CMR_CPRE_CLKB (0xCu << 0) /**< \brief (PWM_CMR) Clock B */
#define PWM_CMR_CALG (0x1u << 8) /**< \brief (PWM_CMR) Channel Alignment */
#define PWM_CMR_CPOL (0x1u << 9) /**< \brief (PWM_CMR) Channel Polarity */
#define PWM_CMR_CES (0x1u << 10) /**< \brief (PWM_CMR) Counter Event Selection */
#define PWM_CMR_DTE (0x1u << 16) /**< \brief (PWM_CMR) Dead-Time Generator Enable */
#define PWM_CMR_DTHI (0x1u << 17) /**< \brief (PWM_CMR) Dead-Time PWMHx Output Inverted */
#define PWM_CMR_DTLI (0x1u << 18) /**< \brief (PWM_CMR) Dead-Time PWMLx Output Inverted */
/* -------- PWM_CDTY : (PWM Offset: N/A) PWM Channel Duty Cycle Register -------- */
#define PWM_CDTY_CDTY_Pos 0
#define PWM_CDTY_CDTY_Msk (0xffffffu << PWM_CDTY_CDTY_Pos) /**< \brief (PWM_CDTY) Channel Duty-Cycle */
#define PWM_CDTY_CDTY(value) ((PWM_CDTY_CDTY_Msk & ((value) << PWM_CDTY_CDTY_Pos)))
/* -------- PWM_CDTYUPD : (PWM Offset: N/A) PWM Channel Duty Cycle Update Register -------- */
#define PWM_CDTYUPD_CDTYUPD_Pos 0
#define PWM_CDTYUPD_CDTYUPD_Msk (0xffffffu << PWM_CDTYUPD_CDTYUPD_Pos) /**< \brief (PWM_CDTYUPD) Channel Duty-Cycle Update */
#define PWM_CDTYUPD_CDTYUPD(value) ((PWM_CDTYUPD_CDTYUPD_Msk & ((value) << PWM_CDTYUPD_CDTYUPD_Pos)))
/* -------- PWM_CPRD : (PWM Offset: N/A) PWM Channel Period Register -------- */
#define PWM_CPRD_CPRD_Pos 0
#define PWM_CPRD_CPRD_Msk (0xffffffu << PWM_CPRD_CPRD_Pos) /**< \brief (PWM_CPRD) Channel Period */
#define PWM_CPRD_CPRD(value) ((PWM_CPRD_CPRD_Msk & ((value) << PWM_CPRD_CPRD_Pos)))
/* -------- PWM_CPRDUPD : (PWM Offset: N/A) PWM Channel Period Update Register -------- */
#define PWM_CPRDUPD_CPRDUPD_Pos 0
#define PWM_CPRDUPD_CPRDUPD_Msk (0xffffffu << PWM_CPRDUPD_CPRDUPD_Pos) /**< \brief (PWM_CPRDUPD) Channel Period Update */
#define PWM_CPRDUPD_CPRDUPD(value) ((PWM_CPRDUPD_CPRDUPD_Msk & ((value) << PWM_CPRDUPD_CPRDUPD_Pos)))
/* -------- PWM_CCNT : (PWM Offset: N/A) PWM Channel Counter Register -------- */
#define PWM_CCNT_CNT_Pos 0
#define PWM_CCNT_CNT_Msk (0xffffffu << PWM_CCNT_CNT_Pos) /**< \brief (PWM_CCNT) Channel Counter Register */
/* -------- PWM_DT : (PWM Offset: N/A) PWM Channel Dead Time Register -------- */
#define PWM_DT_DTH_Pos 0
#define PWM_DT_DTH_Msk (0xffffu << PWM_DT_DTH_Pos) /**< \brief (PWM_DT) Dead-Time Value for PWMHx Output */
#define PWM_DT_DTH(value) ((PWM_DT_DTH_Msk & ((value) << PWM_DT_DTH_Pos)))
#define PWM_DT_DTL_Pos 16
#define PWM_DT_DTL_Msk (0xffffu << PWM_DT_DTL_Pos) /**< \brief (PWM_DT) Dead-Time Value for PWMLx Output */
#define PWM_DT_DTL(value) ((PWM_DT_DTL_Msk & ((value) << PWM_DT_DTL_Pos)))
/* -------- PWM_DTUPD : (PWM Offset: N/A) PWM Channel Dead Time Update Register -------- */
#define PWM_DTUPD_DTHUPD_Pos 0
#define PWM_DTUPD_DTHUPD_Msk (0xffffu << PWM_DTUPD_DTHUPD_Pos) /**< \brief (PWM_DTUPD) Dead-Time Value Update for PWMHx Output */
#define PWM_DTUPD_DTHUPD(value) ((PWM_DTUPD_DTHUPD_Msk & ((value) << PWM_DTUPD_DTHUPD_Pos)))
#define PWM_DTUPD_DTLUPD_Pos 16
#define PWM_DTUPD_DTLUPD_Msk (0xffffu << PWM_DTUPD_DTLUPD_Pos) /**< \brief (PWM_DTUPD) Dead-Time Value Update for PWMLx Output */
#define PWM_DTUPD_DTLUPD(value) ((PWM_DTUPD_DTLUPD_Msk & ((value) << PWM_DTUPD_DTLUPD_Pos)))
/*@}*/ /* end of group SAM3XA_PWM */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Reset Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_RSTC Reset Controller */
/*@{*/

/** \brief Rstc hardware registers */
typedef struct {
  WoReg RSTC_CR; /**< \brief (Rstc Offset: 0x00) Control Register */
  RoReg RSTC_SR; /**< \brief (Rstc Offset: 0x04) Status Register */
  RwReg RSTC_MR; /**< \brief (Rstc Offset: 0x08) Mode Register */
} Rstc;
/* -------- RSTC_CR : (RSTC Offset: 0x00) Control Register -------- */
#define RSTC_CR_PROCRST (0x1u << 0) /**< \brief (RSTC_CR) Processor Reset */
#define RSTC_CR_PERRST (0x1u << 2) /**< \brief (RSTC_CR) Peripheral Reset */
#define RSTC_CR_EXTRST (0x1u << 3) /**< \brief (RSTC_CR) External Reset */
#define RSTC_CR_KEY_Pos 24
#define RSTC_CR_KEY_Msk (0xffu << RSTC_CR_KEY_Pos) /**< \brief (RSTC_CR) Password */
#define RSTC_CR_KEY(value) ((RSTC_CR_KEY_Msk & ((value) << RSTC_CR_KEY_Pos)))
/* -------- RSTC_SR : (RSTC Offset: 0x04) Status Register -------- */
#define RSTC_SR_URSTS (0x1u << 0) /**< \brief (RSTC_SR) User Reset Status */
#define RSTC_SR_RSTTYP_Pos 8
#define RSTC_SR_RSTTYP_Msk (0x7u << RSTC_SR_RSTTYP_Pos) /**< \brief (RSTC_SR) Reset Type */
#define RSTC_SR_NRSTL (0x1u << 16) /**< \brief (RSTC_SR) NRST Pin Level */
#define RSTC_SR_SRCMP (0x1u << 17) /**< \brief (RSTC_SR) Software Reset Command in Progress */
/* -------- RSTC_MR : (RSTC Offset: 0x08) Mode Register -------- */
#define RSTC_MR_URSTEN (0x1u << 0) /**< \brief (RSTC_MR) User Reset Enable */
#define RSTC_MR_URSTIEN (0x1u << 4) /**< \brief (RSTC_MR) User Reset Interrupt Enable */
#define RSTC_MR_ERSTL_Pos 8
#define RSTC_MR_ERSTL_Msk (0xfu << RSTC_MR_ERSTL_Pos) /**< \brief (RSTC_MR) External Reset Length */
#define RSTC_MR_ERSTL(value) ((RSTC_MR_ERSTL_Msk & ((value) << RSTC_MR_ERSTL_Pos)))
#define RSTC_MR_KEY_Pos 24
#define RSTC_MR_KEY_Msk (0xffu << RSTC_MR_KEY_Pos) /**< \brief (RSTC_MR) Password */
#define RSTC_MR_KEY(value) ((RSTC_MR_KEY_Msk & ((value) << RSTC_MR_KEY_Pos)))
/*@}*/ /* end of group SAM3XA_RSTC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Real-time Clock */
/* ============================================================================= */
/** \addtogroup SAM3XA_RTC Real-time Clock */
/*@{*/

/** \brief Rtc hardware registers */
typedef struct {
  RwReg RTC_CR;        /**< \brief (Rtc Offset: 0x00) Control Register */
  RwReg RTC_MR;        /**< \brief (Rtc Offset: 0x04) Mode Register */
  RwReg RTC_TIMR;      /**< \brief (Rtc Offset: 0x08) Time Register */
  RwReg RTC_CALR;      /**< \brief (Rtc Offset: 0x0C) Calendar Register */
  RwReg RTC_TIMALR;    /**< \brief (Rtc Offset: 0x10) Time Alarm Register */
  RwReg RTC_CALALR;    /**< \brief (Rtc Offset: 0x14) Calendar Alarm Register */
  RoReg RTC_SR;        /**< \brief (Rtc Offset: 0x18) Status Register */
  WoReg RTC_SCCR;      /**< \brief (Rtc Offset: 0x1C) Status Clear Command Register */
  WoReg RTC_IER;       /**< \brief (Rtc Offset: 0x20) Interrupt Enable Register */
  WoReg RTC_IDR;       /**< \brief (Rtc Offset: 0x24) Interrupt Disable Register */
  RoReg RTC_IMR;       /**< \brief (Rtc Offset: 0x28) Interrupt Mask Register */
  RoReg RTC_VER;       /**< \brief (Rtc Offset: 0x2C) Valid Entry Register */
  RoReg Reserved1[45];
  RwReg RTC_WPMR;      /**< \brief (Rtc Offset: 0xE4) Write Protect Mode Register */
} Rtc;
/* -------- RTC_CR : (RTC Offset: 0x00) Control Register -------- */
#define RTC_CR_UPDTIM (0x1u << 0) /**< \brief (RTC_CR) Update Request Time Register */
#define RTC_CR_UPDCAL (0x1u << 1) /**< \brief (RTC_CR) Update Request Calendar Register */
#define RTC_CR_TIMEVSEL_Pos 8
#define RTC_CR_TIMEVSEL_Msk (0x3u << RTC_CR_TIMEVSEL_Pos) /**< \brief (RTC_CR) Time Event Selection */
#define   RTC_CR_TIMEVSEL_MINUTE (0x0u << 8) /**< \brief (RTC_CR) Minute change */
#define   RTC_CR_TIMEVSEL_HOUR (0x1u << 8) /**< \brief (RTC_CR) Hour change */
#define   RTC_CR_TIMEVSEL_MIDNIGHT (0x2u << 8) /**< \brief (RTC_CR) Every day at midnight */
#define   RTC_CR_TIMEVSEL_NOON (0x3u << 8) /**< \brief (RTC_CR) Every day at noon */
#define RTC_CR_CALEVSEL_Pos 16
#define RTC_CR_CALEVSEL_Msk (0x3u << RTC_CR_CALEVSEL_Pos) /**< \brief (RTC_CR) Calendar Event Selection */
#define   RTC_CR_CALEVSEL_WEEK (0x0u << 16) /**< \brief (RTC_CR) Week change (every Monday at time 00:00:00) */
#define   RTC_CR_CALEVSEL_MONTH (0x1u << 16) /**< \brief (RTC_CR) Month change (every 01 of each month at time 00:00:00) */
#define   RTC_CR_CALEVSEL_YEAR (0x2u << 16) /**< \brief (RTC_CR) Year change (every January 1 at time 00:00:00) */
/* -------- RTC_MR : (RTC Offset: 0x04) Mode Register -------- */
#define RTC_MR_HRMOD (0x1u << 0) /**< \brief (RTC_MR) 12-/24-hour Mode */
/* -------- RTC_TIMR : (RTC Offset: 0x08) Time Register -------- */
#define RTC_TIMR_SEC_Pos 0
#define RTC_TIMR_SEC_Msk (0x7fu << RTC_TIMR_SEC_Pos) /**< \brief (RTC_TIMR) Current Second */
#define RTC_TIMR_SEC(value) ((RTC_TIMR_SEC_Msk & ((value) << RTC_TIMR_SEC_Pos)))
#define RTC_TIMR_MIN_Pos 8
#define RTC_TIMR_MIN_Msk (0x7fu << RTC_TIMR_MIN_Pos) /**< \brief (RTC_TIMR) Current Minute */
#define RTC_TIMR_MIN(value) ((RTC_TIMR_MIN_Msk & ((value) << RTC_TIMR_MIN_Pos)))
#define RTC_TIMR_HOUR_Pos 16
#define RTC_TIMR_HOUR_Msk (0x3fu << RTC_TIMR_HOUR_Pos) /**< \brief (RTC_TIMR) Current Hour */
#define RTC_TIMR_HOUR(value) ((RTC_TIMR_HOUR_Msk & ((value) << RTC_TIMR_HOUR_Pos)))
#define RTC_TIMR_AMPM (0x1u << 22) /**< \brief (RTC_TIMR) Ante Meridiem Post Meridiem Indicator */
/* -------- RTC_CALR : (RTC Offset: 0x0C) Calendar Register -------- */
#define RTC_CALR_CENT_Pos 0
#define RTC_CALR_CENT_Msk (0x7fu << RTC_CALR_CENT_Pos) /**< \brief (RTC_CALR) Current Century */
#define RTC_CALR_CENT(value) ((RTC_CALR_CENT_Msk & ((value) << RTC_CALR_CENT_Pos)))
#define RTC_CALR_YEAR_Pos 8
#define RTC_CALR_YEAR_Msk (0xffu << RTC_CALR_YEAR_Pos) /**< \brief (RTC_CALR) Current Year */
#define RTC_CALR_YEAR(value) ((RTC_CALR_YEAR_Msk & ((value) << RTC_CALR_YEAR_Pos)))
#define RTC_CALR_MONTH_Pos 16
#define RTC_CALR_MONTH_Msk (0x1fu << RTC_CALR_MONTH_Pos) /**< \brief (RTC_CALR) Current Month */
#define RTC_CALR_MONTH(value) ((RTC_CALR_MONTH_Msk & ((value) << RTC_CALR_MONTH_Pos)))
#define RTC_CALR_DAY_Pos 21
#define RTC_CALR_DAY_Msk (0x7u << RTC_CALR_DAY_Pos) /**< \brief (RTC_CALR) Current Day in Current Week */
#define RTC_CALR_DAY(value) ((RTC_CALR_DAY_Msk & ((value) << RTC_CALR_DAY_Pos)))
#define RTC_CALR_DATE_Pos 24
#define RTC_CALR_DATE_Msk (0x3fu << RTC_CALR_DATE_Pos) /**< \brief (RTC_CALR) Current Day in Current Month */
#define RTC_CALR_DATE(value) ((RTC_CALR_DATE_Msk & ((value) << RTC_CALR_DATE_Pos)))
/* -------- RTC_TIMALR : (RTC Offset: 0x10) Time Alarm Register -------- */
#define RTC_TIMALR_SEC_Pos 0
#define RTC_TIMALR_SEC_Msk (0x7fu << RTC_TIMALR_SEC_Pos) /**< \brief (RTC_TIMALR) Second Alarm */
#define RTC_TIMALR_SEC(value) ((RTC_TIMALR_SEC_Msk & ((value) << RTC_TIMALR_SEC_Pos)))
#define RTC_TIMALR_SECEN (0x1u << 7) /**< \brief (RTC_TIMALR) Second Alarm Enable */
#define RTC_TIMALR_MIN_Pos 8
#define RTC_TIMALR_MIN_Msk (0x7fu << RTC_TIMALR_MIN_Pos) /**< \brief (RTC_TIMALR) Minute Alarm */
#define RTC_TIMALR_MIN(value) ((RTC_TIMALR_MIN_Msk & ((value) << RTC_TIMALR_MIN_Pos)))
#define RTC_TIMALR_MINEN (0x1u << 15) /**< \brief (RTC_TIMALR) Minute Alarm Enable */
#define RTC_TIMALR_HOUR_Pos 16
#define RTC_TIMALR_HOUR_Msk (0x3fu << RTC_TIMALR_HOUR_Pos) /**< \brief (RTC_TIMALR) Hour Alarm */
#define RTC_TIMALR_HOUR(value) ((RTC_TIMALR_HOUR_Msk & ((value) << RTC_TIMALR_HOUR_Pos)))
#define RTC_TIMALR_AMPM (0x1u << 22) /**< \brief (RTC_TIMALR) AM/PM Indicator */
#define RTC_TIMALR_HOUREN (0x1u << 23) /**< \brief (RTC_TIMALR) Hour Alarm Enable */
/* -------- RTC_CALALR : (RTC Offset: 0x14) Calendar Alarm Register -------- */
#define RTC_CALALR_MONTH_Pos 16
#define RTC_CALALR_MONTH_Msk (0x1fu << RTC_CALALR_MONTH_Pos) /**< \brief (RTC_CALALR) Month Alarm */
#define RTC_CALALR_MONTH(value) ((RTC_CALALR_MONTH_Msk & ((value) << RTC_CALALR_MONTH_Pos)))
#define RTC_CALALR_MTHEN (0x1u << 23) /**< \brief (RTC_CALALR) Month Alarm Enable */
#define RTC_CALALR_DATE_Pos 24
#define RTC_CALALR_DATE_Msk (0x3fu << RTC_CALALR_DATE_Pos) /**< \brief (RTC_CALALR) Date Alarm */
#define RTC_CALALR_DATE(value) ((RTC_CALALR_DATE_Msk & ((value) << RTC_CALALR_DATE_Pos)))
#define RTC_CALALR_DATEEN (0x1u << 31) /**< \brief (RTC_CALALR) Date Alarm Enable */
/* -------- RTC_SR : (RTC Offset: 0x18) Status Register -------- */
#define RTC_SR_ACKUPD (0x1u << 0) /**< \brief (RTC_SR) Acknowledge for Update */
#define RTC_SR_ALARM (0x1u << 1) /**< \brief (RTC_SR) Alarm Flag */
#define RTC_SR_SEC (0x1u << 2) /**< \brief (RTC_SR) Second Event */
#define RTC_SR_TIMEV (0x1u << 3) /**< \brief (RTC_SR) Time Event */
#define RTC_SR_CALEV (0x1u << 4) /**< \brief (RTC_SR) Calendar Event */
/* -------- RTC_SCCR : (RTC Offset: 0x1C) Status Clear Command Register -------- */
#define RTC_SCCR_ACKCLR (0x1u << 0) /**< \brief (RTC_SCCR) Acknowledge Clear */
#define RTC_SCCR_ALRCLR (0x1u << 1) /**< \brief (RTC_SCCR) Alarm Clear */
#define RTC_SCCR_SECCLR (0x1u << 2) /**< \brief (RTC_SCCR) Second Clear */
#define RTC_SCCR_TIMCLR (0x1u << 3) /**< \brief (RTC_SCCR) Time Clear */
#define RTC_SCCR_CALCLR (0x1u << 4) /**< \brief (RTC_SCCR) Calendar Clear */
/* -------- RTC_IER : (RTC Offset: 0x20) Interrupt Enable Register -------- */
#define RTC_IER_ACKEN (0x1u << 0) /**< \brief (RTC_IER) Acknowledge Update Interrupt Enable */
#define RTC_IER_ALREN (0x1u << 1) /**< \brief (RTC_IER) Alarm Interrupt Enable */
#define RTC_IER_SECEN (0x1u << 2) /**< \brief (RTC_IER) Second Event Interrupt Enable */
#define RTC_IER_TIMEN (0x1u << 3) /**< \brief (RTC_IER) Time Event Interrupt Enable */
#define RTC_IER_CALEN (0x1u << 4) /**< \brief (RTC_IER) Calendar Event Interrupt Enable */
/* -------- RTC_IDR : (RTC Offset: 0x24) Interrupt Disable Register -------- */
#define RTC_IDR_ACKDIS (0x1u << 0) /**< \brief (RTC_IDR) Acknowledge Update Interrupt Disable */
#define RTC_IDR_ALRDIS (0x1u << 1) /**< \brief (RTC_IDR) Alarm Interrupt Disable */
#define RTC_IDR_SECDIS (0x1u << 2) /**< \brief (RTC_IDR) Second Event Interrupt Disable */
#define RTC_IDR_TIMDIS (0x1u << 3) /**< \brief (RTC_IDR) Time Event Interrupt Disable */
#define RTC_IDR_CALDIS (0x1u << 4) /**< \brief (RTC_IDR) Calendar Event Interrupt Disable */
/* -------- RTC_IMR : (RTC Offset: 0x28) Interrupt Mask Register -------- */
#define RTC_IMR_ACK (0x1u << 0) /**< \brief (RTC_IMR) Acknowledge Update Interrupt Mask */
#define RTC_IMR_ALR (0x1u << 1) /**< \brief (RTC_IMR) Alarm Interrupt Mask */
#define RTC_IMR_SEC (0x1u << 2) /**< \brief (RTC_IMR) Second Event Interrupt Mask */
#define RTC_IMR_TIM (0x1u << 3) /**< \brief (RTC_IMR) Time Event Interrupt Mask */
#define RTC_IMR_CAL (0x1u << 4) /**< \brief (RTC_IMR) Calendar Event Interrupt Mask */
/* -------- RTC_VER : (RTC Offset: 0x2C) Valid Entry Register -------- */
#define RTC_VER_NVTIM (0x1u << 0) /**< \brief (RTC_VER) Non-valid Time */
#define RTC_VER_NVCAL (0x1u << 1) /**< \brief (RTC_VER) Non-valid Calendar */
#define RTC_VER_NVTIMALR (0x1u << 2) /**< \brief (RTC_VER) Non-valid Time Alarm */
#define RTC_VER_NVCALALR (0x1u << 3) /**< \brief (RTC_VER) Non-valid Calendar Alarm */
/* -------- RTC_WPMR : (RTC Offset: 0xE4) Write Protect Mode Register -------- */
#define RTC_WPMR_WPEN (0x1u << 0) /**< \brief (RTC_WPMR) Write Protect Enable */
#define RTC_WPMR_WPKEY_Pos 8
#define RTC_WPMR_WPKEY_Msk (0xffffffu << RTC_WPMR_WPKEY_Pos) /**< \brief (RTC_WPMR)  */
#define RTC_WPMR_WPKEY(value) ((RTC_WPMR_WPKEY_Msk & ((value) << RTC_WPMR_WPKEY_Pos)))
/*@}*/ /* end of group SAM3XA_RTC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Real-time Timer */
/* ============================================================================= */
/** \addtogroup SAM3XA_RTT Real-time Timer */
/*@{*/

/** \brief Rtt hardware registers */
typedef struct {
  RwReg RTT_MR; /**< \brief (Rtt Offset: 0x00) Mode Register */
  RwReg RTT_AR; /**< \brief (Rtt Offset: 0x04) Alarm Register */
  RoReg RTT_VR; /**< \brief (Rtt Offset: 0x08) Value Register */
  RoReg RTT_SR; /**< \brief (Rtt Offset: 0x0C) Status Register */
} Rtt;
/* -------- RTT_MR : (RTT Offset: 0x00) Mode Register -------- */
#define RTT_MR_RTPRES_Pos 0
#define RTT_MR_RTPRES_Msk (0xffffu << RTT_MR_RTPRES_Pos) /**< \brief (RTT_MR) Real-time Timer Prescaler Value */
#define RTT_MR_RTPRES(value) ((RTT_MR_RTPRES_Msk & ((value) << RTT_MR_RTPRES_Pos)))
#define RTT_MR_ALMIEN (0x1u << 16) /**< \brief (RTT_MR) Alarm Interrupt Enable */
#define RTT_MR_RTTINCIEN (0x1u << 17) /**< \brief (RTT_MR) Real-time Timer Increment Interrupt Enable */
#define RTT_MR_RTTRST (0x1u << 18) /**< \brief (RTT_MR) Real-time Timer Restart */
/* -------- RTT_AR : (RTT Offset: 0x04) Alarm Register -------- */
#define RTT_AR_ALMV_Pos 0
#define RTT_AR_ALMV_Msk (0xffffffffu << RTT_AR_ALMV_Pos) /**< \brief (RTT_AR) Alarm Value */
#define RTT_AR_ALMV(value) ((RTT_AR_ALMV_Msk & ((value) << RTT_AR_ALMV_Pos)))
/* -------- RTT_VR : (RTT Offset: 0x08) Value Register -------- */
#define RTT_VR_CRTV_Pos 0
#define RTT_VR_CRTV_Msk (0xffffffffu << RTT_VR_CRTV_Pos) /**< \brief (RTT_VR) Current Real-time Value */
/* -------- RTT_SR : (RTT Offset: 0x0C) Status Register -------- */
#define RTT_SR_ALMS (0x1u << 0) /**< \brief (RTT_SR) Real-time Alarm Status */
#define RTT_SR_RTTINC (0x1u << 1) /**< \brief (RTT_SR) Real-time Timer Increment */
/*@}*/ /* end of group SAM3XA_RTT */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR SDRAM Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_SDRAMC SDRAM Controller */
/*@{*/

/** \brief Sdramc hardware registers */
typedef struct {
  RwReg SDRAMC_MR;    /**< \brief (Sdramc Offset: 0x00) SDRAMC Mode Register */
  RwReg SDRAMC_TR;    /**< \brief (Sdramc Offset: 0x04) SDRAMC Refresh Timer Register */
  RwReg SDRAMC_CR;    /**< \brief (Sdramc Offset: 0x08) SDRAMC Configuration Register */
  RoReg Reserved1[1];
  RwReg SDRAMC_LPR;   /**< \brief (Sdramc Offset: 0x10) SDRAMC Low Power Register */
  WoReg SDRAMC_IER;   /**< \brief (Sdramc Offset: 0x14) SDRAMC Interrupt Enable Register */
  WoReg SDRAMC_IDR;   /**< \brief (Sdramc Offset: 0x18) SDRAMC Interrupt Disable Register */
  RoReg SDRAMC_IMR;   /**< \brief (Sdramc Offset: 0x1C) SDRAMC Interrupt Mask Register */
  RoReg SDRAMC_ISR;   /**< \brief (Sdramc Offset: 0x20) SDRAMC Interrupt Status Register */
  RwReg SDRAMC_MDR;   /**< \brief (Sdramc Offset: 0x24) SDRAMC Memory Device Register */
  RwReg SDRAMC_CR1;   /**< \brief (Sdramc Offset: 0x28) SDRAMC Configuration Register 1 */
  RwReg SDRAMC_OCMS;  /**< \brief (Sdramc Offset: 0x2C) SDRAMC OCMS Register 1 */
} Sdramc;
/* -------- SDRAMC_MR : (SDRAMC Offset: 0x00) SDRAMC Mode Register -------- */
#define SDRAMC_MR_MODE_Pos 0
#define SDRAMC_MR_MODE_Msk (0x7u << SDRAMC_MR_MODE_Pos) /**< \brief (SDRAMC_MR) SDRAMC Command Mode */
#define   SDRAMC_MR_MODE_NORMAL (0x0u << 0) /**< \brief (SDRAMC_MR) Normal mode. Any access to the SDRAM is decoded normally. To activate this mode, command must be followed by a write to the SDRAM. */
#define   SDRAMC_MR_MODE_NOP (0x1u << 0) /**< \brief (SDRAMC_MR) The SDRAM Controller issues a NOP command when the SDRAM device is accessed regardless of the cycle. To activate this mode, command must be followed by a write to the SDRAM. */
#define   SDRAMC_MR_MODE_ALLBANKS_PRECHARGE (0x2u << 0) /**< \brief (SDRAMC_MR) The SDRAM Controller issues an "All Banks Precharge" command when the SDRAM device is accessed regardless of the cycle. To activate this mode, command must be followed by a write to the SDRAM. */
#define   SDRAMC_MR_MODE_LOAD_MODEREG (0x3u << 0) /**< \brief (SDRAMC_MR) The SDRAM Controller issues a "Load Mode Register" command when the SDRAM device is accessed regardless of the cycle. To activate this mode, command must be followed by a write to the SDRAM. */
#define   SDRAMC_MR_MODE_AUTO_REFRESH (0x4u << 0) /**< \brief (SDRAMC_MR) The SDRAM Controller issues an "Auto-Refresh" Command when the SDRAM device is accessed regardless of the cycle. Previously, an "All Banks Precharge" command must be issued. To activate this mode, command must be followed by a write to the SDRAM. */
#define   SDRAMC_MR_MODE_EXT_LOAD_MODEREG (0x5u << 0) /**< \brief (SDRAMC_MR) The SDRAM Controller issues an "Extended Load Mode Register" command when the SDRAM device is accessed regardless of the cycle. To activate this mode, the "Extended Load Mode Register" command must be followed by a write to the SDRAM. The write in the SDRAM must be done in the appropriate bank; most low-power SDRAM devices use the bank 1. */
#define   SDRAMC_MR_MODE_DEEP_POWERDOWN (0x6u << 0) /**< \brief (SDRAMC_MR) Deep power-down mode. Enters deep power-down mode. */
/* -------- SDRAMC_TR : (SDRAMC Offset: 0x04) SDRAMC Refresh Timer Register -------- */
#define SDRAMC_TR_COUNT_Pos 0
#define SDRAMC_TR_COUNT_Msk (0xfffu << SDRAMC_TR_COUNT_Pos) /**< \brief (SDRAMC_TR) SDRAMC Refresh Timer Count */
#define SDRAMC_TR_COUNT(value) ((SDRAMC_TR_COUNT_Msk & ((value) << SDRAMC_TR_COUNT_Pos)))
/* -------- SDRAMC_CR : (SDRAMC Offset: 0x08) SDRAMC Configuration Register -------- */
#define SDRAMC_CR_NC_Pos 0
#define SDRAMC_CR_NC_Msk (0x3u << SDRAMC_CR_NC_Pos) /**< \brief (SDRAMC_CR) Number of Column Bits */
#define   SDRAMC_CR_NC_COL8 (0x0u << 0) /**< \brief (SDRAMC_CR) 8 column bits */
#define   SDRAMC_CR_NC_COL9 (0x1u << 0) /**< \brief (SDRAMC_CR) 9 column bits */
#define   SDRAMC_CR_NC_COL10 (0x2u << 0) /**< \brief (SDRAMC_CR) 10 column bits */
#define   SDRAMC_CR_NC_COL11 (0x3u << 0) /**< \brief (SDRAMC_CR) 11 column bits */
#define SDRAMC_CR_NR_Pos 2
#define SDRAMC_CR_NR_Msk (0x3u << SDRAMC_CR_NR_Pos) /**< \brief (SDRAMC_CR) Number of Row Bits */
#define   SDRAMC_CR_NR_ROW11 (0x0u << 2) /**< \brief (SDRAMC_CR) 11 row bits */
#define   SDRAMC_CR_NR_ROW12 (0x1u << 2) /**< \brief (SDRAMC_CR) 12 row bits */
#define   SDRAMC_CR_NR_ROW13 (0x2u << 2) /**< \brief (SDRAMC_CR) 13 row bits */
#define SDRAMC_CR_NB (0x1u << 4) /**< \brief (SDRAMC_CR) Number of Banks */
#define   SDRAMC_CR_NB_BANK2 (0x0u << 4) /**< \brief (SDRAMC_CR) 2 banks */
#define   SDRAMC_CR_NB_BANK4 (0x1u << 4) /**< \brief (SDRAMC_CR) 4 banks */
#define SDRAMC_CR_CAS_Pos 5
#define SDRAMC_CR_CAS_Msk (0x3u << SDRAMC_CR_CAS_Pos) /**< \brief (SDRAMC_CR) CAS Latency */
#define   SDRAMC_CR_CAS_LATENCY1 (0x1u << 5) /**< \brief (SDRAMC_CR) 1 cycle CAS latency */
#define   SDRAMC_CR_CAS_LATENCY2 (0x2u << 5) /**< \brief (SDRAMC_CR) 2 cycle CAS latency */
#define   SDRAMC_CR_CAS_LATENCY3 (0x3u << 5) /**< \brief (SDRAMC_CR) 3 cycle CAS latency */
#define SDRAMC_CR_DBW (0x1u << 7) /**< \brief (SDRAMC_CR) Data Bus Width */
#define SDRAMC_CR_TWR_Pos 8
#define SDRAMC_CR_TWR_Msk (0xfu << SDRAMC_CR_TWR_Pos) /**< \brief (SDRAMC_CR) Write Recovery Delay */
#define SDRAMC_CR_TWR(value) ((SDRAMC_CR_TWR_Msk & ((value) << SDRAMC_CR_TWR_Pos)))
#define SDRAMC_CR_TRC_TRFC_Pos 12
#define SDRAMC_CR_TRC_TRFC_Msk (0xfu << SDRAMC_CR_TRC_TRFC_Pos) /**< \brief (SDRAMC_CR) Row Cycle Delay and Row Refresh Cycle */
#define SDRAMC_CR_TRC_TRFC(value) ((SDRAMC_CR_TRC_TRFC_Msk & ((value) << SDRAMC_CR_TRC_TRFC_Pos)))
#define SDRAMC_CR_TRP_Pos 16
#define SDRAMC_CR_TRP_Msk (0xfu << SDRAMC_CR_TRP_Pos) /**< \brief (SDRAMC_CR) Row Precharge Delay */
#define SDRAMC_CR_TRP(value) ((SDRAMC_CR_TRP_Msk & ((value) << SDRAMC_CR_TRP_Pos)))
#define SDRAMC_CR_TRCD_Pos 20
#define SDRAMC_CR_TRCD_Msk (0xfu << SDRAMC_CR_TRCD_Pos) /**< \brief (SDRAMC_CR) Row to Column Delay */
#define SDRAMC_CR_TRCD(value) ((SDRAMC_CR_TRCD_Msk & ((value) << SDRAMC_CR_TRCD_Pos)))
#define SDRAMC_CR_TRAS_Pos 24
#define SDRAMC_CR_TRAS_Msk (0xfu << SDRAMC_CR_TRAS_Pos) /**< \brief (SDRAMC_CR) Active to Precharge Delay */
#define SDRAMC_CR_TRAS(value) ((SDRAMC_CR_TRAS_Msk & ((value) << SDRAMC_CR_TRAS_Pos)))
#define SDRAMC_CR_TXSR_Pos 28
#define SDRAMC_CR_TXSR_Msk (0xfu << SDRAMC_CR_TXSR_Pos) /**< \brief (SDRAMC_CR) Exit Self Refresh to Active Delay */
#define SDRAMC_CR_TXSR(value) ((SDRAMC_CR_TXSR_Msk & ((value) << SDRAMC_CR_TXSR_Pos)))
/* -------- SDRAMC_LPR : (SDRAMC Offset: 0x10) SDRAMC Low Power Register -------- */
#define SDRAMC_LPR_LPCB_Pos 0
#define SDRAMC_LPR_LPCB_Msk (0x3u << SDRAMC_LPR_LPCB_Pos) /**< \brief (SDRAMC_LPR) Low-power Configuration Bits */
#define   SDRAMC_LPR_LPCB_DISABLED (0x0u << 0) /**< \brief (SDRAMC_LPR) Low Power Feature is inhibited: no Power-down, Self-refresh or Deep Power-down command is issued to the SDRAM device. */
#define   SDRAMC_LPR_LPCB_SELF_REFRESH (0x1u << 0) /**< \brief (SDRAMC_LPR) The SDRAM Controller issues a Self-refresh command to the SDRAM device, the SDCK clock is deactivated and the SDCKE signal is set low. The SDRAM device leaves the Self Refresh Mode when accessed and enters it after the access. */
#define   SDRAMC_LPR_LPCB_POWER_DOWN (0x2u << 0) /**< \brief (SDRAMC_LPR) The SDRAM Controller issues a Power-down Command to the SDRAM device after each access, the SDCKE signal is set to low. The SDRAM device leaves the Power-down Mode when accessed and enters it after the access. */
#define   SDRAMC_LPR_LPCB_DEEP_POWER_DOWN (0x3u << 0) /**< \brief (SDRAMC_LPR) The SDRAM Controller issues a Deep Power-down command to the SDRAM device. This mode is unique to low-power SDRAM. */
#define SDRAMC_LPR_PASR_Pos 4
#define SDRAMC_LPR_PASR_Msk (0x7u << SDRAMC_LPR_PASR_Pos) /**< \brief (SDRAMC_LPR) Partial Array Self-refresh (only for low-power SDRAM) */
#define SDRAMC_LPR_PASR(value) ((SDRAMC_LPR_PASR_Msk & ((value) << SDRAMC_LPR_PASR_Pos)))
#define SDRAMC_LPR_TCSR_Pos 8
#define SDRAMC_LPR_TCSR_Msk (0x3u << SDRAMC_LPR_TCSR_Pos) /**< \brief (SDRAMC_LPR) Temperature Compensated Self-Refresh (only for low-power SDRAM) */
#define SDRAMC_LPR_TCSR(value) ((SDRAMC_LPR_TCSR_Msk & ((value) << SDRAMC_LPR_TCSR_Pos)))
#define SDRAMC_LPR_DS_Pos 10
#define SDRAMC_LPR_DS_Msk (0x3u << SDRAMC_LPR_DS_Pos) /**< \brief (SDRAMC_LPR) Drive Strength (only for low-power SDRAM) */
#define SDRAMC_LPR_DS(value) ((SDRAMC_LPR_DS_Msk & ((value) << SDRAMC_LPR_DS_Pos)))
#define SDRAMC_LPR_TIMEOUT_Pos 12
#define SDRAMC_LPR_TIMEOUT_Msk (0x3u << SDRAMC_LPR_TIMEOUT_Pos) /**< \brief (SDRAMC_LPR) Time to define when low-power mode is enable */
#define   SDRAMC_LPR_TIMEOUT_LP_LAST_XFER (0x0u << 12) /**< \brief (SDRAMC_LPR) The SDRAM controller activates the SDRAM low-power mode immediately after the end of the last transfer. */
#define   SDRAMC_LPR_TIMEOUT_LP_LAST_XFER_64 (0x1u << 12) /**< \brief (SDRAMC_LPR) The SDRAM controller activates the SDRAM low-power mode 64 clock cycles after the end of the last transfer. */
#define   SDRAMC_LPR_TIMEOUT_LP_LAST_XFER_128 (0x2u << 12) /**< \brief (SDRAMC_LPR) The SDRAM controller activates the SDRAM low-power mode 128 clock cycles after the end of the last transfer. */
/* -------- SDRAMC_IER : (SDRAMC Offset: 0x14) SDRAMC Interrupt Enable Register -------- */
#define SDRAMC_IER_RES (0x1u << 0) /**< \brief (SDRAMC_IER) Refresh Error Status */
/* -------- SDRAMC_IDR : (SDRAMC Offset: 0x18) SDRAMC Interrupt Disable Register -------- */
#define SDRAMC_IDR_RES (0x1u << 0) /**< \brief (SDRAMC_IDR) Refresh Error Status */
/* -------- SDRAMC_IMR : (SDRAMC Offset: 0x1C) SDRAMC Interrupt Mask Register -------- */
#define SDRAMC_IMR_RES (0x1u << 0) /**< \brief (SDRAMC_IMR) Refresh Error Status */
/* -------- SDRAMC_ISR : (SDRAMC Offset: 0x20) SDRAMC Interrupt Status Register -------- */
#define SDRAMC_ISR_RES (0x1u << 0) /**< \brief (SDRAMC_ISR) Refresh Error Status */
/* -------- SDRAMC_MDR : (SDRAMC Offset: 0x24) SDRAMC Memory Device Register -------- */
#define SDRAMC_MDR_MD_Pos 0
#define SDRAMC_MDR_MD_Msk (0x3u << SDRAMC_MDR_MD_Pos) /**< \brief (SDRAMC_MDR) Memory Device Type */
#define   SDRAMC_MDR_MD_SDRAM (0x0u << 0) /**< \brief (SDRAMC_MDR) SDRAM */
#define   SDRAMC_MDR_MD_LPSDRAM (0x1u << 0) /**< \brief (SDRAMC_MDR) Low-power SDRAM */
/* -------- SDRAMC_CR1 : (SDRAMC Offset: 0x28) SDRAMC Configuration Register 1 -------- */
#define SDRAMC_CR1_NC_Pos 0
#define SDRAMC_CR1_NC_Msk (0x3u << SDRAMC_CR1_NC_Pos) /**< \brief (SDRAMC_CR1) Number of Column Bits */
#define   SDRAMC_CR1_NC_COL8 (0x0u << 0) /**< \brief (SDRAMC_CR1) 8 column bits */
#define   SDRAMC_CR1_NC_COL9 (0x1u << 0) /**< \brief (SDRAMC_CR1) 9 column bits */
#define   SDRAMC_CR1_NC_COL10 (0x2u << 0) /**< \brief (SDRAMC_CR1) 10 column bits */
#define   SDRAMC_CR1_NC_COL11 (0x3u << 0) /**< \brief (SDRAMC_CR1) 11 column bits */
#define SDRAMC_CR1_NR_Pos 2
#define SDRAMC_CR1_NR_Msk (0x3u << SDRAMC_CR1_NR_Pos) /**< \brief (SDRAMC_CR1) Number of Row Bits */
#define   SDRAMC_CR1_NR_ROW11 (0x0u << 2) /**< \brief (SDRAMC_CR1) 11 row bits */
#define   SDRAMC_CR1_NR_ROW12 (0x1u << 2) /**< \brief (SDRAMC_CR1) 12 row bits */
#define   SDRAMC_CR1_NR_ROW13 (0x2u << 2) /**< \brief (SDRAMC_CR1) 13 row bits */
#define SDRAMC_CR1_NB (0x1u << 4) /**< \brief (SDRAMC_CR1) Number of Banks */
#define   SDRAMC_CR1_NB_BANK2 (0x0u << 4) /**< \brief (SDRAMC_CR1) 2 banks */
#define   SDRAMC_CR1_NB_BANK4 (0x1u << 4) /**< \brief (SDRAMC_CR1) 4 banks */
#define SDRAMC_CR1_CAS_Pos 5
#define SDRAMC_CR1_CAS_Msk (0x3u << SDRAMC_CR1_CAS_Pos) /**< \brief (SDRAMC_CR1) CAS Latency */
#define   SDRAMC_CR1_CAS_LATENCY1 (0x1u << 5) /**< \brief (SDRAMC_CR1) 1 cycle CAS latency */
#define   SDRAMC_CR1_CAS_LATENCY2 (0x2u << 5) /**< \brief (SDRAMC_CR1) 2 cycle CAS latency */
#define   SDRAMC_CR1_CAS_LATENCY3 (0x3u << 5) /**< \brief (SDRAMC_CR1) 3 cycle CAS latency */
#define SDRAMC_CR1_DBW (0x1u << 7) /**< \brief (SDRAMC_CR1) Data Bus Width */
#define SDRAMC_CR1_TWR_Pos 8
#define SDRAMC_CR1_TWR_Msk (0xfu << SDRAMC_CR1_TWR_Pos) /**< \brief (SDRAMC_CR1) Write Recovery Delay */
#define SDRAMC_CR1_TWR(value) ((SDRAMC_CR1_TWR_Msk & ((value) << SDRAMC_CR1_TWR_Pos)))
#define SDRAMC_CR1_TRC_TRFC_Pos 12
#define SDRAMC_CR1_TRC_TRFC_Msk (0xfu << SDRAMC_CR1_TRC_TRFC_Pos) /**< \brief (SDRAMC_CR1) Row Cycle Delay and Row Refresh Cycle */
#define SDRAMC_CR1_TRC_TRFC(value) ((SDRAMC_CR1_TRC_TRFC_Msk & ((value) << SDRAMC_CR1_TRC_TRFC_Pos)))
#define SDRAMC_CR1_TRP_Pos 16
#define SDRAMC_CR1_TRP_Msk (0xfu << SDRAMC_CR1_TRP_Pos) /**< \brief (SDRAMC_CR1) Row Precharge Delay */
#define SDRAMC_CR1_TRP(value) ((SDRAMC_CR1_TRP_Msk & ((value) << SDRAMC_CR1_TRP_Pos)))
#define SDRAMC_CR1_TRCD_Pos 20
#define SDRAMC_CR1_TRCD_Msk (0xfu << SDRAMC_CR1_TRCD_Pos) /**< \brief (SDRAMC_CR1) Row to Column Delay */
#define SDRAMC_CR1_TRCD(value) ((SDRAMC_CR1_TRCD_Msk & ((value) << SDRAMC_CR1_TRCD_Pos)))
#define SDRAMC_CR1_TRAS_Pos 24
#define SDRAMC_CR1_TRAS_Msk (0xfu << SDRAMC_CR1_TRAS_Pos) /**< \brief (SDRAMC_CR1) Active to Precharge Delay */
#define SDRAMC_CR1_TRAS(value) ((SDRAMC_CR1_TRAS_Msk & ((value) << SDRAMC_CR1_TRAS_Pos)))
#define SDRAMC_CR1_TXSR_Pos 28
#define SDRAMC_CR1_TXSR_Msk (0xfu << SDRAMC_CR1_TXSR_Pos) /**< \brief (SDRAMC_CR1) Exit Self Refresh to Active Delay */
#define SDRAMC_CR1_TXSR(value) ((SDRAMC_CR1_TXSR_Msk & ((value) << SDRAMC_CR1_TXSR_Pos)))
/* -------- SDRAMC_OCMS : (SDRAMC Offset: 0x2C) SDRAMC OCMS Register 1 -------- */
#define SDRAMC_OCMS_SDR_SE (0x1u << 0) /**< \brief (SDRAMC_OCMS) SDRAM Memory Controller Scrambling Enable */
/*@}*/ /* end of group SAM3XA_SDRAMC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Static Memory Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_SMC Static Memory Controller */
/*@{*/

/** \brief SmcCs_number hardware registers */
typedef struct {
  RwReg         SMC_SETUP;        /**< \brief (SmcCs_number Offset: 0x0) SMC Setup Register */
  RwReg         SMC_PULSE;        /**< \brief (SmcCs_number Offset: 0x4) SMC Pulse Register */
  RwReg         SMC_CYCLE;        /**< \brief (SmcCs_number Offset: 0x8) SMC Cycle Register */
  RwReg         SMC_TIMINGS;      /**< \brief (SmcCs_number Offset: 0xC) SMC Timings Register */
  RwReg         SMC_MODE;         /**< \brief (SmcCs_number Offset: 0x10) SMC Mode Register */
} SmcCs_number;
/** \brief Smc hardware registers */
#define SMCCS_NUMBER_NUMBER 8
typedef struct {
  RwReg         SMC_CFG;          /**< \brief (Smc Offset: 0x000) SMC NFC Configuration Register */
  WoReg         SMC_CTRL;         /**< \brief (Smc Offset: 0x004) SMC NFC Control Register */
  RoReg         SMC_SR;           /**< \brief (Smc Offset: 0x008) SMC NFC Status Register */
  WoReg         SMC_IER;          /**< \brief (Smc Offset: 0x00C) SMC NFC Interrupt Enable Register */
  WoReg         SMC_IDR;          /**< \brief (Smc Offset: 0x010) SMC NFC Interrupt Disable Register */
  RoReg         SMC_IMR;          /**< \brief (Smc Offset: 0x014) SMC NFC Interrupt Mask Register */
  RwReg         SMC_ADDR;         /**< \brief (Smc Offset: 0x018) SMC NFC Address Cycle Zero Register */
  RwReg         SMC_BANK;         /**< \brief (Smc Offset: 0x01C) SMC Bank Address Register */
  WoReg         SMC_ECC_CTRL;     /**< \brief (Smc Offset: 0x020) SMC ECC Control Register */
  RwReg         SMC_ECC_MD;       /**< \brief (Smc Offset: 0x024) SMC ECC Mode Register */
  RoReg         SMC_ECC_SR1;      /**< \brief (Smc Offset: 0x028) SMC ECC Status 1 Register */
  RoReg         SMC_ECC_PR0;      /**< \brief (Smc Offset: 0x02C) SMC ECC Parity 0 Register */
  RoReg         SMC_ECC_PR1;      /**< \brief (Smc Offset: 0x030) SMC ECC parity 1 Register */
  RoReg         SMC_ECC_SR2;      /**< \brief (Smc Offset: 0x034) SMC ECC status 2 Register */
  RoReg         SMC_ECC_PR2;      /**< \brief (Smc Offset: 0x038) SMC ECC parity 2 Register */
  RoReg         SMC_ECC_PR3;      /**< \brief (Smc Offset: 0x03C) SMC ECC parity 3 Register */
  RoReg         SMC_ECC_PR4;      /**< \brief (Smc Offset: 0x040) SMC ECC parity 4 Register */
  RoReg         SMC_ECC_PR5;      /**< \brief (Smc Offset: 0x044) SMC ECC parity 5 Register */
  RoReg         SMC_ECC_PR6;      /**< \brief (Smc Offset: 0x048) SMC ECC parity 6 Register */
  RoReg         SMC_ECC_PR7;      /**< \brief (Smc Offset: 0x04C) SMC ECC parity 7 Register */
  RoReg         SMC_ECC_PR8;      /**< \brief (Smc Offset: 0x050) SMC ECC parity 8 Register */
  RoReg         SMC_ECC_PR9;      /**< \brief (Smc Offset: 0x054) SMC ECC parity 9 Register */
  RoReg         SMC_ECC_PR10;     /**< \brief (Smc Offset: 0x058) SMC ECC parity 10 Register */
  RoReg         SMC_ECC_PR11;     /**< \brief (Smc Offset: 0x05C) SMC ECC parity 11 Register */
  RoReg         SMC_ECC_PR12;     /**< \brief (Smc Offset: 0x060) SMC ECC parity 12 Register */
  RoReg         SMC_ECC_PR13;     /**< \brief (Smc Offset: 0x064) SMC ECC parity 13 Register */
  RoReg         SMC_ECC_PR14;     /**< \brief (Smc Offset: 0x068) SMC ECC parity 14 Register */
  RoReg         SMC_ECC_PR15;     /**< \brief (Smc Offset: 0x06C) SMC ECC parity 15 Register */
  SmcCs_number  SMC_CS_NUMBER[SMCCS_NUMBER_NUMBER]; /**< \brief (Smc Offset: 0x70) CS_number = 0 .. 7 */
  RwReg         SMC_OCMS;         /**< \brief (Smc Offset: 0x110) SMC OCMS Register */
  WoReg         SMC_KEY1;         /**< \brief (Smc Offset: 0x114) SMC OCMS KEY1 Register */
  WoReg         SMC_KEY2;         /**< \brief (Smc Offset: 0x118) SMC OCMS KEY2 Register */
  RoReg         Reserved1[50];
  WoReg         SMC_WPCR;         /**< \brief (Smc Offset: 0x1E4) Write Protection Control Register */
  RoReg         SMC_WPSR;         /**< \brief (Smc Offset: 0x1E8) Write Protection Status Register */
} Smc;
/* -------- SMC_CFG : (SMC Offset: 0x000) SMC NFC Configuration Register -------- */
#define SMC_CFG_PAGESIZE_Pos 0
#define SMC_CFG_PAGESIZE_Msk (0x3u << SMC_CFG_PAGESIZE_Pos) /**< \brief (SMC_CFG)  */
#define   SMC_CFG_PAGESIZE_PS512_16 (0x0u << 0) /**< \brief (SMC_CFG) Main area 512 Bytes + Spare area 16 Bytes = 528 Bytes */
#define   SMC_CFG_PAGESIZE_PS1024_32 (0x1u << 0) /**< \brief (SMC_CFG) Main area 1024 Bytes + Spare area 32 Bytes = 1056 Bytes */
#define   SMC_CFG_PAGESIZE_PS2048_64 (0x2u << 0) /**< \brief (SMC_CFG) Main area 2048 Bytes + Spare area 64 Bytes = 2112 Bytes */
#define   SMC_CFG_PAGESIZE_PS4096_128 (0x3u << 0) /**< \brief (SMC_CFG) Main area 4096 Bytes + Spare area 128 Bytes = 4224 Bytes */
#define SMC_CFG_WSPARE (0x1u << 8) /**< \brief (SMC_CFG) Write Spare Area */
#define SMC_CFG_RSPARE (0x1u << 9) /**< \brief (SMC_CFG) Read Spare Area */
#define SMC_CFG_EDGECTRL (0x1u << 12) /**< \brief (SMC_CFG) Rising/Falling Edge Detection Control */
#define SMC_CFG_RBEDGE (0x1u << 13) /**< \brief (SMC_CFG) Ready/Busy Signal Edge Detection */
#define SMC_CFG_DTOCYC_Pos 16
#define SMC_CFG_DTOCYC_Msk (0xfu << SMC_CFG_DTOCYC_Pos) /**< \brief (SMC_CFG) Data Timeout Cycle Number */
#define SMC_CFG_DTOCYC(value) ((SMC_CFG_DTOCYC_Msk & ((value) << SMC_CFG_DTOCYC_Pos)))
#define SMC_CFG_DTOMUL_Pos 20
#define SMC_CFG_DTOMUL_Msk (0x7u << SMC_CFG_DTOMUL_Pos) /**< \brief (SMC_CFG) Data Timeout Multiplier */
#define   SMC_CFG_DTOMUL_X1 (0x0u << 20) /**< \brief (SMC_CFG) DTOCYC */
#define   SMC_CFG_DTOMUL_X16 (0x1u << 20) /**< \brief (SMC_CFG) DTOCYC x 16 */
#define   SMC_CFG_DTOMUL_X128 (0x2u << 20) /**< \brief (SMC_CFG) DTOCYC x 128 */
#define   SMC_CFG_DTOMUL_X256 (0x3u << 20) /**< \brief (SMC_CFG) DTOCYC x 256 */
#define   SMC_CFG_DTOMUL_X1024 (0x4u << 20) /**< \brief (SMC_CFG) DTOCYC x 1024 */
#define   SMC_CFG_DTOMUL_X4096 (0x5u << 20) /**< \brief (SMC_CFG) DTOCYC x 4096 */
#define   SMC_CFG_DTOMUL_X65536 (0x6u << 20) /**< \brief (SMC_CFG) DTOCYC x 65536 */
#define   SMC_CFG_DTOMUL_X1048576 (0x7u << 20) /**< \brief (SMC_CFG) DTOCYC x 1048576 */
/* -------- SMC_CTRL : (SMC Offset: 0x004) SMC NFC Control Register -------- */
#define SMC_CTRL_NFCEN (0x1u << 0) /**< \brief (SMC_CTRL) NAND Flash Controller Enable */
#define SMC_CTRL_NFCDIS (0x1u << 1) /**< \brief (SMC_CTRL) NAND Flash Controller Disable */
/* -------- SMC_SR : (SMC Offset: 0x008) SMC NFC Status Register -------- */
#define SMC_SR_SMCSTS (0x1u << 0) /**< \brief (SMC_SR) NAND Flash Controller status (this field cannot be reset) */
#define SMC_SR_RB_RISE (0x1u << 4) /**< \brief (SMC_SR) Selected Ready Busy Rising Edge Detected */
#define SMC_SR_RB_FALL (0x1u << 5) /**< \brief (SMC_SR) Selected Ready Busy Falling Edge Detected */
#define SMC_SR_NFCBUSY (0x1u << 8) /**< \brief (SMC_SR) NFC Busy (this field cannot be reset) */
#define SMC_SR_NFCWR (0x1u << 11) /**< \brief (SMC_SR) NFC Write/Read Operation (this field cannot be reset) */
#define SMC_SR_NFCSID_Pos 12
#define SMC_SR_NFCSID_Msk (0x7u << SMC_SR_NFCSID_Pos) /**< \brief (SMC_SR) NFC Chip Select ID (this field cannot be reset) */
#define SMC_SR_XFRDONE (0x1u << 16) /**< \brief (SMC_SR) NFC Data Transfer Terminated */
#define SMC_SR_CMDDONE (0x1u << 17) /**< \brief (SMC_SR) Command Done */
#define SMC_SR_DTOE (0x1u << 20) /**< \brief (SMC_SR) Data Timeout Error */
#define SMC_SR_UNDEF (0x1u << 21) /**< \brief (SMC_SR) Undefined Area Error */
#define SMC_SR_AWB (0x1u << 22) /**< \brief (SMC_SR) Accessing While Busy */
#define SMC_SR_NFCASE (0x1u << 23) /**< \brief (SMC_SR) NFC Access Size Error */
#define SMC_SR_RB_EDGE0 (0x1u << 24) /**< \brief (SMC_SR) Ready/Busy Line 0 Edge Detected */
/* -------- SMC_IER : (SMC Offset: 0x00C) SMC NFC Interrupt Enable Register -------- */
#define SMC_IER_RB_RISE (0x1u << 4) /**< \brief (SMC_IER) Ready Busy Rising Edge Detection Interrupt Enable */
#define SMC_IER_RB_FALL (0x1u << 5) /**< \brief (SMC_IER) Ready Busy Falling Edge Detection Interrupt Enable */
#define SMC_IER_XFRDONE (0x1u << 16) /**< \brief (SMC_IER) Transfer Done Interrupt Enable */
#define SMC_IER_CMDDONE (0x1u << 17) /**< \brief (SMC_IER) Command Done Interrupt Enable */
#define SMC_IER_DTOE (0x1u << 20) /**< \brief (SMC_IER) Data Timeout Error Interrupt Enable */
#define SMC_IER_UNDEF (0x1u << 21) /**< \brief (SMC_IER) Undefined Area Access Interrupt Enable */
#define SMC_IER_AWB (0x1u << 22) /**< \brief (SMC_IER) Accessing While Busy Interrupt Enable */
#define SMC_IER_NFCASE (0x1u << 23) /**< \brief (SMC_IER) NFC Access Size Error Interrupt Enable */
#define SMC_IER_RB_EDGE0 (0x1u << 24) /**< \brief (SMC_IER) Ready/Busy Line 0 Interrupt Enable */
/* -------- SMC_IDR : (SMC Offset: 0x010) SMC NFC Interrupt Disable Register -------- */
#define SMC_IDR_RB_RISE (0x1u << 4) /**< \brief (SMC_IDR) Ready Busy Rising Edge Detection Interrupt Disable */
#define SMC_IDR_RB_FALL (0x1u << 5) /**< \brief (SMC_IDR) Ready Busy Falling Edge Detection Interrupt Disable */
#define SMC_IDR_XFRDONE (0x1u << 16) /**< \brief (SMC_IDR) Transfer Done Interrupt Disable */
#define SMC_IDR_CMDDONE (0x1u << 17) /**< \brief (SMC_IDR) Command Done Interrupt Disable */
#define SMC_IDR_DTOE (0x1u << 20) /**< \brief (SMC_IDR) Data Timeout Error Interrupt Disable */
#define SMC_IDR_UNDEF (0x1u << 21) /**< \brief (SMC_IDR) Undefined Area Access Interrupt Disable */
#define SMC_IDR_AWB (0x1u << 22) /**< \brief (SMC_IDR) Accessing While Busy Interrupt Disable */
#define SMC_IDR_NFCASE (0x1u << 23) /**< \brief (SMC_IDR) NFC Access Size Error Interrupt Disable */
#define SMC_IDR_RB_EDGE0 (0x1u << 24) /**< \brief (SMC_IDR) Ready/Busy Line 0 Interrupt Disable */
/* -------- SMC_IMR : (SMC Offset: 0x014) SMC NFC Interrupt Mask Register -------- */
#define SMC_IMR_RB_RISE (0x1u << 4) /**< \brief (SMC_IMR) Ready Busy Rising Edge Detection Interrupt Mask */
#define SMC_IMR_RB_FALL (0x1u << 5) /**< \brief (SMC_IMR) Ready Busy Falling Edge Detection Interrupt Mask */
#define SMC_IMR_XFRDONE (0x1u << 16) /**< \brief (SMC_IMR) Transfer Done Interrupt Mask */
#define SMC_IMR_CMDDONE (0x1u << 17) /**< \brief (SMC_IMR) Command Done Interrupt Mask */
#define SMC_IMR_DTOE (0x1u << 20) /**< \brief (SMC_IMR) Data Timeout Error Interrupt Mask */
#define SMC_IMR_UNDEF (0x1u << 21) /**< \brief (SMC_IMR) Undefined Area Access Interrupt Mask5 */
#define SMC_IMR_AWB (0x1u << 22) /**< \brief (SMC_IMR) Accessing While Busy Interrupt Mask */
#define SMC_IMR_NFCASE (0x1u << 23) /**< \brief (SMC_IMR) NFC Access Size Error Interrupt Mask */
#define SMC_IMR_RB_EDGE0 (0x1u << 24) /**< \brief (SMC_IMR) Ready/Busy Line 0 Interrupt Mask */
/* -------- SMC_ADDR : (SMC Offset: 0x018) SMC NFC Address Cycle Zero Register -------- */
#define SMC_ADDR_ADDR_CYCLE0_Pos 0
#define SMC_ADDR_ADDR_CYCLE0_Msk (0xffu << SMC_ADDR_ADDR_CYCLE0_Pos) /**< \brief (SMC_ADDR) NAND Flash Array Address cycle 0 */
#define SMC_ADDR_ADDR_CYCLE0(value) ((SMC_ADDR_ADDR_CYCLE0_Msk & ((value) << SMC_ADDR_ADDR_CYCLE0_Pos)))
/* -------- SMC_BANK : (SMC Offset: 0x01C) SMC Bank Address Register -------- */
#define SMC_BANK_BANK_Pos 0
#define SMC_BANK_BANK_Msk (0x7u << SMC_BANK_BANK_Pos) /**< \brief (SMC_BANK) Bank Identifier */
#define SMC_BANK_BANK(value) ((SMC_BANK_BANK_Msk & ((value) << SMC_BANK_BANK_Pos)))
/* -------- SMC_ECC_CTRL : (SMC Offset: 0x020) SMC ECC Control Register -------- */
#define SMC_ECC_CTRL_RST (0x1u << 0) /**< \brief (SMC_ECC_CTRL) Reset ECC */
#define SMC_ECC_CTRL_SWRST (0x1u << 1) /**< \brief (SMC_ECC_CTRL) Software Reset */
/* -------- SMC_ECC_MD : (SMC Offset: 0x024) SMC ECC Mode Register -------- */
#define SMC_ECC_MD_ECC_PAGESIZE_Pos 0
#define SMC_ECC_MD_ECC_PAGESIZE_Msk (0x3u << SMC_ECC_MD_ECC_PAGESIZE_Pos) /**< \brief (SMC_ECC_MD) ECC Page Size */
#define   SMC_ECC_MD_ECC_PAGESIZE_PS512_16 (0x0u << 0) /**< \brief (SMC_ECC_MD) Main area 512 Bytes + Spare area 16 Bytes = 528 Bytes */
#define   SMC_ECC_MD_ECC_PAGESIZE_PS1024_32 (0x1u << 0) /**< \brief (SMC_ECC_MD) Main area 1024 Bytes + Spare area 32 Bytes = 1056 Bytes */
#define   SMC_ECC_MD_ECC_PAGESIZE_PS2048_64 (0x2u << 0) /**< \brief (SMC_ECC_MD) Main area 2048 Bytes + Spare area 64 Bytes = 2112 Bytes */
#define   SMC_ECC_MD_ECC_PAGESIZE_PS4096_128 (0x3u << 0) /**< \brief (SMC_ECC_MD) Main area 4096 Bytes + Spare area 128 Bytes = 4224 Bytes */
#define SMC_ECC_MD_TYPCORREC_Pos 4
#define SMC_ECC_MD_TYPCORREC_Msk (0x3u << SMC_ECC_MD_TYPCORREC_Pos) /**< \brief (SMC_ECC_MD) Type of Correction */
#define   SMC_ECC_MD_TYPCORREC_CPAGE (0x0u << 4) /**< \brief (SMC_ECC_MD) 1 bit correction for a page of 512/1024/2048/4096 Bytes  (for 8 or 16-bit NAND Flash) */
#define   SMC_ECC_MD_TYPCORREC_C256B (0x1u << 4) /**< \brief (SMC_ECC_MD) 1 bit correction for 256 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only) */
#define   SMC_ECC_MD_TYPCORREC_C512B (0x2u << 4) /**< \brief (SMC_ECC_MD) 1 bit correction for 512 Bytes of data for a page of 512/2048/4096 bytes (for 8-bit NAND Flash only) */
/* -------- SMC_ECC_SR1 : (SMC Offset: 0x028) SMC ECC Status 1 Register -------- */
#define SMC_ECC_SR1_RECERR0 (0x1u << 0) /**< \brief (SMC_ECC_SR1) Recoverable Error */
#define SMC_ECC_SR1_ECCERR0_Pos 1
#define SMC_ECC_SR1_ECCERR0_Msk (0x3u << SMC_ECC_SR1_ECCERR0_Pos) /**< \brief (SMC_ECC_SR1) ECC Error */
#define SMC_ECC_SR1_RECERR1 (0x1u << 4) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 256th and the 511th bytes or the 512nd and the 1023rd bytes */
#define SMC_ECC_SR1_ECCERR1 (0x1u << 5) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes */
#define SMC_ECC_SR1_MULERR1 (0x1u << 6) /**< \brief (SMC_ECC_SR1) Multiple Error in the page between the 256th and the 511th bytes or between the 512nd and the 1023rd bytes */
#define SMC_ECC_SR1_RECERR2 (0x1u << 8) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes */
#define SMC_ECC_SR1_ECCERR2 (0x1u << 9) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes */
#define SMC_ECC_SR1_MULERR2 (0x1u << 10) /**< \brief (SMC_ECC_SR1) Multiple Error in the page between the 512nd and the 767th bytes or between the 1024th and the 1535th bytes */
#define SMC_ECC_SR1_RECERR3 (0x1u << 12) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes */
#define SMC_ECC_SR1_ECCERR3 (0x1u << 13) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes */
#define SMC_ECC_SR1_MULERR3 (0x1u << 14) /**< \brief (SMC_ECC_SR1) Multiple Error in the page between the 768th and the 1023rd bytes or between the 1536th and the 2047th bytes */
#define SMC_ECC_SR1_RECERR4 (0x1u << 16) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes */
#define SMC_ECC_SR1_ECCERR4_Pos 17
#define SMC_ECC_SR1_ECCERR4_Msk (0x3u << SMC_ECC_SR1_ECCERR4_Pos) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 1024th and the 1279th bytes or between the 2048th and the 2559th bytes */
#define SMC_ECC_SR1_RECERR5 (0x1u << 20) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes */
#define SMC_ECC_SR1_ECCERR5_Pos 21
#define SMC_ECC_SR1_ECCERR5_Msk (0x3u << SMC_ECC_SR1_ECCERR5_Pos) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 1280th and the 1535th bytes or between the 2560th and the 3071st bytes */
#define SMC_ECC_SR1_RECERR6 (0x1u << 24) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes */
#define SMC_ECC_SR1_ECCERR6_Pos 25
#define SMC_ECC_SR1_ECCERR6_Msk (0x3u << SMC_ECC_SR1_ECCERR6_Pos) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 1536th and the 1791st bytes or between the 3072nd and the 3583rd bytes */
#define SMC_ECC_SR1_RECERR7 (0x1u << 28) /**< \brief (SMC_ECC_SR1) Recoverable Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes */
#define SMC_ECC_SR1_ECCERR7_Pos 29
#define SMC_ECC_SR1_ECCERR7_Msk (0x3u << SMC_ECC_SR1_ECCERR7_Pos) /**< \brief (SMC_ECC_SR1) ECC Error in the page between the 1792nd and the 2047th bytes or between the 3584th and the 4095th bytes */
/* -------- SMC_ECC_PR0 : (SMC Offset: 0x02C) SMC ECC Parity 0 Register -------- */
#define SMC_ECC_PR0_BITADDR_Pos 0
#define SMC_ECC_PR0_BITADDR_Msk (0xfu << SMC_ECC_PR0_BITADDR_Pos) /**< \brief (SMC_ECC_PR0) Bit Address */
#define SMC_ECC_PR0_WORDADDR_Pos 4
#define SMC_ECC_PR0_WORDADDR_Msk (0xfffu << SMC_ECC_PR0_WORDADDR_Pos) /**< \brief (SMC_ECC_PR0) Word Address */
#define SMC_ECC_PR0_BITADDR_W9BIT_Pos 0
#define SMC_ECC_PR0_BITADDR_W9BIT_Msk (0x7u << SMC_ECC_PR0_BITADDR_W9BIT_Pos) /**< \brief (SMC_ECC_PR0) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR0_WORDADDR_W9BIT_Pos 3
#define SMC_ECC_PR0_WORDADDR_W9BIT_Msk (0x1ffu << SMC_ECC_PR0_WORDADDR_W9BIT_Pos) /**< \brief (SMC_ECC_PR0) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR0_NPARITY_Pos 12
#define SMC_ECC_PR0_NPARITY_Msk (0xfffu << SMC_ECC_PR0_NPARITY_Pos) /**< \brief (SMC_ECC_PR0) Parity N */
#define SMC_ECC_PR0_BITADDR_W8BIT_Pos 0
#define SMC_ECC_PR0_BITADDR_W8BIT_Msk (0x7u << SMC_ECC_PR0_BITADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR0) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR0_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR0_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR0_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR0) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR0_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR0_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR0_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR0) Parity N */
/* -------- SMC_ECC_PR1 : (SMC Offset: 0x030) SMC ECC parity 1 Register -------- */
#define SMC_ECC_PR1_NPARITY_Pos 0
#define SMC_ECC_PR1_NPARITY_Msk (0xffffu << SMC_ECC_PR1_NPARITY_Pos) /**< \brief (SMC_ECC_PR1) Parity N */
#define SMC_ECC_PR1_BITADDR_Pos 0
#define SMC_ECC_PR1_BITADDR_Msk (0x7u << SMC_ECC_PR1_BITADDR_Pos) /**< \brief (SMC_ECC_PR1) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR1_WORDADDR_Pos 3
#define SMC_ECC_PR1_WORDADDR_Msk (0x1ffu << SMC_ECC_PR1_WORDADDR_Pos) /**< \brief (SMC_ECC_PR1) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR1_NPARITY_W9BIT_Pos 12
#define SMC_ECC_PR1_NPARITY_W9BIT_Msk (0xfffu << SMC_ECC_PR1_NPARITY_W9BIT_Pos) /**< \brief (SMC_ECC_PR1) Parity N */
#define SMC_ECC_PR1_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR1_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR1_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR1) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR1_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR1_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR1_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR1) Parity N */
/* -------- SMC_ECC_SR2 : (SMC Offset: 0x034) SMC ECC status 2 Register -------- */
#define SMC_ECC_SR2_RECERR8 (0x1u << 0) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 2048th and the 2303rd bytes */
#define SMC_ECC_SR2_ECCERR8_Pos 1
#define SMC_ECC_SR2_ECCERR8_Msk (0x3u << SMC_ECC_SR2_ECCERR8_Pos) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 2048th and the 2303rd bytes */
#define SMC_ECC_SR2_RECERR9 (0x1u << 4) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 2304th and the 2559th bytes */
#define SMC_ECC_SR2_ECCERR9 (0x1u << 5) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 2304th and the 2559th bytes */
#define SMC_ECC_SR2_MULERR9 (0x1u << 6) /**< \brief (SMC_ECC_SR2) Multiple Error in the page between the 2304th and the 2559th bytes */
#define SMC_ECC_SR2_RECERR10 (0x1u << 8) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 2560th and the 2815th bytes */
#define SMC_ECC_SR2_ECCERR10 (0x1u << 9) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 2560th and the 2815th bytes */
#define SMC_ECC_SR2_MULERR10 (0x1u << 10) /**< \brief (SMC_ECC_SR2) Multiple Error in the page between the 2560th and the 2815th bytes */
#define SMC_ECC_SR2_RECERR11 (0x1u << 12) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 2816th and the 3071st bytes */
#define SMC_ECC_SR2_ECCERR11 (0x1u << 13) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 2816th and the 3071st bytes */
#define SMC_ECC_SR2_MULERR11 (0x1u << 14) /**< \brief (SMC_ECC_SR2) Multiple Error in the page between the 2816th and the 3071st bytes */
#define SMC_ECC_SR2_RECERR12 (0x1u << 16) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 3072nd and the 3327th bytes */
#define SMC_ECC_SR2_ECCERR12_Pos 17
#define SMC_ECC_SR2_ECCERR12_Msk (0x3u << SMC_ECC_SR2_ECCERR12_Pos) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 3072nd and the 3327th bytes */
#define SMC_ECC_SR2_RECERR13 (0x1u << 20) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 3328th and the 3583rd bytes */
#define SMC_ECC_SR2_ECCERR13_Pos 21
#define SMC_ECC_SR2_ECCERR13_Msk (0x3u << SMC_ECC_SR2_ECCERR13_Pos) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 3328th and the 3583rd bytes */
#define SMC_ECC_SR2_RECERR14 (0x1u << 24) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 3584th and the 3839th bytes */
#define SMC_ECC_SR2_ECCERR14_Pos 25
#define SMC_ECC_SR2_ECCERR14_Msk (0x3u << SMC_ECC_SR2_ECCERR14_Pos) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 3584th and the 3839th bytes */
#define SMC_ECC_SR2_RECERR15 (0x1u << 28) /**< \brief (SMC_ECC_SR2) Recoverable Error in the page between the 3840th and the 4095th bytes */
#define SMC_ECC_SR2_ECCERR15_Pos 29
#define SMC_ECC_SR2_ECCERR15_Msk (0x3u << SMC_ECC_SR2_ECCERR15_Pos) /**< \brief (SMC_ECC_SR2) ECC Error in the page between the 3840th and the 4095th bytes */
/* -------- SMC_ECC_PR2 : (SMC Offset: 0x038) SMC ECC parity 2 Register -------- */
#define SMC_ECC_PR2_BITADDR_Pos 0
#define SMC_ECC_PR2_BITADDR_Msk (0x7u << SMC_ECC_PR2_BITADDR_Pos) /**< \brief (SMC_ECC_PR2) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR2_WORDADDR_Pos 3
#define SMC_ECC_PR2_WORDADDR_Msk (0x1ffu << SMC_ECC_PR2_WORDADDR_Pos) /**< \brief (SMC_ECC_PR2) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR2_NPARITY_Pos 12
#define SMC_ECC_PR2_NPARITY_Msk (0xfffu << SMC_ECC_PR2_NPARITY_Pos) /**< \brief (SMC_ECC_PR2) Parity N */
#define SMC_ECC_PR2_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR2_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR2_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR2) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR2_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR2_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR2_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR2) Parity N */
/* -------- SMC_ECC_PR3 : (SMC Offset: 0x03C) SMC ECC parity 3 Register -------- */
#define SMC_ECC_PR3_BITADDR_Pos 0
#define SMC_ECC_PR3_BITADDR_Msk (0x7u << SMC_ECC_PR3_BITADDR_Pos) /**< \brief (SMC_ECC_PR3) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR3_WORDADDR_Pos 3
#define SMC_ECC_PR3_WORDADDR_Msk (0x1ffu << SMC_ECC_PR3_WORDADDR_Pos) /**< \brief (SMC_ECC_PR3) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR3_NPARITY_Pos 12
#define SMC_ECC_PR3_NPARITY_Msk (0xfffu << SMC_ECC_PR3_NPARITY_Pos) /**< \brief (SMC_ECC_PR3) Parity N */
#define SMC_ECC_PR3_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR3_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR3_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR3) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR3_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR3_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR3_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR3) Parity N */
/* -------- SMC_ECC_PR4 : (SMC Offset: 0x040) SMC ECC parity 4 Register -------- */
#define SMC_ECC_PR4_BITADDR_Pos 0
#define SMC_ECC_PR4_BITADDR_Msk (0x7u << SMC_ECC_PR4_BITADDR_Pos) /**< \brief (SMC_ECC_PR4) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR4_WORDADDR_Pos 3
#define SMC_ECC_PR4_WORDADDR_Msk (0x1ffu << SMC_ECC_PR4_WORDADDR_Pos) /**< \brief (SMC_ECC_PR4) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR4_NPARITY_Pos 12
#define SMC_ECC_PR4_NPARITY_Msk (0xfffu << SMC_ECC_PR4_NPARITY_Pos) /**< \brief (SMC_ECC_PR4) Parity N */
#define SMC_ECC_PR4_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR4_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR4_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR4) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR4_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR4_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR4_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR4) Parity N */
/* -------- SMC_ECC_PR5 : (SMC Offset: 0x044) SMC ECC parity 5 Register -------- */
#define SMC_ECC_PR5_BITADDR_Pos 0
#define SMC_ECC_PR5_BITADDR_Msk (0x7u << SMC_ECC_PR5_BITADDR_Pos) /**< \brief (SMC_ECC_PR5) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR5_WORDADDR_Pos 3
#define SMC_ECC_PR5_WORDADDR_Msk (0x1ffu << SMC_ECC_PR5_WORDADDR_Pos) /**< \brief (SMC_ECC_PR5) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR5_NPARITY_Pos 12
#define SMC_ECC_PR5_NPARITY_Msk (0xfffu << SMC_ECC_PR5_NPARITY_Pos) /**< \brief (SMC_ECC_PR5) Parity N */
#define SMC_ECC_PR5_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR5_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR5_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR5) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR5_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR5_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR5_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR5) Parity N */
/* -------- SMC_ECC_PR6 : (SMC Offset: 0x048) SMC ECC parity 6 Register -------- */
#define SMC_ECC_PR6_BITADDR_Pos 0
#define SMC_ECC_PR6_BITADDR_Msk (0x7u << SMC_ECC_PR6_BITADDR_Pos) /**< \brief (SMC_ECC_PR6) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR6_WORDADDR_Pos 3
#define SMC_ECC_PR6_WORDADDR_Msk (0x1ffu << SMC_ECC_PR6_WORDADDR_Pos) /**< \brief (SMC_ECC_PR6) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR6_NPARITY_Pos 12
#define SMC_ECC_PR6_NPARITY_Msk (0xfffu << SMC_ECC_PR6_NPARITY_Pos) /**< \brief (SMC_ECC_PR6) Parity N */
#define SMC_ECC_PR6_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR6_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR6_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR6) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR6_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR6_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR6_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR6) Parity N */
/* -------- SMC_ECC_PR7 : (SMC Offset: 0x04C) SMC ECC parity 7 Register -------- */
#define SMC_ECC_PR7_BITADDR_Pos 0
#define SMC_ECC_PR7_BITADDR_Msk (0x7u << SMC_ECC_PR7_BITADDR_Pos) /**< \brief (SMC_ECC_PR7) Corrupted Bit Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR7_WORDADDR_Pos 3
#define SMC_ECC_PR7_WORDADDR_Msk (0x1ffu << SMC_ECC_PR7_WORDADDR_Pos) /**< \brief (SMC_ECC_PR7) Corrupted Word Address in the Page between (i x 512) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR7_NPARITY_Pos 12
#define SMC_ECC_PR7_NPARITY_Msk (0xfffu << SMC_ECC_PR7_NPARITY_Pos) /**< \brief (SMC_ECC_PR7) Parity N */
#define SMC_ECC_PR7_WORDADDR_W8BIT_Pos 3
#define SMC_ECC_PR7_WORDADDR_W8BIT_Msk (0xffu << SMC_ECC_PR7_WORDADDR_W8BIT_Pos) /**< \brief (SMC_ECC_PR7) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR7_NPARITY_W8BIT_Pos 12
#define SMC_ECC_PR7_NPARITY_W8BIT_Msk (0x7ffu << SMC_ECC_PR7_NPARITY_W8BIT_Pos) /**< \brief (SMC_ECC_PR7) Parity N */
/* -------- SMC_ECC_PR8 : (SMC Offset: 0x050) SMC ECC parity 8 Register -------- */
#define SMC_ECC_PR8_BITADDR_Pos 0
#define SMC_ECC_PR8_BITADDR_Msk (0x7u << SMC_ECC_PR8_BITADDR_Pos) /**< \brief (SMC_ECC_PR8) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR8_WORDADDR_Pos 3
#define SMC_ECC_PR8_WORDADDR_Msk (0xffu << SMC_ECC_PR8_WORDADDR_Pos) /**< \brief (SMC_ECC_PR8) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR8_NPARITY_Pos 12
#define SMC_ECC_PR8_NPARITY_Msk (0x7ffu << SMC_ECC_PR8_NPARITY_Pos) /**< \brief (SMC_ECC_PR8) Parity N */
/* -------- SMC_ECC_PR9 : (SMC Offset: 0x054) SMC ECC parity 9 Register -------- */
#define SMC_ECC_PR9_BITADDR_Pos 0
#define SMC_ECC_PR9_BITADDR_Msk (0x7u << SMC_ECC_PR9_BITADDR_Pos) /**< \brief (SMC_ECC_PR9) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR9_WORDADDR_Pos 3
#define SMC_ECC_PR9_WORDADDR_Msk (0xffu << SMC_ECC_PR9_WORDADDR_Pos) /**< \brief (SMC_ECC_PR9) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR9_NPARITY_Pos 12
#define SMC_ECC_PR9_NPARITY_Msk (0x7ffu << SMC_ECC_PR9_NPARITY_Pos) /**< \brief (SMC_ECC_PR9) Parity N */
/* -------- SMC_ECC_PR10 : (SMC Offset: 0x058) SMC ECC parity 10 Register -------- */
#define SMC_ECC_PR10_BITADDR_Pos 0
#define SMC_ECC_PR10_BITADDR_Msk (0x7u << SMC_ECC_PR10_BITADDR_Pos) /**< \brief (SMC_ECC_PR10) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR10_WORDADDR_Pos 3
#define SMC_ECC_PR10_WORDADDR_Msk (0xffu << SMC_ECC_PR10_WORDADDR_Pos) /**< \brief (SMC_ECC_PR10) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR10_NPARITY_Pos 12
#define SMC_ECC_PR10_NPARITY_Msk (0x7ffu << SMC_ECC_PR10_NPARITY_Pos) /**< \brief (SMC_ECC_PR10) Parity N */
/* -------- SMC_ECC_PR11 : (SMC Offset: 0x05C) SMC ECC parity 11 Register -------- */
#define SMC_ECC_PR11_BITADDR_Pos 0
#define SMC_ECC_PR11_BITADDR_Msk (0x7u << SMC_ECC_PR11_BITADDR_Pos) /**< \brief (SMC_ECC_PR11) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR11_WORDADDR_Pos 3
#define SMC_ECC_PR11_WORDADDR_Msk (0xffu << SMC_ECC_PR11_WORDADDR_Pos) /**< \brief (SMC_ECC_PR11) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR11_NPARITY_Pos 12
#define SMC_ECC_PR11_NPARITY_Msk (0x7ffu << SMC_ECC_PR11_NPARITY_Pos) /**< \brief (SMC_ECC_PR11) Parity N */
/* -------- SMC_ECC_PR12 : (SMC Offset: 0x060) SMC ECC parity 12 Register -------- */
#define SMC_ECC_PR12_BITADDR_Pos 0
#define SMC_ECC_PR12_BITADDR_Msk (0x7u << SMC_ECC_PR12_BITADDR_Pos) /**< \brief (SMC_ECC_PR12) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR12_WORDADDR_Pos 3
#define SMC_ECC_PR12_WORDADDR_Msk (0xffu << SMC_ECC_PR12_WORDADDR_Pos) /**< \brief (SMC_ECC_PR12) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR12_NPARITY_Pos 12
#define SMC_ECC_PR12_NPARITY_Msk (0x7ffu << SMC_ECC_PR12_NPARITY_Pos) /**< \brief (SMC_ECC_PR12) Parity N */
/* -------- SMC_ECC_PR13 : (SMC Offset: 0x064) SMC ECC parity 13 Register -------- */
#define SMC_ECC_PR13_BITADDR_Pos 0
#define SMC_ECC_PR13_BITADDR_Msk (0x7u << SMC_ECC_PR13_BITADDR_Pos) /**< \brief (SMC_ECC_PR13) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR13_WORDADDR_Pos 3
#define SMC_ECC_PR13_WORDADDR_Msk (0xffu << SMC_ECC_PR13_WORDADDR_Pos) /**< \brief (SMC_ECC_PR13) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR13_NPARITY_Pos 12
#define SMC_ECC_PR13_NPARITY_Msk (0x7ffu << SMC_ECC_PR13_NPARITY_Pos) /**< \brief (SMC_ECC_PR13) Parity N */
/* -------- SMC_ECC_PR14 : (SMC Offset: 0x068) SMC ECC parity 14 Register -------- */
#define SMC_ECC_PR14_BITADDR_Pos 0
#define SMC_ECC_PR14_BITADDR_Msk (0x7u << SMC_ECC_PR14_BITADDR_Pos) /**< \brief (SMC_ECC_PR14) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR14_WORDADDR_Pos 3
#define SMC_ECC_PR14_WORDADDR_Msk (0xffu << SMC_ECC_PR14_WORDADDR_Pos) /**< \brief (SMC_ECC_PR14) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR14_NPARITY_Pos 12
#define SMC_ECC_PR14_NPARITY_Msk (0x7ffu << SMC_ECC_PR14_NPARITY_Pos) /**< \brief (SMC_ECC_PR14) Parity N */
/* -------- SMC_ECC_PR15 : (SMC Offset: 0x06C) SMC ECC parity 15 Register -------- */
#define SMC_ECC_PR15_BITADDR_Pos 0
#define SMC_ECC_PR15_BITADDR_Msk (0x7u << SMC_ECC_PR15_BITADDR_Pos) /**< \brief (SMC_ECC_PR15) Corrupted Bit Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR15_WORDADDR_Pos 3
#define SMC_ECC_PR15_WORDADDR_Msk (0xffu << SMC_ECC_PR15_WORDADDR_Pos) /**< \brief (SMC_ECC_PR15) Corrupted Word Address in the Page between (i x 256) and ((i + 1) x 512) - 1) Bytes */
#define SMC_ECC_PR15_NPARITY_Pos 12
#define SMC_ECC_PR15_NPARITY_Msk (0x7ffu << SMC_ECC_PR15_NPARITY_Pos) /**< \brief (SMC_ECC_PR15) Parity N */
/* -------- SMC_SETUP : (SMC Offset: N/A) SMC Setup Register -------- */
#define SMC_SETUP_NWE_SETUP_Pos 0
#define SMC_SETUP_NWE_SETUP_Msk (0x3fu << SMC_SETUP_NWE_SETUP_Pos) /**< \brief (SMC_SETUP) NWE Setup Length */
#define SMC_SETUP_NWE_SETUP(value) ((SMC_SETUP_NWE_SETUP_Msk & ((value) << SMC_SETUP_NWE_SETUP_Pos)))
#define SMC_SETUP_NCS_WR_SETUP_Pos 8
#define SMC_SETUP_NCS_WR_SETUP_Msk (0x3fu << SMC_SETUP_NCS_WR_SETUP_Pos) /**< \brief (SMC_SETUP) NCS Setup Length in Write Access */
#define SMC_SETUP_NCS_WR_SETUP(value) ((SMC_SETUP_NCS_WR_SETUP_Msk & ((value) << SMC_SETUP_NCS_WR_SETUP_Pos)))
#define SMC_SETUP_NRD_SETUP_Pos 16
#define SMC_SETUP_NRD_SETUP_Msk (0x3fu << SMC_SETUP_NRD_SETUP_Pos) /**< \brief (SMC_SETUP) NRD Setup Length */
#define SMC_SETUP_NRD_SETUP(value) ((SMC_SETUP_NRD_SETUP_Msk & ((value) << SMC_SETUP_NRD_SETUP_Pos)))
#define SMC_SETUP_NCS_RD_SETUP_Pos 24
#define SMC_SETUP_NCS_RD_SETUP_Msk (0x3fu << SMC_SETUP_NCS_RD_SETUP_Pos) /**< \brief (SMC_SETUP) NCS Setup Length in Read Access */
#define SMC_SETUP_NCS_RD_SETUP(value) ((SMC_SETUP_NCS_RD_SETUP_Msk & ((value) << SMC_SETUP_NCS_RD_SETUP_Pos)))
/* -------- SMC_PULSE : (SMC Offset: N/A) SMC Pulse Register -------- */
#define SMC_PULSE_NWE_PULSE_Pos 0
#define SMC_PULSE_NWE_PULSE_Msk (0x3fu << SMC_PULSE_NWE_PULSE_Pos) /**< \brief (SMC_PULSE) NWE Pulse Length */
#define SMC_PULSE_NWE_PULSE(value) ((SMC_PULSE_NWE_PULSE_Msk & ((value) << SMC_PULSE_NWE_PULSE_Pos)))
#define SMC_PULSE_NCS_WR_PULSE_Pos 8
#define SMC_PULSE_NCS_WR_PULSE_Msk (0x3fu << SMC_PULSE_NCS_WR_PULSE_Pos) /**< \brief (SMC_PULSE) NCS Pulse Length in WRITE Access */
#define SMC_PULSE_NCS_WR_PULSE(value) ((SMC_PULSE_NCS_WR_PULSE_Msk & ((value) << SMC_PULSE_NCS_WR_PULSE_Pos)))
#define SMC_PULSE_NRD_PULSE_Pos 16
#define SMC_PULSE_NRD_PULSE_Msk (0x3fu << SMC_PULSE_NRD_PULSE_Pos) /**< \brief (SMC_PULSE) NRD Pulse Length */
#define SMC_PULSE_NRD_PULSE(value) ((SMC_PULSE_NRD_PULSE_Msk & ((value) << SMC_PULSE_NRD_PULSE_Pos)))
#define SMC_PULSE_NCS_RD_PULSE_Pos 24
#define SMC_PULSE_NCS_RD_PULSE_Msk (0x3fu << SMC_PULSE_NCS_RD_PULSE_Pos) /**< \brief (SMC_PULSE) NCS Pulse Length in READ Access */
#define SMC_PULSE_NCS_RD_PULSE(value) ((SMC_PULSE_NCS_RD_PULSE_Msk & ((value) << SMC_PULSE_NCS_RD_PULSE_Pos)))
/* -------- SMC_CYCLE : (SMC Offset: N/A) SMC Cycle Register -------- */
#define SMC_CYCLE_NWE_CYCLE_Pos 0
#define SMC_CYCLE_NWE_CYCLE_Msk (0x1ffu << SMC_CYCLE_NWE_CYCLE_Pos) /**< \brief (SMC_CYCLE) Total Write Cycle Length */
#define SMC_CYCLE_NWE_CYCLE(value) ((SMC_CYCLE_NWE_CYCLE_Msk & ((value) << SMC_CYCLE_NWE_CYCLE_Pos)))
#define SMC_CYCLE_NRD_CYCLE_Pos 16
#define SMC_CYCLE_NRD_CYCLE_Msk (0x1ffu << SMC_CYCLE_NRD_CYCLE_Pos) /**< \brief (SMC_CYCLE) Total Read Cycle Length */
#define SMC_CYCLE_NRD_CYCLE(value) ((SMC_CYCLE_NRD_CYCLE_Msk & ((value) << SMC_CYCLE_NRD_CYCLE_Pos)))
/* -------- SMC_TIMINGS : (SMC Offset: N/A) SMC Timings Register -------- */
#define SMC_TIMINGS_TCLR_Pos 0
#define SMC_TIMINGS_TCLR_Msk (0xfu << SMC_TIMINGS_TCLR_Pos) /**< \brief (SMC_TIMINGS) CLE to REN Low Delay */
#define SMC_TIMINGS_TCLR(value) ((SMC_TIMINGS_TCLR_Msk & ((value) << SMC_TIMINGS_TCLR_Pos)))
#define SMC_TIMINGS_TADL_Pos 4
#define SMC_TIMINGS_TADL_Msk (0xfu << SMC_TIMINGS_TADL_Pos) /**< \brief (SMC_TIMINGS) ALE to Data Start */
#define SMC_TIMINGS_TADL(value) ((SMC_TIMINGS_TADL_Msk & ((value) << SMC_TIMINGS_TADL_Pos)))
#define SMC_TIMINGS_TAR_Pos 8
#define SMC_TIMINGS_TAR_Msk (0xfu << SMC_TIMINGS_TAR_Pos) /**< \brief (SMC_TIMINGS) ALE to REN Low Delay */
#define SMC_TIMINGS_TAR(value) ((SMC_TIMINGS_TAR_Msk & ((value) << SMC_TIMINGS_TAR_Pos)))
#define SMC_TIMINGS_OCMS (0x1u << 12) /**< \brief (SMC_TIMINGS) Off Chip Memory Scrambling Enable */
#define SMC_TIMINGS_TRR_Pos 16
#define SMC_TIMINGS_TRR_Msk (0xfu << SMC_TIMINGS_TRR_Pos) /**< \brief (SMC_TIMINGS) Ready to REN Low Delay */
#define SMC_TIMINGS_TRR(value) ((SMC_TIMINGS_TRR_Msk & ((value) << SMC_TIMINGS_TRR_Pos)))
#define SMC_TIMINGS_TWB_Pos 24
#define SMC_TIMINGS_TWB_Msk (0xfu << SMC_TIMINGS_TWB_Pos) /**< \brief (SMC_TIMINGS) WEN High to REN to Busy */
#define SMC_TIMINGS_TWB(value) ((SMC_TIMINGS_TWB_Msk & ((value) << SMC_TIMINGS_TWB_Pos)))
#define SMC_TIMINGS_RBNSEL_Pos 28
#define SMC_TIMINGS_RBNSEL_Msk (0x7u << SMC_TIMINGS_RBNSEL_Pos) /**< \brief (SMC_TIMINGS) Ready/Busy Line Selection */
#define SMC_TIMINGS_RBNSEL(value) ((SMC_TIMINGS_RBNSEL_Msk & ((value) << SMC_TIMINGS_RBNSEL_Pos)))
#define SMC_TIMINGS_NFSEL (0x1u << 31) /**< \brief (SMC_TIMINGS) NAND Flash Selection */
/* -------- SMC_MODE : (SMC Offset: N/A) SMC Mode Register -------- */
#define SMC_MODE_READ_MODE (0x1u << 0) /**< \brief (SMC_MODE)  */
#define   SMC_MODE_READ_MODE_NCS_CTRL (0x0u << 0) /**< \brief (SMC_MODE) The Read operation is controlled by the NCS signal. */
#define   SMC_MODE_READ_MODE_NRD_CTRL (0x1u << 0) /**< \brief (SMC_MODE) The Read operation is controlled by the NRD signal. */
#define SMC_MODE_WRITE_MODE (0x1u << 1) /**< \brief (SMC_MODE)  */
#define   SMC_MODE_WRITE_MODE_NCS_CTRL (0x0u << 1) /**< \brief (SMC_MODE) The Write operation is controller by the NCS signal. */
#define   SMC_MODE_WRITE_MODE_NWE_CTRL (0x1u << 1) /**< \brief (SMC_MODE) The Write operation is controlled by the NWE signal. */
#define SMC_MODE_EXNW_MODE_Pos 4
#define SMC_MODE_EXNW_MODE_Msk (0x3u << SMC_MODE_EXNW_MODE_Pos) /**< \brief (SMC_MODE) NWAIT Mode */
#define   SMC_MODE_EXNW_MODE_DISABLED (0x0u << 4) /**< \brief (SMC_MODE) Disabled */
#define   SMC_MODE_EXNW_MODE_FROZEN (0x2u << 4) /**< \brief (SMC_MODE) Frozen Mode */
#define   SMC_MODE_EXNW_MODE_READY (0x3u << 4) /**< \brief (SMC_MODE) Ready Mode */
#define SMC_MODE_BAT (0x1u << 8) /**< \brief (SMC_MODE) Byte Access Type */
#define SMC_MODE_DBW (0x1u << 12) /**< \brief (SMC_MODE) Data Bus Width */
#define   SMC_MODE_DBW_BIT_8 (0x0u << 12) /**< \brief (SMC_MODE) 8-bit bus */
#define   SMC_MODE_DBW_BIT_16 (0x1u << 12) /**< \brief (SMC_MODE) 16-bit bus */
#define SMC_MODE_TDF_CYCLES_Pos 16
#define SMC_MODE_TDF_CYCLES_Msk (0xfu << SMC_MODE_TDF_CYCLES_Pos) /**< \brief (SMC_MODE) Data Float Time */
#define SMC_MODE_TDF_CYCLES(value) ((SMC_MODE_TDF_CYCLES_Msk & ((value) << SMC_MODE_TDF_CYCLES_Pos)))
#define SMC_MODE_TDF_MODE (0x1u << 20) /**< \brief (SMC_MODE) TDF Optimization */
/* -------- SMC_OCMS : (SMC Offset: 0x110) SMC OCMS Register -------- */
#define SMC_OCMS_SMSE (0x1u << 0) /**< \brief (SMC_OCMS) Static Memory Controller Scrambling Enable */
#define SMC_OCMS_SRSE (0x1u << 1) /**< \brief (SMC_OCMS) SRAM Scrambling Enable */
/* -------- SMC_KEY1 : (SMC Offset: 0x114) SMC OCMS KEY1 Register -------- */
#define SMC_KEY1_KEY1_Pos 0
#define SMC_KEY1_KEY1_Msk (0xffffffffu << SMC_KEY1_KEY1_Pos) /**< \brief (SMC_KEY1) Off Chip Memory Scrambling (OCMS) Key Part 1 */
#define SMC_KEY1_KEY1(value) ((SMC_KEY1_KEY1_Msk & ((value) << SMC_KEY1_KEY1_Pos)))
/* -------- SMC_KEY2 : (SMC Offset: 0x118) SMC OCMS KEY2 Register -------- */
#define SMC_KEY2_KEY2_Pos 0
#define SMC_KEY2_KEY2_Msk (0xffffffffu << SMC_KEY2_KEY2_Pos) /**< \brief (SMC_KEY2) Off Chip Memory Scrambling (OCMS) Key Part 2 */
#define SMC_KEY2_KEY2(value) ((SMC_KEY2_KEY2_Msk & ((value) << SMC_KEY2_KEY2_Pos)))
/* -------- SMC_WPCR : (SMC Offset: 0x1E4) Write Protection Control Register -------- */
#define SMC_WPCR_WP_EN (0x1u << 0) /**< \brief (SMC_WPCR) Write Protection Enable */
#define SMC_WPCR_WP_KEY_Pos 8
#define SMC_WPCR_WP_KEY_Msk (0xffffffu << SMC_WPCR_WP_KEY_Pos) /**< \brief (SMC_WPCR) Write Protection KEY password */
#define SMC_WPCR_WP_KEY(value) ((SMC_WPCR_WP_KEY_Msk & ((value) << SMC_WPCR_WP_KEY_Pos)))
/* -------- SMC_WPSR : (SMC Offset: 0x1E8) Write Protection Status Register -------- */
#define SMC_WPSR_WP_VS_Pos 0
#define SMC_WPSR_WP_VS_Msk (0xfu << SMC_WPSR_WP_VS_Pos) /**< \brief (SMC_WPSR) Write Protection Violation Status */
#define SMC_WPSR_WP_VSRC_Pos 8
#define SMC_WPSR_WP_VSRC_Msk (0xffffu << SMC_WPSR_WP_VSRC_Pos) /**< \brief (SMC_WPSR) Write Protection Violation Source */
/*@}*/ /* end of group SAM3XA_SMC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Serial Peripheral Interface */
/* ============================================================================= */
/** \addtogroup SAM3XA_SPI Serial Peripheral Interface */
/*@{*/

/** \brief Spi hardware registers */
typedef struct {
  WoReg SPI_CR;        /**< \brief (Spi Offset: 0x00) Control Register */
  RwReg SPI_MR;        /**< \brief (Spi Offset: 0x04) Mode Register */
  RoReg SPI_RDR;       /**< \brief (Spi Offset: 0x08) Receive Data Register */
  WoReg SPI_TDR;       /**< \brief (Spi Offset: 0x0C) Transmit Data Register */
  RoReg SPI_SR;        /**< \brief (Spi Offset: 0x10) Status Register */
  WoReg SPI_IER;       /**< \brief (Spi Offset: 0x14) Interrupt Enable Register */
  WoReg SPI_IDR;       /**< \brief (Spi Offset: 0x18) Interrupt Disable Register */
  RoReg SPI_IMR;       /**< \brief (Spi Offset: 0x1C) Interrupt Mask Register */
  RoReg Reserved1[4];
  RwReg SPI_CSR[4];    /**< \brief (Spi Offset: 0x30) Chip Select Register */
  RoReg Reserved2[41];
  RwReg SPI_WPMR;      /**< \brief (Spi Offset: 0xE4) Write Protection Control Register */
  RoReg SPI_WPSR;      /**< \brief (Spi Offset: 0xE8) Write Protection Status Register */
} Spi;
/* -------- SPI_CR : (SPI Offset: 0x00) Control Register -------- */
#define SPI_CR_SPIEN (0x1u << 0) /**< \brief (SPI_CR) SPI Enable */
#define SPI_CR_SPIDIS (0x1u << 1) /**< \brief (SPI_CR) SPI Disable */
#define SPI_CR_SWRST (0x1u << 7) /**< \brief (SPI_CR) SPI Software Reset */
#define SPI_CR_LASTXFER (0x1u << 24) /**< \brief (SPI_CR) Last Transfer */
/* -------- SPI_MR : (SPI Offset: 0x04) Mode Register -------- */
#define SPI_MR_MSTR (0x1u << 0) /**< \brief (SPI_MR) Master/Slave Mode */
#define SPI_MR_PS (0x1u << 1) /**< \brief (SPI_MR) Peripheral Select */
#define SPI_MR_PCSDEC (0x1u << 2) /**< \brief (SPI_MR) Chip Select Decode */
#define SPI_MR_MODFDIS (0x1u << 4) /**< \brief (SPI_MR) Mode Fault Detection */
#define SPI_MR_WDRBT (0x1u << 5) /**< \brief (SPI_MR) Wait Data Read Before Transfer */
#define SPI_MR_LLB (0x1u << 7) /**< \brief (SPI_MR) Local Loopback Enable */
#define SPI_MR_PCS_Pos 16
#define SPI_MR_PCS_Msk (0xfu << SPI_MR_PCS_Pos) /**< \brief (SPI_MR) Peripheral Chip Select */
#define SPI_MR_PCS(value) ((SPI_MR_PCS_Msk & ((value) << SPI_MR_PCS_Pos)))
#define SPI_MR_DLYBCS_Pos 24
#define SPI_MR_DLYBCS_Msk (0xffu << SPI_MR_DLYBCS_Pos) /**< \brief (SPI_MR) Delay Between Chip Selects */
#define SPI_MR_DLYBCS(value) ((SPI_MR_DLYBCS_Msk & ((value) << SPI_MR_DLYBCS_Pos)))
/* -------- SPI_RDR : (SPI Offset: 0x08) Receive Data Register -------- */
#define SPI_RDR_RD_Pos 0
#define SPI_RDR_RD_Msk (0xffffu << SPI_RDR_RD_Pos) /**< \brief (SPI_RDR) Receive Data */
#define SPI_RDR_PCS_Pos 16
#define SPI_RDR_PCS_Msk (0xfu << SPI_RDR_PCS_Pos) /**< \brief (SPI_RDR) Peripheral Chip Select */
/* -------- SPI_TDR : (SPI Offset: 0x0C) Transmit Data Register -------- */
#define SPI_TDR_TD_Pos 0
#define SPI_TDR_TD_Msk (0xffffu << SPI_TDR_TD_Pos) /**< \brief (SPI_TDR) Transmit Data */
#define SPI_TDR_TD(value) ((SPI_TDR_TD_Msk & ((value) << SPI_TDR_TD_Pos)))
#define SPI_TDR_PCS_Pos 16
#define SPI_TDR_PCS_Msk (0xfu << SPI_TDR_PCS_Pos) /**< \brief (SPI_TDR) Peripheral Chip Select */
#define SPI_TDR_PCS(value) ((SPI_TDR_PCS_Msk & ((value) << SPI_TDR_PCS_Pos)))
#define SPI_TDR_LASTXFER (0x1u << 24) /**< \brief (SPI_TDR) Last Transfer */
/* -------- SPI_SR : (SPI Offset: 0x10) Status Register -------- */
#define SPI_SR_RDRF (0x1u << 0) /**< \brief (SPI_SR) Receive Data Register Full */
#define SPI_SR_TDRE (0x1u << 1) /**< \brief (SPI_SR) Transmit Data Register Empty */
#define SPI_SR_MODF (0x1u << 2) /**< \brief (SPI_SR) Mode Fault Error */
#define SPI_SR_OVRES (0x1u << 3) /**< \brief (SPI_SR) Overrun Error Status */
#define SPI_SR_NSSR (0x1u << 8) /**< \brief (SPI_SR) NSS Rising */
#define SPI_SR_TXEMPTY (0x1u << 9) /**< \brief (SPI_SR) Transmission Registers Empty */
#define SPI_SR_UNDES (0x1u << 10) /**< \brief (SPI_SR) Underrun Error Status (Slave Mode Only) */
#define SPI_SR_SPIENS (0x1u << 16) /**< \brief (SPI_SR) SPI Enable Status */
/* -------- SPI_IER : (SPI Offset: 0x14) Interrupt Enable Register -------- */
#define SPI_IER_RDRF (0x1u << 0) /**< \brief (SPI_IER) Receive Data Register Full Interrupt Enable */
#define SPI_IER_TDRE (0x1u << 1) /**< \brief (SPI_IER) SPI Transmit Data Register Empty Interrupt Enable */
#define SPI_IER_MODF (0x1u << 2) /**< \brief (SPI_IER) Mode Fault Error Interrupt Enable */
#define SPI_IER_OVRES (0x1u << 3) /**< \brief (SPI_IER) Overrun Error Interrupt Enable */
#define SPI_IER_NSSR (0x1u << 8) /**< \brief (SPI_IER) NSS Rising Interrupt Enable */
#define SPI_IER_TXEMPTY (0x1u << 9) /**< \brief (SPI_IER) Transmission Registers Empty Enable */
#define SPI_IER_UNDES (0x1u << 10) /**< \brief (SPI_IER) Underrun Error Interrupt Enable */
/* -------- SPI_IDR : (SPI Offset: 0x18) Interrupt Disable Register -------- */
#define SPI_IDR_RDRF (0x1u << 0) /**< \brief (SPI_IDR) Receive Data Register Full Interrupt Disable */
#define SPI_IDR_TDRE (0x1u << 1) /**< \brief (SPI_IDR) SPI Transmit Data Register Empty Interrupt Disable */
#define SPI_IDR_MODF (0x1u << 2) /**< \brief (SPI_IDR) Mode Fault Error Interrupt Disable */
#define SPI_IDR_OVRES (0x1u << 3) /**< \brief (SPI_IDR) Overrun Error Interrupt Disable */
#define SPI_IDR_NSSR (0x1u << 8) /**< \brief (SPI_IDR) NSS Rising Interrupt Disable */
#define SPI_IDR_TXEMPTY (0x1u << 9) /**< \brief (SPI_IDR) Transmission Registers Empty Disable */
#define SPI_IDR_UNDES (0x1u << 10) /**< \brief (SPI_IDR) Underrun Error Interrupt Disable */
/* -------- SPI_IMR : (SPI Offset: 0x1C) Interrupt Mask Register -------- */
#define SPI_IMR_RDRF (0x1u << 0) /**< \brief (SPI_IMR) Receive Data Register Full Interrupt Mask */
#define SPI_IMR_TDRE (0x1u << 1) /**< \brief (SPI_IMR) SPI Transmit Data Register Empty Interrupt Mask */
#define SPI_IMR_MODF (0x1u << 2) /**< \brief (SPI_IMR) Mode Fault Error Interrupt Mask */
#define SPI_IMR_OVRES (0x1u << 3) /**< \brief (SPI_IMR) Overrun Error Interrupt Mask */
#define SPI_IMR_NSSR (0x1u << 8) /**< \brief (SPI_IMR) NSS Rising Interrupt Mask */
#define SPI_IMR_TXEMPTY (0x1u << 9) /**< \brief (SPI_IMR) Transmission Registers Empty Mask */
#define SPI_IMR_UNDES (0x1u << 10) /**< \brief (SPI_IMR) Underrun Error Interrupt Mask */
/* -------- SPI_CSR[4] : (SPI Offset: 0x30) Chip Select Register -------- */
#define SPI_CSR_CPOL (0x1u << 0) /**< \brief (SPI_CSR[4]) Clock Polarity */
#define SPI_CSR_NCPHA (0x1u << 1) /**< \brief (SPI_CSR[4]) Clock Phase */
#define SPI_CSR_CSNAAT (0x1u << 2) /**< \brief (SPI_CSR[4]) Chip Select Not Active After Transfer (Ignored if CSAAT = 1) */
#define SPI_CSR_CSAAT (0x1u << 3) /**< \brief (SPI_CSR[4]) Chip Select Not Active After Transfer (Ignored if CSAAT = 1) */
#define SPI_CSR_BITS_Pos 4
#define SPI_CSR_BITS_Msk (0xfu << SPI_CSR_BITS_Pos) /**< \brief (SPI_CSR[4]) Bits Per Transfer */
#define   SPI_CSR_BITS_8_BIT (0x0u << 4) /**< \brief (SPI_CSR[4]) 8 bits for transfer */
#define   SPI_CSR_BITS_9_BIT (0x1u << 4) /**< \brief (SPI_CSR[4]) 9 bits for transfer */
#define   SPI_CSR_BITS_10_BIT (0x2u << 4) /**< \brief (SPI_CSR[4]) 10 bits for transfer */
#define   SPI_CSR_BITS_11_BIT (0x3u << 4) /**< \brief (SPI_CSR[4]) 11 bits for transfer */
#define   SPI_CSR_BITS_12_BIT (0x4u << 4) /**< \brief (SPI_CSR[4]) 12 bits for transfer */
#define   SPI_CSR_BITS_13_BIT (0x5u << 4) /**< \brief (SPI_CSR[4]) 13 bits for transfer */
#define   SPI_CSR_BITS_14_BIT (0x6u << 4) /**< \brief (SPI_CSR[4]) 14 bits for transfer */
#define   SPI_CSR_BITS_15_BIT (0x7u << 4) /**< \brief (SPI_CSR[4]) 15 bits for transfer */
#define   SPI_CSR_BITS_16_BIT (0x8u << 4) /**< \brief (SPI_CSR[4]) 16 bits for transfer */
#define SPI_CSR_SCBR_Pos 8
#define SPI_CSR_SCBR_Msk (0xffu << SPI_CSR_SCBR_Pos) /**< \brief (SPI_CSR[4]) Serial Clock Baud Rate */
#define SPI_CSR_SCBR(value) ((SPI_CSR_SCBR_Msk & ((value) << SPI_CSR_SCBR_Pos)))
#define SPI_CSR_DLYBS_Pos 16
#define SPI_CSR_DLYBS_Msk (0xffu << SPI_CSR_DLYBS_Pos) /**< \brief (SPI_CSR[4]) Delay Before SPCK */
#define SPI_CSR_DLYBS(value) ((SPI_CSR_DLYBS_Msk & ((value) << SPI_CSR_DLYBS_Pos)))
#define SPI_CSR_DLYBCT_Pos 24
#define SPI_CSR_DLYBCT_Msk (0xffu << SPI_CSR_DLYBCT_Pos) /**< \brief (SPI_CSR[4]) Delay Between Consecutive Transfers */
#define SPI_CSR_DLYBCT(value) ((SPI_CSR_DLYBCT_Msk & ((value) << SPI_CSR_DLYBCT_Pos)))
/* -------- SPI_WPMR : (SPI Offset: 0xE4) Write Protection Control Register -------- */
#define SPI_WPMR_WPEN (0x1u << 0) /**< \brief (SPI_WPMR) Write Protection Enable */
#define SPI_WPMR_WPKEY_Pos 8
#define SPI_WPMR_WPKEY_Msk (0xffffffu << SPI_WPMR_WPKEY_Pos) /**< \brief (SPI_WPMR) Write Protection Key Password */
#define SPI_WPMR_WPKEY(value) ((SPI_WPMR_WPKEY_Msk & ((value) << SPI_WPMR_WPKEY_Pos)))
/* -------- SPI_WPSR : (SPI Offset: 0xE8) Write Protection Status Register -------- */
#define SPI_WPSR_WPVS (0x1u << 0) /**< \brief (SPI_WPSR) Write Protection Violation Status */
#define SPI_WPSR_WPVSRC_Pos 8
#define SPI_WPSR_WPVSRC_Msk (0xffu << SPI_WPSR_WPVSRC_Pos) /**< \brief (SPI_WPSR) Write Protection Violation Source */
/*@}*/ /* end of group SAM3XA_SPI */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Synchronous Serial Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_SSC Synchronous Serial Controller */
/*@{*/

/** \brief Ssc hardware registers */
typedef struct {
  WoReg SSC_CR;        /**< \brief (Ssc Offset: 0x0) Control Register */
  RwReg SSC_CMR;       /**< \brief (Ssc Offset: 0x4) Clock Mode Register */
  RoReg Reserved1[2];
  RwReg SSC_RCMR;      /**< \brief (Ssc Offset: 0x10) Receive Clock Mode Register */
  RwReg SSC_RFMR;      /**< \brief (Ssc Offset: 0x14) Receive Frame Mode Register */
  RwReg SSC_TCMR;      /**< \brief (Ssc Offset: 0x18) Transmit Clock Mode Register */
  RwReg SSC_TFMR;      /**< \brief (Ssc Offset: 0x1C) Transmit Frame Mode Register */
  RoReg SSC_RHR;       /**< \brief (Ssc Offset: 0x20) Receive Holding Register */
  WoReg SSC_THR;       /**< \brief (Ssc Offset: 0x24) Transmit Holding Register */
  RoReg Reserved2[2];
  RoReg SSC_RSHR;      /**< \brief (Ssc Offset: 0x30) Receive Sync. Holding Register */
  RwReg SSC_TSHR;      /**< \brief (Ssc Offset: 0x34) Transmit Sync. Holding Register */
  RwReg SSC_RC0R;      /**< \brief (Ssc Offset: 0x38) Receive Compare 0 Register */
  RwReg SSC_RC1R;      /**< \brief (Ssc Offset: 0x3C) Receive Compare 1 Register */
  RoReg SSC_SR;        /**< \brief (Ssc Offset: 0x40) Status Register */
  WoReg SSC_IER;       /**< \brief (Ssc Offset: 0x44) Interrupt Enable Register */
  WoReg SSC_IDR;       /**< \brief (Ssc Offset: 0x48) Interrupt Disable Register */
  RoReg SSC_IMR;       /**< \brief (Ssc Offset: 0x4C) Interrupt Mask Register */
  RoReg Reserved3[37];
  RwReg SSC_WPMR;      /**< \brief (Ssc Offset: 0xE4) Write Protect Mode Register */
  RoReg SSC_WPSR;      /**< \brief (Ssc Offset: 0xE8) Write Protect Status Register */
} Ssc;
/* -------- SSC_CR : (SSC Offset: 0x0) Control Register -------- */
#define SSC_CR_RXEN (0x1u << 0) /**< \brief (SSC_CR) Receive Enable */
#define SSC_CR_RXDIS (0x1u << 1) /**< \brief (SSC_CR) Receive Disable */
#define SSC_CR_TXEN (0x1u << 8) /**< \brief (SSC_CR) Transmit Enable */
#define SSC_CR_TXDIS (0x1u << 9) /**< \brief (SSC_CR) Transmit Disable */
#define SSC_CR_SWRST (0x1u << 15) /**< \brief (SSC_CR) Software Reset */
/* -------- SSC_CMR : (SSC Offset: 0x4) Clock Mode Register -------- */
#define SSC_CMR_DIV_Pos 0
#define SSC_CMR_DIV_Msk (0xfffu << SSC_CMR_DIV_Pos) /**< \brief (SSC_CMR) Clock Divider */
#define SSC_CMR_DIV(value) ((SSC_CMR_DIV_Msk & ((value) << SSC_CMR_DIV_Pos)))
/* -------- SSC_RCMR : (SSC Offset: 0x10) Receive Clock Mode Register -------- */
#define SSC_RCMR_CKS_Pos 0
#define SSC_RCMR_CKS_Msk (0x3u << SSC_RCMR_CKS_Pos) /**< \brief (SSC_RCMR) Receive Clock Selection */
#define   SSC_RCMR_CKS_MCK (0x0u << 0) /**< \brief (SSC_RCMR) Divided Clock */
#define   SSC_RCMR_CKS_TK (0x1u << 0) /**< \brief (SSC_RCMR) TK Clock signal */
#define   SSC_RCMR_CKS_RK (0x2u << 0) /**< \brief (SSC_RCMR) RK pin */
#define SSC_RCMR_CKO_Pos 2
#define SSC_RCMR_CKO_Msk (0x7u << SSC_RCMR_CKO_Pos) /**< \brief (SSC_RCMR) Receive Clock Output Mode Selection */
#define   SSC_RCMR_CKO_NONE (0x0u << 2) /**< \brief (SSC_RCMR) None */
#define   SSC_RCMR_CKO_CONTINUOUS (0x1u << 2) /**< \brief (SSC_RCMR) Continuous Receive Clock */
#define   SSC_RCMR_CKO_TRANSFER (0x2u << 2) /**< \brief (SSC_RCMR) Receive Clock only during data transfers */
#define SSC_RCMR_CKI (0x1u << 5) /**< \brief (SSC_RCMR) Receive Clock Inversion */
#define SSC_RCMR_CKG_Pos 6
#define SSC_RCMR_CKG_Msk (0x3u << SSC_RCMR_CKG_Pos) /**< \brief (SSC_RCMR) Receive Clock Gating Selection */
#define   SSC_RCMR_CKG_NONE (0x0u << 6) /**< \brief (SSC_RCMR) None */
#define   SSC_RCMR_CKG_CONTINUOUS (0x1u << 6) /**< \brief (SSC_RCMR) Continuous Receive Clock */
#define   SSC_RCMR_CKG_TRANSFER (0x2u << 6) /**< \brief (SSC_RCMR) Receive Clock only during data transfers */
#define SSC_RCMR_START_Pos 8
#define SSC_RCMR_START_Msk (0xfu << SSC_RCMR_START_Pos) /**< \brief (SSC_RCMR) Receive Start Selection */
#define   SSC_RCMR_START_CONTINUOUS (0x0u << 8) /**< \brief (SSC_RCMR) Continuous, as soon as the receiver is enabled, and immediately after the end of transfer of the previous data. */
#define   SSC_RCMR_START_TRANSMIT (0x1u << 8) /**< \brief (SSC_RCMR) Transmit start */
#define   SSC_RCMR_START_RF_LOW (0x2u << 8) /**< \brief (SSC_RCMR) Detection of a low level on RF signal */
#define   SSC_RCMR_START_RF_HIGH (0x3u << 8) /**< \brief (SSC_RCMR) Detection of a high level on RF signal */
#define   SSC_RCMR_START_RF_FALLING (0x4u << 8) /**< \brief (SSC_RCMR) Detection of a falling edge on RF signal */
#define   SSC_RCMR_START_RF_RISING (0x5u << 8) /**< \brief (SSC_RCMR) Detection of a rising edge on RF signal */
#define   SSC_RCMR_START_RF_LEVEL (0x6u << 8) /**< \brief (SSC_RCMR) Detection of any level change on RF signal */
#define   SSC_RCMR_START_RF_EDGE (0x7u << 8) /**< \brief (SSC_RCMR) Detection of any edge on RF signal */
#define   SSC_RCMR_START_CMP_0 (0x8u << 8) /**< \brief (SSC_RCMR) Compare 0 */
#define SSC_RCMR_STOP (0x1u << 12) /**< \brief (SSC_RCMR) Receive Stop Selection */
#define SSC_RCMR_STTDLY_Pos 16
#define SSC_RCMR_STTDLY_Msk (0xffu << SSC_RCMR_STTDLY_Pos) /**< \brief (SSC_RCMR) Receive Start Delay */
#define SSC_RCMR_STTDLY(value) ((SSC_RCMR_STTDLY_Msk & ((value) << SSC_RCMR_STTDLY_Pos)))
#define SSC_RCMR_PERIOD_Pos 24
#define SSC_RCMR_PERIOD_Msk (0xffu << SSC_RCMR_PERIOD_Pos) /**< \brief (SSC_RCMR) Receive Period Divider Selection */
#define SSC_RCMR_PERIOD(value) ((SSC_RCMR_PERIOD_Msk & ((value) << SSC_RCMR_PERIOD_Pos)))
/* -------- SSC_RFMR : (SSC Offset: 0x14) Receive Frame Mode Register -------- */
#define SSC_RFMR_DATLEN_Pos 0
#define SSC_RFMR_DATLEN_Msk (0x1fu << SSC_RFMR_DATLEN_Pos) /**< \brief (SSC_RFMR) Data Length */
#define SSC_RFMR_DATLEN(value) ((SSC_RFMR_DATLEN_Msk & ((value) << SSC_RFMR_DATLEN_Pos)))
#define SSC_RFMR_LOOP (0x1u << 5) /**< \brief (SSC_RFMR) Loop Mode */
#define SSC_RFMR_MSBF (0x1u << 7) /**< \brief (SSC_RFMR) Most Significant Bit First */
#define SSC_RFMR_DATNB_Pos 8
#define SSC_RFMR_DATNB_Msk (0xfu << SSC_RFMR_DATNB_Pos) /**< \brief (SSC_RFMR) Data Number per Frame */
#define SSC_RFMR_DATNB(value) ((SSC_RFMR_DATNB_Msk & ((value) << SSC_RFMR_DATNB_Pos)))
#define SSC_RFMR_FSLEN_Pos 16
#define SSC_RFMR_FSLEN_Msk (0xfu << SSC_RFMR_FSLEN_Pos) /**< \brief (SSC_RFMR) Receive Frame Sync Length */
#define SSC_RFMR_FSLEN(value) ((SSC_RFMR_FSLEN_Msk & ((value) << SSC_RFMR_FSLEN_Pos)))
#define SSC_RFMR_FSOS_Pos 20
#define SSC_RFMR_FSOS_Msk (0x7u << SSC_RFMR_FSOS_Pos) /**< \brief (SSC_RFMR) Receive Frame Sync Output Selection */
#define   SSC_RFMR_FSOS_NONE (0x0u << 20) /**< \brief (SSC_RFMR) None */
#define   SSC_RFMR_FSOS_NEGATIVE (0x1u << 20) /**< \brief (SSC_RFMR) Negative Pulse */
#define   SSC_RFMR_FSOS_POSITIVE (0x2u << 20) /**< \brief (SSC_RFMR) Positive Pulse */
#define   SSC_RFMR_FSOS_LOW (0x3u << 20) /**< \brief (SSC_RFMR) Driven Low during data transfer */
#define   SSC_RFMR_FSOS_HIGH (0x4u << 20) /**< \brief (SSC_RFMR) Driven High during data transfer */
#define   SSC_RFMR_FSOS_TOGGLING (0x5u << 20) /**< \brief (SSC_RFMR) Toggling at each start of data transfer */
#define SSC_RFMR_FSEDGE (0x1u << 24) /**< \brief (SSC_RFMR) Frame Sync Edge Detection */
#define   SSC_RFMR_FSEDGE_POSITIVE (0x0u << 24) /**< \brief (SSC_RFMR) Positive Edge Detection */
#define   SSC_RFMR_FSEDGE_NEGATIVE (0x1u << 24) /**< \brief (SSC_RFMR) Negative Edge Detection */
#define SSC_RFMR_FSLEN_EXT_Pos 28
#define SSC_RFMR_FSLEN_EXT_Msk (0xfu << SSC_RFMR_FSLEN_EXT_Pos) /**< \brief (SSC_RFMR) FSLEN Field Extension */
#define SSC_RFMR_FSLEN_EXT(value) ((SSC_RFMR_FSLEN_EXT_Msk & ((value) << SSC_RFMR_FSLEN_EXT_Pos)))
/* -------- SSC_TCMR : (SSC Offset: 0x18) Transmit Clock Mode Register -------- */
#define SSC_TCMR_CKS_Pos 0
#define SSC_TCMR_CKS_Msk (0x3u << SSC_TCMR_CKS_Pos) /**< \brief (SSC_TCMR) Transmit Clock Selection */
#define   SSC_TCMR_CKS_MCK (0x0u << 0) /**< \brief (SSC_TCMR) Divided Clock */
#define   SSC_TCMR_CKS_TK (0x1u << 0) /**< \brief (SSC_TCMR) TK Clock signal */
#define   SSC_TCMR_CKS_RK (0x2u << 0) /**< \brief (SSC_TCMR) RK pin */
#define SSC_TCMR_CKO_Pos 2
#define SSC_TCMR_CKO_Msk (0x7u << SSC_TCMR_CKO_Pos) /**< \brief (SSC_TCMR) Transmit Clock Output Mode Selection */
#define   SSC_TCMR_CKO_NONE (0x0u << 2) /**< \brief (SSC_TCMR) None */
#define   SSC_TCMR_CKO_CONTINUOUS (0x1u << 2) /**< \brief (SSC_TCMR) Continuous Receive Clock */
#define   SSC_TCMR_CKO_TRANSFER (0x2u << 2) /**< \brief (SSC_TCMR) Transmit Clock only during data transfers */
#define SSC_TCMR_CKI (0x1u << 5) /**< \brief (SSC_TCMR) Transmit Clock Inversion */
#define SSC_TCMR_CKG_Pos 6
#define SSC_TCMR_CKG_Msk (0x3u << SSC_TCMR_CKG_Pos) /**< \brief (SSC_TCMR) Transmit Clock Gating Selection */
#define   SSC_TCMR_CKG_NONE (0x0u << 6) /**< \brief (SSC_TCMR) None */
#define   SSC_TCMR_CKG_CONTINUOUS (0x1u << 6) /**< \brief (SSC_TCMR) Transmit Clock enabled only if TF Low */
#define   SSC_TCMR_CKG_TRANSFER (0x2u << 6) /**< \brief (SSC_TCMR) Transmit Clock enabled only if TF High */
#define SSC_TCMR_START_Pos 8
#define SSC_TCMR_START_Msk (0xfu << SSC_TCMR_START_Pos) /**< \brief (SSC_TCMR) Transmit Start Selection */
#define   SSC_TCMR_START_CONTINUOUS (0x0u << 8) /**< \brief (SSC_TCMR) Continuous, as soon as a word is written in the SSC_THR Register (if Transmit is enabled), and immediately after the end of transfer of the previous data. */
#define   SSC_TCMR_START_RECEIVE (0x1u << 8) /**< \brief (SSC_TCMR) Receive start */
#define   SSC_TCMR_START_RF_LOW (0x2u << 8) /**< \brief (SSC_TCMR) Detection of a low level on TF signal */
#define   SSC_TCMR_START_RF_HIGH (0x3u << 8) /**< \brief (SSC_TCMR) Detection of a high level on TF signal */
#define   SSC_TCMR_START_RF_FALLING (0x4u << 8) /**< \brief (SSC_TCMR) Detection of a falling edge on TF signal */
#define   SSC_TCMR_START_RF_RISING (0x5u << 8) /**< \brief (SSC_TCMR) Detection of a rising edge on TF signal */
#define   SSC_TCMR_START_RF_LEVEL (0x6u << 8) /**< \brief (SSC_TCMR) Detection of any level change on TF signal */
#define   SSC_TCMR_START_RF_EDGE (0x7u << 8) /**< \brief (SSC_TCMR) Detection of any edge on TF signal */
#define   SSC_TCMR_START_CMP_0 (0x8u << 8) /**< \brief (SSC_TCMR) Compare 0 */
#define SSC_TCMR_STTDLY_Pos 16
#define SSC_TCMR_STTDLY_Msk (0xffu << SSC_TCMR_STTDLY_Pos) /**< \brief (SSC_TCMR) Transmit Start Delay */
#define SSC_TCMR_STTDLY(value) ((SSC_TCMR_STTDLY_Msk & ((value) << SSC_TCMR_STTDLY_Pos)))
#define SSC_TCMR_PERIOD_Pos 24
#define SSC_TCMR_PERIOD_Msk (0xffu << SSC_TCMR_PERIOD_Pos) /**< \brief (SSC_TCMR) Transmit Period Divider Selection */
#define SSC_TCMR_PERIOD(value) ((SSC_TCMR_PERIOD_Msk & ((value) << SSC_TCMR_PERIOD_Pos)))
/* -------- SSC_TFMR : (SSC Offset: 0x1C) Transmit Frame Mode Register -------- */
#define SSC_TFMR_DATLEN_Pos 0
#define SSC_TFMR_DATLEN_Msk (0x1fu << SSC_TFMR_DATLEN_Pos) /**< \brief (SSC_TFMR) Data Length */
#define SSC_TFMR_DATLEN(value) ((SSC_TFMR_DATLEN_Msk & ((value) << SSC_TFMR_DATLEN_Pos)))
#define SSC_TFMR_DATDEF (0x1u << 5) /**< \brief (SSC_TFMR) Data Default Value */
#define SSC_TFMR_MSBF (0x1u << 7) /**< \brief (SSC_TFMR) Most Significant Bit First */
#define SSC_TFMR_DATNB_Pos 8
#define SSC_TFMR_DATNB_Msk (0xfu << SSC_TFMR_DATNB_Pos) /**< \brief (SSC_TFMR) Data Number per frame */
#define SSC_TFMR_DATNB(value) ((SSC_TFMR_DATNB_Msk & ((value) << SSC_TFMR_DATNB_Pos)))
#define SSC_TFMR_FSLEN_Pos 16
#define SSC_TFMR_FSLEN_Msk (0xfu << SSC_TFMR_FSLEN_Pos) /**< \brief (SSC_TFMR) Transmit Frame Sync Length */
#define SSC_TFMR_FSLEN(value) ((SSC_TFMR_FSLEN_Msk & ((value) << SSC_TFMR_FSLEN_Pos)))
#define SSC_TFMR_FSOS_Pos 20
#define SSC_TFMR_FSOS_Msk (0x7u << SSC_TFMR_FSOS_Pos) /**< \brief (SSC_TFMR) Transmit Frame Sync Output Selection */
#define   SSC_TFMR_FSOS_NONE (0x0u << 20) /**< \brief (SSC_TFMR) None */
#define   SSC_TFMR_FSOS_NEGATIVE (0x1u << 20) /**< \brief (SSC_TFMR) Negative Pulse */
#define   SSC_TFMR_FSOS_POSITIVE (0x2u << 20) /**< \brief (SSC_TFMR) Positive Pulse */
#define   SSC_TFMR_FSOS_LOW (0x3u << 20) /**< \brief (SSC_TFMR) Driven Low during data transfer */
#define   SSC_TFMR_FSOS_HIGH (0x4u << 20) /**< \brief (SSC_TFMR) Driven High during data transfer */
#define   SSC_TFMR_FSOS_TOGGLING (0x5u << 20) /**< \brief (SSC_TFMR) Toggling at each start of data transfer */
#define SSC_TFMR_FSDEN (0x1u << 23) /**< \brief (SSC_TFMR) Frame Sync Data Enable */
#define SSC_TFMR_FSEDGE (0x1u << 24) /**< \brief (SSC_TFMR) Frame Sync Edge Detection */
#define   SSC_TFMR_FSEDGE_POSITIVE (0x0u << 24) /**< \brief (SSC_TFMR) Positive Edge Detection */
#define   SSC_TFMR_FSEDGE_NEGATIVE (0x1u << 24) /**< \brief (SSC_TFMR) Negative Edge Detection */
#define SSC_TFMR_FSLEN_EXT_Pos 28
#define SSC_TFMR_FSLEN_EXT_Msk (0xfu << SSC_TFMR_FSLEN_EXT_Pos) /**< \brief (SSC_TFMR) FSLEN Field Extension */
#define SSC_TFMR_FSLEN_EXT(value) ((SSC_TFMR_FSLEN_EXT_Msk & ((value) << SSC_TFMR_FSLEN_EXT_Pos)))
/* -------- SSC_RHR : (SSC Offset: 0x20) Receive Holding Register -------- */
#define SSC_RHR_RDAT_Pos 0
#define SSC_RHR_RDAT_Msk (0xffffffffu << SSC_RHR_RDAT_Pos) /**< \brief (SSC_RHR) Receive Data */
/* -------- SSC_THR : (SSC Offset: 0x24) Transmit Holding Register -------- */
#define SSC_THR_TDAT_Pos 0
#define SSC_THR_TDAT_Msk (0xffffffffu << SSC_THR_TDAT_Pos) /**< \brief (SSC_THR) Transmit Data */
#define SSC_THR_TDAT(value) ((SSC_THR_TDAT_Msk & ((value) << SSC_THR_TDAT_Pos)))
/* -------- SSC_RSHR : (SSC Offset: 0x30) Receive Sync. Holding Register -------- */
#define SSC_RSHR_RSDAT_Pos 0
#define SSC_RSHR_RSDAT_Msk (0xffffu << SSC_RSHR_RSDAT_Pos) /**< \brief (SSC_RSHR) Receive Synchronization Data */
/* -------- SSC_TSHR : (SSC Offset: 0x34) Transmit Sync. Holding Register -------- */
#define SSC_TSHR_TSDAT_Pos 0
#define SSC_TSHR_TSDAT_Msk (0xffffu << SSC_TSHR_TSDAT_Pos) /**< \brief (SSC_TSHR) Transmit Synchronization Data */
#define SSC_TSHR_TSDAT(value) ((SSC_TSHR_TSDAT_Msk & ((value) << SSC_TSHR_TSDAT_Pos)))
/* -------- SSC_RC0R : (SSC Offset: 0x38) Receive Compare 0 Register -------- */
#define SSC_RC0R_CP0_Pos 0
#define SSC_RC0R_CP0_Msk (0xffffu << SSC_RC0R_CP0_Pos) /**< \brief (SSC_RC0R) Receive Compare Data 0 */
#define SSC_RC0R_CP0(value) ((SSC_RC0R_CP0_Msk & ((value) << SSC_RC0R_CP0_Pos)))
/* -------- SSC_RC1R : (SSC Offset: 0x3C) Receive Compare 1 Register -------- */
#define SSC_RC1R_CP1_Pos 0
#define SSC_RC1R_CP1_Msk (0xffffu << SSC_RC1R_CP1_Pos) /**< \brief (SSC_RC1R) Receive Compare Data 1 */
#define SSC_RC1R_CP1(value) ((SSC_RC1R_CP1_Msk & ((value) << SSC_RC1R_CP1_Pos)))
/* -------- SSC_SR : (SSC Offset: 0x40) Status Register -------- */
#define SSC_SR_TXRDY (0x1u << 0) /**< \brief (SSC_SR) Transmit Ready */
#define SSC_SR_TXEMPTY (0x1u << 1) /**< \brief (SSC_SR) Transmit Empty */
#define SSC_SR_RXRDY (0x1u << 4) /**< \brief (SSC_SR) Receive Ready */
#define SSC_SR_OVRUN (0x1u << 5) /**< \brief (SSC_SR) Receive Overrun */
#define SSC_SR_CP0 (0x1u << 8) /**< \brief (SSC_SR) Compare 0 */
#define SSC_SR_CP1 (0x1u << 9) /**< \brief (SSC_SR) Compare 1 */
#define SSC_SR_TXSYN (0x1u << 10) /**< \brief (SSC_SR) Transmit Sync */
#define SSC_SR_RXSYN (0x1u << 11) /**< \brief (SSC_SR) Receive Sync */
#define SSC_SR_TXEN (0x1u << 16) /**< \brief (SSC_SR) Transmit Enable */
#define SSC_SR_RXEN (0x1u << 17) /**< \brief (SSC_SR) Receive Enable */
/* -------- SSC_IER : (SSC Offset: 0x44) Interrupt Enable Register -------- */
#define SSC_IER_TXRDY (0x1u << 0) /**< \brief (SSC_IER) Transmit Ready Interrupt Enable */
#define SSC_IER_TXEMPTY (0x1u << 1) /**< \brief (SSC_IER) Transmit Empty Interrupt Enable */
#define SSC_IER_RXRDY (0x1u << 4) /**< \brief (SSC_IER) Receive Ready Interrupt Enable */
#define SSC_IER_OVRUN (0x1u << 5) /**< \brief (SSC_IER) Receive Overrun Interrupt Enable */
#define SSC_IER_CP0 (0x1u << 8) /**< \brief (SSC_IER) Compare 0 Interrupt Enable */
#define SSC_IER_CP1 (0x1u << 9) /**< \brief (SSC_IER) Compare 1 Interrupt Enable */
#define SSC_IER_TXSYN (0x1u << 10) /**< \brief (SSC_IER) Tx Sync Interrupt Enable */
#define SSC_IER_RXSYN (0x1u << 11) /**< \brief (SSC_IER) Rx Sync Interrupt Enable */
/* -------- SSC_IDR : (SSC Offset: 0x48) Interrupt Disable Register -------- */
#define SSC_IDR_TXRDY (0x1u << 0) /**< \brief (SSC_IDR) Transmit Ready Interrupt Disable */
#define SSC_IDR_TXEMPTY (0x1u << 1) /**< \brief (SSC_IDR) Transmit Empty Interrupt Disable */
#define SSC_IDR_RXRDY (0x1u << 4) /**< \brief (SSC_IDR) Receive Ready Interrupt Disable */
#define SSC_IDR_OVRUN (0x1u << 5) /**< \brief (SSC_IDR) Receive Overrun Interrupt Disable */
#define SSC_IDR_CP0 (0x1u << 8) /**< \brief (SSC_IDR) Compare 0 Interrupt Disable */
#define SSC_IDR_CP1 (0x1u << 9) /**< \brief (SSC_IDR) Compare 1 Interrupt Disable */
#define SSC_IDR_TXSYN (0x1u << 10) /**< \brief (SSC_IDR) Tx Sync Interrupt Enable */
#define SSC_IDR_RXSYN (0x1u << 11) /**< \brief (SSC_IDR) Rx Sync Interrupt Enable */
/* -------- SSC_IMR : (SSC Offset: 0x4C) Interrupt Mask Register -------- */
#define SSC_IMR_TXRDY (0x1u << 0) /**< \brief (SSC_IMR) Transmit Ready Interrupt Mask */
#define SSC_IMR_TXEMPTY (0x1u << 1) /**< \brief (SSC_IMR) Transmit Empty Interrupt Mask */
#define SSC_IMR_RXRDY (0x1u << 4) /**< \brief (SSC_IMR) Receive Ready Interrupt Mask */
#define SSC_IMR_OVRUN (0x1u << 5) /**< \brief (SSC_IMR) Receive Overrun Interrupt Mask */
#define SSC_IMR_CP0 (0x1u << 8) /**< \brief (SSC_IMR) Compare 0 Interrupt Mask */
#define SSC_IMR_CP1 (0x1u << 9) /**< \brief (SSC_IMR) Compare 1 Interrupt Mask */
#define SSC_IMR_TXSYN (0x1u << 10) /**< \brief (SSC_IMR) Tx Sync Interrupt Mask */
#define SSC_IMR_RXSYN (0x1u << 11) /**< \brief (SSC_IMR) Rx Sync Interrupt Mask */
/* -------- SSC_WPMR : (SSC Offset: 0xE4) Write Protect Mode Register -------- */
#define SSC_WPMR_WPEN (0x1u << 0) /**< \brief (SSC_WPMR) Write Protect Enable */
#define SSC_WPMR_WPKEY_Pos 8
#define SSC_WPMR_WPKEY_Msk (0xffffffu << SSC_WPMR_WPKEY_Pos) /**< \brief (SSC_WPMR) Write Protect KEY */
#define SSC_WPMR_WPKEY(value) ((SSC_WPMR_WPKEY_Msk & ((value) << SSC_WPMR_WPKEY_Pos)))
/* -------- SSC_WPSR : (SSC Offset: 0xE8) Write Protect Status Register -------- */
#define SSC_WPSR_WPVS (0x1u << 0) /**< \brief (SSC_WPSR) Write Protect Violation Status */
#define SSC_WPSR_WPVSRC_Pos 8
#define SSC_WPSR_WPVSRC_Msk (0xffffu << SSC_WPSR_WPVSRC_Pos) /**< \brief (SSC_WPSR) Write Protect Violation Source */
/*@}*/ /* end of group SAM3XA_SSC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Supply Controller */
/* ============================================================================= */
/** \addtogroup SAM3XA_SUPC Supply Controller */
/*@{*/

/** \brief Supc hardware registers */
typedef struct {
  WoReg SUPC_CR;   /**< \brief (Supc Offset: 0x00) Supply Controller Control Register */
  RwReg SUPC_SMMR; /**< \brief (Supc Offset: 0x04) Supply Controller Supply Monitor Mode Register */
  RwReg SUPC_MR;   /**< \brief (Supc Offset: 0x08) Supply Controller Mode Register */
  RwReg SUPC_WUMR; /**< \brief (Supc Offset: 0x0C) Supply Controller Wake Up Mode Register */
  RwReg SUPC_WUIR; /**< \brief (Supc Offset: 0x10) Supply Controller Wake Up Inputs Register */
  RoReg SUPC_SR;   /**< \brief (Supc Offset: 0x14) Supply Controller Status Register */
} Supc;
/* -------- SUPC_CR : (SUPC Offset: 0x00) Supply Controller Control Register -------- */
#define SUPC_CR_VROFF (0x1u << 2) /**< \brief (SUPC_CR) Voltage Regulator Off */
#define   SUPC_CR_VROFF_NO_EFFECT (0x0u << 2) /**< \brief (SUPC_CR) no effect. */
#define   SUPC_CR_VROFF_STOP_VREG (0x1u << 2) /**< \brief (SUPC_CR) if KEY is correct, asserts vddcore_nreset and stops the voltage regulator. */
#define SUPC_CR_XTALSEL (0x1u << 3) /**< \brief (SUPC_CR) Crystal Oscillator Select */
#define   SUPC_CR_XTALSEL_NO_EFFECT (0x0u << 3) /**< \brief (SUPC_CR) no effect. */
#define   SUPC_CR_XTALSEL_CRYSTAL_SEL (0x1u << 3) /**< \brief (SUPC_CR) if KEY is correct, switches the slow clock on the crystal oscillator output. */
#define SUPC_CR_KEY_Pos 24
#define SUPC_CR_KEY_Msk (0xffu << SUPC_CR_KEY_Pos) /**< \brief (SUPC_CR) Password */
#define SUPC_CR_KEY(value) ((SUPC_CR_KEY_Msk & ((value) << SUPC_CR_KEY_Pos)))
/* -------- SUPC_SMMR : (SUPC Offset: 0x04) Supply Controller Supply Monitor Mode Register -------- */
#define SUPC_SMMR_SMTH_Pos 0
#define SUPC_SMMR_SMTH_Msk (0xfu << SUPC_SMMR_SMTH_Pos) /**< \brief (SUPC_SMMR) Supply Monitor Threshold */
#define   SUPC_SMMR_SMTH_1_9V (0x0u << 0) /**< \brief (SUPC_SMMR) 1.9 V */
#define   SUPC_SMMR_SMTH_2_0V (0x1u << 0) /**< \brief (SUPC_SMMR) 2.0 V */
#define   SUPC_SMMR_SMTH_2_1V (0x2u << 0) /**< \brief (SUPC_SMMR) 2.1 V */
#define   SUPC_SMMR_SMTH_2_2V (0x3u << 0) /**< \brief (SUPC_SMMR) 2.2 V */
#define   SUPC_SMMR_SMTH_2_3V (0x4u << 0) /**< \brief (SUPC_SMMR) 2.3 V */
#define   SUPC_SMMR_SMTH_2_4V (0x5u << 0) /**< \brief (SUPC_SMMR) 2.4 V */
#define   SUPC_SMMR_SMTH_2_5V (0x6u << 0) /**< \brief (SUPC_SMMR) 2.5 V */
#define   SUPC_SMMR_SMTH_2_6V (0x7u << 0) /**< \brief (SUPC_SMMR) 2.6 V */
#define   SUPC_SMMR_SMTH_2_7V (0x8u << 0) /**< \brief (SUPC_SMMR) 2.7 V */
#define   SUPC_SMMR_SMTH_2_8V (0x9u << 0) /**< \brief (SUPC_SMMR) 2.8 V */
#define   SUPC_SMMR_SMTH_2_9V (0xAu << 0) /**< \brief (SUPC_SMMR) 2.9 V */
#define   SUPC_SMMR_SMTH_3_0V (0xBu << 0) /**< \brief (SUPC_SMMR) 3.0 V */
#define   SUPC_SMMR_SMTH_3_1V (0xCu << 0) /**< \brief (SUPC_SMMR) 3.1 V */
#define   SUPC_SMMR_SMTH_3_2V (0xDu << 0) /**< \brief (SUPC_SMMR) 3.2 V */
#define   SUPC_SMMR_SMTH_3_3V (0xEu << 0) /**< \brief (SUPC_SMMR) 3.3 V */
#define   SUPC_SMMR_SMTH_3_4V (0xFu << 0) /**< \brief (SUPC_SMMR) 3.4 V */
#define SUPC_SMMR_SMSMPL_Pos 8
#define SUPC_SMMR_SMSMPL_Msk (0x7u << SUPC_SMMR_SMSMPL_Pos) /**< \brief (SUPC_SMMR) Supply Monitor Sampling Period */
#define   SUPC_SMMR_SMSMPL_SMD (0x0u << 8) /**< \brief (SUPC_SMMR) Supply Monitor disabled */
#define   SUPC_SMMR_SMSMPL_CSM (0x1u << 8) /**< \brief (SUPC_SMMR) Continuous Supply Monitor */
#define   SUPC_SMMR_SMSMPL_32SLCK (0x2u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 32 SLCK periods */
#define   SUPC_SMMR_SMSMPL_256SLCK (0x3u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 256 SLCK periods */
#define   SUPC_SMMR_SMSMPL_2048SLCK (0x4u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 2,048 SLCK periods */
#define SUPC_SMMR_SMRSTEN (0x1u << 12) /**< \brief (SUPC_SMMR) Supply Monitor Reset Enable */
#define   SUPC_SMMR_SMRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_SMMR) the core reset signal "vddcore_nreset" is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_SMMR) the core reset signal, vddcore_nreset is asserted when a supply monitor detection occurs. */
#define SUPC_SMMR_SMIEN (0x1u << 13) /**< \brief (SUPC_SMMR) Supply Monitor Interrupt Enable */
#define   SUPC_SMMR_SMIEN_NOT_ENABLE (0x0u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMIEN_ENABLE (0x1u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is asserted when a supply monitor detection occurs. */
/* -------- SUPC_MR : (SUPC Offset: 0x08) Supply Controller Mode Register -------- */
#define SUPC_MR_BODRSTEN (0x1u << 12) /**< \brief (SUPC_MR) Brownout Detector Reset Enable */
#define   SUPC_MR_BODRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_MR) the core reset signal "vddcore_nreset" is not affected when a brownout detection occurs. */
#define   SUPC_MR_BODRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_MR) the core reset signal, vddcore_nreset is asserted when a brownout detection occurs. */
#define SUPC_MR_BODDIS (0x1u << 13) /**< \brief (SUPC_MR) Brownout Detector Disable */
#define   SUPC_MR_BODDIS_ENABLE (0x0u << 13) /**< \brief (SUPC_MR) the core brownout detector is enabled. */
#define   SUPC_MR_BODDIS_DISABLE (0x1u << 13) /**< \brief (SUPC_MR) the core brownout detector is disabled. */
#define SUPC_MR_VDDIORDYONREG (0x1u << 14) /**< \brief (SUPC_MR)  */
#define SUPC_MR_OSCBYPASS (0x1u << 20) /**< \brief (SUPC_MR) Oscillator Bypass */
#define   SUPC_MR_OSCBYPASS_NO_EFFECT (0x0u << 20) /**< \brief (SUPC_MR) no effect. Clock selection depends on XTALSEL value. */
#define   SUPC_MR_OSCBYPASS_BYPASS (0x1u << 20) /**< \brief (SUPC_MR) the 32-KHz XTAL oscillator is selected and is put in bypass mode. */
#define SUPC_MR_KEY_Pos 24
#define SUPC_MR_KEY_Msk (0xffu << SUPC_MR_KEY_Pos) /**< \brief (SUPC_MR) Password Key */
#define SUPC_MR_KEY(value) ((SUPC_MR_KEY_Msk & ((value) << SUPC_MR_KEY_Pos)))
/* -------- SUPC_WUMR : (SUPC Offset: 0x0C) Supply Controller Wake Up Mode Register -------- */
#define SUPC_WUMR_FWUPEN (0x1u << 0) /**< \brief (SUPC_WUMR) Force Wake Up Enable */
#define   SUPC_WUMR_FWUPEN_NOT_ENABLE (0x0u << 0) /**< \brief (SUPC_WUMR) the Force Wake Up pin has no wake up effect. */
#define   SUPC_WUMR_FWUPEN_ENABLE (0x1u << 0) /**< \brief (SUPC_WUMR) the Force Wake Up pin low forces the wake up of the core power supply. */
#define SUPC_WUMR_SMEN (0x1u << 1) /**< \brief (SUPC_WUMR) Supply Monitor Wake Up Enable */
#define   SUPC_WUMR_SMEN_NOT_ENABLE (0x0u << 1) /**< \brief (SUPC_WUMR) the supply monitor detection has no wake up effect. */
#define   SUPC_WUMR_SMEN_ENABLE (0x1u << 1) /**< \brief (SUPC_WUMR) the supply monitor detection forces the wake up of the core power supply. */
#define SUPC_WUMR_RTTEN (0x1u << 2) /**< \brief (SUPC_WUMR) Real Time Timer Wake Up Enable */
#define   SUPC_WUMR_RTTEN_NOT_ENABLE (0x0u << 2) /**< \brief (SUPC_WUMR) the RTT alarm signal has no wake up effect. */
#define   SUPC_WUMR_RTTEN_ENABLE (0x1u << 2) /**< \brief (SUPC_WUMR) the RTT alarm signal forces the wake up of the core power supply. */
#define SUPC_WUMR_RTCEN (0x1u << 3) /**< \brief (SUPC_WUMR) Real Time Clock Wake Up Enable */
#define   SUPC_WUMR_RTCEN_NOT_ENABLE (0x0u << 3) /**< \brief (SUPC_WUMR) the RTC alarm signal has no wake up effect. */
#define   SUPC_WUMR_RTCEN_ENABLE (0x1u << 3) /**< \brief (SUPC_WUMR) the RTC alarm signal forces the wake up of the core power supply. */
#define SUPC_WUMR_FWUPDBC_Pos 8
#define SUPC_WUMR_FWUPDBC_Msk (0x7u << SUPC_WUMR_FWUPDBC_Pos) /**< \brief (SUPC_WUMR) Force Wake Up Debouncer Period */
#define   SUPC_WUMR_FWUPDBC_IMMEDIATE (0x0u << 8) /**< \brief (SUPC_WUMR) Immediate, no debouncing, detected active at least on one Slow Clock edge. */
#define   SUPC_WUMR_FWUPDBC_3_SCLK (0x1u << 8) /**< \brief (SUPC_WUMR) FWUP shall be low for at least 3 SLCK periods */
#define   SUPC_WUMR_FWUPDBC_32_SCLK (0x2u << 8) /**< \brief (SUPC_WUMR) FWUP shall be low for at least 32 SLCK periods */
#define   SUPC_WUMR_FWUPDBC_512_SCLK (0x3u << 8) /**< \brief (SUPC_WUMR) FWUP shall be low for at least 512 SLCK periods */
#define   SUPC_WUMR_FWUPDBC_4096_SCLK (0x4u << 8) /**< \brief (SUPC_WUMR) FWUP shall be low for at least 4,096 SLCK periods */
#define   SUPC_WUMR_FWUPDBC_32768_SCLK (0x5u << 8) /**< \brief (SUPC_WUMR) FWUP shall be low for at least 32,768 SLCK periods */
#define SUPC_WUMR_WKUPDBC_Pos 12
#define SUPC_WUMR_WKUPDBC_Msk (0x7u << SUPC_WUMR_WKUPDBC_Pos) /**< \brief (SUPC_WUMR) Wake Up Inputs Debouncer Period */
#define   SUPC_WUMR_WKUPDBC_IMMEDIATE (0x0u << 12) /**< \brief (SUPC_WUMR) Immediate, no debouncing, detected active at least on one Slow Clock edge. */
#define   SUPC_WUMR_WKUPDBC_3_SCLK (0x1u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 3 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_32_SCLK (0x2u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 32 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_512_SCLK (0x3u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 512 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_4096_SCLK (0x4u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 4,096 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_32768_SCLK (0x5u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 32,768 SLCK periods */
/* -------- SUPC_WUIR : (SUPC Offset: 0x10) Supply Controller Wake Up Inputs Register -------- */
#define SUPC_WUIR_WKUPEN0 (0x1u << 0) /**< \brief (SUPC_WUIR) Wake Up Input Enable 0 */
#define   SUPC_WUIR_WKUPEN0_NOT_ENABLE (0x0u << 0) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN0_ENABLE (0x1u << 0) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN1 (0x1u << 1) /**< \brief (SUPC_WUIR) Wake Up Input Enable 1 */
#define   SUPC_WUIR_WKUPEN1_NOT_ENABLE (0x0u << 1) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN1_ENABLE (0x1u << 1) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN2 (0x1u << 2) /**< \brief (SUPC_WUIR) Wake Up Input Enable 2 */
#define   SUPC_WUIR_WKUPEN2_NOT_ENABLE (0x0u << 2) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN2_ENABLE (0x1u << 2) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN3 (0x1u << 3) /**< \brief (SUPC_WUIR) Wake Up Input Enable 3 */
#define   SUPC_WUIR_WKUPEN3_NOT_ENABLE (0x0u << 3) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN3_ENABLE (0x1u << 3) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN4 (0x1u << 4) /**< \brief (SUPC_WUIR) Wake Up Input Enable 4 */
#define   SUPC_WUIR_WKUPEN4_NOT_ENABLE (0x0u << 4) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN4_ENABLE (0x1u << 4) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN5 (0x1u << 5) /**< \brief (SUPC_WUIR) Wake Up Input Enable 5 */
#define   SUPC_WUIR_WKUPEN5_NOT_ENABLE (0x0u << 5) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN5_ENABLE (0x1u << 5) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN6 (0x1u << 6) /**< \brief (SUPC_WUIR) Wake Up Input Enable 6 */
#define   SUPC_WUIR_WKUPEN6_NOT_ENABLE (0x0u << 6) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN6_ENABLE (0x1u << 6) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN7 (0x1u << 7) /**< \brief (SUPC_WUIR) Wake Up Input Enable 7 */
#define   SUPC_WUIR_WKUPEN7_NOT_ENABLE (0x0u << 7) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN7_ENABLE (0x1u << 7) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN8 (0x1u << 8) /**< \brief (SUPC_WUIR) Wake Up Input Enable 8 */
#define   SUPC_WUIR_WKUPEN8_NOT_ENABLE (0x0u << 8) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN8_ENABLE (0x1u << 8) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN9 (0x1u << 9) /**< \brief (SUPC_WUIR) Wake Up Input Enable 9 */
#define   SUPC_WUIR_WKUPEN9_NOT_ENABLE (0x0u << 9) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN9_ENABLE (0x1u << 9) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN10 (0x1u << 10) /**< \brief (SUPC_WUIR) Wake Up Input Enable 10 */
#define   SUPC_WUIR_WKUPEN10_NOT_ENABLE (0x0u << 10) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN10_ENABLE (0x1u << 10) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN11 (0x1u << 11) /**< \brief (SUPC_WUIR) Wake Up Input Enable 11 */
#define   SUPC_WUIR_WKUPEN11_NOT_ENABLE (0x0u << 11) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN11_ENABLE (0x1u << 11) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN12 (0x1u << 12) /**< \brief (SUPC_WUIR) Wake Up Input Enable 12 */
#define   SUPC_WUIR_WKUPEN12_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN12_ENABLE (0x1u << 12) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN13 (0x1u << 13) /**< \brief (SUPC_WUIR) Wake Up Input Enable 13 */
#define   SUPC_WUIR_WKUPEN13_NOT_ENABLE (0x0u << 13) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN13_ENABLE (0x1u << 13) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN14 (0x1u << 14) /**< \brief (SUPC_WUIR) Wake Up Input Enable 14 */
#define   SUPC_WUIR_WKUPEN14_NOT_ENABLE (0x0u << 14) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN14_ENABLE (0x1u << 14) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN15 (0x1u << 15) /**< \brief (SUPC_WUIR) Wake Up Input Enable 15 */
#define   SUPC_WUIR_WKUPEN15_NOT_ENABLE (0x0u << 15) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN15_ENABLE (0x1u << 15) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT0 (0x1u << 16) /**< \brief (SUPC_WUIR) Wake Up Input Transition 0 */
#define   SUPC_WUIR_WKUPT0_HIGH_TO_LOW (0x0u << 16) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT0_LOW_TO_HIGH (0x1u << 16) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT1 (0x1u << 17) /**< \brief (SUPC_WUIR) Wake Up Input Transition 1 */
#define   SUPC_WUIR_WKUPT1_HIGH_TO_LOW (0x0u << 17) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT1_LOW_TO_HIGH (0x1u << 17) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT2 (0x1u << 18) /**< \brief (SUPC_WUIR) Wake Up Input Transition 2 */
#define   SUPC_WUIR_WKUPT2_HIGH_TO_LOW (0x0u << 18) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT2_LOW_TO_HIGH (0x1u << 18) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT3 (0x1u << 19) /**< \brief (SUPC_WUIR) Wake Up Input Transition 3 */
#define   SUPC_WUIR_WKUPT3_HIGH_TO_LOW (0x0u << 19) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT3_LOW_TO_HIGH (0x1u << 19) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT4 (0x1u << 20) /**< \brief (SUPC_WUIR) Wake Up Input Transition 4 */
#define   SUPC_WUIR_WKUPT4_HIGH_TO_LOW (0x0u << 20) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT4_LOW_TO_HIGH (0x1u << 20) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT5 (0x1u << 21) /**< \brief (SUPC_WUIR) Wake Up Input Transition 5 */
#define   SUPC_WUIR_WKUPT5_HIGH_TO_LOW (0x0u << 21) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT5_LOW_TO_HIGH (0x1u << 21) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT6 (0x1u << 22) /**< \brief (SUPC_WUIR) Wake Up Input Transition 6 */
#define   SUPC_WUIR_WKUPT6_HIGH_TO_LOW (0x0u << 22) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT6_LOW_TO_HIGH (0x1u << 22) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT7 (0x1u << 23) /**< \brief (SUPC_WUIR) Wake Up Input Transition 7 */
#define   SUPC_WUIR_WKUPT7_HIGH_TO_LOW (0x0u << 23) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT7_LOW_TO_HIGH (0x1u << 23) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT8 (0x1u << 24) /**< \brief (SUPC_WUIR) Wake Up Input Transition 8 */
#define   SUPC_WUIR_WKUPT8_HIGH_TO_LOW (0x0u << 24) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT8_LOW_TO_HIGH (0x1u << 24) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT9 (0x1u << 25) /**< \brief (SUPC_WUIR) Wake Up Input Transition 9 */
#define   SUPC_WUIR_WKUPT9_HIGH_TO_LOW (0x0u << 25) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT9_LOW_TO_HIGH (0x1u << 25) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT10 (0x1u << 26) /**< \brief (SUPC_WUIR) Wake Up Input Transition 10 */
#define   SUPC_WUIR_WKUPT10_HIGH_TO_LOW (0x0u << 26) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT10_LOW_TO_HIGH (0x1u << 26) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT11 (0x1u << 27) /**< \brief (SUPC_WUIR) Wake Up Input Transition 11 */
#define   SUPC_WUIR_WKUPT11_HIGH_TO_LOW (0x0u << 27) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT11_LOW_TO_HIGH (0x1u << 27) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT12 (0x1u << 28) /**< \brief (SUPC_WUIR) Wake Up Input Transition 12 */
#define   SUPC_WUIR_WKUPT12_HIGH_TO_LOW (0x0u << 28) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT12_LOW_TO_HIGH (0x1u << 28) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT13 (0x1u << 29) /**< \brief (SUPC_WUIR) Wake Up Input Transition 13 */
#define   SUPC_WUIR_WKUPT13_HIGH_TO_LOW (0x0u << 29) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT13_LOW_TO_HIGH (0x1u << 29) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT14 (0x1u << 30) /**< \brief (SUPC_WUIR) Wake Up Input Transition 14 */
#define   SUPC_WUIR_WKUPT14_HIGH_TO_LOW (0x0u << 30) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT14_LOW_TO_HIGH (0x1u << 30) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT15 (0x1u << 31) /**< \brief (SUPC_WUIR) Wake Up Input Transition 15 */
#define   SUPC_WUIR_WKUPT15_HIGH_TO_LOW (0x0u << 31) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT15_LOW_TO_HIGH (0x1u << 31) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
/* -------- SUPC_SR : (SUPC Offset: 0x14) Supply Controller Status Register -------- */
#define SUPC_SR_FWUPS (0x1u << 0) /**< \brief (SUPC_SR) FWUP Wake Up Status */
#define   SUPC_SR_FWUPS_NO (0x0u << 0) /**< \brief (SUPC_SR) no wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR. */
#define   SUPC_SR_FWUPS_PRESENT (0x1u << 0) /**< \brief (SUPC_SR) at least one wake up due to the assertion of the FWUP pin has occurred since the last read of SUPC_SR. */
#define SUPC_SR_WKUPS (0x1u << 1) /**< \brief (SUPC_SR) WKUP Wake Up Status */
#define   SUPC_SR_WKUPS_NO (0x0u << 1) /**< \brief (SUPC_SR) no wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR. */
#define   SUPC_SR_WKUPS_PRESENT (0x1u << 1) /**< \brief (SUPC_SR) at least one wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR. */
#define SUPC_SR_SMWS (0x1u << 2) /**< \brief (SUPC_SR) Supply Monitor Detection Wake Up Status */
#define   SUPC_SR_SMWS_NO (0x0u << 2) /**< \brief (SUPC_SR) no wake up due to a supply monitor detection has occurred since the last read of SUPC_SR. */
#define   SUPC_SR_SMWS_PRESENT (0x1u << 2) /**< \brief (SUPC_SR) at least one wake up due to a supply monitor detection has occurred since the last read of SUPC_SR. */
#define SUPC_SR_BODRSTS (0x1u << 3) /**< \brief (SUPC_SR) Brownout Detector Reset Status */
#define   SUPC_SR_BODRSTS_NO (0x0u << 3) /**< \brief (SUPC_SR) no core brownout rising edge event has been detected since the last read of the SUPC_SR. */
#define   SUPC_SR_BODRSTS_PRESENT (0x1u << 3) /**< \brief (SUPC_SR) at least one brownout output rising edge event has been detected since the last read of the SUPC_SR. */
#define SUPC_SR_SMRSTS (0x1u << 4) /**< \brief (SUPC_SR) Supply Monitor Reset Status */
#define   SUPC_SR_SMRSTS_NO (0x0u << 4) /**< \brief (SUPC_SR) no supply monitor detection has generated a core reset since the last read of the SUPC_SR. */
#define   SUPC_SR_SMRSTS_PRESENT (0x1u << 4) /**< \brief (SUPC_SR) at least one supply monitor detection has generated a core reset since the last read of the SUPC_SR. */
#define SUPC_SR_SMS (0x1u << 5) /**< \brief (SUPC_SR) Supply Monitor Status */
#define   SUPC_SR_SMS_NO (0x0u << 5) /**< \brief (SUPC_SR) no supply monitor detection since the last read of SUPC_SR. */
#define   SUPC_SR_SMS_PRESENT (0x1u << 5) /**< \brief (SUPC_SR) at least one supply monitor detection since the last read of SUPC_SR. */
#define SUPC_SR_SMOS (0x1u << 6) /**< \brief (SUPC_SR) Supply Monitor Output Status */
#define   SUPC_SR_SMOS_HIGH (0x0u << 6) /**< \brief (SUPC_SR) the supply monitor detected VDDUTMI higher than its threshold at its last measurement. */
#define   SUPC_SR_SMOS_LOW (0x1u << 6) /**< \brief (SUPC_SR) the supply monitor detected VDDUTMI lower than its threshold at its last measurement. */
#define SUPC_SR_OSCSEL (0x1u << 7) /**< \brief (SUPC_SR) 32-kHz Oscillator Selection Status */
#define   SUPC_SR_OSCSEL_RC (0x0u << 7) /**< \brief (SUPC_SR) the slow clock, SLCK is generated by the embedded 32-kHz RC oscillator. */
#define   SUPC_SR_OSCSEL_CRYST (0x1u << 7) /**< \brief (SUPC_SR) the slow clock, SLCK is generated by the 32-kHz crystal oscillator. */
#define SUPC_SR_FWUPIS (0x1u << 12) /**< \brief (SUPC_SR) FWUP Input Status */
#define   SUPC_SR_FWUPIS_LOW (0x0u << 12) /**< \brief (SUPC_SR) FWUP input is tied low. */
#define   SUPC_SR_FWUPIS_HIGH (0x1u << 12) /**< \brief (SUPC_SR) FWUP input is tied high. */
#define SUPC_SR_WKUPIS0 (0x1u << 16) /**< \brief (SUPC_SR) WKUP Input Status 0 */
#define   SUPC_SR_WKUPIS0_DIS (0x0u << 16) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS0_EN (0x1u << 16) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS1 (0x1u << 17) /**< \brief (SUPC_SR) WKUP Input Status 1 */
#define   SUPC_SR_WKUPIS1_DIS (0x0u << 17) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS1_EN (0x1u << 17) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS2 (0x1u << 18) /**< \brief (SUPC_SR) WKUP Input Status 2 */
#define   SUPC_SR_WKUPIS2_DIS (0x0u << 18) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS2_EN (0x1u << 18) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS3 (0x1u << 19) /**< \brief (SUPC_SR) WKUP Input Status 3 */
#define   SUPC_SR_WKUPIS3_DIS (0x0u << 19) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS3_EN (0x1u << 19) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS4 (0x1u << 20) /**< \brief (SUPC_SR) WKUP Input Status 4 */
#define   SUPC_SR_WKUPIS4_DIS (0x0u << 20) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS4_EN (0x1u << 20) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS5 (0x1u << 21) /**< \brief (SUPC_SR) WKUP Input Status 5 */
#define   SUPC_SR_WKUPIS5_DIS (0x0u << 21) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS5_EN (0x1u << 21) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS6 (0x1u << 22) /**< \brief (SUPC_SR) WKUP Input Status 6 */
#define   SUPC_SR_WKUPIS6_DIS (0x0u << 22) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS6_EN (0x1u << 22) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS7 (0x1u << 23) /**< \brief (SUPC_SR) WKUP Input Status 7 */
#define   SUPC_SR_WKUPIS7_DIS (0x0u << 23) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS7_EN (0x1u << 23) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS8 (0x1u << 24) /**< \brief (SUPC_SR) WKUP Input Status 8 */
#define   SUPC_SR_WKUPIS8_DIS (0x0u << 24) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS8_EN (0x1u << 24) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS9 (0x1u << 25) /**< \brief (SUPC_SR) WKUP Input Status 9 */
#define   SUPC_SR_WKUPIS9_DIS (0x0u << 25) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS9_EN (0x1u << 25) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS10 (0x1u << 26) /**< \brief (SUPC_SR) WKUP Input Status 10 */
#define   SUPC_SR_WKUPIS10_DIS (0x0u << 26) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS10_EN (0x1u << 26) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS11 (0x1u << 27) /**< \brief (SUPC_SR) WKUP Input Status 11 */
#define   SUPC_SR_WKUPIS11_DIS (0x0u << 27) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS11_EN (0x1u << 27) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS12 (0x1u << 28) /**< \brief (SUPC_SR) WKUP Input Status 12 */
#define   SUPC_SR_WKUPIS12_DIS (0x0u << 28) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS12_EN (0x1u << 28) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS13 (0x1u << 29) /**< \brief (SUPC_SR) WKUP Input Status 13 */
#define   SUPC_SR_WKUPIS13_DIS (0x0u << 29) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS13_EN (0x1u << 29) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS14 (0x1u << 30) /**< \brief (SUPC_SR) WKUP Input Status 14 */
#define   SUPC_SR_WKUPIS14_DIS (0x0u << 30) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS14_EN (0x1u << 30) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS15 (0x1u << 31) /**< \brief (SUPC_SR) WKUP Input Status 15 */
#define   SUPC_SR_WKUPIS15_DIS (0x0u << 31) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS15_EN (0x1u << 31) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
/*@}*/ /* end of group SAM3XA_SUPC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Timer Counter */
/* ============================================================================= */
/** \addtogroup SAM3XA_TC Timer Counter */
/*@{*/

/** \brief TcChannel hardware registers */
typedef struct {
  RwReg      TC_CCR;        /**< \brief (TcChannel Offset: 0x0) Channel Control Register */
  RwReg      TC_CMR;        /**< \brief (TcChannel Offset: 0x4) Channel Mode Register */
  RwReg      TC_SMMR;       /**< \brief (TcChannel Offset: 0x8) Stepper Motor Mode Register */
  RoReg      Reserved1[1];
  RwReg      TC_CV;         /**< \brief (TcChannel Offset: 0x10) Counter Value */
  RwReg      TC_RA;         /**< \brief (TcChannel Offset: 0x14) Register A */
  RwReg      TC_RB;         /**< \brief (TcChannel Offset: 0x18) Register B */
  RwReg      TC_RC;         /**< \brief (TcChannel Offset: 0x1C) Register C */
  RwReg      TC_SR;         /**< \brief (TcChannel Offset: 0x20) Status Register */
  RwReg      TC_IER;        /**< \brief (TcChannel Offset: 0x24) Interrupt Enable Register */
  RwReg      TC_IDR;        /**< \brief (TcChannel Offset: 0x28) Interrupt Disable Register */
  RwReg      TC_IMR;        /**< \brief (TcChannel Offset: 0x2C) Interrupt Mask Register */
  RoReg      Reserved2[4];
} TcChannel;
/** \brief Tc hardware registers */
#define TCCHANNEL_NUMBER 3
typedef struct {
  TcChannel  TC_CHANNEL[TCCHANNEL_NUMBER]; /**< \brief (Tc Offset: 0x0) channel = 0 .. 2 */
  WoReg      TC_BCR;        /**< \brief (Tc Offset: 0xC0) Block Control Register */
  RwReg      TC_BMR;        /**< \brief (Tc Offset: 0xC4) Block Mode Register */
  WoReg      TC_QIER;       /**< \brief (Tc Offset: 0xC8) QDEC Interrupt Enable Register */
  WoReg      TC_QIDR;       /**< \brief (Tc Offset: 0xCC) QDEC Interrupt Disable Register */
  RoReg      TC_QIMR;       /**< \brief (Tc Offset: 0xD0) QDEC Interrupt Mask Register */
  RoReg      TC_QISR;       /**< \brief (Tc Offset: 0xD4) QDEC Interrupt Status Register */
  RwReg      TC_FMR;        /**< \brief (Tc Offset: 0xD8) Fault Mode Register */
  RoReg      Reserved1[2];
  RwReg      TC_WPMR;       /**< \brief (Tc Offset: 0xE4) Write Protect Mode Register */
} Tc;
/* -------- TC_CCR : (TC Offset: N/A) Channel Control Register -------- */
#define TC_CCR_CLKEN (0x1u << 0) /**< \brief (TC_CCR) Counter Clock Enable Command */
#define TC_CCR_CLKDIS (0x1u << 1) /**< \brief (TC_CCR) Counter Clock Disable Command */
#define TC_CCR_SWTRG (0x1u << 2) /**< \brief (TC_CCR) Software Trigger Command */
/* -------- TC_CMR : (TC Offset: N/A) Channel Mode Register -------- */
#define TC_CMR_TCCLKS_Pos 0
#define TC_CMR_TCCLKS_Msk (0x7u << TC_CMR_TCCLKS_Pos) /**< \brief (TC_CMR) Clock Selection */
#define   TC_CMR_TCCLKS_TIMER_CLOCK1 (0x0u << 0) /**< \brief (TC_CMR) Clock selected: TCLK1 */
#define   TC_CMR_TCCLKS_TIMER_CLOCK2 (0x1u << 0) /**< \brief (TC_CMR) Clock selected: TCLK2 */
#define   TC_CMR_TCCLKS_TIMER_CLOCK3 (0x2u << 0) /**< \brief (TC_CMR) Clock selected: TCLK3 */
#define   TC_CMR_TCCLKS_TIMER_CLOCK4 (0x3u << 0) /**< \brief (TC_CMR) Clock selected: TCLK4 */
#define   TC_CMR_TCCLKS_TIMER_CLOCK5 (0x4u << 0) /**< \brief (TC_CMR) Clock selected: TCLK5 */
#define   TC_CMR_TCCLKS_XC0 (0x5u << 0) /**< \brief (TC_CMR) Clock selected: XC0 */
#define   TC_CMR_TCCLKS_XC1 (0x6u << 0) /**< \brief (TC_CMR) Clock selected: XC1 */
#define   TC_CMR_TCCLKS_XC2 (0x7u << 0) /**< \brief (TC_CMR) Clock selected: XC2 */
#define TC_CMR_CLKI (0x1u << 3) /**< \brief (TC_CMR) Clock Invert */
#define TC_CMR_BURST_Pos 4
#define TC_CMR_BURST_Msk (0x3u << TC_CMR_BURST_Pos) /**< \brief (TC_CMR) Burst Signal Selection */
#define   TC_CMR_BURST_NONE (0x0u << 4) /**< \brief (TC_CMR) The clock is not gated by an external signal. */
#define   TC_CMR_BURST_XC0 (0x1u << 4) /**< \brief (TC_CMR) XC0 is ANDed with the selected clock. */
#define   TC_CMR_BURST_XC1 (0x2u << 4) /**< \brief (TC_CMR) XC1 is ANDed with the selected clock. */
#define   TC_CMR_BURST_XC2 (0x3u << 4) /**< \brief (TC_CMR) XC2 is ANDed with the selected clock. */
#define TC_CMR_LDBSTOP (0x1u << 6) /**< \brief (TC_CMR) Counter Clock Stopped with RB Loading */
#define TC_CMR_LDBDIS (0x1u << 7) /**< \brief (TC_CMR) Counter Clock Disable with RB Loading */
#define TC_CMR_ETRGEDG_Pos 8
#define TC_CMR_ETRGEDG_Msk (0x3u << TC_CMR_ETRGEDG_Pos) /**< \brief (TC_CMR) External Trigger Edge Selection */
#define   TC_CMR_ETRGEDG_NONE (0x0u << 8) /**< \brief (TC_CMR) The clock is not gated by an external signal. */
#define   TC_CMR_ETRGEDG_RISING (0x1u << 8) /**< \brief (TC_CMR) Rising edge */
#define   TC_CMR_ETRGEDG_FALLING (0x2u << 8) /**< \brief (TC_CMR) Falling edge */
#define   TC_CMR_ETRGEDG_EDGE (0x3u << 8) /**< \brief (TC_CMR) Each edge */
#define TC_CMR_ABETRG (0x1u << 10) /**< \brief (TC_CMR) TIOA or TIOB External Trigger Selection */
#define TC_CMR_CPCTRG (0x1u << 14) /**< \brief (TC_CMR) RC Compare Trigger Enable */
#define TC_CMR_WAVE (0x1u << 15) /**< \brief (TC_CMR) Waveform Mode */
#define TC_CMR_LDRA_Pos 16
#define TC_CMR_LDRA_Msk (0x3u << TC_CMR_LDRA_Pos) /**< \brief (TC_CMR) RA Loading Edge Selection */
#define   TC_CMR_LDRA_NONE (0x0u << 16) /**< \brief (TC_CMR) None */
#define   TC_CMR_LDRA_RISING (0x1u << 16) /**< \brief (TC_CMR) Rising edge of TIOA */
#define   TC_CMR_LDRA_FALLING (0x2u << 16) /**< \brief (TC_CMR) Falling edge of TIOA */
#define   TC_CMR_LDRA_EDGE (0x3u << 16) /**< \brief (TC_CMR) Each edge of TIOA */
#define TC_CMR_LDRB_Pos 18
#define TC_CMR_LDRB_Msk (0x3u << TC_CMR_LDRB_Pos) /**< \brief (TC_CMR) RB Loading Edge Selection */
#define   TC_CMR_LDRB_NONE (0x0u << 18) /**< \brief (TC_CMR) None */
#define   TC_CMR_LDRB_RISING (0x1u << 18) /**< \brief (TC_CMR) Rising edge of TIOA */
#define   TC_CMR_LDRB_FALLING (0x2u << 18) /**< \brief (TC_CMR) Falling edge of TIOA */
#define   TC_CMR_LDRB_EDGE (0x3u << 18) /**< \brief (TC_CMR) Each edge of TIOA */
#define TC_CMR_CPCSTOP (0x1u << 6) /**< \brief (TC_CMR) Counter Clock Stopped with RC Compare */
#define TC_CMR_CPCDIS (0x1u << 7) /**< \brief (TC_CMR) Counter Clock Disable with RC Compare */
#define TC_CMR_EEVTEDG_Pos 8
#define TC_CMR_EEVTEDG_Msk (0x3u << TC_CMR_EEVTEDG_Pos) /**< \brief (TC_CMR) External Event Edge Selection */
#define   TC_CMR_EEVTEDG_NONE (0x0u << 8) /**< \brief (TC_CMR) None */
#define   TC_CMR_EEVTEDG_RISING (0x1u << 8) /**< \brief (TC_CMR) Rising edge */
#define   TC_CMR_EEVTEDG_FALLING (0x2u << 8) /**< \brief (TC_CMR) Falling edge */
#define   TC_CMR_EEVTEDG_EDGE (0x3u << 8) /**< \brief (TC_CMR) Each edge */
#define TC_CMR_EEVT_Pos 10
#define TC_CMR_EEVT_Msk (0x3u << TC_CMR_EEVT_Pos) /**< \brief (TC_CMR) External Event Selection */
#define   TC_CMR_EEVT_TIOB (0x0u << 10) /**< \brief (TC_CMR) TIOB */
#define   TC_CMR_EEVT_XC0 (0x1u << 10) /**< \brief (TC_CMR) XC0 */
#define   TC_CMR_EEVT_XC1 (0x2u << 10) /**< \brief (TC_CMR) XC1 */
#define   TC_CMR_EEVT_XC2 (0x3u << 10) /**< \brief (TC_CMR) XC2 */
#define TC_CMR_ENETRG (0x1u << 12) /**< \brief (TC_CMR) External Event Trigger Enable */
#define TC_CMR_WAVSEL_Pos 13
#define TC_CMR_WAVSEL_Msk (0x3u << TC_CMR_WAVSEL_Pos) /**< \brief (TC_CMR) Waveform Selection */
#define   TC_CMR_WAVSEL_UP (0x0u << 13) /**< \brief (TC_CMR) UP mode without automatic trigger on RC Compare */
#define   TC_CMR_WAVSEL_UPDOWN (0x1u << 13) /**< \brief (TC_CMR) UPDOWN mode without automatic trigger on RC Compare */
#define   TC_CMR_WAVSEL_UP_RC (0x2u << 13) /**< \brief (TC_CMR) UP mode with automatic trigger on RC Compare */
#define   TC_CMR_WAVSEL_UPDOWN_RC (0x3u << 13) /**< \brief (TC_CMR) UPDOWN mode with automatic trigger on RC Compare */
#define TC_CMR_ACPA_Pos 16
#define TC_CMR_ACPA_Msk (0x3u << TC_CMR_ACPA_Pos) /**< \brief (TC_CMR) RA Compare Effect on TIOA */
#define   TC_CMR_ACPA_NONE (0x0u << 16) /**< \brief (TC_CMR) None */
#define   TC_CMR_ACPA_SET (0x1u << 16) /**< \brief (TC_CMR) Set */
#define   TC_CMR_ACPA_CLEAR (0x2u << 16) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_ACPA_TOGGLE (0x3u << 16) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_ACPC_Pos 18
#define TC_CMR_ACPC_Msk (0x3u << TC_CMR_ACPC_Pos) /**< \brief (TC_CMR) RC Compare Effect on TIOA */
#define   TC_CMR_ACPC_NONE (0x0u << 18) /**< \brief (TC_CMR) None */
#define   TC_CMR_ACPC_SET (0x1u << 18) /**< \brief (TC_CMR) Set */
#define   TC_CMR_ACPC_CLEAR (0x2u << 18) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_ACPC_TOGGLE (0x3u << 18) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_AEEVT_Pos 20
#define TC_CMR_AEEVT_Msk (0x3u << TC_CMR_AEEVT_Pos) /**< \brief (TC_CMR) External Event Effect on TIOA */
#define   TC_CMR_AEEVT_NONE (0x0u << 20) /**< \brief (TC_CMR) None */
#define   TC_CMR_AEEVT_SET (0x1u << 20) /**< \brief (TC_CMR) Set */
#define   TC_CMR_AEEVT_CLEAR (0x2u << 20) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_AEEVT_TOGGLE (0x3u << 20) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_ASWTRG_Pos 22
#define TC_CMR_ASWTRG_Msk (0x3u << TC_CMR_ASWTRG_Pos) /**< \brief (TC_CMR) Software Trigger Effect on TIOA */
#define   TC_CMR_ASWTRG_NONE (0x0u << 22) /**< \brief (TC_CMR) None */
#define   TC_CMR_ASWTRG_SET (0x1u << 22) /**< \brief (TC_CMR) Set */
#define   TC_CMR_ASWTRG_CLEAR (0x2u << 22) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_ASWTRG_TOGGLE (0x3u << 22) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_BCPB_Pos 24
#define TC_CMR_BCPB_Msk (0x3u << TC_CMR_BCPB_Pos) /**< \brief (TC_CMR) RB Compare Effect on TIOB */
#define   TC_CMR_BCPB_NONE (0x0u << 24) /**< \brief (TC_CMR) None */
#define   TC_CMR_BCPB_SET (0x1u << 24) /**< \brief (TC_CMR) Set */
#define   TC_CMR_BCPB_CLEAR (0x2u << 24) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_BCPB_TOGGLE (0x3u << 24) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_BCPC_Pos 26
#define TC_CMR_BCPC_Msk (0x3u << TC_CMR_BCPC_Pos) /**< \brief (TC_CMR) RC Compare Effect on TIOB */
#define   TC_CMR_BCPC_NONE (0x0u << 26) /**< \brief (TC_CMR) None */
#define   TC_CMR_BCPC_SET (0x1u << 26) /**< \brief (TC_CMR) Set */
#define   TC_CMR_BCPC_CLEAR (0x2u << 26) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_BCPC_TOGGLE (0x3u << 26) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_BEEVT_Pos 28
#define TC_CMR_BEEVT_Msk (0x3u << TC_CMR_BEEVT_Pos) /**< \brief (TC_CMR) External Event Effect on TIOB */
#define   TC_CMR_BEEVT_NONE (0x0u << 28) /**< \brief (TC_CMR) None */
#define   TC_CMR_BEEVT_SET (0x1u << 28) /**< \brief (TC_CMR) Set */
#define   TC_CMR_BEEVT_CLEAR (0x2u << 28) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_BEEVT_TOGGLE (0x3u << 28) /**< \brief (TC_CMR) Toggle */
#define TC_CMR_BSWTRG_Pos 30
#define TC_CMR_BSWTRG_Msk (0x3u << TC_CMR_BSWTRG_Pos) /**< \brief (TC_CMR) Software Trigger Effect on TIOB */
#define   TC_CMR_BSWTRG_NONE (0x0u << 30) /**< \brief (TC_CMR) None */
#define   TC_CMR_BSWTRG_SET (0x1u << 30) /**< \brief (TC_CMR) Set */
#define   TC_CMR_BSWTRG_CLEAR (0x2u << 30) /**< \brief (TC_CMR) Clear */
#define   TC_CMR_BSWTRG_TOGGLE (0x3u << 30) /**< \brief (TC_CMR) Toggle */
/* -------- TC_SMMR : (TC Offset: N/A) Stepper Motor Mode Register -------- */
#define TC_SMMR_GCEN (0x1u << 0) /**< \brief (TC_SMMR) Gray Count Enable */
#define TC_SMMR_DOWN (0x1u << 1) /**< \brief (TC_SMMR) DOWN Count */
/* -------- TC_CV : (TC Offset: N/A) Counter Value -------- */
#define TC_CV_CV_Pos 0
#define TC_CV_CV_Msk (0xffffffffu << TC_CV_CV_Pos) /**< \brief (TC_CV) Counter Value */
/* -------- TC_RA : (TC Offset: N/A) Register A -------- */
#define TC_RA_RA_Pos 0
#define TC_RA_RA_Msk (0xffffffffu << TC_RA_RA_Pos) /**< \brief (TC_RA) Register A */
#define TC_RA_RA(value) ((TC_RA_RA_Msk & ((value) << TC_RA_RA_Pos)))
/* -------- TC_RB : (TC Offset: N/A) Register B -------- */
#define TC_RB_RB_Pos 0
#define TC_RB_RB_Msk (0xffffffffu << TC_RB_RB_Pos) /**< \brief (TC_RB) Register B */
#define TC_RB_RB(value) ((TC_RB_RB_Msk & ((value) << TC_RB_RB_Pos)))
/* -------- TC_RC : (TC Offset: N/A) Register C -------- */
#define TC_RC_RC_Pos 0
#define TC_RC_RC_Msk (0xffffffffu << TC_RC_RC_Pos) /**< \brief (TC_RC) Register C */
#define TC_RC_RC(value) ((TC_RC_RC_Msk & ((value) << TC_RC_RC_Pos)))
/* -------- TC_SR : (TC Offset: N/A) Status Register -------- */
#define TC_SR_COVFS (0x1u << 0) /**< \brief (TC_SR) Counter Overflow Status */
#define TC_SR_LOVRS (0x1u << 1) /**< \brief (TC_SR) Load Overrun Status */
#define TC_SR_CPAS (0x1u << 2) /**< \brief (TC_SR) RA Compare Status */
#define TC_SR_CPBS (0x1u << 3) /**< \brief (TC_SR) RB Compare Status */
#define TC_SR_CPCS (0x1u << 4) /**< \brief (TC_SR) RC Compare Status */
#define TC_SR_LDRAS (0x1u << 5) /**< \brief (TC_SR) RA Loading Status */
#define TC_SR_LDRBS (0x1u << 6) /**< \brief (TC_SR) RB Loading Status */
#define TC_SR_ETRGS (0x1u << 7) /**< \brief (TC_SR) External Trigger Status */
#define TC_SR_CLKSTA (0x1u << 16) /**< \brief (TC_SR) Clock Enabling Status */
#define TC_SR_MTIOA (0x1u << 17) /**< \brief (TC_SR) TIOA Mirror */
#define TC_SR_MTIOB (0x1u << 18) /**< \brief (TC_SR) TIOB Mirror */
/* -------- TC_IER : (TC Offset: N/A) Interrupt Enable Register -------- */
#define TC_IER_COVFS (0x1u << 0) /**< \brief (TC_IER) Counter Overflow */
#define TC_IER_LOVRS (0x1u << 1) /**< \brief (TC_IER) Load Overrun */
#define TC_IER_CPAS (0x1u << 2) /**< \brief (TC_IER) RA Compare */
#define TC_IER_CPBS (0x1u << 3) /**< \brief (TC_IER) RB Compare */
#define TC_IER_CPCS (0x1u << 4) /**< \brief (TC_IER) RC Compare */
#define TC_IER_LDRAS (0x1u << 5) /**< \brief (TC_IER) RA Loading */
#define TC_IER_LDRBS (0x1u << 6) /**< \brief (TC_IER) RB Loading */
#define TC_IER_ETRGS (0x1u << 7) /**< \brief (TC_IER) External Trigger */
/* -------- TC_IDR : (TC Offset: N/A) Interrupt Disable Register -------- */
#define TC_IDR_COVFS (0x1u << 0) /**< \brief (TC_IDR) Counter Overflow */
#define TC_IDR_LOVRS (0x1u << 1) /**< \brief (TC_IDR) Load Overrun */
#define TC_IDR_CPAS (0x1u << 2) /**< \brief (TC_IDR) RA Compare */
#define TC_IDR_CPBS (0x1u << 3) /**< \brief (TC_IDR) RB Compare */
#define TC_IDR_CPCS (0x1u << 4) /**< \brief (TC_IDR) RC Compare */
#define TC_IDR_LDRAS (0x1u << 5) /**< \brief (TC_IDR) RA Loading */
#define TC_IDR_LDRBS (0x1u << 6) /**< \brief (TC_IDR) RB Loading */
#define TC_IDR_ETRGS (0x1u << 7) /**< \brief (TC_IDR) External Trigger */
/* -------- TC_IMR : (TC Offset: N/A) Interrupt Mask Register -------- */
#define TC_IMR_COVFS (0x1u << 0) /**< \brief (TC_IMR) Counter Overflow */
#define TC_IMR_LOVRS (0x1u << 1) /**< \brief (TC_IMR) Load Overrun */
#define TC_IMR_CPAS (0x1u << 2) /**< \brief (TC_IMR) RA Compare */
#define TC_IMR_CPBS (0x1u << 3) /**< \brief (TC_IMR) RB Compare */
#define TC_IMR_CPCS (0x1u << 4) /**< \brief (TC_IMR) RC Compare */
#define TC_IMR_LDRAS (0x1u << 5) /**< \brief (TC_IMR) RA Loading */
#define TC_IMR_LDRBS (0x1u << 6) /**< \brief (TC_IMR) RB Loading */
#define TC_IMR_ETRGS (0x1u << 7) /**< \brief (TC_IMR) External Trigger */
/* -------- TC_BCR : (TC Offset: 0xC0) Block Control Register -------- */
#define TC_BCR_SYNC (0x1u << 0) /**< \brief (TC_BCR) Synchro Command */
/* -------- TC_BMR : (TC Offset: 0xC4) Block Mode Register -------- */
#define TC_BMR_TC0XC0S_Pos 0
#define TC_BMR_TC0XC0S_Msk (0x3u << TC_BMR_TC0XC0S_Pos) /**< \brief (TC_BMR) External Clock Signal 0 Selection */
#define   TC_BMR_TC0XC0S_TCLK0 (0x0u << 0) /**< \brief (TC_BMR) Signal connected to XC0: TCLK0 */
#define   TC_BMR_TC0XC0S_TIOA1 (0x2u << 0) /**< \brief (TC_BMR) Signal connected to XC0: TIOA1 */
#define   TC_BMR_TC0XC0S_TIOA2 (0x3u << 0) /**< \brief (TC_BMR) Signal connected to XC0: TIOA2 */
#define TC_BMR_TC1XC1S_Pos 2
#define TC_BMR_TC1XC1S_Msk (0x3u << TC_BMR_TC1XC1S_Pos) /**< \brief (TC_BMR) External Clock Signal 1 Selection */
#define   TC_BMR_TC1XC1S_TCLK1 (0x0u << 2) /**< \brief (TC_BMR) Signal connected to XC1: TCLK1 */
#define   TC_BMR_TC1XC1S_TIOA0 (0x2u << 2) /**< \brief (TC_BMR) Signal connected to XC1: TIOA0 */
#define   TC_BMR_TC1XC1S_TIOA2 (0x3u << 2) /**< \brief (TC_BMR) Signal connected to XC1: TIOA2 */
#define TC_BMR_TC2XC2S_Pos 4
#define TC_BMR_TC2XC2S_Msk (0x3u << TC_BMR_TC2XC2S_Pos) /**< \brief (TC_BMR) External Clock Signal 2 Selection */
#define   TC_BMR_TC2XC2S_TCLK2 (0x0u << 4) /**< \brief (TC_BMR) Signal connected to XC2: TCLK2 */
#define   TC_BMR_TC2XC2S_TIOA1 (0x2u << 4) /**< \brief (TC_BMR) Signal connected to XC2: TIOA1 */
#define   TC_BMR_TC2XC2S_TIOA2 (0x3u << 4) /**< \brief (TC_BMR) Signal connected to XC2: TIOA2 */
#define TC_BMR_QDEN (0x1u << 8) /**< \brief (TC_BMR) Quadrature Decoder ENabled */
#define TC_BMR_POSEN (0x1u << 9) /**< \brief (TC_BMR) POSition ENabled */
#define TC_BMR_SPEEDEN (0x1u << 10) /**< \brief (TC_BMR) SPEED ENabled */
#define TC_BMR_QDTRANS (0x1u << 11) /**< \brief (TC_BMR) Quadrature Decoding TRANSparent */
#define TC_BMR_EDGPHA (0x1u << 12) /**< \brief (TC_BMR) EDGe on PHA count mode */
#define TC_BMR_INVA (0x1u << 13) /**< \brief (TC_BMR) INVerted phA */
#define TC_BMR_INVB (0x1u << 14) /**< \brief (TC_BMR) INVerted phB */
#define TC_BMR_INVIDX (0x1u << 15) /**< \brief (TC_BMR) INVerted InDeX */
#define TC_BMR_SWAP (0x1u << 16) /**< \brief (TC_BMR) SWAP PHA and PHB */
#define TC_BMR_IDXPHB (0x1u << 17) /**< \brief (TC_BMR) InDeX pin is PHB pin */
#define TC_BMR_FILTER (0x1u << 19) /**< \brief (TC_BMR)  */
#define TC_BMR_MAXFILT_Pos 20
#define TC_BMR_MAXFILT_Msk (0x3fu << TC_BMR_MAXFILT_Pos) /**< \brief (TC_BMR) MAXimum FILTer */
#define TC_BMR_MAXFILT(value) ((TC_BMR_MAXFILT_Msk & ((value) << TC_BMR_MAXFILT_Pos)))
/* -------- TC_QIER : (TC Offset: 0xC8) QDEC Interrupt Enable Register -------- */
#define TC_QIER_IDX (0x1u << 0) /**< \brief (TC_QIER) InDeX */
#define TC_QIER_DIRCHG (0x1u << 1) /**< \brief (TC_QIER) DIRection CHanGe */
#define TC_QIER_QERR (0x1u << 2) /**< \brief (TC_QIER) Quadrature ERRor */
/* -------- TC_QIDR : (TC Offset: 0xCC) QDEC Interrupt Disable Register -------- */
#define TC_QIDR_IDX (0x1u << 0) /**< \brief (TC_QIDR) InDeX */
#define TC_QIDR_DIRCHG (0x1u << 1) /**< \brief (TC_QIDR) DIRection CHanGe */
#define TC_QIDR_QERR (0x1u << 2) /**< \brief (TC_QIDR) Quadrature ERRor */
/* -------- TC_QIMR : (TC Offset: 0xD0) QDEC Interrupt Mask Register -------- */
#define TC_QIMR_IDX (0x1u << 0) /**< \brief (TC_QIMR) InDeX */
#define TC_QIMR_DIRCHG (0x1u << 1) /**< \brief (TC_QIMR) DIRection CHanGe */
#define TC_QIMR_QERR (0x1u << 2) /**< \brief (TC_QIMR) Quadrature ERRor */
/* -------- TC_QISR : (TC Offset: 0xD4) QDEC Interrupt Status Register -------- */
#define TC_QISR_IDX (0x1u << 0) /**< \brief (TC_QISR) InDeX */
#define TC_QISR_DIRCHG (0x1u << 1) /**< \brief (TC_QISR) DIRection CHanGe */
#define TC_QISR_QERR (0x1u << 2) /**< \brief (TC_QISR) Quadrature ERRor */
#define TC_QISR_DIR (0x1u << 8) /**< \brief (TC_QISR) Direction */
/* -------- TC_FMR : (TC Offset: 0xD8) Fault Mode Register -------- */
#define TC_FMR_ENCF0 (0x1u << 0) /**< \brief (TC_FMR) ENable Compare Fault Channel 0 */
#define TC_FMR_ENCF1 (0x1u << 1) /**< \brief (TC_FMR) ENable Compare Fault Channel 1 */
/* -------- TC_WPMR : (TC Offset: 0xE4) Write Protect Mode Register -------- */
#define TC_WPMR_WPEN (0x1u << 0) /**< \brief (TC_WPMR) Write Protect Enable */
#define TC_WPMR_WPKEY_Pos 8
#define TC_WPMR_WPKEY_Msk (0xffffffu << TC_WPMR_WPKEY_Pos) /**< \brief (TC_WPMR) Write Protect KEY */
#define TC_WPMR_WPKEY(value) ((TC_WPMR_WPKEY_Msk & ((value) << TC_WPMR_WPKEY_Pos)))
/*@}*/ /* end of group SAM3XA_TC */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR True Random Number Generator */
/* ============================================================================= */
/** \addtogroup SAM3XA_TRNG True Random Number Generator */
/*@{*/

/** \brief Trng hardware registers */
typedef struct {
  WoReg TRNG_CR;       /**< \brief (Trng Offset: 0x00) Control Register */
  RoReg Reserved1[3];
  WoReg TRNG_IER;      /**< \brief (Trng Offset: 0x10) Interrupt Enable Register */
  WoReg TRNG_IDR;      /**< \brief (Trng Offset: 0x14) Interrupt Disable Register */
  RoReg TRNG_IMR;      /**< \brief (Trng Offset: 0x18) Interrupt Mask Register */
  RoReg TRNG_ISR;      /**< \brief (Trng Offset: 0x1C) Interrupt Status Register */
  RoReg Reserved2[12];
  RoReg TRNG_ODATA;    /**< \brief (Trng Offset: 0x50) Output Data Register */
} Trng;
/* -------- TRNG_CR : (TRNG Offset: 0x00) Control Register -------- */
#define TRNG_CR_ENABLE (0x1u << 0) /**< \brief (TRNG_CR) Enables the TRNG to provide random values */
#define TRNG_CR_KEY_Pos 8
#define TRNG_CR_KEY_Msk (0xffffffu << TRNG_CR_KEY_Pos) /**< \brief (TRNG_CR) Security Key */
#define TRNG_CR_KEY(value) ((TRNG_CR_KEY_Msk & ((value) << TRNG_CR_KEY_Pos)))
/* -------- TRNG_IER : (TRNG Offset: 0x10) Interrupt Enable Register -------- */
#define TRNG_IER_DATRDY (0x1u << 0) /**< \brief (TRNG_IER) Data Ready Interrupt Enable */
/* -------- TRNG_IDR : (TRNG Offset: 0x14) Interrupt Disable Register -------- */
#define TRNG_IDR_DATRDY (0x1u << 0) /**< \brief (TRNG_IDR) Data Ready Interrupt Disable */
/* -------- TRNG_IMR : (TRNG Offset: 0x18) Interrupt Mask Register -------- */
#define TRNG_IMR_DATRDY (0x1u << 0) /**< \brief (TRNG_IMR) Data Ready Interrupt Mask */
/* -------- TRNG_ISR : (TRNG Offset: 0x1C) Interrupt Status Register -------- */
#define TRNG_ISR_DATRDY (0x1u << 0) /**< \brief (TRNG_ISR) Data Ready */
/* -------- TRNG_ODATA : (TRNG Offset: 0x50) Output Data Register -------- */
#define TRNG_ODATA_ODATA_Pos 0
#define TRNG_ODATA_ODATA_Msk (0xffffffffu << TRNG_ODATA_ODATA_Pos) /**< \brief (TRNG_ODATA) Output Data */
/*@}*/ /* end of group SAM3XA_TRNG */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Two-wire Interface */
/* ============================================================================= */
/** \addtogroup SAM3XA_TWI Two-wire Interface */
/*@{*/

/** \brief Twi hardware registers */
typedef struct {
  WoReg TWI_CR;        /**< \brief (Twi Offset: 0x00) Control Register */
  RwReg TWI_MMR;       /**< \brief (Twi Offset: 0x04) Master Mode Register */
  RwReg TWI_SMR;       /**< \brief (Twi Offset: 0x08) Slave Mode Register */
  RwReg TWI_IADR;      /**< \brief (Twi Offset: 0x0C) Internal Address Register */
  RwReg TWI_CWGR;      /**< \brief (Twi Offset: 0x10) Clock Waveform Generator Register */
  RoReg Reserved1[3];
  RoReg TWI_SR;        /**< \brief (Twi Offset: 0x20) Status Register */
  WoReg TWI_IER;       /**< \brief (Twi Offset: 0x24) Interrupt Enable Register */
  WoReg TWI_IDR;       /**< \brief (Twi Offset: 0x28) Interrupt Disable Register */
  RoReg TWI_IMR;       /**< \brief (Twi Offset: 0x2C) Interrupt Mask Register */
  RoReg TWI_RHR;       /**< \brief (Twi Offset: 0x30) Receive Holding Register */
  WoReg TWI_THR;       /**< \brief (Twi Offset: 0x34) Transmit Holding Register */
  RoReg Reserved2[50];
  RwReg TWI_RPR;       /**< \brief (Twi Offset: 0x100) Receive Pointer Register */
  RwReg TWI_RCR;       /**< \brief (Twi Offset: 0x104) Receive Counter Register */
  RwReg TWI_TPR;       /**< \brief (Twi Offset: 0x108) Transmit Pointer Register */
  RwReg TWI_TCR;       /**< \brief (Twi Offset: 0x10C) Transmit Counter Register */
  RwReg TWI_RNPR;      /**< \brief (Twi Offset: 0x110) Receive Next Pointer Register */
  RwReg TWI_RNCR;      /**< \brief (Twi Offset: 0x114) Receive Next Counter Register */
  RwReg TWI_TNPR;      /**< \brief (Twi Offset: 0x118) Transmit Next Pointer Register */
  RwReg TWI_TNCR;      /**< \brief (Twi Offset: 0x11C) Transmit Next Counter Register */
  WoReg TWI_PTCR;      /**< \brief (Twi Offset: 0x120) Transfer Control Register */
  RoReg TWI_PTSR;      /**< \brief (Twi Offset: 0x124) Transfer Status Register */
} Twi;
/* -------- TWI_CR : (TWI Offset: 0x00) Control Register -------- */
#define TWI_CR_START (0x1u << 0) /**< \brief (TWI_CR) Send a START Condition */
#define TWI_CR_STOP (0x1u << 1) /**< \brief (TWI_CR) Send a STOP Condition */
#define TWI_CR_MSEN (0x1u << 2) /**< \brief (TWI_CR) TWI Master Mode Enabled */
#define TWI_CR_MSDIS (0x1u << 3) /**< \brief (TWI_CR) TWI Master Mode Disabled */
#define TWI_CR_SVEN (0x1u << 4) /**< \brief (TWI_CR) TWI Slave Mode Enabled */
#define TWI_CR_SVDIS (0x1u << 5) /**< \brief (TWI_CR) TWI Slave Mode Disabled */
#define TWI_CR_QUICK (0x1u << 6) /**< \brief (TWI_CR) SMBUS Quick Command */
#define TWI_CR_SWRST (0x1u << 7) /**< \brief (TWI_CR) Software Reset */
/* -------- TWI_MMR : (TWI Offset: 0x04) Master Mode Register -------- */
#define TWI_MMR_IADRSZ_Pos 8
#define TWI_MMR_IADRSZ_Msk (0x3u << TWI_MMR_IADRSZ_Pos) /**< \brief (TWI_MMR) Internal Device Address Size */
#define   TWI_MMR_IADRSZ_NONE (0x0u << 8) /**< \brief (TWI_MMR) No internal device address */
#define   TWI_MMR_IADRSZ_1_BYTE (0x1u << 8) /**< \brief (TWI_MMR) One-byte internal device address */
#define   TWI_MMR_IADRSZ_2_BYTE (0x2u << 8) /**< \brief (TWI_MMR) Two-byte internal device address */
#define   TWI_MMR_IADRSZ_3_BYTE (0x3u << 8) /**< \brief (TWI_MMR) Three-byte internal device address */
#define TWI_MMR_MREAD (0x1u << 12) /**< \brief (TWI_MMR) Master Read Direction */
#define TWI_MMR_DADR_Pos 16
#define TWI_MMR_DADR_Msk (0x7fu << TWI_MMR_DADR_Pos) /**< \brief (TWI_MMR) Device Address */
#define TWI_MMR_DADR(value) ((TWI_MMR_DADR_Msk & ((value) << TWI_MMR_DADR_Pos)))
/* -------- TWI_SMR : (TWI Offset: 0x08) Slave Mode Register -------- */
#define TWI_SMR_SADR_Pos 16
#define TWI_SMR_SADR_Msk (0x7fu << TWI_SMR_SADR_Pos) /**< \brief (TWI_SMR) Slave Address */
#define TWI_SMR_SADR(value) ((TWI_SMR_SADR_Msk & ((value) << TWI_SMR_SADR_Pos)))
/* -------- TWI_IADR : (TWI Offset: 0x0C) Internal Address Register -------- */
#define TWI_IADR_IADR_Pos 0
#define TWI_IADR_IADR_Msk (0xffffffu << TWI_IADR_IADR_Pos) /**< \brief (TWI_IADR) Internal Address */
#define TWI_IADR_IADR(value) ((TWI_IADR_IADR_Msk & ((value) << TWI_IADR_IADR_Pos)))
/* -------- TWI_CWGR : (TWI Offset: 0x10) Clock Waveform Generator Register -------- */
#define TWI_CWGR_CLDIV_Pos 0
#define TWI_CWGR_CLDIV_Msk (0xffu << TWI_CWGR_CLDIV_Pos) /**< \brief (TWI_CWGR) Clock Low Divider */
#define TWI_CWGR_CLDIV(value) ((TWI_CWGR_CLDIV_Msk & ((value) << TWI_CWGR_CLDIV_Pos)))
#define TWI_CWGR_CHDIV_Pos 8
#define TWI_CWGR_CHDIV_Msk (0xffu << TWI_CWGR_CHDIV_Pos) /**< \brief (TWI_CWGR) Clock High Divider */
#define TWI_CWGR_CHDIV(value) ((TWI_CWGR_CHDIV_Msk & ((value) << TWI_CWGR_CHDIV_Pos)))
#define TWI_CWGR_CKDIV_Pos 16
#define TWI_CWGR_CKDIV_Msk (0x7u << TWI_CWGR_CKDIV_Pos) /**< \brief (TWI_CWGR) Clock Divider */
#define TWI_CWGR_CKDIV(value) ((TWI_CWGR_CKDIV_Msk & ((value) << TWI_CWGR_CKDIV_Pos)))
/* -------- TWI_SR : (TWI Offset: 0x20) Status Register -------- */
#define TWI_SR_TXCOMP (0x1u << 0) /**< \brief (TWI_SR) Transmission Completed (automatically set / reset) */
#define TWI_SR_RXRDY (0x1u << 1) /**< \brief (TWI_SR) Receive Holding Register Ready (automatically set / reset) */
#define TWI_SR_TXRDY (0x1u << 2) /**< \brief (TWI_SR) Transmit Holding Register Ready (automatically set / reset) */
#define TWI_SR_SVREAD (0x1u << 3) /**< \brief (TWI_SR) Slave Read (automatically set / reset) */
#define TWI_SR_SVACC (0x1u << 4) /**< \brief (TWI_SR) Slave Access (automatically set / reset) */
#define TWI_SR_GACC (0x1u << 5) /**< \brief (TWI_SR) General Call Access (clear on read) */
#define TWI_SR_OVRE (0x1u << 6) /**< \brief (TWI_SR) Overrun Error (clear on read) */
#define TWI_SR_NACK (0x1u << 8) /**< \brief (TWI_SR) Not Acknowledged (clear on read) */
#define TWI_SR_ARBLST (0x1u << 9) /**< \brief (TWI_SR) Arbitration Lost (clear on read) */
#define TWI_SR_SCLWS (0x1u << 10) /**< \brief (TWI_SR) Clock Wait State (automatically set / reset) */
#define TWI_SR_EOSACC (0x1u << 11) /**< \brief (TWI_SR) End Of Slave Access (clear on read) */
#define TWI_SR_ENDRX (0x1u << 12) /**< \brief (TWI_SR) End of RX buffer */
#define TWI_SR_ENDTX (0x1u << 13) /**< \brief (TWI_SR) End of TX buffer */
#define TWI_SR_RXBUFF (0x1u << 14) /**< \brief (TWI_SR) RX Buffer Full */
#define TWI_SR_TXBUFE (0x1u << 15) /**< \brief (TWI_SR) TX Buffer Empty */
/* -------- TWI_IER : (TWI Offset: 0x24) Interrupt Enable Register -------- */
#define TWI_IER_TXCOMP (0x1u << 0) /**< \brief (TWI_IER) Transmission Completed Interrupt Enable */
#define TWI_IER_RXRDY (0x1u << 1) /**< \brief (TWI_IER) Receive Holding Register Ready Interrupt Enable */
#define TWI_IER_TXRDY (0x1u << 2) /**< \brief (TWI_IER) Transmit Holding Register Ready Interrupt Enable */
#define TWI_IER_SVACC (0x1u << 4) /**< \brief (TWI_IER) Slave Access Interrupt Enable */
#define TWI_IER_GACC (0x1u << 5) /**< \brief (TWI_IER) General Call Access Interrupt Enable */
#define TWI_IER_OVRE (0x1u << 6) /**< \brief (TWI_IER) Overrun Error Interrupt Enable */
#define TWI_IER_NACK (0x1u << 8) /**< \brief (TWI_IER) Not Acknowledge Interrupt Enable */
#define TWI_IER_ARBLST (0x1u << 9) /**< \brief (TWI_IER) Arbitration Lost Interrupt Enable */
#define TWI_IER_SCL_WS (0x1u << 10) /**< \brief (TWI_IER) Clock Wait State Interrupt Enable */
#define TWI_IER_EOSACC (0x1u << 11) /**< \brief (TWI_IER) End Of Slave Access Interrupt Enable */
#define TWI_IER_ENDRX (0x1u << 12) /**< \brief (TWI_IER) End of Receive Buffer Interrupt Enable */
#define TWI_IER_ENDTX (0x1u << 13) /**< \brief (TWI_IER) End of Transmit Buffer Interrupt Enable */
#define TWI_IER_RXBUFF (0x1u << 14) /**< \brief (TWI_IER) Receive Buffer Full Interrupt Enable */
#define TWI_IER_TXBUFE (0x1u << 15) /**< \brief (TWI_IER) Transmit Buffer Empty Interrupt Enable */
/* -------- TWI_IDR : (TWI Offset: 0x28) Interrupt Disable Register -------- */
#define TWI_IDR_TXCOMP (0x1u << 0) /**< \brief (TWI_IDR) Transmission Completed Interrupt Disable */
#define TWI_IDR_RXRDY (0x1u << 1) /**< \brief (TWI_IDR) Receive Holding Register Ready Interrupt Disable */
#define TWI_IDR_TXRDY (0x1u << 2) /**< \brief (TWI_IDR) Transmit Holding Register Ready Interrupt Disable */
#define TWI_IDR_SVACC (0x1u << 4) /**< \brief (TWI_IDR) Slave Access Interrupt Disable */
#define TWI_IDR_GACC (0x1u << 5) /**< \brief (TWI_IDR) General Call Access Interrupt Disable */
#define TWI_IDR_OVRE (0x1u << 6) /**< \brief (TWI_IDR) Overrun Error Interrupt Disable */
#define TWI_IDR_NACK (0x1u << 8) /**< \brief (TWI_IDR) Not Acknowledge Interrupt Disable */
#define TWI_IDR_ARBLST (0x1u << 9) /**< \brief (TWI_IDR) Arbitration Lost Interrupt Disable */
#define TWI_IDR_SCL_WS (0x1u << 10) /**< \brief (TWI_IDR) Clock Wait State Interrupt Disable */
#define TWI_IDR_EOSACC (0x1u << 11) /**< \brief (TWI_IDR) End Of Slave Access Interrupt Disable */
#define TWI_IDR_ENDRX (0x1u << 12) /**< \brief (TWI_IDR) End of Receive Buffer Interrupt Disable */
#define TWI_IDR_ENDTX (0x1u << 13) /**< \brief (TWI_IDR) End of Transmit Buffer Interrupt Disable */
#define TWI_IDR_RXBUFF (0x1u << 14) /**< \brief (TWI_IDR) Receive Buffer Full Interrupt Disable */
#define TWI_IDR_TXBUFE (0x1u << 15) /**< \brief (TWI_IDR) Transmit Buffer Empty Interrupt Disable */
/* -------- TWI_IMR : (TWI Offset: 0x2C) Interrupt Mask Register -------- */
#define TWI_IMR_TXCOMP (0x1u << 0) /**< \brief (TWI_IMR) Transmission Completed Interrupt Mask */
#define TWI_IMR_RXRDY (0x1u << 1) /**< \brief (TWI_IMR) Receive Holding Register Ready Interrupt Mask */
#define TWI_IMR_TXRDY (0x1u << 2) /**< \brief (TWI_IMR) Transmit Holding Register Ready Interrupt Mask */
#define TWI_IMR_SVACC (0x1u << 4) /**< \brief (TWI_IMR) Slave Access Interrupt Mask */
#define TWI_IMR_GACC (0x1u << 5) /**< \brief (TWI_IMR) General Call Access Interrupt Mask */
#define TWI_IMR_OVRE (0x1u << 6) /**< \brief (TWI_IMR) Overrun Error Interrupt Mask */
#define TWI_IMR_NACK (0x1u << 8) /**< \brief (TWI_IMR) Not Acknowledge Interrupt Mask */
#define TWI_IMR_ARBLST (0x1u << 9) /**< \brief (TWI_IMR) Arbitration Lost Interrupt Mask */
#define TWI_IMR_SCL_WS (0x1u << 10) /**< \brief (TWI_IMR) Clock Wait State Interrupt Mask */
#define TWI_IMR_EOSACC (0x1u << 11) /**< \brief (TWI_IMR) End Of Slave Access Interrupt Mask */
#define TWI_IMR_ENDRX (0x1u << 12) /**< \brief (TWI_IMR) End of Receive Buffer Interrupt Mask */
#define TWI_IMR_ENDTX (0x1u << 13) /**< \brief (TWI_IMR) End of Transmit Buffer Interrupt Mask */
#define TWI_IMR_RXBUFF (0x1u << 14) /**< \brief (TWI_IMR) Receive Buffer Full Interrupt Mask */
#define TWI_IMR_TXBUFE (0x1u << 15) /**< \brief (TWI_IMR) Transmit Buffer Empty Interrupt Mask */
/* -------- TWI_RHR : (TWI Offset: 0x30) Receive Holding Register -------- */
#define TWI_RHR_RXDATA_Pos 0
#define TWI_RHR_RXDATA_Msk (0xffu << TWI_RHR_RXDATA_Pos) /**< \brief (TWI_RHR) Master or Slave Receive Holding Data */
/* -------- TWI_THR : (TWI Offset: 0x34) Transmit Holding Register -------- */
#define TWI_THR_TXDATA_Pos 0
#define TWI_THR_TXDATA_Msk (0xffu << TWI_THR_TXDATA_Pos) /**< \brief (TWI_THR) Master or Slave Transmit Holding Data */
#define TWI_THR_TXDATA(value) ((TWI_THR_TXDATA_Msk & ((value) << TWI_THR_TXDATA_Pos)))
/* -------- TWI_RPR : (TWI Offset: 0x100) Receive Pointer Register -------- */
#define TWI_RPR_RXPTR_Pos 0
#define TWI_RPR_RXPTR_Msk (0xffffffffu << TWI_RPR_RXPTR_Pos) /**< \brief (TWI_RPR) Receive Pointer Register */
#define TWI_RPR_RXPTR(value) ((TWI_RPR_RXPTR_Msk & ((value) << TWI_RPR_RXPTR_Pos)))
/* -------- TWI_RCR : (TWI Offset: 0x104) Receive Counter Register -------- */
#define TWI_RCR_RXCTR_Pos 0
#define TWI_RCR_RXCTR_Msk (0xffffu << TWI_RCR_RXCTR_Pos) /**< \brief (TWI_RCR) Receive Counter Register */
#define TWI_RCR_RXCTR(value) ((TWI_RCR_RXCTR_Msk & ((value) << TWI_RCR_RXCTR_Pos)))
/* -------- TWI_TPR : (TWI Offset: 0x108) Transmit Pointer Register -------- */
#define TWI_TPR_TXPTR_Pos 0
#define TWI_TPR_TXPTR_Msk (0xffffffffu << TWI_TPR_TXPTR_Pos) /**< \brief (TWI_TPR) Transmit Counter Register */
#define TWI_TPR_TXPTR(value) ((TWI_TPR_TXPTR_Msk & ((value) << TWI_TPR_TXPTR_Pos)))
/* -------- TWI_TCR : (TWI Offset: 0x10C) Transmit Counter Register -------- */
#define TWI_TCR_TXCTR_Pos 0
#define TWI_TCR_TXCTR_Msk (0xffffu << TWI_TCR_TXCTR_Pos) /**< \brief (TWI_TCR) Transmit Counter Register */
#define TWI_TCR_TXCTR(value) ((TWI_TCR_TXCTR_Msk & ((value) << TWI_TCR_TXCTR_Pos)))
/* -------- TWI_RNPR : (TWI Offset: 0x110) Receive Next Pointer Register -------- */
#define TWI_RNPR_RXNPTR_Pos 0
#define TWI_RNPR_RXNPTR_Msk (0xffffffffu << TWI_RNPR_RXNPTR_Pos) /**< \brief (TWI_RNPR) Receive Next Pointer */
#define TWI_RNPR_RXNPTR(value) ((TWI_RNPR_RXNPTR_Msk & ((value) << TWI_RNPR_RXNPTR_Pos)))
/* -------- TWI_RNCR : (TWI Offset: 0x114) Receive Next Counter Register -------- */
#define TWI_RNCR_RXNCTR_Pos 0
#define TWI_RNCR_RXNCTR_Msk (0xffffu << TWI_RNCR_RXNCTR_Pos) /**< \brief (TWI_RNCR) Receive Next Counter */
#define TWI_RNCR_RXNCTR(value) ((TWI_RNCR_RXNCTR_Msk & ((value) << TWI_RNCR_RXNCTR_Pos)))
/* -------- TWI_TNPR : (TWI Offset: 0x118) Transmit Next Pointer Register -------- */
#define TWI_TNPR_TXNPTR_Pos 0
#define TWI_TNPR_TXNPTR_Msk (0xffffffffu << TWI_TNPR_TXNPTR_Pos) /**< \brief (TWI_TNPR) Transmit Next Pointer */
#define TWI_TNPR_TXNPTR(value) ((TWI_TNPR_TXNPTR_Msk & ((value) << TWI_TNPR_TXNPTR_Pos)))
/* -------- TWI_TNCR : (TWI Offset: 0x11C) Transmit Next Counter Register -------- */
#define TWI_TNCR_TXNCTR_Pos 0
#define TWI_TNCR_TXNCTR_Msk (0xffffu << TWI_TNCR_TXNCTR_Pos) /**< \brief (TWI_TNCR) Transmit Counter Next */
#define TWI_TNCR_TXNCTR(value) ((TWI_TNCR_TXNCTR_Msk & ((value) << TWI_TNCR_TXNCTR_Pos)))
/* -------- TWI_PTCR : (TWI Offset: 0x120) Transfer Control Register -------- */
#define TWI_PTCR_RXTEN (0x1u << 0) /**< \brief (TWI_PTCR) Receiver Transfer Enable */
#define TWI_PTCR_RXTDIS (0x1u << 1) /**< \brief (TWI_PTCR) Receiver Transfer Disable */
#define TWI_PTCR_TXTEN (0x1u << 8) /**< \brief (TWI_PTCR) Transmitter Transfer Enable */
#define TWI_PTCR_TXTDIS (0x1u << 9) /**< \brief (TWI_PTCR) Transmitter Transfer Disable */
/* -------- TWI_PTSR : (TWI Offset: 0x124) Transfer Status Register -------- */
#define TWI_PTSR_RXTEN (0x1u << 0) /**< \brief (TWI_PTSR) Receiver Transfer Enable */
#define TWI_PTSR_TXTEN (0x1u << 8) /**< \brief (TWI_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_TWI */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Universal Asynchronous Receiver Transmitter */
/* ============================================================================= */
/** \addtogroup SAM3XA_UART Universal Asynchronous Receiver Transmitter */
/*@{*/

/** \brief Uart hardware registers */
typedef struct {
  WoReg UART_CR;       /**< \brief (Uart Offset: 0x0000) Control Register */
  RwReg UART_MR;       /**< \brief (Uart Offset: 0x0004) Mode Register */
  WoReg UART_IER;      /**< \brief (Uart Offset: 0x0008) Interrupt Enable Register */
  WoReg UART_IDR;      /**< \brief (Uart Offset: 0x000C) Interrupt Disable Register */
  RoReg UART_IMR;      /**< \brief (Uart Offset: 0x0010) Interrupt Mask Register */
  RoReg UART_SR;       /**< \brief (Uart Offset: 0x0014) Status Register */
  RoReg UART_RHR;      /**< \brief (Uart Offset: 0x0018) Receive Holding Register */
  WoReg UART_THR;      /**< \brief (Uart Offset: 0x001C) Transmit Holding Register */
  RwReg UART_BRGR;     /**< \brief (Uart Offset: 0x0020) Baud Rate Generator Register */
  RoReg Reserved1[55];
  RwReg UART_RPR;      /**< \brief (Uart Offset: 0x100) Receive Pointer Register */
  RwReg UART_RCR;      /**< \brief (Uart Offset: 0x104) Receive Counter Register */
  RwReg UART_TPR;      /**< \brief (Uart Offset: 0x108) Transmit Pointer Register */
  RwReg UART_TCR;      /**< \brief (Uart Offset: 0x10C) Transmit Counter Register */
  RwReg UART_RNPR;     /**< \brief (Uart Offset: 0x110) Receive Next Pointer Register */
  RwReg UART_RNCR;     /**< \brief (Uart Offset: 0x114) Receive Next Counter Register */
  RwReg UART_TNPR;     /**< \brief (Uart Offset: 0x118) Transmit Next Pointer Register */
  RwReg UART_TNCR;     /**< \brief (Uart Offset: 0x11C) Transmit Next Counter Register */
  WoReg UART_PTCR;     /**< \brief (Uart Offset: 0x120) Transfer Control Register */
  RoReg UART_PTSR;     /**< \brief (Uart Offset: 0x124) Transfer Status Register */
} Uart;
/* -------- UART_CR : (UART Offset: 0x0000) Control Register -------- */
#define UART_CR_RSTRX (0x1u << 2) /**< \brief (UART_CR) Reset Receiver */
#define UART_CR_RSTTX (0x1u << 3) /**< \brief (UART_CR) Reset Transmitter */
#define UART_CR_RXEN (0x1u << 4) /**< \brief (UART_CR) Receiver Enable */
#define UART_CR_RXDIS (0x1u << 5) /**< \brief (UART_CR) Receiver Disable */
#define UART_CR_TXEN (0x1u << 6) /**< \brief (UART_CR) Transmitter Enable */
#define UART_CR_TXDIS (0x1u << 7) /**< \brief (UART_CR) Transmitter Disable */
#define UART_CR_RSTSTA (0x1u << 8) /**< \brief (UART_CR) Reset Status Bits */
/* -------- UART_MR : (UART Offset: 0x0004) Mode Register -------- */
#define UART_MR_PAR_Pos 9
#define UART_MR_PAR_Msk (0x7u << UART_MR_PAR_Pos) /**< \brief (UART_MR) Parity Type */
#define   UART_MR_PAR_EVEN (0x0u << 9) /**< \brief (UART_MR) Even parity */
#define   UART_MR_PAR_ODD (0x1u << 9) /**< \brief (UART_MR) Odd parity */
#define   UART_MR_PAR_SPACE (0x2u << 9) /**< \brief (UART_MR) Space: parity forced to 0 */
#define   UART_MR_PAR_MARK (0x3u << 9) /**< \brief (UART_MR) Mark: parity forced to 1 */
#define   UART_MR_PAR_NO (0x4u << 9) /**< \brief (UART_MR) No parity */
#define UART_MR_CHMODE_Pos 14
#define UART_MR_CHMODE_Msk (0x3u << UART_MR_CHMODE_Pos) /**< \brief (UART_MR) Channel Mode */
#define   UART_MR_CHMODE_NORMAL (0x0u << 14) /**< \brief (UART_MR) Normal Mode */
#define   UART_MR_CHMODE_AUTOMATIC (0x1u << 14) /**< \brief (UART_MR) Automatic Echo */
#define   UART_MR_CHMODE_LOCAL_LOOPBACK (0x2u << 14) /**< \brief (UART_MR) Local Loopback */
#define   UART_MR_CHMODE_REMOTE_LOOPBACK (0x3u << 14) /**< \brief (UART_MR) Remote Loopback */
/* -------- UART_IER : (UART Offset: 0x0008) Interrupt Enable Register -------- */
#define UART_IER_RXRDY (0x1u << 0) /**< \brief (UART_IER) Enable RXRDY Interrupt */
#define UART_IER_TXRDY (0x1u << 1) /**< \brief (UART_IER) Enable TXRDY Interrupt */
#define UART_IER_ENDRX (0x1u << 3) /**< \brief (UART_IER) Enable End of Receive Transfer Interrupt */
#define UART_IER_ENDTX (0x1u << 4) /**< \brief (UART_IER) Enable End of Transmit Interrupt */
#define UART_IER_OVRE (0x1u << 5) /**< \brief (UART_IER) Enable Overrun Error Interrupt */
#define UART_IER_FRAME (0x1u << 6) /**< \brief (UART_IER) Enable Framing Error Interrupt */
#define UART_IER_PARE (0x1u << 7) /**< \brief (UART_IER) Enable Parity Error Interrupt */
#define UART_IER_TXEMPTY (0x1u << 9) /**< \brief (UART_IER) Enable TXEMPTY Interrupt */
#define UART_IER_TXBUFE (0x1u << 11) /**< \brief (UART_IER) Enable Buffer Empty Interrupt */
#define UART_IER_RXBUFF (0x1u << 12) /**< \brief (UART_IER) Enable Buffer Full Interrupt */
/* -------- UART_IDR : (UART Offset: 0x000C) Interrupt Disable Register -------- */
#define UART_IDR_RXRDY (0x1u << 0) /**< \brief (UART_IDR) Disable RXRDY Interrupt */
#define UART_IDR_TXRDY (0x1u << 1) /**< \brief (UART_IDR) Disable TXRDY Interrupt */
#define UART_IDR_ENDRX (0x1u << 3) /**< \brief (UART_IDR) Disable End of Receive Transfer Interrupt */
#define UART_IDR_ENDTX (0x1u << 4) /**< \brief (UART_IDR) Disable End of Transmit Interrupt */
#define UART_IDR_OVRE (0x1u << 5) /**< \brief (UART_IDR) Disable Overrun Error Interrupt */
#define UART_IDR_FRAME (0x1u << 6) /**< \brief (UART_IDR) Disable Framing Error Interrupt */
#define UART_IDR_PARE (0x1u << 7) /**< \brief (UART_IDR) Disable Parity Error Interrupt */
#define UART_IDR_TXEMPTY (0x1u << 9) /**< \brief (UART_IDR) Disable TXEMPTY Interrupt */
#define UART_IDR_TXBUFE (0x1u << 11) /**< \brief (UART_IDR) Disable Buffer Empty Interrupt */
#define UART_IDR_RXBUFF (0x1u << 12) /**< \brief (UART_IDR) Disable Buffer Full Interrupt */
/* -------- UART_IMR : (UART Offset: 0x0010) Interrupt Mask Register -------- */
#define UART_IMR_RXRDY (0x1u << 0) /**< \brief (UART_IMR) Mask RXRDY Interrupt */
#define UART_IMR_TXRDY (0x1u << 1) /**< \brief (UART_IMR) Disable TXRDY Interrupt */
#define UART_IMR_ENDRX (0x1u << 3) /**< \brief (UART_IMR) Mask End of Receive Transfer Interrupt */
#define UART_IMR_ENDTX (0x1u << 4) /**< \brief (UART_IMR) Mask End of Transmit Interrupt */
#define UART_IMR_OVRE (0x1u << 5) /**< \brief (UART_IMR) Mask Overrun Error Interrupt */
#define UART_IMR_FRAME (0x1u << 6) /**< \brief (UART_IMR) Mask Framing Error Interrupt */
#define UART_IMR_PARE (0x1u << 7) /**< \brief (UART_IMR) Mask Parity Error Interrupt */
#define UART_IMR_TXEMPTY (0x1u << 9) /**< \brief (UART_IMR) Mask TXEMPTY Interrupt */
#define UART_IMR_TXBUFE (0x1u << 11) /**< \brief (UART_IMR) Mask TXBUFE Interrupt */
#define UART_IMR_RXBUFF (0x1u << 12) /**< \brief (UART_IMR) Mask RXBUFF Interrupt */
/* -------- UART_SR : (UART Offset: 0x0014) Status Register -------- */
#define UART_SR_RXRDY (0x1u << 0) /**< \brief (UART_SR) Receiver Ready */
#define UART_SR_TXRDY (0x1u << 1) /**< \brief (UART_SR) Transmitter Ready */
#define UART_SR_ENDRX (0x1u << 3) /**< \brief (UART_SR) End of Receiver Transfer */
#define UART_SR_ENDTX (0x1u << 4) /**< \brief (UART_SR) End of Transmitter Transfer */
#define UART_SR_OVRE (0x1u << 5) /**< \brief (UART_SR) Overrun Error */
#define UART_SR_FRAME (0x1u << 6) /**< \brief (UART_SR) Framing Error */
#define UART_SR_PARE (0x1u << 7) /**< \brief (UART_SR) Parity Error */
#define UART_SR_TXEMPTY (0x1u << 9) /**< \brief (UART_SR) Transmitter Empty */
#define UART_SR_TXBUFE (0x1u << 11) /**< \brief (UART_SR) Transmission Buffer Empty */
#define UART_SR_RXBUFF (0x1u << 12) /**< \brief (UART_SR) Receive Buffer Full */
/* -------- UART_RHR : (UART Offset: 0x0018) Receive Holding Register -------- */
#define UART_RHR_RXCHR_Pos 0
#define UART_RHR_RXCHR_Msk (0xffu << UART_RHR_RXCHR_Pos) /**< \brief (UART_RHR) Received Character */
/* -------- UART_THR : (UART Offset: 0x001C) Transmit Holding Register -------- */
#define UART_THR_TXCHR_Pos 0
#define UART_THR_TXCHR_Msk (0xffu << UART_THR_TXCHR_Pos) /**< \brief (UART_THR) Character to be Transmitted */
#define UART_THR_TXCHR(value) ((UART_THR_TXCHR_Msk & ((value) << UART_THR_TXCHR_Pos)))
/* -------- UART_BRGR : (UART Offset: 0x0020) Baud Rate Generator Register -------- */
#define UART_BRGR_CD_Pos 0
#define UART_BRGR_CD_Msk (0xffffu << UART_BRGR_CD_Pos) /**< \brief (UART_BRGR) Clock Divisor */
#define UART_BRGR_CD(value) ((UART_BRGR_CD_Msk & ((value) << UART_BRGR_CD_Pos)))
/* -------- UART_RPR : (UART Offset: 0x100) Receive Pointer Register -------- */
#define UART_RPR_RXPTR_Pos 0
#define UART_RPR_RXPTR_Msk (0xffffffffu << UART_RPR_RXPTR_Pos) /**< \brief (UART_RPR) Receive Pointer Register */
#define UART_RPR_RXPTR(value) ((UART_RPR_RXPTR_Msk & ((value) << UART_RPR_RXPTR_Pos)))
/* -------- UART_RCR : (UART Offset: 0x104) Receive Counter Register -------- */
#define UART_RCR_RXCTR_Pos 0
#define UART_RCR_RXCTR_Msk (0xffffu << UART_RCR_RXCTR_Pos) /**< \brief (UART_RCR) Receive Counter Register */
#define UART_RCR_RXCTR(value) ((UART_RCR_RXCTR_Msk & ((value) << UART_RCR_RXCTR_Pos)))
/* -------- UART_TPR : (UART Offset: 0x108) Transmit Pointer Register -------- */
#define UART_TPR_TXPTR_Pos 0
#define UART_TPR_TXPTR_Msk (0xffffffffu << UART_TPR_TXPTR_Pos) /**< \brief (UART_TPR) Transmit Counter Register */
#define UART_TPR_TXPTR(value) ((UART_TPR_TXPTR_Msk & ((value) << UART_TPR_TXPTR_Pos)))
/* -------- UART_TCR : (UART Offset: 0x10C) Transmit Counter Register -------- */
#define UART_TCR_TXCTR_Pos 0
#define UART_TCR_TXCTR_Msk (0xffffu << UART_TCR_TXCTR_Pos) /**< \brief (UART_TCR) Transmit Counter Register */
#define UART_TCR_TXCTR(value) ((UART_TCR_TXCTR_Msk & ((value) << UART_TCR_TXCTR_Pos)))
/* -------- UART_RNPR : (UART Offset: 0x110) Receive Next Pointer Register -------- */
#define UART_RNPR_RXNPTR_Pos 0
#define UART_RNPR_RXNPTR_Msk (0xffffffffu << UART_RNPR_RXNPTR_Pos) /**< \brief (UART_RNPR) Receive Next Pointer */
#define UART_RNPR_RXNPTR(value) ((UART_RNPR_RXNPTR_Msk & ((value) << UART_RNPR_RXNPTR_Pos)))
/* -------- UART_RNCR : (UART Offset: 0x114) Receive Next Counter Register -------- */
#define UART_RNCR_RXNCTR_Pos 0
#define UART_RNCR_RXNCTR_Msk (0xffffu << UART_RNCR_RXNCTR_Pos) /**< \brief (UART_RNCR) Receive Next Counter */
#define UART_RNCR_RXNCTR(value) ((UART_RNCR_RXNCTR_Msk & ((value) << UART_RNCR_RXNCTR_Pos)))
/* -------- UART_TNPR : (UART Offset: 0x118) Transmit Next Pointer Register -------- */
#define UART_TNPR_TXNPTR_Pos 0
#define UART_TNPR_TXNPTR_Msk (0xffffffffu << UART_TNPR_TXNPTR_Pos) /**< \brief (UART_TNPR) Transmit Next Pointer */
#define UART_TNPR_TXNPTR(value) ((UART_TNPR_TXNPTR_Msk & ((value) << UART_TNPR_TXNPTR_Pos)))
/* -------- UART_TNCR : (UART Offset: 0x11C) Transmit Next Counter Register -------- */
#define UART_TNCR_TXNCTR_Pos 0
#define UART_TNCR_TXNCTR_Msk (0xffffu << UART_TNCR_TXNCTR_Pos) /**< \brief (UART_TNCR) Transmit Counter Next */
#define UART_TNCR_TXNCTR(value) ((UART_TNCR_TXNCTR_Msk & ((value) << UART_TNCR_TXNCTR_Pos)))
/* -------- UART_PTCR : (UART Offset: 0x120) Transfer Control Register -------- */
#define UART_PTCR_RXTEN (0x1u << 0) /**< \brief (UART_PTCR) Receiver Transfer Enable */
#define UART_PTCR_RXTDIS (0x1u << 1) /**< \brief (UART_PTCR) Receiver Transfer Disable */
#define UART_PTCR_TXTEN (0x1u << 8) /**< \brief (UART_PTCR) Transmitter Transfer Enable */
#define UART_PTCR_TXTDIS (0x1u << 9) /**< \brief (UART_PTCR) Transmitter Transfer Disable */
/* -------- UART_PTSR : (UART Offset: 0x124) Transfer Status Register -------- */
#define UART_PTSR_RXTEN (0x1u << 0) /**< \brief (UART_PTSR) Receiver Transfer Enable */
#define UART_PTSR_TXTEN (0x1u << 8) /**< \brief (UART_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_UART */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR USB On-The-Go Interface */
/* ============================================================================= */
/** \addtogroup SAM3XA_UOTGHS USB On-The-Go Interface */
/*@{*/

/** \brief UotghsDevdma hardware registers */
typedef struct {
  RwReg         UOTGHS_DEVDMANXTDSC;   /**< \brief (UotghsDevdma Offset: 0x0) Device DMA Channel Next Descriptor Address Register */
  RwReg         UOTGHS_DEVDMAADDRESS;  /**< \brief (UotghsDevdma Offset: 0x4) Device DMA Channel Address Register */
  RwReg         UOTGHS_DEVDMACONTROL;  /**< \brief (UotghsDevdma Offset: 0x8) Device DMA Channel Control Register */
  RwReg         UOTGHS_DEVDMASTATUS;   /**< \brief (UotghsDevdma Offset: 0xC) Device DMA Channel Status Register */
} UotghsDevdma;
/** \brief UotghsHstdma hardware registers */
typedef struct {
  RwReg         UOTGHS_HSTDMANXTDSC;   /**< \brief (UotghsHstdma Offset: 0x0) Host DMA Channel Next Descriptor Address Register */
  RwReg         UOTGHS_HSTDMAADDRESS;  /**< \brief (UotghsHstdma Offset: 0x4) Host DMA Channel Address Register */
  RwReg         UOTGHS_HSTDMACONTROL;  /**< \brief (UotghsHstdma Offset: 0x8) Host DMA Channel Control Register */
  RwReg         UOTGHS_HSTDMASTATUS;   /**< \brief (UotghsHstdma Offset: 0xC) Host DMA Channel Status Register */
} UotghsHstdma;
/** \brief Uotghs hardware registers */
#define UOTGHSDEVDMA_NUMBER 7
#define UOTGHSHSTDMA_NUMBER 7
typedef struct {
  RwReg         UOTGHS_DEVCTRL;        /**< \brief (Uotghs Offset: 0x0000) Device General Control Register */
  RoReg         UOTGHS_DEVISR;         /**< \brief (Uotghs Offset: 0x0004) Device Global Interrupt Status Register */
  WoReg         UOTGHS_DEVICR;         /**< \brief (Uotghs Offset: 0x0008) Device Global Interrupt Clear Register */
  WoReg         UOTGHS_DEVIFR;         /**< \brief (Uotghs Offset: 0x000C) Device Global Interrupt Set Register */
  RoReg         UOTGHS_DEVIMR;         /**< \brief (Uotghs Offset: 0x0010) Device Global Interrupt Mask Register */
  WoReg         UOTGHS_DEVIDR;         /**< \brief (Uotghs Offset: 0x0014) Device Global Interrupt Disable Register */
  WoReg         UOTGHS_DEVIER;         /**< \brief (Uotghs Offset: 0x0018) Device Global Interrupt Enable Register */
  RwReg         UOTGHS_DEVEPT;         /**< \brief (Uotghs Offset: 0x001C) Device Endpoint Register */
  RoReg         UOTGHS_DEVFNUM;        /**< \brief (Uotghs Offset: 0x0020) Device Frame Number Register */
  RoReg         Reserved1[55];
  RwReg         UOTGHS_DEVEPTCFG[10];  /**< \brief (Uotghs Offset: 0x100) Device Endpoint Configuration Register (n = 0) */
  RoReg         Reserved2[2];
  RoReg         UOTGHS_DEVEPTISR[10];  /**< \brief (Uotghs Offset: 0x130) Device Endpoint Status Register (n = 0) */
  RoReg         Reserved3[2];
  WoReg         UOTGHS_DEVEPTICR[10];  /**< \brief (Uotghs Offset: 0x160) Device Endpoint Clear Register (n = 0) */
  RoReg         Reserved4[2];
  WoReg         UOTGHS_DEVEPTIFR[10];  /**< \brief (Uotghs Offset: 0x190) Device Endpoint Set Register (n = 0) */
  RoReg         Reserved5[2];
  RoReg         UOTGHS_DEVEPTIMR[10];  /**< \brief (Uotghs Offset: 0x1C0) Device Endpoint Mask Register (n = 0) */
  RoReg         Reserved6[2];
  WoReg         UOTGHS_DEVEPTIER[10];  /**< \brief (Uotghs Offset: 0x1F0) Device Endpoint Enable Register (n = 0) */
  RoReg         Reserved7[2];
  WoReg         UOTGHS_DEVEPTIDR[10];  /**< \brief (Uotghs Offset: 0x220) Device Endpoint Disable Register (n = 0) */
  RoReg         Reserved8[50];
  UotghsDevdma  UOTGHS_DEVDMA[UOTGHSDEVDMA_NUMBER]; /**< \brief (Uotghs Offset: 0x310) n = 1 .. 7 */
  RoReg         Reserved9[32];
  RwReg         UOTGHS_HSTCTRL;        /**< \brief (Uotghs Offset: 0x0400) Host General Control Register */
  RoReg         UOTGHS_HSTISR;         /**< \brief (Uotghs Offset: 0x0404) Host Global Interrupt Status Register */
  WoReg         UOTGHS_HSTICR;         /**< \brief (Uotghs Offset: 0x0408) Host Global Interrupt Clear Register */
  WoReg         UOTGHS_HSTIFR;         /**< \brief (Uotghs Offset: 0x040C) Host Global Interrupt Set Register */
  RoReg         UOTGHS_HSTIMR;         /**< \brief (Uotghs Offset: 0x0410) Host Global Interrupt Mask Register */
  WoReg         UOTGHS_HSTIDR;         /**< \brief (Uotghs Offset: 0x0414) Host Global Interrupt Disable Register */
  WoReg         UOTGHS_HSTIER;         /**< \brief (Uotghs Offset: 0x0418) Host Global Interrupt Enable Register */
  RwReg         UOTGHS_HSTPIP;         /**< \brief (Uotghs Offset: 0x0041C) Host Pipe Register */
  RwReg         UOTGHS_HSTFNUM;        /**< \brief (Uotghs Offset: 0x0420) Host Frame Number Register */
  RwReg         UOTGHS_HSTADDR1;       /**< \brief (Uotghs Offset: 0x0424) Host Address 1 Register */
  RwReg         UOTGHS_HSTADDR2;       /**< \brief (Uotghs Offset: 0x0428) Host Address 2 Register */
  RwReg         UOTGHS_HSTADDR3;       /**< \brief (Uotghs Offset: 0x042C) Host Address 3 Register */
  RoReg         Reserved10[52];
  RwReg         UOTGHS_HSTPIPCFG[10];  /**< \brief (Uotghs Offset: 0x500) Host Pipe Configuration Register (n = 0) */
  RoReg         Reserved11[2];
  RoReg         UOTGHS_HSTPIPISR[10];  /**< \brief (Uotghs Offset: 0x530) Host Pipe Status Register (n = 0) */
  RoReg         Reserved12[2];
  WoReg         UOTGHS_HSTPIPICR[10];  /**< \brief (Uotghs Offset: 0x560) Host Pipe Clear Register (n = 0) */
  RoReg         Reserved13[2];
  WoReg         UOTGHS_HSTPIPIFR[10];  /**< \brief (Uotghs Offset: 0x590) Host Pipe Set Register (n = 0) */
  RoReg         Reserved14[2];
  RoReg         UOTGHS_HSTPIPIMR[10];  /**< \brief (Uotghs Offset: 0x5C0) Host Pipe Mask Register (n = 0) */
  RoReg         Reserved15[2];
  WoReg         UOTGHS_HSTPIPIER[10];  /**< \brief (Uotghs Offset: 0x5F0) Host Pipe Enable Register (n = 0) */
  RoReg         Reserved16[2];
  WoReg         UOTGHS_HSTPIPIDR[10];  /**< \brief (Uotghs Offset: 0x620) Host Pipe Disable Register (n = 0) */
  RoReg         Reserved17[2];
  RwReg         UOTGHS_HSTPIPINRQ[10]; /**< \brief (Uotghs Offset: 0x650) Host Pipe IN Request Register (n = 0) */
  RoReg         Reserved18[2];
  RwReg         UOTGHS_HSTPIPERR[10];  /**< \brief (Uotghs Offset: 0x680) Host Pipe Error Register (n = 0) */
  RoReg         Reserved19[26];
  UotghsHstdma  UOTGHS_HSTDMA[UOTGHSHSTDMA_NUMBER]; /**< \brief (Uotghs Offset: 0x710) n = 1 .. 7 */
  RoReg         Reserved20[32];
  RwReg         UOTGHS_CTRL;           /**< \brief (Uotghs Offset: 0x0800) General Control Register */
  RoReg         UOTGHS_SR;             /**< \brief (Uotghs Offset: 0x0804) General Status Register */
  WoReg         UOTGHS_SCR;            /**< \brief (Uotghs Offset: 0x0808) General Status Clear Register */
  WoReg         UOTGHS_SFR;            /**< \brief (Uotghs Offset: 0x080C) General Status Set Register */
  RoReg         Reserved21[7];
  RoReg         UOTGHS_FSM;            /**< \brief (Uotghs Offset: 0x082C) General Finite State Machine Register */
} Uotghs;
/* -------- UOTGHS_DEVCTRL : (UOTGHS Offset: 0x0000) Device General Control Register -------- */
#define UOTGHS_DEVCTRL_UADD_Pos 0
#define UOTGHS_DEVCTRL_UADD_Msk (0x7fu << UOTGHS_DEVCTRL_UADD_Pos) /**< \brief (UOTGHS_DEVCTRL) USB Address */
#define UOTGHS_DEVCTRL_UADD(value) ((UOTGHS_DEVCTRL_UADD_Msk & ((value) << UOTGHS_DEVCTRL_UADD_Pos)))
#define UOTGHS_DEVCTRL_ADDEN (0x1u << 7) /**< \brief (UOTGHS_DEVCTRL) Address Enable */
#define UOTGHS_DEVCTRL_DETACH (0x1u << 8) /**< \brief (UOTGHS_DEVCTRL) Detach */
#define UOTGHS_DEVCTRL_RMWKUP (0x1u << 9) /**< \brief (UOTGHS_DEVCTRL) Remote Wake-Up */
#define UOTGHS_DEVCTRL_SPDCONF_Pos 10
#define UOTGHS_DEVCTRL_SPDCONF_Msk (0x3u << UOTGHS_DEVCTRL_SPDCONF_Pos) /**< \brief (UOTGHS_DEVCTRL) Mode Configuration */
#define   UOTGHS_DEVCTRL_SPDCONF_NORMAL (0x0u << 10) /**< \brief (UOTGHS_DEVCTRL) The peripheral starts in full-speed mode and performs a high-speed reset to switch to the high-speed mode if the host is high-speed capable. */
#define   UOTGHS_DEVCTRL_SPDCONF_LOW_POWER (0x1u << 10) /**< \brief (UOTGHS_DEVCTRL) For a better consumption, if high-speed is not needed. */
#define   UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED (0x2u << 10) /**< \brief (UOTGHS_DEVCTRL) Forced high speed. */
#define   UOTGHS_DEVCTRL_SPDCONF_FORCED_FS (0x3u << 10) /**< \brief (UOTGHS_DEVCTRL) The peripheral remains in full-speed mode whatever the host speed capability. */
#define UOTGHS_DEVCTRL_LS (0x1u << 12) /**< \brief (UOTGHS_DEVCTRL) Low-Speed Mode Force */
#define UOTGHS_DEVCTRL_TSTJ (0x1u << 13) /**< \brief (UOTGHS_DEVCTRL) Test mode J */
#define UOTGHS_DEVCTRL_TSTK (0x1u << 14) /**< \brief (UOTGHS_DEVCTRL) Test mode K */
#define UOTGHS_DEVCTRL_TSTPCKT (0x1u << 15) /**< \brief (UOTGHS_DEVCTRL) Test packet mode */
#define UOTGHS_DEVCTRL_OPMODE2 (0x1u << 16) /**< \brief (UOTGHS_DEVCTRL) Specific Operational mode */
/* -------- UOTGHS_DEVISR : (UOTGHS Offset: 0x0004) Device Global Interrupt Status Register -------- */
#define UOTGHS_DEVISR_SUSP (0x1u << 0) /**< \brief (UOTGHS_DEVISR) Suspend Interrupt */
#define UOTGHS_DEVISR_MSOF (0x1u << 1) /**< \brief (UOTGHS_DEVISR) Micro Start of Frame Interrupt */
#define UOTGHS_DEVISR_SOF (0x1u << 2) /**< \brief (UOTGHS_DEVISR) Start of Frame Interrupt */
#define UOTGHS_DEVISR_EORST (0x1u << 3) /**< \brief (UOTGHS_DEVISR) End of Reset Interrupt */
#define UOTGHS_DEVISR_WAKEUP (0x1u << 4) /**< \brief (UOTGHS_DEVISR) Wake-Up Interrupt */
#define UOTGHS_DEVISR_EORSM (0x1u << 5) /**< \brief (UOTGHS_DEVISR) End of Resume Interrupt */
#define UOTGHS_DEVISR_UPRSM (0x1u << 6) /**< \brief (UOTGHS_DEVISR) Upstream Resume Interrupt */
#define UOTGHS_DEVISR_PEP_0 (0x1u << 12) /**< \brief (UOTGHS_DEVISR) Endpoint 0 Interrupt */
#define UOTGHS_DEVISR_PEP_1 (0x1u << 13) /**< \brief (UOTGHS_DEVISR) Endpoint 1 Interrupt */
#define UOTGHS_DEVISR_PEP_2 (0x1u << 14) /**< \brief (UOTGHS_DEVISR) Endpoint 2 Interrupt */
#define UOTGHS_DEVISR_PEP_3 (0x1u << 15) /**< \brief (UOTGHS_DEVISR) Endpoint 3 Interrupt */
#define UOTGHS_DEVISR_PEP_4 (0x1u << 16) /**< \brief (UOTGHS_DEVISR) Endpoint 4 Interrupt */
#define UOTGHS_DEVISR_PEP_5 (0x1u << 17) /**< \brief (UOTGHS_DEVISR) Endpoint 5 Interrupt */
#define UOTGHS_DEVISR_PEP_6 (0x1u << 18) /**< \brief (UOTGHS_DEVISR) Endpoint 6 Interrupt */
#define UOTGHS_DEVISR_PEP_7 (0x1u << 19) /**< \brief (UOTGHS_DEVISR) Endpoint 7 Interrupt */
#define UOTGHS_DEVISR_PEP_8 (0x1u << 20) /**< \brief (UOTGHS_DEVISR) Endpoint 8 Interrupt */
#define UOTGHS_DEVISR_PEP_9 (0x1u << 21) /**< \brief (UOTGHS_DEVISR) Endpoint 9 Interrupt */
#define UOTGHS_DEVISR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_DEVISR) DMA Channel 1 Interrupt */
#define UOTGHS_DEVISR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_DEVISR) DMA Channel 2 Interrupt */
#define UOTGHS_DEVISR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_DEVISR) DMA Channel 3 Interrupt */
#define UOTGHS_DEVISR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_DEVISR) DMA Channel 4 Interrupt */
#define UOTGHS_DEVISR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_DEVISR) DMA Channel 5 Interrupt */
#define UOTGHS_DEVISR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_DEVISR) DMA Channel 6 Interrupt */
/* -------- UOTGHS_DEVICR : (UOTGHS Offset: 0x0008) Device Global Interrupt Clear Register -------- */
#define UOTGHS_DEVICR_SUSPC (0x1u << 0) /**< \brief (UOTGHS_DEVICR) Suspend Interrupt Clear */
#define UOTGHS_DEVICR_MSOFC (0x1u << 1) /**< \brief (UOTGHS_DEVICR) Micro Start of Frame Interrupt Clear */
#define UOTGHS_DEVICR_SOFC (0x1u << 2) /**< \brief (UOTGHS_DEVICR) Start of Frame Interrupt Clear */
#define UOTGHS_DEVICR_EORSTC (0x1u << 3) /**< \brief (UOTGHS_DEVICR) End of Reset Interrupt Clear */
#define UOTGHS_DEVICR_WAKEUPC (0x1u << 4) /**< \brief (UOTGHS_DEVICR) Wake-Up Interrupt Clear */
#define UOTGHS_DEVICR_EORSMC (0x1u << 5) /**< \brief (UOTGHS_DEVICR) End of Resume Interrupt Clear */
#define UOTGHS_DEVICR_UPRSMC (0x1u << 6) /**< \brief (UOTGHS_DEVICR) Upstream Resume Interrupt Clear */
/* -------- UOTGHS_DEVIFR : (UOTGHS Offset: 0x000C) Device Global Interrupt Set Register -------- */
#define UOTGHS_DEVIFR_SUSPS (0x1u << 0) /**< \brief (UOTGHS_DEVIFR) Suspend Interrupt Set */
#define UOTGHS_DEVIFR_MSOFS (0x1u << 1) /**< \brief (UOTGHS_DEVIFR) Micro Start of Frame Interrupt Set */
#define UOTGHS_DEVIFR_SOFS (0x1u << 2) /**< \brief (UOTGHS_DEVIFR) Start of Frame Interrupt Set */
#define UOTGHS_DEVIFR_EORSTS (0x1u << 3) /**< \brief (UOTGHS_DEVIFR) End of Reset Interrupt Set */
#define UOTGHS_DEVIFR_WAKEUPS (0x1u << 4) /**< \brief (UOTGHS_DEVIFR) Wake-Up Interrupt Set */
#define UOTGHS_DEVIFR_EORSMS (0x1u << 5) /**< \brief (UOTGHS_DEVIFR) End of Resume Interrupt Set */
#define UOTGHS_DEVIFR_UPRSMS (0x1u << 6) /**< \brief (UOTGHS_DEVIFR) Upstream Resume Interrupt Set */
#define UOTGHS_DEVIFR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_DEVIFR) DMA Channel 1 Interrupt Set */
#define UOTGHS_DEVIFR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_DEVIFR) DMA Channel 2 Interrupt Set */
#define UOTGHS_DEVIFR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_DEVIFR) DMA Channel 3 Interrupt Set */
#define UOTGHS_DEVIFR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_DEVIFR) DMA Channel 4 Interrupt Set */
#define UOTGHS_DEVIFR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_DEVIFR) DMA Channel 5 Interrupt Set */
#define UOTGHS_DEVIFR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_DEVIFR) DMA Channel 6 Interrupt Set */
/* -------- UOTGHS_DEVIMR : (UOTGHS Offset: 0x0010) Device Global Interrupt Mask Register -------- */
#define UOTGHS_DEVIMR_SUSPE (0x1u << 0) /**< \brief (UOTGHS_DEVIMR) Suspend Interrupt Mask */
#define UOTGHS_DEVIMR_MSOFE (0x1u << 1) /**< \brief (UOTGHS_DEVIMR) Micro Start of Frame Interrupt Mask */
#define UOTGHS_DEVIMR_SOFE (0x1u << 2) /**< \brief (UOTGHS_DEVIMR) Start of Frame Interrupt Mask */
#define UOTGHS_DEVIMR_EORSTE (0x1u << 3) /**< \brief (UOTGHS_DEVIMR) End of Reset Interrupt Mask */
#define UOTGHS_DEVIMR_WAKEUPE (0x1u << 4) /**< \brief (UOTGHS_DEVIMR) Wake-Up Interrupt Mask */
#define UOTGHS_DEVIMR_EORSME (0x1u << 5) /**< \brief (UOTGHS_DEVIMR) End of Resume Interrupt Mask */
#define UOTGHS_DEVIMR_UPRSME (0x1u << 6) /**< \brief (UOTGHS_DEVIMR) Upstream Resume Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_0 (0x1u << 12) /**< \brief (UOTGHS_DEVIMR) Endpoint 0 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_1 (0x1u << 13) /**< \brief (UOTGHS_DEVIMR) Endpoint 1 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_2 (0x1u << 14) /**< \brief (UOTGHS_DEVIMR) Endpoint 2 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_3 (0x1u << 15) /**< \brief (UOTGHS_DEVIMR) Endpoint 3 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_4 (0x1u << 16) /**< \brief (UOTGHS_DEVIMR) Endpoint 4 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_5 (0x1u << 17) /**< \brief (UOTGHS_DEVIMR) Endpoint 5 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_6 (0x1u << 18) /**< \brief (UOTGHS_DEVIMR) Endpoint 6 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_7 (0x1u << 19) /**< \brief (UOTGHS_DEVIMR) Endpoint 7 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_8 (0x1u << 20) /**< \brief (UOTGHS_DEVIMR) Endpoint 8 Interrupt Mask */
#define UOTGHS_DEVIMR_PEP_9 (0x1u << 21) /**< \brief (UOTGHS_DEVIMR) Endpoint 9 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_DEVIMR) DMA Channel 1 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_DEVIMR) DMA Channel 2 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_DEVIMR) DMA Channel 3 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_DEVIMR) DMA Channel 4 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_DEVIMR) DMA Channel 5 Interrupt Mask */
#define UOTGHS_DEVIMR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_DEVIMR) DMA Channel 6 Interrupt Mask */
/* -------- UOTGHS_DEVIDR : (UOTGHS Offset: 0x0014) Device Global Interrupt Disable Register -------- */
#define UOTGHS_DEVIDR_SUSPEC (0x1u << 0) /**< \brief (UOTGHS_DEVIDR) Suspend Interrupt Disable */
#define UOTGHS_DEVIDR_MSOFEC (0x1u << 1) /**< \brief (UOTGHS_DEVIDR) Micro Start of Frame Interrupt Disable */
#define UOTGHS_DEVIDR_SOFEC (0x1u << 2) /**< \brief (UOTGHS_DEVIDR) Start of Frame Interrupt Disable */
#define UOTGHS_DEVIDR_EORSTEC (0x1u << 3) /**< \brief (UOTGHS_DEVIDR) End of Reset Interrupt Disable */
#define UOTGHS_DEVIDR_WAKEUPEC (0x1u << 4) /**< \brief (UOTGHS_DEVIDR) Wake-Up Interrupt Disable */
#define UOTGHS_DEVIDR_EORSMEC (0x1u << 5) /**< \brief (UOTGHS_DEVIDR) End of Resume Interrupt Disable */
#define UOTGHS_DEVIDR_UPRSMEC (0x1u << 6) /**< \brief (UOTGHS_DEVIDR) Upstream Resume Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_0 (0x1u << 12) /**< \brief (UOTGHS_DEVIDR) Endpoint 0 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_1 (0x1u << 13) /**< \brief (UOTGHS_DEVIDR) Endpoint 1 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_2 (0x1u << 14) /**< \brief (UOTGHS_DEVIDR) Endpoint 2 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_3 (0x1u << 15) /**< \brief (UOTGHS_DEVIDR) Endpoint 3 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_4 (0x1u << 16) /**< \brief (UOTGHS_DEVIDR) Endpoint 4 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_5 (0x1u << 17) /**< \brief (UOTGHS_DEVIDR) Endpoint 5 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_6 (0x1u << 18) /**< \brief (UOTGHS_DEVIDR) Endpoint 6 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_7 (0x1u << 19) /**< \brief (UOTGHS_DEVIDR) Endpoint 7 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_8 (0x1u << 20) /**< \brief (UOTGHS_DEVIDR) Endpoint 8 Interrupt Disable */
#define UOTGHS_DEVIDR_PEP_9 (0x1u << 21) /**< \brief (UOTGHS_DEVIDR) Endpoint 9 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_DEVIDR) DMA Channel 1 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_DEVIDR) DMA Channel 2 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_DEVIDR) DMA Channel 3 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_DEVIDR) DMA Channel 4 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_DEVIDR) DMA Channel 5 Interrupt Disable */
#define UOTGHS_DEVIDR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_DEVIDR) DMA Channel 6 Interrupt Disable */
/* -------- UOTGHS_DEVIER : (UOTGHS Offset: 0x0018) Device Global Interrupt Enable Register -------- */
#define UOTGHS_DEVIER_SUSPES (0x1u << 0) /**< \brief (UOTGHS_DEVIER) Suspend Interrupt Enable */
#define UOTGHS_DEVIER_MSOFES (0x1u << 1) /**< \brief (UOTGHS_DEVIER) Micro Start of Frame Interrupt Enable */
#define UOTGHS_DEVIER_SOFES (0x1u << 2) /**< \brief (UOTGHS_DEVIER) Start of Frame Interrupt Enable */
#define UOTGHS_DEVIER_EORSTES (0x1u << 3) /**< \brief (UOTGHS_DEVIER) End of Reset Interrupt Enable */
#define UOTGHS_DEVIER_WAKEUPES (0x1u << 4) /**< \brief (UOTGHS_DEVIER) Wake-Up Interrupt Enable */
#define UOTGHS_DEVIER_EORSMES (0x1u << 5) /**< \brief (UOTGHS_DEVIER) End of Resume Interrupt Enable */
#define UOTGHS_DEVIER_UPRSMES (0x1u << 6) /**< \brief (UOTGHS_DEVIER) Upstream Resume Interrupt Enable */
#define UOTGHS_DEVIER_PEP_0 (0x1u << 12) /**< \brief (UOTGHS_DEVIER) Endpoint 0 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_1 (0x1u << 13) /**< \brief (UOTGHS_DEVIER) Endpoint 1 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_2 (0x1u << 14) /**< \brief (UOTGHS_DEVIER) Endpoint 2 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_3 (0x1u << 15) /**< \brief (UOTGHS_DEVIER) Endpoint 3 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_4 (0x1u << 16) /**< \brief (UOTGHS_DEVIER) Endpoint 4 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_5 (0x1u << 17) /**< \brief (UOTGHS_DEVIER) Endpoint 5 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_6 (0x1u << 18) /**< \brief (UOTGHS_DEVIER) Endpoint 6 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_7 (0x1u << 19) /**< \brief (UOTGHS_DEVIER) Endpoint 7 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_8 (0x1u << 20) /**< \brief (UOTGHS_DEVIER) Endpoint 8 Interrupt Enable */
#define UOTGHS_DEVIER_PEP_9 (0x1u << 21) /**< \brief (UOTGHS_DEVIER) Endpoint 9 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_DEVIER) DMA Channel 1 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_DEVIER) DMA Channel 2 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_DEVIER) DMA Channel 3 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_DEVIER) DMA Channel 4 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_DEVIER) DMA Channel 5 Interrupt Enable */
#define UOTGHS_DEVIER_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_DEVIER) DMA Channel 6 Interrupt Enable */
/* -------- UOTGHS_DEVEPT : (UOTGHS Offset: 0x001C) Device Endpoint Register -------- */
#define UOTGHS_DEVEPT_EPEN0 (0x1u << 0) /**< \brief (UOTGHS_DEVEPT) Endpoint 0 Enable */
#define UOTGHS_DEVEPT_EPEN1 (0x1u << 1) /**< \brief (UOTGHS_DEVEPT) Endpoint 1 Enable */
#define UOTGHS_DEVEPT_EPEN2 (0x1u << 2) /**< \brief (UOTGHS_DEVEPT) Endpoint 2 Enable */
#define UOTGHS_DEVEPT_EPEN3 (0x1u << 3) /**< \brief (UOTGHS_DEVEPT) Endpoint 3 Enable */
#define UOTGHS_DEVEPT_EPEN4 (0x1u << 4) /**< \brief (UOTGHS_DEVEPT) Endpoint 4 Enable */
#define UOTGHS_DEVEPT_EPEN5 (0x1u << 5) /**< \brief (UOTGHS_DEVEPT) Endpoint 5 Enable */
#define UOTGHS_DEVEPT_EPEN6 (0x1u << 6) /**< \brief (UOTGHS_DEVEPT) Endpoint 6 Enable */
#define UOTGHS_DEVEPT_EPEN7 (0x1u << 7) /**< \brief (UOTGHS_DEVEPT) Endpoint 7 Enable */
#define UOTGHS_DEVEPT_EPEN8 (0x1u << 8) /**< \brief (UOTGHS_DEVEPT) Endpoint 8 Enable */
#define UOTGHS_DEVEPT_EPRST0 (0x1u << 16) /**< \brief (UOTGHS_DEVEPT) Endpoint 0 Reset */
#define UOTGHS_DEVEPT_EPRST1 (0x1u << 17) /**< \brief (UOTGHS_DEVEPT) Endpoint 1 Reset */
#define UOTGHS_DEVEPT_EPRST2 (0x1u << 18) /**< \brief (UOTGHS_DEVEPT) Endpoint 2 Reset */
#define UOTGHS_DEVEPT_EPRST3 (0x1u << 19) /**< \brief (UOTGHS_DEVEPT) Endpoint 3 Reset */
#define UOTGHS_DEVEPT_EPRST4 (0x1u << 20) /**< \brief (UOTGHS_DEVEPT) Endpoint 4 Reset */
#define UOTGHS_DEVEPT_EPRST5 (0x1u << 21) /**< \brief (UOTGHS_DEVEPT) Endpoint 5 Reset */
#define UOTGHS_DEVEPT_EPRST6 (0x1u << 22) /**< \brief (UOTGHS_DEVEPT) Endpoint 6 Reset */
#define UOTGHS_DEVEPT_EPRST7 (0x1u << 23) /**< \brief (UOTGHS_DEVEPT) Endpoint 7 Reset */
#define UOTGHS_DEVEPT_EPRST8 (0x1u << 24) /**< \brief (UOTGHS_DEVEPT) Endpoint 8 Reset */
/* -------- UOTGHS_DEVFNUM : (UOTGHS Offset: 0x0020) Device Frame Number Register -------- */
#define UOTGHS_DEVFNUM_MFNUM_Pos 0
#define UOTGHS_DEVFNUM_MFNUM_Msk (0x7u << UOTGHS_DEVFNUM_MFNUM_Pos) /**< \brief (UOTGHS_DEVFNUM) Micro Frame Number */
#define UOTGHS_DEVFNUM_FNUM_Pos 3
#define UOTGHS_DEVFNUM_FNUM_Msk (0x7ffu << UOTGHS_DEVFNUM_FNUM_Pos) /**< \brief (UOTGHS_DEVFNUM) Frame Number */
#define UOTGHS_DEVFNUM_FNCERR (0x1u << 15) /**< \brief (UOTGHS_DEVFNUM) Frame Number CRC Error */
/* -------- UOTGHS_DEVEPTCFG[10] : (UOTGHS Offset: 0x100) Device Endpoint Configuration Register (n = 0) -------- */
#define UOTGHS_DEVEPTCFG_ALLOC (0x1u << 1) /**< \brief (UOTGHS_DEVEPTCFG[10]) Endpoint Memory Allocate */
#define UOTGHS_DEVEPTCFG_EPBK_Pos 2
#define UOTGHS_DEVEPTCFG_EPBK_Msk (0x3u << UOTGHS_DEVEPTCFG_EPBK_Pos) /**< \brief (UOTGHS_DEVEPTCFG[10]) Endpoint Banks */
#define   UOTGHS_DEVEPTCFG_EPBK_1_BANK (0x0u << 2) /**< \brief (UOTGHS_DEVEPTCFG[10]) Single-bank endpoint */
#define   UOTGHS_DEVEPTCFG_EPBK_2_BANK (0x1u << 2) /**< \brief (UOTGHS_DEVEPTCFG[10]) Double-bank endpoint */
#define   UOTGHS_DEVEPTCFG_EPBK_3_BANK (0x2u << 2) /**< \brief (UOTGHS_DEVEPTCFG[10]) Triple-bank endpoint */
#define UOTGHS_DEVEPTCFG_EPSIZE_Pos 4
#define UOTGHS_DEVEPTCFG_EPSIZE_Msk (0x7u << UOTGHS_DEVEPTCFG_EPSIZE_Pos) /**< \brief (UOTGHS_DEVEPTCFG[10]) Endpoint Size */
#define   UOTGHS_DEVEPTCFG_EPSIZE_8_BYTE (0x0u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 8 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_16_BYTE (0x1u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 16 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_32_BYTE (0x2u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 32 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_64_BYTE (0x3u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 64 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_128_BYTE (0x4u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 128 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_256_BYTE (0x5u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 256 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_512_BYTE (0x6u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 512 bytes */
#define   UOTGHS_DEVEPTCFG_EPSIZE_1024_BYTE (0x7u << 4) /**< \brief (UOTGHS_DEVEPTCFG[10]) 1024 bytes */
#define UOTGHS_DEVEPTCFG_EPDIR (0x1u << 8) /**< \brief (UOTGHS_DEVEPTCFG[10]) Endpoint Direction */
#define   UOTGHS_DEVEPTCFG_EPDIR_OUT (0x0u << 8) /**< \brief (UOTGHS_DEVEPTCFG[10]) The endpoint direction is OUT. */
#define   UOTGHS_DEVEPTCFG_EPDIR_IN (0x1u << 8) /**< \brief (UOTGHS_DEVEPTCFG[10]) The endpoint direction is IN (nor for control endpoints). */
#define UOTGHS_DEVEPTCFG_AUTOSW (0x1u << 9) /**< \brief (UOTGHS_DEVEPTCFG[10]) Automatic Switch */
#define UOTGHS_DEVEPTCFG_EPTYPE_Pos 11
#define UOTGHS_DEVEPTCFG_EPTYPE_Msk (0x3u << UOTGHS_DEVEPTCFG_EPTYPE_Pos) /**< \brief (UOTGHS_DEVEPTCFG[10]) Endpoint Type */
#define   UOTGHS_DEVEPTCFG_EPTYPE_CTRL (0x0u << 11) /**< \brief (UOTGHS_DEVEPTCFG[10]) Control */
#define   UOTGHS_DEVEPTCFG_EPTYPE_ISO (0x1u << 11) /**< \brief (UOTGHS_DEVEPTCFG[10]) Isochronous */
#define   UOTGHS_DEVEPTCFG_EPTYPE_BLK (0x2u << 11) /**< \brief (UOTGHS_DEVEPTCFG[10]) Bulk */
#define   UOTGHS_DEVEPTCFG_EPTYPE_INTRPT (0x3u << 11) /**< \brief (UOTGHS_DEVEPTCFG[10]) Interrupt */
#define UOTGHS_DEVEPTCFG_NBTRANS_Pos 13
#define UOTGHS_DEVEPTCFG_NBTRANS_Msk (0x3u << UOTGHS_DEVEPTCFG_NBTRANS_Pos) /**< \brief (UOTGHS_DEVEPTCFG[10]) Number of transaction per microframe for isochronous endpoint */
#define   UOTGHS_DEVEPTCFG_NBTRANS_0_TRANS (0x0u << 13) /**< \brief (UOTGHS_DEVEPTCFG[10]) reserved to endpoint that does not have the high-bandwidth isochronous capability. */
#define   UOTGHS_DEVEPTCFG_NBTRANS_1_TRANS (0x1u << 13) /**< \brief (UOTGHS_DEVEPTCFG[10]) default value: one transaction per micro-frame. */
#define   UOTGHS_DEVEPTCFG_NBTRANS_2_TRANS (0x2u << 13) /**< \brief (UOTGHS_DEVEPTCFG[10]) 2 transactions per micro-frame. This endpoint should be configured as double-bank. */
#define   UOTGHS_DEVEPTCFG_NBTRANS_3_TRANS (0x3u << 13) /**< \brief (UOTGHS_DEVEPTCFG[10]) 3 transactions per micro-frame. This endpoint should be configured as triple-bank. */
/* -------- UOTGHS_DEVEPTISR[10] : (UOTGHS Offset: 0x130) Device Endpoint Status Register (n = 0) -------- */
#define UOTGHS_DEVEPTISR_TXINI (0x1u << 0) /**< \brief (UOTGHS_DEVEPTISR[10]) Transmitted IN Data Interrupt */
#define UOTGHS_DEVEPTISR_RXOUTI (0x1u << 1) /**< \brief (UOTGHS_DEVEPTISR[10]) Received OUT Data Interrupt */
#define UOTGHS_DEVEPTISR_RXSTPI (0x1u << 2) /**< \brief (UOTGHS_DEVEPTISR[10]) Received SETUP Interrupt */
#define UOTGHS_DEVEPTISR_UNDERFI (0x1u << 2) /**< \brief (UOTGHS_DEVEPTISR[10]) Underflow Interrupt */
#define UOTGHS_DEVEPTISR_NAKOUTI (0x1u << 3) /**< \brief (UOTGHS_DEVEPTISR[10]) NAKed OUT Interrupt */
#define UOTGHS_DEVEPTISR_HBISOINERRI (0x1u << 3) /**< \brief (UOTGHS_DEVEPTISR[10]) High bandwidth isochronous IN Underflow Error Interrupt */
#define UOTGHS_DEVEPTISR_NAKINI (0x1u << 4) /**< \brief (UOTGHS_DEVEPTISR[10]) NAKed IN Interrupt */
#define UOTGHS_DEVEPTISR_HBISOFLUSHI (0x1u << 4) /**< \brief (UOTGHS_DEVEPTISR[10]) High Bandwidth Isochronous IN Flush Interrupt */
#define UOTGHS_DEVEPTISR_OVERFI (0x1u << 5) /**< \brief (UOTGHS_DEVEPTISR[10]) Overflow Interrupt */
#define UOTGHS_DEVEPTISR_STALLEDI (0x1u << 6) /**< \brief (UOTGHS_DEVEPTISR[10]) STALLed Interrupt */
#define UOTGHS_DEVEPTISR_CRCERRI (0x1u << 6) /**< \brief (UOTGHS_DEVEPTISR[10]) CRC Error Interrupt */
#define UOTGHS_DEVEPTISR_SHORTPACKET (0x1u << 7) /**< \brief (UOTGHS_DEVEPTISR[10]) Short Packet Interrupt */
#define UOTGHS_DEVEPTISR_DTSEQ_Pos 8
#define UOTGHS_DEVEPTISR_DTSEQ_Msk (0x3u << UOTGHS_DEVEPTISR_DTSEQ_Pos) /**< \brief (UOTGHS_DEVEPTISR[10]) Data Toggle Sequence */
#define   UOTGHS_DEVEPTISR_DTSEQ_DATA0 (0x0u << 8) /**< \brief (UOTGHS_DEVEPTISR[10]) Data0 toggle sequence */
#define   UOTGHS_DEVEPTISR_DTSEQ_DATA1 (0x1u << 8) /**< \brief (UOTGHS_DEVEPTISR[10]) Data1 toggle sequence */
#define   UOTGHS_DEVEPTISR_DTSEQ_DATA2 (0x2u << 8) /**< \brief (UOTGHS_DEVEPTISR[10]) Data2 toggle sequence (for high-bandwidth isochronous endpoint) */
#define   UOTGHS_DEVEPTISR_DTSEQ_MDATA (0x3u << 8) /**< \brief (UOTGHS_DEVEPTISR[10]) MData toggle sequence (for high-bandwidth isochronous endpoint) */
#define UOTGHS_DEVEPTISR_ERRORTRANS (0x1u << 10) /**< \brief (UOTGHS_DEVEPTISR[10]) High-bandwidth isochronous OUT endpoint transaction error Interrupt */
#define UOTGHS_DEVEPTISR_NBUSYBK_Pos 12
#define UOTGHS_DEVEPTISR_NBUSYBK_Msk (0x3u << UOTGHS_DEVEPTISR_NBUSYBK_Pos) /**< \brief (UOTGHS_DEVEPTISR[10]) Number of Busy Banks */
#define   UOTGHS_DEVEPTISR_NBUSYBK_0_BUSY (0x0u << 12) /**< \brief (UOTGHS_DEVEPTISR[10]) 0 busy bank (all banks free) */
#define   UOTGHS_DEVEPTISR_NBUSYBK_1_BUSY (0x1u << 12) /**< \brief (UOTGHS_DEVEPTISR[10]) 1 busy bank */
#define   UOTGHS_DEVEPTISR_NBUSYBK_2_BUSY (0x2u << 12) /**< \brief (UOTGHS_DEVEPTISR[10]) 2 busy banks */
#define   UOTGHS_DEVEPTISR_NBUSYBK_3_BUSY (0x3u << 12) /**< \brief (UOTGHS_DEVEPTISR[10]) 3 busy banks */
#define UOTGHS_DEVEPTISR_CURRBK_Pos 14
#define UOTGHS_DEVEPTISR_CURRBK_Msk (0x3u << UOTGHS_DEVEPTISR_CURRBK_Pos) /**< \brief (UOTGHS_DEVEPTISR[10]) Current Bank */
#define   UOTGHS_DEVEPTISR_CURRBK_BANK0 (0x0u << 14) /**< \brief (UOTGHS_DEVEPTISR[10]) Current bank is bank0 */
#define   UOTGHS_DEVEPTISR_CURRBK_BANK1 (0x1u << 14) /**< \brief (UOTGHS_DEVEPTISR[10]) Current bank is bank1 */
#define   UOTGHS_DEVEPTISR_CURRBK_BANK2 (0x2u << 14) /**< \brief (UOTGHS_DEVEPTISR[10]) Current bank is bank2 */
#define UOTGHS_DEVEPTISR_RWALL (0x1u << 16) /**< \brief (UOTGHS_DEVEPTISR[10]) Read-write Allowed */
#define UOTGHS_DEVEPTISR_CTRLDIR (0x1u << 17) /**< \brief (UOTGHS_DEVEPTISR[10]) Control Direction */
#define UOTGHS_DEVEPTISR_CFGOK (0x1u << 18) /**< \brief (UOTGHS_DEVEPTISR[10]) Configuration OK Status */
#define UOTGHS_DEVEPTISR_BYCT_Pos 20
#define UOTGHS_DEVEPTISR_BYCT_Msk (0x7ffu << UOTGHS_DEVEPTISR_BYCT_Pos) /**< \brief (UOTGHS_DEVEPTISR[10]) Byte Count */
/* -------- UOTGHS_DEVEPTICR[10] : (UOTGHS Offset: 0x160) Device Endpoint Clear Register (n = 0) -------- */
#define UOTGHS_DEVEPTICR_TXINIC (0x1u << 0) /**< \brief (UOTGHS_DEVEPTICR[10]) Transmitted IN Data Interrupt Clear */
#define UOTGHS_DEVEPTICR_RXOUTIC (0x1u << 1) /**< \brief (UOTGHS_DEVEPTICR[10]) Received OUT Data Interrupt Clear */
#define UOTGHS_DEVEPTICR_RXSTPIC (0x1u << 2) /**< \brief (UOTGHS_DEVEPTICR[10]) Received SETUP Interrupt Clear */
#define UOTGHS_DEVEPTICR_UNDERFIC (0x1u << 2) /**< \brief (UOTGHS_DEVEPTICR[10]) Underflow Interrupt Clear */
#define UOTGHS_DEVEPTICR_NAKOUTIC (0x1u << 3) /**< \brief (UOTGHS_DEVEPTICR[10]) NAKed OUT Interrupt Clear */
#define UOTGHS_DEVEPTICR_HBISOINERRIC (0x1u << 3) /**< \brief (UOTGHS_DEVEPTICR[10]) High bandwidth isochronous IN Underflow Error Interrupt Clear */
#define UOTGHS_DEVEPTICR_NAKINIC (0x1u << 4) /**< \brief (UOTGHS_DEVEPTICR[10]) NAKed IN Interrupt Clear */
#define UOTGHS_DEVEPTICR_HBISOFLUSHIC (0x1u << 4) /**< \brief (UOTGHS_DEVEPTICR[10]) High Bandwidth Isochronous IN Flush Interrupt Clear */
#define UOTGHS_DEVEPTICR_OVERFIC (0x1u << 5) /**< \brief (UOTGHS_DEVEPTICR[10]) Overflow Interrupt Clear */
#define UOTGHS_DEVEPTICR_STALLEDIC (0x1u << 6) /**< \brief (UOTGHS_DEVEPTICR[10]) STALLed Interrupt Clear */
#define UOTGHS_DEVEPTICR_CRCERRIC (0x1u << 6) /**< \brief (UOTGHS_DEVEPTICR[10]) CRC Error Interrupt Clear */
#define UOTGHS_DEVEPTICR_SHORTPACKETC (0x1u << 7) /**< \brief (UOTGHS_DEVEPTICR[10]) Short Packet Interrupt Clear */
/* -------- UOTGHS_DEVEPTIFR[10] : (UOTGHS Offset: 0x190) Device Endpoint Set Register (n = 0) -------- */
#define UOTGHS_DEVEPTIFR_TXINIS (0x1u << 0) /**< \brief (UOTGHS_DEVEPTIFR[10]) Transmitted IN Data Interrupt Set */
#define UOTGHS_DEVEPTIFR_RXOUTIS (0x1u << 1) /**< \brief (UOTGHS_DEVEPTIFR[10]) Received OUT Data Interrupt Set */
#define UOTGHS_DEVEPTIFR_RXSTPIS (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIFR[10]) Received SETUP Interrupt Set */
#define UOTGHS_DEVEPTIFR_UNDERFIS (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIFR[10]) Underflow Interrupt Set */
#define UOTGHS_DEVEPTIFR_NAKOUTIS (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIFR[10]) NAKed OUT Interrupt Set */
#define UOTGHS_DEVEPTIFR_HBISOINERRIS (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIFR[10]) High bandwidth isochronous IN Underflow Error Interrupt Set */
#define UOTGHS_DEVEPTIFR_NAKINIS (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIFR[10]) NAKed IN Interrupt Set */
#define UOTGHS_DEVEPTIFR_HBISOFLUSHIS (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIFR[10]) High Bandwidth Isochronous IN Flush Interrupt Set */
#define UOTGHS_DEVEPTIFR_OVERFIS (0x1u << 5) /**< \brief (UOTGHS_DEVEPTIFR[10]) Overflow Interrupt Set */
#define UOTGHS_DEVEPTIFR_STALLEDIS (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIFR[10]) STALLed Interrupt Set */
#define UOTGHS_DEVEPTIFR_CRCERRIS (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIFR[10]) CRC Error Interrupt Set */
#define UOTGHS_DEVEPTIFR_SHORTPACKETS (0x1u << 7) /**< \brief (UOTGHS_DEVEPTIFR[10]) Short Packet Interrupt Set */
#define UOTGHS_DEVEPTIFR_NBUSYBKS (0x1u << 12) /**< \brief (UOTGHS_DEVEPTIFR[10]) Number of Busy Banks Interrupt Set */
/* -------- UOTGHS_DEVEPTIMR[10] : (UOTGHS Offset: 0x1C0) Device Endpoint Mask Register (n = 0) -------- */
#define UOTGHS_DEVEPTIMR_TXINE (0x1u << 0) /**< \brief (UOTGHS_DEVEPTIMR[10]) Transmitted IN Data Interrupt */
#define UOTGHS_DEVEPTIMR_RXOUTE (0x1u << 1) /**< \brief (UOTGHS_DEVEPTIMR[10]) Received OUT Data Interrupt */
#define UOTGHS_DEVEPTIMR_RXSTPE (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIMR[10]) Received SETUP Interrupt */
#define UOTGHS_DEVEPTIMR_UNDERFE (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIMR[10]) Underflow Interrupt */
#define UOTGHS_DEVEPTIMR_NAKOUTE (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIMR[10]) NAKed OUT Interrupt */
#define UOTGHS_DEVEPTIMR_HBISOINERRE (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIMR[10]) High Bandwidth Isochronous IN Error Interrupt */
#define UOTGHS_DEVEPTIMR_NAKINE (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIMR[10]) NAKed IN Interrupt */
#define UOTGHS_DEVEPTIMR_HBISOFLUSHE (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIMR[10]) High Bandwidth Isochronous IN Flush Interrupt */
#define UOTGHS_DEVEPTIMR_OVERFE (0x1u << 5) /**< \brief (UOTGHS_DEVEPTIMR[10]) Overflow Interrupt */
#define UOTGHS_DEVEPTIMR_STALLEDE (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIMR[10]) STALLed Interrupt */
#define UOTGHS_DEVEPTIMR_CRCERRE (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIMR[10]) CRC Error Interrupt */
#define UOTGHS_DEVEPTIMR_SHORTPACKETE (0x1u << 7) /**< \brief (UOTGHS_DEVEPTIMR[10]) Short Packet Interrupt */
#define UOTGHS_DEVEPTIMR_MDATAE (0x1u << 8) /**< \brief (UOTGHS_DEVEPTIMR[10]) MData Interrupt */
#define UOTGHS_DEVEPTIMR_DATAXE (0x1u << 9) /**< \brief (UOTGHS_DEVEPTIMR[10]) DataX Interrupt */
#define UOTGHS_DEVEPTIMR_ERRORTRANSE (0x1u << 10) /**< \brief (UOTGHS_DEVEPTIMR[10]) Transaction Error Interrupt */
#define UOTGHS_DEVEPTIMR_NBUSYBKE (0x1u << 12) /**< \brief (UOTGHS_DEVEPTIMR[10]) Number of Busy Banks Interrupt */
#define UOTGHS_DEVEPTIMR_KILLBK (0x1u << 13) /**< \brief (UOTGHS_DEVEPTIMR[10]) Kill IN Bank */
#define UOTGHS_DEVEPTIMR_FIFOCON (0x1u << 14) /**< \brief (UOTGHS_DEVEPTIMR[10]) FIFO Control */
#define UOTGHS_DEVEPTIMR_EPDISHDMA (0x1u << 16) /**< \brief (UOTGHS_DEVEPTIMR[10]) Endpoint Interrupts Disable HDMA Request */
#define UOTGHS_DEVEPTIMR_NYETDIS (0x1u << 17) /**< \brief (UOTGHS_DEVEPTIMR[10]) NYET Token Disable */
#define UOTGHS_DEVEPTIMR_RSTDT (0x1u << 18) /**< \brief (UOTGHS_DEVEPTIMR[10]) Reset Data Toggle */
#define UOTGHS_DEVEPTIMR_STALLRQ (0x1u << 19) /**< \brief (UOTGHS_DEVEPTIMR[10]) STALL Request */
/* -------- UOTGHS_DEVEPTIER[10] : (UOTGHS Offset: 0x1F0) Device Endpoint Enable Register (n = 0) -------- */
#define UOTGHS_DEVEPTIER_TXINES (0x1u << 0) /**< \brief (UOTGHS_DEVEPTIER[10]) Transmitted IN Data Interrupt Enable */
#define UOTGHS_DEVEPTIER_RXOUTES (0x1u << 1) /**< \brief (UOTGHS_DEVEPTIER[10]) Received OUT Data Interrupt Enable */
#define UOTGHS_DEVEPTIER_RXSTPES (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIER[10]) Received SETUP Interrupt Enable */
#define UOTGHS_DEVEPTIER_UNDERFES (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIER[10]) Underflow Interrupt Enable */
#define UOTGHS_DEVEPTIER_NAKOUTES (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIER[10]) NAKed OUT Interrupt Enable */
#define UOTGHS_DEVEPTIER_HBISOINERRES (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIER[10]) High Bandwidth Isochronous IN Error Interrupt Enable */
#define UOTGHS_DEVEPTIER_NAKINES (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIER[10]) NAKed IN Interrupt Enable */
#define UOTGHS_DEVEPTIER_HBISOFLUSHES (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIER[10]) High Bandwidth Isochronous IN Flush Interrupt Enable */
#define UOTGHS_DEVEPTIER_OVERFES (0x1u << 5) /**< \brief (UOTGHS_DEVEPTIER[10]) Overflow Interrupt Enable */
#define UOTGHS_DEVEPTIER_STALLEDES (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIER[10]) STALLed Interrupt Enable */
#define UOTGHS_DEVEPTIER_CRCERRES (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIER[10]) CRC Error Interrupt Enable */
#define UOTGHS_DEVEPTIER_SHORTPACKETES (0x1u << 7) /**< \brief (UOTGHS_DEVEPTIER[10]) Short Packet Interrupt Enable */
#define UOTGHS_DEVEPTIER_MDATAES (0x1u << 8) /**< \brief (UOTGHS_DEVEPTIER[10]) MData Interrupt Enable */
#define UOTGHS_DEVEPTIER_DATAXES (0x1u << 9) /**< \brief (UOTGHS_DEVEPTIER[10]) DataX Interrupt Enable */
#define UOTGHS_DEVEPTIER_ERRORTRANSES (0x1u << 10) /**< \brief (UOTGHS_DEVEPTIER[10]) Transaction Error Interrupt Enable */
#define UOTGHS_DEVEPTIER_NBUSYBKES (0x1u << 12) /**< \brief (UOTGHS_DEVEPTIER[10]) Number of Busy Banks Interrupt Enable */
#define UOTGHS_DEVEPTIER_KILLBKS (0x1u << 13) /**< \brief (UOTGHS_DEVEPTIER[10]) Kill IN Bank */
#define UOTGHS_DEVEPTIER_EPDISHDMAS (0x1u << 16) /**< \brief (UOTGHS_DEVEPTIER[10]) Endpoint Interrupts Disable HDMA Request Enable */
#define UOTGHS_DEVEPTIER_NYETDISS (0x1u << 17) /**< \brief (UOTGHS_DEVEPTIER[10]) NYET Token Disable Enable */
#define UOTGHS_DEVEPTIER_RSTDTS (0x1u << 18) /**< \brief (UOTGHS_DEVEPTIER[10]) Reset Data Toggle Enable */
#define UOTGHS_DEVEPTIER_STALLRQS (0x1u << 19) /**< \brief (UOTGHS_DEVEPTIER[10]) STALL Request Enable */
/* -------- UOTGHS_DEVEPTIDR[10] : (UOTGHS Offset: 0x220) Device Endpoint Disable Register (n = 0) -------- */
#define UOTGHS_DEVEPTIDR_TXINEC (0x1u << 0) /**< \brief (UOTGHS_DEVEPTIDR[10]) Transmitted IN Interrupt Clear */
#define UOTGHS_DEVEPTIDR_RXOUTEC (0x1u << 1) /**< \brief (UOTGHS_DEVEPTIDR[10]) Received OUT Data Interrupt Clear */
#define UOTGHS_DEVEPTIDR_RXSTPEC (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIDR[10]) Received SETUP Interrupt Clear */
#define UOTGHS_DEVEPTIDR_UNDERFEC (0x1u << 2) /**< \brief (UOTGHS_DEVEPTIDR[10]) Underflow Interrupt Clear */
#define UOTGHS_DEVEPTIDR_NAKOUTEC (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIDR[10]) NAKed OUT Interrupt Clear */
#define UOTGHS_DEVEPTIDR_HBISOINERREC (0x1u << 3) /**< \brief (UOTGHS_DEVEPTIDR[10]) High Bandwidth Isochronous IN Error Interrupt Clear */
#define UOTGHS_DEVEPTIDR_NAKINEC (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIDR[10]) NAKed IN Interrupt Clear */
#define UOTGHS_DEVEPTIDR_HBISOFLUSHEC (0x1u << 4) /**< \brief (UOTGHS_DEVEPTIDR[10]) High Bandwidth Isochronous IN Flush Interrupt Clear */
#define UOTGHS_DEVEPTIDR_OVERFEC (0x1u << 5) /**< \brief (UOTGHS_DEVEPTIDR[10]) Overflow Interrupt Clear */
#define UOTGHS_DEVEPTIDR_STALLEDEC (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIDR[10]) STALLed Interrupt Clear */
#define UOTGHS_DEVEPTIDR_CRCERREC (0x1u << 6) /**< \brief (UOTGHS_DEVEPTIDR[10]) CRC Error Interrupt Clear */
#define UOTGHS_DEVEPTIDR_SHORTPACKETEC (0x1u << 7) /**< \brief (UOTGHS_DEVEPTIDR[10]) Shortpacket Interrupt Clear */
#define UOTGHS_DEVEPTIDR_MDATEC (0x1u << 8) /**< \brief (UOTGHS_DEVEPTIDR[10]) MData Interrupt Clear */
#define UOTGHS_DEVEPTIDR_DATAXEC (0x1u << 9) /**< \brief (UOTGHS_DEVEPTIDR[10]) DataX Interrupt Clear */
#define UOTGHS_DEVEPTIDR_ERRORTRANSEC (0x1u << 10) /**< \brief (UOTGHS_DEVEPTIDR[10]) Transaction Error Interrupt Clear */
#define UOTGHS_DEVEPTIDR_NBUSYBKEC (0x1u << 12) /**< \brief (UOTGHS_DEVEPTIDR[10]) Number of Busy Banks Interrupt Clear */
#define UOTGHS_DEVEPTIDR_FIFOCONC (0x1u << 14) /**< \brief (UOTGHS_DEVEPTIDR[10]) FIFO Control Clear */
#define UOTGHS_DEVEPTIDR_EPDISHDMAC (0x1u << 16) /**< \brief (UOTGHS_DEVEPTIDR[10]) Endpoint Interrupts Disable HDMA Request Clear */
#define UOTGHS_DEVEPTIDR_NYETDISC (0x1u << 17) /**< \brief (UOTGHS_DEVEPTIDR[10]) NYET Token Disable Clear */
#define UOTGHS_DEVEPTIDR_STALLRQC (0x1u << 19) /**< \brief (UOTGHS_DEVEPTIDR[10]) STALL Request Clear */
/* -------- UOTGHS_DEVDMANXTDSC : (UOTGHS Offset: N/A) Device DMA Channel Next Descriptor Address Register -------- */
#define UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD_Pos 0
#define UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD_Msk (0xffffffffu << UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD_Pos) /**< \brief (UOTGHS_DEVDMANXTDSC) Next Descriptor Address */
#define UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD(value) ((UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD_Msk & ((value) << UOTGHS_DEVDMANXTDSC_NXT_DSC_ADD_Pos)))
/* -------- UOTGHS_DEVDMAADDRESS : (UOTGHS Offset: N/A) Device DMA Channel Address Register -------- */
#define UOTGHS_DEVDMAADDRESS_BUFF_ADD_Pos 0
#define UOTGHS_DEVDMAADDRESS_BUFF_ADD_Msk (0xffffffffu << UOTGHS_DEVDMAADDRESS_BUFF_ADD_Pos) /**< \brief (UOTGHS_DEVDMAADDRESS) Buffer Address */
#define UOTGHS_DEVDMAADDRESS_BUFF_ADD(value) ((UOTGHS_DEVDMAADDRESS_BUFF_ADD_Msk & ((value) << UOTGHS_DEVDMAADDRESS_BUFF_ADD_Pos)))
/* -------- UOTGHS_DEVDMACONTROL : (UOTGHS Offset: N/A) Device DMA Channel Control Register -------- */
#define UOTGHS_DEVDMACONTROL_CHANN_ENB (0x1u << 0) /**< \brief (UOTGHS_DEVDMACONTROL) Channel Enable Command */
#define UOTGHS_DEVDMACONTROL_LDNXT_DSC (0x1u << 1) /**< \brief (UOTGHS_DEVDMACONTROL) Load Next Channel Transfer Descriptor Enable Command */
#define UOTGHS_DEVDMACONTROL_END_TR_EN (0x1u << 2) /**< \brief (UOTGHS_DEVDMACONTROL) End of Transfer Enable Control */
#define UOTGHS_DEVDMACONTROL_END_B_EN (0x1u << 3) /**< \brief (UOTGHS_DEVDMACONTROL) End of Buffer Enable Control */
#define UOTGHS_DEVDMACONTROL_END_TR_IT (0x1u << 4) /**< \brief (UOTGHS_DEVDMACONTROL) End of Transfer Interrupt Enable */
#define UOTGHS_DEVDMACONTROL_END_BUFFIT (0x1u << 5) /**< \brief (UOTGHS_DEVDMACONTROL) End of Buffer Interrupt Enable */
#define UOTGHS_DEVDMACONTROL_DESC_LD_IT (0x1u << 6) /**< \brief (UOTGHS_DEVDMACONTROL) Descriptor Loaded Interrupt Enable */
#define UOTGHS_DEVDMACONTROL_BURST_LCK (0x1u << 7) /**< \brief (UOTGHS_DEVDMACONTROL) Burst Lock Enable */
#define UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Pos 16
#define UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Msk (0xffffu << UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Pos) /**< \brief (UOTGHS_DEVDMACONTROL) Buffer Byte Length (Write-only) */
#define UOTGHS_DEVDMACONTROL_BUFF_LENGTH(value) ((UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Msk & ((value) << UOTGHS_DEVDMACONTROL_BUFF_LENGTH_Pos)))
/* -------- UOTGHS_DEVDMASTATUS : (UOTGHS Offset: N/A) Device DMA Channel Status Register -------- */
#define UOTGHS_DEVDMASTATUS_CHANN_ENB (0x1u << 0) /**< \brief (UOTGHS_DEVDMASTATUS) Channel Enable Status */
#define UOTGHS_DEVDMASTATUS_CHANN_ACT (0x1u << 1) /**< \brief (UOTGHS_DEVDMASTATUS) Channel Active Status */
#define UOTGHS_DEVDMASTATUS_END_TR_ST (0x1u << 4) /**< \brief (UOTGHS_DEVDMASTATUS) End of Channel Transfer Status */
#define UOTGHS_DEVDMASTATUS_END_BF_ST (0x1u << 5) /**< \brief (UOTGHS_DEVDMASTATUS) End of Channel Buffer Status */
#define UOTGHS_DEVDMASTATUS_DESC_LDST (0x1u << 6) /**< \brief (UOTGHS_DEVDMASTATUS) Descriptor Loaded Status */
#define UOTGHS_DEVDMASTATUS_BUFF_COUNT_Pos 16
#define UOTGHS_DEVDMASTATUS_BUFF_COUNT_Msk (0xffffu << UOTGHS_DEVDMASTATUS_BUFF_COUNT_Pos) /**< \brief (UOTGHS_DEVDMASTATUS) Buffer Byte Count */
#define UOTGHS_DEVDMASTATUS_BUFF_COUNT(value) ((UOTGHS_DEVDMASTATUS_BUFF_COUNT_Msk & ((value) << UOTGHS_DEVDMASTATUS_BUFF_COUNT_Pos)))
/* -------- UOTGHS_HSTCTRL : (UOTGHS Offset: 0x0400) Host General Control Register -------- */
#define UOTGHS_HSTCTRL_SOFE (0x1u << 8) /**< \brief (UOTGHS_HSTCTRL) Start of Frame Generation Enable */
#define UOTGHS_HSTCTRL_RESET (0x1u << 9) /**< \brief (UOTGHS_HSTCTRL) Send USB Reset */
#define UOTGHS_HSTCTRL_RESUME (0x1u << 10) /**< \brief (UOTGHS_HSTCTRL) Send USB Resume */
#define UOTGHS_HSTCTRL_SPDCONF_Pos 12
#define UOTGHS_HSTCTRL_SPDCONF_Msk (0x3u << UOTGHS_HSTCTRL_SPDCONF_Pos) /**< \brief (UOTGHS_HSTCTRL) Mode Configuration */
#define   UOTGHS_HSTCTRL_SPDCONF_NORMAL (0x0u << 12) /**< \brief (UOTGHS_HSTCTRL) The host starts in full-speed mode and performs a high-speed reset to switch to the high-speed mode if the downstream peripheral is high-speed capable. */
#define   UOTGHS_HSTCTRL_SPDCONF_LOW_POWER (0x1u << 12) /**< \brief (UOTGHS_HSTCTRL) For a better consumption, if high-speed is not needed. */
#define   UOTGHS_HSTCTRL_SPDCONF_HIGH_SPEED (0x2u << 12) /**< \brief (UOTGHS_HSTCTRL) Forced high speed. */
#define   UOTGHS_HSTCTRL_SPDCONF_FORCED_FS (0x3u << 12) /**< \brief (UOTGHS_HSTCTRL) The host remains to full-speed mode whatever the peripheral speed capability. */
/* -------- UOTGHS_HSTISR : (UOTGHS Offset: 0x0404) Host Global Interrupt Status Register -------- */
#define UOTGHS_HSTISR_DCONNI (0x1u << 0) /**< \brief (UOTGHS_HSTISR) Device Connection Interrupt */
#define UOTGHS_HSTISR_DDISCI (0x1u << 1) /**< \brief (UOTGHS_HSTISR) Device Disconnection Interrupt */
#define UOTGHS_HSTISR_RSTI (0x1u << 2) /**< \brief (UOTGHS_HSTISR) USB Reset Sent Interrupt */
#define UOTGHS_HSTISR_RSMEDI (0x1u << 3) /**< \brief (UOTGHS_HSTISR) Downstream Resume Sent Interrupt */
#define UOTGHS_HSTISR_RXRSMI (0x1u << 4) /**< \brief (UOTGHS_HSTISR) Upstream Resume Received Interrupt */
#define UOTGHS_HSTISR_HSOFI (0x1u << 5) /**< \brief (UOTGHS_HSTISR) Host Start of Frame Interrupt */
#define UOTGHS_HSTISR_HWUPI (0x1u << 6) /**< \brief (UOTGHS_HSTISR) Host Wake-Up Interrupt */
#define UOTGHS_HSTISR_PEP_0 (0x1u << 8) /**< \brief (UOTGHS_HSTISR) Pipe 0 Interrupt */
#define UOTGHS_HSTISR_PEP_1 (0x1u << 9) /**< \brief (UOTGHS_HSTISR) Pipe 1 Interrupt */
#define UOTGHS_HSTISR_PEP_2 (0x1u << 10) /**< \brief (UOTGHS_HSTISR) Pipe 2 Interrupt */
#define UOTGHS_HSTISR_PEP_3 (0x1u << 11) /**< \brief (UOTGHS_HSTISR) Pipe 3 Interrupt */
#define UOTGHS_HSTISR_PEP_4 (0x1u << 12) /**< \brief (UOTGHS_HSTISR) Pipe 4 Interrupt */
#define UOTGHS_HSTISR_PEP_5 (0x1u << 13) /**< \brief (UOTGHS_HSTISR) Pipe 5 Interrupt */
#define UOTGHS_HSTISR_PEP_6 (0x1u << 14) /**< \brief (UOTGHS_HSTISR) Pipe 6 Interrupt */
#define UOTGHS_HSTISR_PEP_7 (0x1u << 15) /**< \brief (UOTGHS_HSTISR) Pipe 7 Interrupt */
#define UOTGHS_HSTISR_PEP_8 (0x1u << 16) /**< \brief (UOTGHS_HSTISR) Pipe 8 Interrupt */
#define UOTGHS_HSTISR_PEP_9 (0x1u << 17) /**< \brief (UOTGHS_HSTISR) Pipe 9 Interrupt */
#define UOTGHS_HSTISR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_HSTISR) DMA Channel 1 Interrupt */
#define UOTGHS_HSTISR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_HSTISR) DMA Channel 2 Interrupt */
#define UOTGHS_HSTISR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_HSTISR) DMA Channel 3 Interrupt */
#define UOTGHS_HSTISR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_HSTISR) DMA Channel 4 Interrupt */
#define UOTGHS_HSTISR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_HSTISR) DMA Channel 5 Interrupt */
#define UOTGHS_HSTISR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_HSTISR) DMA Channel 6 Interrupt */
/* -------- UOTGHS_HSTICR : (UOTGHS Offset: 0x0408) Host Global Interrupt Clear Register -------- */
#define UOTGHS_HSTICR_DCONNIC (0x1u << 0) /**< \brief (UOTGHS_HSTICR) Device Connection Interrupt Clear */
#define UOTGHS_HSTICR_DDISCIC (0x1u << 1) /**< \brief (UOTGHS_HSTICR) Device Disconnection Interrupt Clear */
#define UOTGHS_HSTICR_RSTIC (0x1u << 2) /**< \brief (UOTGHS_HSTICR) USB Reset Sent Interrupt Clear */
#define UOTGHS_HSTICR_RSMEDIC (0x1u << 3) /**< \brief (UOTGHS_HSTICR) Downstream Resume Sent Interrupt Clear */
#define UOTGHS_HSTICR_RXRSMIC (0x1u << 4) /**< \brief (UOTGHS_HSTICR) Upstream Resume Received Interrupt Clear */
#define UOTGHS_HSTICR_HSOFIC (0x1u << 5) /**< \brief (UOTGHS_HSTICR) Host Start of Frame Interrupt Clear */
#define UOTGHS_HSTICR_HWUPIC (0x1u << 6) /**< \brief (UOTGHS_HSTICR) Host Wake-Up Interrupt Clear */
/* -------- UOTGHS_HSTIFR : (UOTGHS Offset: 0x040C) Host Global Interrupt Set Register -------- */
#define UOTGHS_HSTIFR_DCONNIS (0x1u << 0) /**< \brief (UOTGHS_HSTIFR) Device Connection Interrupt Set */
#define UOTGHS_HSTIFR_DDISCIS (0x1u << 1) /**< \brief (UOTGHS_HSTIFR) Device Disconnection Interrupt Set */
#define UOTGHS_HSTIFR_RSTIS (0x1u << 2) /**< \brief (UOTGHS_HSTIFR) USB Reset Sent Interrupt Set */
#define UOTGHS_HSTIFR_RSMEDIS (0x1u << 3) /**< \brief (UOTGHS_HSTIFR) Downstream Resume Sent Interrupt Set */
#define UOTGHS_HSTIFR_RXRSMIS (0x1u << 4) /**< \brief (UOTGHS_HSTIFR) Upstream Resume Received Interrupt Set */
#define UOTGHS_HSTIFR_HSOFIS (0x1u << 5) /**< \brief (UOTGHS_HSTIFR) Host Start of Frame Interrupt Set */
#define UOTGHS_HSTIFR_HWUPIS (0x1u << 6) /**< \brief (UOTGHS_HSTIFR) Host Wake-Up Interrupt Set */
#define UOTGHS_HSTIFR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_HSTIFR) DMA Channel 1 Interrupt Set */
#define UOTGHS_HSTIFR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_HSTIFR) DMA Channel 2 Interrupt Set */
#define UOTGHS_HSTIFR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_HSTIFR) DMA Channel 3 Interrupt Set */
#define UOTGHS_HSTIFR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_HSTIFR) DMA Channel 4 Interrupt Set */
#define UOTGHS_HSTIFR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_HSTIFR) DMA Channel 5 Interrupt Set */
#define UOTGHS_HSTIFR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_HSTIFR) DMA Channel 6 Interrupt Set */
/* -------- UOTGHS_HSTIMR : (UOTGHS Offset: 0x0410) Host Global Interrupt Mask Register -------- */
#define UOTGHS_HSTIMR_DCONNIE (0x1u << 0) /**< \brief (UOTGHS_HSTIMR) Device Connection Interrupt Enable */
#define UOTGHS_HSTIMR_DDISCIE (0x1u << 1) /**< \brief (UOTGHS_HSTIMR) Device Disconnection Interrupt Enable */
#define UOTGHS_HSTIMR_RSTIE (0x1u << 2) /**< \brief (UOTGHS_HSTIMR) USB Reset Sent Interrupt Enable */
#define UOTGHS_HSTIMR_RSMEDIE (0x1u << 3) /**< \brief (UOTGHS_HSTIMR) Downstream Resume Sent Interrupt Enable */
#define UOTGHS_HSTIMR_RXRSMIE (0x1u << 4) /**< \brief (UOTGHS_HSTIMR) Upstream Resume Received Interrupt Enable */
#define UOTGHS_HSTIMR_HSOFIE (0x1u << 5) /**< \brief (UOTGHS_HSTIMR) Host Start of Frame Interrupt Enable */
#define UOTGHS_HSTIMR_HWUPIE (0x1u << 6) /**< \brief (UOTGHS_HSTIMR) Host Wake-Up Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_0 (0x1u << 8) /**< \brief (UOTGHS_HSTIMR) Pipe 0 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_1 (0x1u << 9) /**< \brief (UOTGHS_HSTIMR) Pipe 1 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_2 (0x1u << 10) /**< \brief (UOTGHS_HSTIMR) Pipe 2 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_3 (0x1u << 11) /**< \brief (UOTGHS_HSTIMR) Pipe 3 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_4 (0x1u << 12) /**< \brief (UOTGHS_HSTIMR) Pipe 4 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_5 (0x1u << 13) /**< \brief (UOTGHS_HSTIMR) Pipe 5 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_6 (0x1u << 14) /**< \brief (UOTGHS_HSTIMR) Pipe 6 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_7 (0x1u << 15) /**< \brief (UOTGHS_HSTIMR) Pipe 7 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_8 (0x1u << 16) /**< \brief (UOTGHS_HSTIMR) Pipe 8 Interrupt Enable */
#define UOTGHS_HSTIMR_PEP_9 (0x1u << 17) /**< \brief (UOTGHS_HSTIMR) Pipe 9 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_HSTIMR) DMA Channel 1 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_HSTIMR) DMA Channel 2 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_HSTIMR) DMA Channel 3 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_HSTIMR) DMA Channel 4 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_HSTIMR) DMA Channel 5 Interrupt Enable */
#define UOTGHS_HSTIMR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_HSTIMR) DMA Channel 6 Interrupt Enable */
/* -------- UOTGHS_HSTIDR : (UOTGHS Offset: 0x0414) Host Global Interrupt Disable Register -------- */
#define UOTGHS_HSTIDR_DCONNIEC (0x1u << 0) /**< \brief (UOTGHS_HSTIDR) Device Connection Interrupt Disable */
#define UOTGHS_HSTIDR_DDISCIEC (0x1u << 1) /**< \brief (UOTGHS_HSTIDR) Device Disconnection Interrupt Disable */
#define UOTGHS_HSTIDR_RSTIEC (0x1u << 2) /**< \brief (UOTGHS_HSTIDR) USB Reset Sent Interrupt Disable */
#define UOTGHS_HSTIDR_RSMEDIEC (0x1u << 3) /**< \brief (UOTGHS_HSTIDR) Downstream Resume Sent Interrupt Disable */
#define UOTGHS_HSTIDR_RXRSMIEC (0x1u << 4) /**< \brief (UOTGHS_HSTIDR) Upstream Resume Received Interrupt Disable */
#define UOTGHS_HSTIDR_HSOFIEC (0x1u << 5) /**< \brief (UOTGHS_HSTIDR) Host Start of Frame Interrupt Disable */
#define UOTGHS_HSTIDR_HWUPIEC (0x1u << 6) /**< \brief (UOTGHS_HSTIDR) Host Wake-Up Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_0 (0x1u << 8) /**< \brief (UOTGHS_HSTIDR) Pipe 0 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_1 (0x1u << 9) /**< \brief (UOTGHS_HSTIDR) Pipe 1 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_2 (0x1u << 10) /**< \brief (UOTGHS_HSTIDR) Pipe 2 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_3 (0x1u << 11) /**< \brief (UOTGHS_HSTIDR) Pipe 3 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_4 (0x1u << 12) /**< \brief (UOTGHS_HSTIDR) Pipe 4 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_5 (0x1u << 13) /**< \brief (UOTGHS_HSTIDR) Pipe 5 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_6 (0x1u << 14) /**< \brief (UOTGHS_HSTIDR) Pipe 6 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_7 (0x1u << 15) /**< \brief (UOTGHS_HSTIDR) Pipe 7 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_8 (0x1u << 16) /**< \brief (UOTGHS_HSTIDR) Pipe 8 Interrupt Disable */
#define UOTGHS_HSTIDR_PEP_9 (0x1u << 17) /**< \brief (UOTGHS_HSTIDR) Pipe 9 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_HSTIDR) DMA Channel 1 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_HSTIDR) DMA Channel 2 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_HSTIDR) DMA Channel 3 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_HSTIDR) DMA Channel 4 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_HSTIDR) DMA Channel 5 Interrupt Disable */
#define UOTGHS_HSTIDR_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_HSTIDR) DMA Channel 6 Interrupt Disable */
/* -------- UOTGHS_HSTIER : (UOTGHS Offset: 0x0418) Host Global Interrupt Enable Register -------- */
#define UOTGHS_HSTIER_DCONNIES (0x1u << 0) /**< \brief (UOTGHS_HSTIER) Device Connection Interrupt Enable */
#define UOTGHS_HSTIER_DDISCIES (0x1u << 1) /**< \brief (UOTGHS_HSTIER) Device Disconnection Interrupt Enable */
#define UOTGHS_HSTIER_RSTIES (0x1u << 2) /**< \brief (UOTGHS_HSTIER) USB Reset Sent Interrupt Enable */
#define UOTGHS_HSTIER_RSMEDIES (0x1u << 3) /**< \brief (UOTGHS_HSTIER) Downstream Resume Sent Interrupt Enable */
#define UOTGHS_HSTIER_RXRSMIES (0x1u << 4) /**< \brief (UOTGHS_HSTIER) Upstream Resume Received Interrupt Enable */
#define UOTGHS_HSTIER_HSOFIES (0x1u << 5) /**< \brief (UOTGHS_HSTIER) Host Start of Frame Interrupt Enable */
#define UOTGHS_HSTIER_HWUPIES (0x1u << 6) /**< \brief (UOTGHS_HSTIER) Host Wake-Up Interrupt Enable */
#define UOTGHS_HSTIER_PEP_0 (0x1u << 8) /**< \brief (UOTGHS_HSTIER) Pipe 0 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_1 (0x1u << 9) /**< \brief (UOTGHS_HSTIER) Pipe 1 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_2 (0x1u << 10) /**< \brief (UOTGHS_HSTIER) Pipe 2 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_3 (0x1u << 11) /**< \brief (UOTGHS_HSTIER) Pipe 3 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_4 (0x1u << 12) /**< \brief (UOTGHS_HSTIER) Pipe 4 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_5 (0x1u << 13) /**< \brief (UOTGHS_HSTIER) Pipe 5 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_6 (0x1u << 14) /**< \brief (UOTGHS_HSTIER) Pipe 6 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_7 (0x1u << 15) /**< \brief (UOTGHS_HSTIER) Pipe 7 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_8 (0x1u << 16) /**< \brief (UOTGHS_HSTIER) Pipe 8 Interrupt Enable */
#define UOTGHS_HSTIER_PEP_9 (0x1u << 17) /**< \brief (UOTGHS_HSTIER) Pipe 9 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_1 (0x1u << 25) /**< \brief (UOTGHS_HSTIER) DMA Channel 1 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_2 (0x1u << 26) /**< \brief (UOTGHS_HSTIER) DMA Channel 2 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_3 (0x1u << 27) /**< \brief (UOTGHS_HSTIER) DMA Channel 3 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_4 (0x1u << 28) /**< \brief (UOTGHS_HSTIER) DMA Channel 4 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_5 (0x1u << 29) /**< \brief (UOTGHS_HSTIER) DMA Channel 5 Interrupt Enable */
#define UOTGHS_HSTIER_DMA_6 (0x1u << 30) /**< \brief (UOTGHS_HSTIER) DMA Channel 6 Interrupt Enable */
/* -------- UOTGHS_HSTPIP : (UOTGHS Offset: 0x0041C) Host Pipe Register -------- */
#define UOTGHS_HSTPIP_PEN0 (0x1u << 0) /**< \brief (UOTGHS_HSTPIP) Pipe 0 Enable */
#define UOTGHS_HSTPIP_PEN1 (0x1u << 1) /**< \brief (UOTGHS_HSTPIP) Pipe 1 Enable */
#define UOTGHS_HSTPIP_PEN2 (0x1u << 2) /**< \brief (UOTGHS_HSTPIP) Pipe 2 Enable */
#define UOTGHS_HSTPIP_PEN3 (0x1u << 3) /**< \brief (UOTGHS_HSTPIP) Pipe 3 Enable */
#define UOTGHS_HSTPIP_PEN4 (0x1u << 4) /**< \brief (UOTGHS_HSTPIP) Pipe 4 Enable */
#define UOTGHS_HSTPIP_PEN5 (0x1u << 5) /**< \brief (UOTGHS_HSTPIP) Pipe 5 Enable */
#define UOTGHS_HSTPIP_PEN6 (0x1u << 6) /**< \brief (UOTGHS_HSTPIP) Pipe 6 Enable */
#define UOTGHS_HSTPIP_PEN7 (0x1u << 7) /**< \brief (UOTGHS_HSTPIP) Pipe 7 Enable */
#define UOTGHS_HSTPIP_PEN8 (0x1u << 8) /**< \brief (UOTGHS_HSTPIP) Pipe 8 Enable */
#define UOTGHS_HSTPIP_PRST0 (0x1u << 16) /**< \brief (UOTGHS_HSTPIP) Pipe 0 Reset */
#define UOTGHS_HSTPIP_PRST1 (0x1u << 17) /**< \brief (UOTGHS_HSTPIP) Pipe 1 Reset */
#define UOTGHS_HSTPIP_PRST2 (0x1u << 18) /**< \brief (UOTGHS_HSTPIP) Pipe 2 Reset */
#define UOTGHS_HSTPIP_PRST3 (0x1u << 19) /**< \brief (UOTGHS_HSTPIP) Pipe 3 Reset */
#define UOTGHS_HSTPIP_PRST4 (0x1u << 20) /**< \brief (UOTGHS_HSTPIP) Pipe 4 Reset */
#define UOTGHS_HSTPIP_PRST5 (0x1u << 21) /**< \brief (UOTGHS_HSTPIP) Pipe 5 Reset */
#define UOTGHS_HSTPIP_PRST6 (0x1u << 22) /**< \brief (UOTGHS_HSTPIP) Pipe 6 Reset */
#define UOTGHS_HSTPIP_PRST7 (0x1u << 23) /**< \brief (UOTGHS_HSTPIP) Pipe 7 Reset */
#define UOTGHS_HSTPIP_PRST8 (0x1u << 24) /**< \brief (UOTGHS_HSTPIP) Pipe 8 Reset */
/* -------- UOTGHS_HSTFNUM : (UOTGHS Offset: 0x0420) Host Frame Number Register -------- */
#define UOTGHS_HSTFNUM_MFNUM_Pos 0
#define UOTGHS_HSTFNUM_MFNUM_Msk (0x7u << UOTGHS_HSTFNUM_MFNUM_Pos) /**< \brief (UOTGHS_HSTFNUM) Micro Frame Number */
#define UOTGHS_HSTFNUM_MFNUM(value) ((UOTGHS_HSTFNUM_MFNUM_Msk & ((value) << UOTGHS_HSTFNUM_MFNUM_Pos)))
#define UOTGHS_HSTFNUM_FNUM_Pos 3
#define UOTGHS_HSTFNUM_FNUM_Msk (0x7ffu << UOTGHS_HSTFNUM_FNUM_Pos) /**< \brief (UOTGHS_HSTFNUM) Frame Number */
#define UOTGHS_HSTFNUM_FNUM(value) ((UOTGHS_HSTFNUM_FNUM_Msk & ((value) << UOTGHS_HSTFNUM_FNUM_Pos)))
#define UOTGHS_HSTFNUM_FLENHIGH_Pos 16
#define UOTGHS_HSTFNUM_FLENHIGH_Msk (0xffu << UOTGHS_HSTFNUM_FLENHIGH_Pos) /**< \brief (UOTGHS_HSTFNUM) Frame Length */
#define UOTGHS_HSTFNUM_FLENHIGH(value) ((UOTGHS_HSTFNUM_FLENHIGH_Msk & ((value) << UOTGHS_HSTFNUM_FLENHIGH_Pos)))
/* -------- UOTGHS_HSTADDR1 : (UOTGHS Offset: 0x0424) Host Address 1 Register -------- */
#define UOTGHS_HSTADDR1_HSTADDRP0_Pos 0
#define UOTGHS_HSTADDR1_HSTADDRP0_Msk (0x7fu << UOTGHS_HSTADDR1_HSTADDRP0_Pos) /**< \brief (UOTGHS_HSTADDR1) USB Host Address */
#define UOTGHS_HSTADDR1_HSTADDRP0(value) ((UOTGHS_HSTADDR1_HSTADDRP0_Msk & ((value) << UOTGHS_HSTADDR1_HSTADDRP0_Pos)))
#define UOTGHS_HSTADDR1_HSTADDRP1_Pos 8
#define UOTGHS_HSTADDR1_HSTADDRP1_Msk (0x7fu << UOTGHS_HSTADDR1_HSTADDRP1_Pos) /**< \brief (UOTGHS_HSTADDR1) USB Host Address */
#define UOTGHS_HSTADDR1_HSTADDRP1(value) ((UOTGHS_HSTADDR1_HSTADDRP1_Msk & ((value) << UOTGHS_HSTADDR1_HSTADDRP1_Pos)))
#define UOTGHS_HSTADDR1_HSTADDRP2_Pos 16
#define UOTGHS_HSTADDR1_HSTADDRP2_Msk (0x7fu << UOTGHS_HSTADDR1_HSTADDRP2_Pos) /**< \brief (UOTGHS_HSTADDR1) USB Host Address */
#define UOTGHS_HSTADDR1_HSTADDRP2(value) ((UOTGHS_HSTADDR1_HSTADDRP2_Msk & ((value) << UOTGHS_HSTADDR1_HSTADDRP2_Pos)))
#define UOTGHS_HSTADDR1_HSTADDRP3_Pos 24
#define UOTGHS_HSTADDR1_HSTADDRP3_Msk (0x7fu << UOTGHS_HSTADDR1_HSTADDRP3_Pos) /**< \brief (UOTGHS_HSTADDR1) USB Host Address */
#define UOTGHS_HSTADDR1_HSTADDRP3(value) ((UOTGHS_HSTADDR1_HSTADDRP3_Msk & ((value) << UOTGHS_HSTADDR1_HSTADDRP3_Pos)))
/* -------- UOTGHS_HSTADDR2 : (UOTGHS Offset: 0x0428) Host Address 2 Register -------- */
#define UOTGHS_HSTADDR2_HSTADDRP4_Pos 0
#define UOTGHS_HSTADDR2_HSTADDRP4_Msk (0x7fu << UOTGHS_HSTADDR2_HSTADDRP4_Pos) /**< \brief (UOTGHS_HSTADDR2) USB Host Address */
#define UOTGHS_HSTADDR2_HSTADDRP4(value) ((UOTGHS_HSTADDR2_HSTADDRP4_Msk & ((value) << UOTGHS_HSTADDR2_HSTADDRP4_Pos)))
#define UOTGHS_HSTADDR2_HSTADDRP5_Pos 8
#define UOTGHS_HSTADDR2_HSTADDRP5_Msk (0x7fu << UOTGHS_HSTADDR2_HSTADDRP5_Pos) /**< \brief (UOTGHS_HSTADDR2) USB Host Address */
#define UOTGHS_HSTADDR2_HSTADDRP5(value) ((UOTGHS_HSTADDR2_HSTADDRP5_Msk & ((value) << UOTGHS_HSTADDR2_HSTADDRP5_Pos)))
#define UOTGHS_HSTADDR2_HSTADDRP6_Pos 16
#define UOTGHS_HSTADDR2_HSTADDRP6_Msk (0x7fu << UOTGHS_HSTADDR2_HSTADDRP6_Pos) /**< \brief (UOTGHS_HSTADDR2) USB Host Address */
#define UOTGHS_HSTADDR2_HSTADDRP6(value) ((UOTGHS_HSTADDR2_HSTADDRP6_Msk & ((value) << UOTGHS_HSTADDR2_HSTADDRP6_Pos)))
#define UOTGHS_HSTADDR2_HSTADDRP7_Pos 24
#define UOTGHS_HSTADDR2_HSTADDRP7_Msk (0x7fu << UOTGHS_HSTADDR2_HSTADDRP7_Pos) /**< \brief (UOTGHS_HSTADDR2) USB Host Address */
#define UOTGHS_HSTADDR2_HSTADDRP7(value) ((UOTGHS_HSTADDR2_HSTADDRP7_Msk & ((value) << UOTGHS_HSTADDR2_HSTADDRP7_Pos)))
/* -------- UOTGHS_HSTADDR3 : (UOTGHS Offset: 0x042C) Host Address 3 Register -------- */
#define UOTGHS_HSTADDR3_HSTADDRP8_Pos 0
#define UOTGHS_HSTADDR3_HSTADDRP8_Msk (0x7fu << UOTGHS_HSTADDR3_HSTADDRP8_Pos) /**< \brief (UOTGHS_HSTADDR3) USB Host Address */
#define UOTGHS_HSTADDR3_HSTADDRP8(value) ((UOTGHS_HSTADDR3_HSTADDRP8_Msk & ((value) << UOTGHS_HSTADDR3_HSTADDRP8_Pos)))
#define UOTGHS_HSTADDR3_HSTADDRP9_Pos 8
#define UOTGHS_HSTADDR3_HSTADDRP9_Msk (0x7fu << UOTGHS_HSTADDR3_HSTADDRP9_Pos) /**< \brief (UOTGHS_HSTADDR3) USB Host Address */
#define UOTGHS_HSTADDR3_HSTADDRP9(value) ((UOTGHS_HSTADDR3_HSTADDRP9_Msk & ((value) << UOTGHS_HSTADDR3_HSTADDRP9_Pos)))
/* -------- UOTGHS_HSTPIPCFG[10] : (UOTGHS Offset: 0x500) Host Pipe Configuration Register (n = 0) -------- */
#define UOTGHS_HSTPIPCFG_ALLOC (0x1u << 1) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Memory Allocate */
#define UOTGHS_HSTPIPCFG_PBK_Pos 2
#define UOTGHS_HSTPIPCFG_PBK_Msk (0x3u << UOTGHS_HSTPIPCFG_PBK_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Banks */
#define   UOTGHS_HSTPIPCFG_PBK_1_BANK (0x0u << 2) /**< \brief (UOTGHS_HSTPIPCFG[10]) Single-bank pipe */
#define   UOTGHS_HSTPIPCFG_PBK_2_BANK (0x1u << 2) /**< \brief (UOTGHS_HSTPIPCFG[10]) Double-bank pipe */
#define   UOTGHS_HSTPIPCFG_PBK_3_BANK (0x2u << 2) /**< \brief (UOTGHS_HSTPIPCFG[10]) Triple-bank pipe */
#define UOTGHS_HSTPIPCFG_PSIZE_Pos 4
#define UOTGHS_HSTPIPCFG_PSIZE_Msk (0x7u << UOTGHS_HSTPIPCFG_PSIZE_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Size */
#define   UOTGHS_HSTPIPCFG_PSIZE_8_BYTE (0x0u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 8 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_16_BYTE (0x1u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 16 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_32_BYTE (0x2u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 32 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_64_BYTE (0x3u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 64 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_128_BYTE (0x4u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 128 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_256_BYTE (0x5u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 256 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_512_BYTE (0x6u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 512 bytes */
#define   UOTGHS_HSTPIPCFG_PSIZE_1024_BYTE (0x7u << 4) /**< \brief (UOTGHS_HSTPIPCFG[10]) 1024 bytes */
#define UOTGHS_HSTPIPCFG_PTOKEN_Pos 8
#define UOTGHS_HSTPIPCFG_PTOKEN_Msk (0x3u << UOTGHS_HSTPIPCFG_PTOKEN_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Token */
#define   UOTGHS_HSTPIPCFG_PTOKEN_SETUP (0x0u << 8) /**< \brief (UOTGHS_HSTPIPCFG[10]) SETUP */
#define   UOTGHS_HSTPIPCFG_PTOKEN_IN (0x1u << 8) /**< \brief (UOTGHS_HSTPIPCFG[10]) IN */
#define   UOTGHS_HSTPIPCFG_PTOKEN_OUT (0x2u << 8) /**< \brief (UOTGHS_HSTPIPCFG[10]) OUT */
#define UOTGHS_HSTPIPCFG_AUTOSW (0x1u << 10) /**< \brief (UOTGHS_HSTPIPCFG[10]) Automatic Switch */
#define UOTGHS_HSTPIPCFG_PTYPE_Pos 12
#define UOTGHS_HSTPIPCFG_PTYPE_Msk (0x3u << UOTGHS_HSTPIPCFG_PTYPE_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Type */
#define   UOTGHS_HSTPIPCFG_PTYPE_CTRL (0x0u << 12) /**< \brief (UOTGHS_HSTPIPCFG[10]) Control */
#define   UOTGHS_HSTPIPCFG_PTYPE_ISO (0x1u << 12) /**< \brief (UOTGHS_HSTPIPCFG[10]) Isochronous */
#define   UOTGHS_HSTPIPCFG_PTYPE_BLK (0x2u << 12) /**< \brief (UOTGHS_HSTPIPCFG[10]) Bulk */
#define   UOTGHS_HSTPIPCFG_PTYPE_INTRPT (0x3u << 12) /**< \brief (UOTGHS_HSTPIPCFG[10]) Interrupt */
#define UOTGHS_HSTPIPCFG_PEPNUM_Pos 16
#define UOTGHS_HSTPIPCFG_PEPNUM_Msk (0xfu << UOTGHS_HSTPIPCFG_PEPNUM_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Endpoint Number */
#define UOTGHS_HSTPIPCFG_PEPNUM(value) ((UOTGHS_HSTPIPCFG_PEPNUM_Msk & ((value) << UOTGHS_HSTPIPCFG_PEPNUM_Pos)))
#define UOTGHS_HSTPIPCFG_PINGEN (0x1u << 20) /**< \brief (UOTGHS_HSTPIPCFG[10]) Ping Enable */
#define UOTGHS_HSTPIPCFG_INTFRQ_Pos 24
#define UOTGHS_HSTPIPCFG_INTFRQ_Msk (0xffu << UOTGHS_HSTPIPCFG_INTFRQ_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) Pipe Interrupt Request Frequency */
#define UOTGHS_HSTPIPCFG_INTFRQ(value) ((UOTGHS_HSTPIPCFG_INTFRQ_Msk & ((value) << UOTGHS_HSTPIPCFG_INTFRQ_Pos)))
#define UOTGHS_HSTPIPCFG_BINTERVAL_Pos 24
#define UOTGHS_HSTPIPCFG_BINTERVAL_Msk (0xffu << UOTGHS_HSTPIPCFG_BINTERVAL_Pos) /**< \brief (UOTGHS_HSTPIPCFG[10]) bInterval parameter for the Bulk-Out/Ping transaction */
#define UOTGHS_HSTPIPCFG_BINTERVAL(value) ((UOTGHS_HSTPIPCFG_BINTERVAL_Msk & ((value) << UOTGHS_HSTPIPCFG_BINTERVAL_Pos)))
/* -------- UOTGHS_HSTPIPISR[10] : (UOTGHS Offset: 0x530) Host Pipe Status Register (n = 0) -------- */
#define UOTGHS_HSTPIPISR_RXINI (0x1u << 0) /**< \brief (UOTGHS_HSTPIPISR[10]) Received IN Data Interrupt */
#define UOTGHS_HSTPIPISR_TXOUTI (0x1u << 1) /**< \brief (UOTGHS_HSTPIPISR[10]) Transmitted OUT Data Interrupt */
#define UOTGHS_HSTPIPISR_TXSTPI (0x1u << 2) /**< \brief (UOTGHS_HSTPIPISR[10]) Transmitted SETUP Interrupt */
#define UOTGHS_HSTPIPISR_UNDERFI (0x1u << 2) /**< \brief (UOTGHS_HSTPIPISR[10]) Underflow Interrupt */
#define UOTGHS_HSTPIPISR_PERRI (0x1u << 3) /**< \brief (UOTGHS_HSTPIPISR[10]) Pipe Error Interrupt */
#define UOTGHS_HSTPIPISR_NAKEDI (0x1u << 4) /**< \brief (UOTGHS_HSTPIPISR[10]) NAKed Interrupt */
#define UOTGHS_HSTPIPISR_OVERFI (0x1u << 5) /**< \brief (UOTGHS_HSTPIPISR[10]) Overflow Interrupt */
#define UOTGHS_HSTPIPISR_RXSTALLDI (0x1u << 6) /**< \brief (UOTGHS_HSTPIPISR[10]) Received STALLed Interrupt */
#define UOTGHS_HSTPIPISR_CRCERRI (0x1u << 6) /**< \brief (UOTGHS_HSTPIPISR[10]) CRC Error Interrupt */
#define UOTGHS_HSTPIPISR_SHORTPACKETI (0x1u << 7) /**< \brief (UOTGHS_HSTPIPISR[10]) Short Packet Interrupt */
#define UOTGHS_HSTPIPISR_DTSEQ_Pos 8
#define UOTGHS_HSTPIPISR_DTSEQ_Msk (0x3u << UOTGHS_HSTPIPISR_DTSEQ_Pos) /**< \brief (UOTGHS_HSTPIPISR[10]) Data Toggle Sequence */
#define   UOTGHS_HSTPIPISR_DTSEQ_DATA0 (0x0u << 8) /**< \brief (UOTGHS_HSTPIPISR[10]) Data0 toggle sequence */
#define   UOTGHS_HSTPIPISR_DTSEQ_DATA1 (0x1u << 8) /**< \brief (UOTGHS_HSTPIPISR[10]) Data1 toggle sequence */
#define UOTGHS_HSTPIPISR_NBUSYBK_Pos 12
#define UOTGHS_HSTPIPISR_NBUSYBK_Msk (0x3u << UOTGHS_HSTPIPISR_NBUSYBK_Pos) /**< \brief (UOTGHS_HSTPIPISR[10]) Number of Busy Banks */
#define   UOTGHS_HSTPIPISR_NBUSYBK_0_BUSY (0x0u << 12) /**< \brief (UOTGHS_HSTPIPISR[10]) 0 busy bank (all banks free) */
#define   UOTGHS_HSTPIPISR_NBUSYBK_1_BUSY (0x1u << 12) /**< \brief (UOTGHS_HSTPIPISR[10]) 1 busy bank */
#define   UOTGHS_HSTPIPISR_NBUSYBK_2_BUSY (0x2u << 12) /**< \brief (UOTGHS_HSTPIPISR[10]) 2 busy banks */
#define   UOTGHS_HSTPIPISR_NBUSYBK_3_BUSY (0x3u << 12) /**< \brief (UOTGHS_HSTPIPISR[10]) 3 busy banks */
#define UOTGHS_HSTPIPISR_CURRBK_Pos 14
#define UOTGHS_HSTPIPISR_CURRBK_Msk (0x3u << UOTGHS_HSTPIPISR_CURRBK_Pos) /**< \brief (UOTGHS_HSTPIPISR[10]) Current Bank */
#define   UOTGHS_HSTPIPISR_CURRBK_BANK0 (0x0u << 14) /**< \brief (UOTGHS_HSTPIPISR[10]) Current bank is bank0 */
#define   UOTGHS_HSTPIPISR_CURRBK_BANK1 (0x1u << 14) /**< \brief (UOTGHS_HSTPIPISR[10]) Current bank is bank1 */
#define   UOTGHS_HSTPIPISR_CURRBK_BANK2 (0x2u << 14) /**< \brief (UOTGHS_HSTPIPISR[10]) Current bank is bank2 */
#define UOTGHS_HSTPIPISR_RWALL (0x1u << 16) /**< \brief (UOTGHS_HSTPIPISR[10]) Read-write Allowed */
#define UOTGHS_HSTPIPISR_CFGOK (0x1u << 18) /**< \brief (UOTGHS_HSTPIPISR[10]) Configuration OK Status */
#define UOTGHS_HSTPIPISR_PBYCT_Pos 20
#define UOTGHS_HSTPIPISR_PBYCT_Msk (0x7ffu << UOTGHS_HSTPIPISR_PBYCT_Pos) /**< \brief (UOTGHS_HSTPIPISR[10]) Pipe Byte Count */
/* -------- UOTGHS_HSTPIPICR[10] : (UOTGHS Offset: 0x560) Host Pipe Clear Register (n = 0) -------- */
#define UOTGHS_HSTPIPICR_RXINIC (0x1u << 0) /**< \brief (UOTGHS_HSTPIPICR[10]) Received IN Data Interrupt Clear */
#define UOTGHS_HSTPIPICR_TXOUTIC (0x1u << 1) /**< \brief (UOTGHS_HSTPIPICR[10]) Transmitted OUT Data Interrupt Clear */
#define UOTGHS_HSTPIPICR_TXSTPIC (0x1u << 2) /**< \brief (UOTGHS_HSTPIPICR[10]) Transmitted SETUP Interrupt Clear */
#define UOTGHS_HSTPIPICR_UNDERFIC (0x1u << 2) /**< \brief (UOTGHS_HSTPIPICR[10]) Underflow Interrupt Clear */
#define UOTGHS_HSTPIPICR_NAKEDIC (0x1u << 4) /**< \brief (UOTGHS_HSTPIPICR[10]) NAKed Interrupt Clear */
#define UOTGHS_HSTPIPICR_OVERFIC (0x1u << 5) /**< \brief (UOTGHS_HSTPIPICR[10]) Overflow Interrupt Clear */
#define UOTGHS_HSTPIPICR_RXSTALLDIC (0x1u << 6) /**< \brief (UOTGHS_HSTPIPICR[10]) Received STALLed Interrupt Clear */
#define UOTGHS_HSTPIPICR_CRCERRIC (0x1u << 6) /**< \brief (UOTGHS_HSTPIPICR[10]) CRC Error Interrupt Clear */
#define UOTGHS_HSTPIPICR_SHORTPACKETIC (0x1u << 7) /**< \brief (UOTGHS_HSTPIPICR[10]) Short Packet Interrupt Clear */
/* -------- UOTGHS_HSTPIPIFR[10] : (UOTGHS Offset: 0x590) Host Pipe Set Register (n = 0) -------- */
#define UOTGHS_HSTPIPIFR_RXINIS (0x1u << 0) /**< \brief (UOTGHS_HSTPIPIFR[10]) Received IN Data Interrupt Set */
#define UOTGHS_HSTPIPIFR_TXOUTIS (0x1u << 1) /**< \brief (UOTGHS_HSTPIPIFR[10]) Transmitted OUT Data Interrupt Set */
#define UOTGHS_HSTPIPIFR_TXSTPIS (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIFR[10]) Transmitted SETUP Interrupt Set */
#define UOTGHS_HSTPIPIFR_UNDERFIS (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIFR[10]) Underflow Interrupt Set */
#define UOTGHS_HSTPIPIFR_PERRIS (0x1u << 3) /**< \brief (UOTGHS_HSTPIPIFR[10]) Pipe Error Interrupt Set */
#define UOTGHS_HSTPIPIFR_NAKEDIS (0x1u << 4) /**< \brief (UOTGHS_HSTPIPIFR[10]) NAKed Interrupt Set */
#define UOTGHS_HSTPIPIFR_OVERFIS (0x1u << 5) /**< \brief (UOTGHS_HSTPIPIFR[10]) Overflow Interrupt Set */
#define UOTGHS_HSTPIPIFR_RXSTALLDIS (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIFR[10]) Received STALLed Interrupt Set */
#define UOTGHS_HSTPIPIFR_CRCERRIS (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIFR[10]) CRC Error Interrupt Set */
#define UOTGHS_HSTPIPIFR_SHORTPACKETIS (0x1u << 7) /**< \brief (UOTGHS_HSTPIPIFR[10]) Short Packet Interrupt Set */
#define UOTGHS_HSTPIPIFR_NBUSYBKS (0x1u << 12) /**< \brief (UOTGHS_HSTPIPIFR[10]) Number of Busy Banks Set */
/* -------- UOTGHS_HSTPIPIMR[10] : (UOTGHS Offset: 0x5C0) Host Pipe Mask Register (n = 0) -------- */
#define UOTGHS_HSTPIPIMR_RXINE (0x1u << 0) /**< \brief (UOTGHS_HSTPIPIMR[10]) Received IN Data Interrupt Enable */
#define UOTGHS_HSTPIPIMR_TXOUTE (0x1u << 1) /**< \brief (UOTGHS_HSTPIPIMR[10]) Transmitted OUT Data Interrupt Enable */
#define UOTGHS_HSTPIPIMR_TXSTPE (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIMR[10]) Transmitted SETUP Interrupt Enable */
#define UOTGHS_HSTPIPIMR_UNDERFIE (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIMR[10]) Underflow Interrupt Enable */
#define UOTGHS_HSTPIPIMR_PERRE (0x1u << 3) /**< \brief (UOTGHS_HSTPIPIMR[10]) Pipe Error Interrupt Enable */
#define UOTGHS_HSTPIPIMR_NAKEDE (0x1u << 4) /**< \brief (UOTGHS_HSTPIPIMR[10]) NAKed Interrupt Enable */
#define UOTGHS_HSTPIPIMR_OVERFIE (0x1u << 5) /**< \brief (UOTGHS_HSTPIPIMR[10]) Overflow Interrupt Enable */
#define UOTGHS_HSTPIPIMR_RXSTALLDE (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIMR[10]) Received STALLed Interrupt Enable */
#define UOTGHS_HSTPIPIMR_CRCERRE (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIMR[10]) CRC Error Interrupt Enable */
#define UOTGHS_HSTPIPIMR_SHORTPACKETIE (0x1u << 7) /**< \brief (UOTGHS_HSTPIPIMR[10]) Short Packet Interrupt Enable */
#define UOTGHS_HSTPIPIMR_NBUSYBKE (0x1u << 12) /**< \brief (UOTGHS_HSTPIPIMR[10]) Number of Busy Banks Interrupt Enable */
#define UOTGHS_HSTPIPIMR_FIFOCON (0x1u << 14) /**< \brief (UOTGHS_HSTPIPIMR[10]) FIFO Control */
#define UOTGHS_HSTPIPIMR_PDISHDMA (0x1u << 16) /**< \brief (UOTGHS_HSTPIPIMR[10]) Pipe Interrupts Disable HDMA Request Enable */
#define UOTGHS_HSTPIPIMR_PFREEZE (0x1u << 17) /**< \brief (UOTGHS_HSTPIPIMR[10]) Pipe Freeze */
#define UOTGHS_HSTPIPIMR_RSTDT (0x1u << 18) /**< \brief (UOTGHS_HSTPIPIMR[10]) Reset Data Toggle */
/* -------- UOTGHS_HSTPIPIER[10] : (UOTGHS Offset: 0x5F0) Host Pipe Enable Register (n = 0) -------- */
#define UOTGHS_HSTPIPIER_RXINES (0x1u << 0) /**< \brief (UOTGHS_HSTPIPIER[10]) Received IN Data Interrupt Enable */
#define UOTGHS_HSTPIPIER_TXOUTES (0x1u << 1) /**< \brief (UOTGHS_HSTPIPIER[10]) Transmitted OUT Data Interrupt Enable */
#define UOTGHS_HSTPIPIER_TXSTPES (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIER[10]) Transmitted SETUP Interrupt Enable */
#define UOTGHS_HSTPIPIER_UNDERFIES (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIER[10]) Underflow Interrupt Enable */
#define UOTGHS_HSTPIPIER_PERRES (0x1u << 3) /**< \brief (UOTGHS_HSTPIPIER[10]) Pipe Error Interrupt Enable */
#define UOTGHS_HSTPIPIER_NAKEDES (0x1u << 4) /**< \brief (UOTGHS_HSTPIPIER[10]) NAKed Interrupt Enable */
#define UOTGHS_HSTPIPIER_OVERFIES (0x1u << 5) /**< \brief (UOTGHS_HSTPIPIER[10]) Overflow Interrupt Enable */
#define UOTGHS_HSTPIPIER_RXSTALLDES (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIER[10]) Received STALLed Interrupt Enable */
#define UOTGHS_HSTPIPIER_CRCERRES (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIER[10]) CRC Error Interrupt Enable */
#define UOTGHS_HSTPIPIER_SHORTPACKETIES (0x1u << 7) /**< \brief (UOTGHS_HSTPIPIER[10]) Short Packet Interrupt Enable */
#define UOTGHS_HSTPIPIER_NBUSYBKES (0x1u << 12) /**< \brief (UOTGHS_HSTPIPIER[10]) Number of Busy Banks Enable */
#define UOTGHS_HSTPIPIER_PDISHDMAS (0x1u << 16) /**< \brief (UOTGHS_HSTPIPIER[10]) Pipe Interrupts Disable HDMA Request Enable */
#define UOTGHS_HSTPIPIER_PFREEZES (0x1u << 17) /**< \brief (UOTGHS_HSTPIPIER[10]) Pipe Freeze Enable */
#define UOTGHS_HSTPIPIER_RSTDTS (0x1u << 18) /**< \brief (UOTGHS_HSTPIPIER[10]) Reset Data Toggle Enable */
/* -------- UOTGHS_HSTPIPIDR[10] : (UOTGHS Offset: 0x620) Host Pipe Disable Register (n = 0) -------- */
#define UOTGHS_HSTPIPIDR_RXINEC (0x1u << 0) /**< \brief (UOTGHS_HSTPIPIDR[10]) Received IN Data Interrupt Disable */
#define UOTGHS_HSTPIPIDR_TXOUTEC (0x1u << 1) /**< \brief (UOTGHS_HSTPIPIDR[10]) Transmitted OUT Data Interrupt Disable */
#define UOTGHS_HSTPIPIDR_TXSTPEC (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIDR[10]) Transmitted SETUP Interrupt Disable */
#define UOTGHS_HSTPIPIDR_UNDERFIEC (0x1u << 2) /**< \brief (UOTGHS_HSTPIPIDR[10]) Underflow Interrupt Disable */
#define UOTGHS_HSTPIPIDR_PERREC (0x1u << 3) /**< \brief (UOTGHS_HSTPIPIDR[10]) Pipe Error Interrupt Disable */
#define UOTGHS_HSTPIPIDR_NAKEDEC (0x1u << 4) /**< \brief (UOTGHS_HSTPIPIDR[10]) NAKed Interrupt Disable */
#define UOTGHS_HSTPIPIDR_OVERFIEC (0x1u << 5) /**< \brief (UOTGHS_HSTPIPIDR[10]) Overflow Interrupt Disable */
#define UOTGHS_HSTPIPIDR_RXSTALLDEC (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIDR[10]) Received STALLed Interrupt Disable */
#define UOTGHS_HSTPIPIDR_CRCERREC (0x1u << 6) /**< \brief (UOTGHS_HSTPIPIDR[10]) CRC Error Interrupt Disable */
#define UOTGHS_HSTPIPIDR_SHORTPACKETIEC (0x1u << 7) /**< \brief (UOTGHS_HSTPIPIDR[10]) Short Packet Interrupt Disable */
#define UOTGHS_HSTPIPIDR_NBUSYBKEC (0x1u << 12) /**< \brief (UOTGHS_HSTPIPIDR[10]) Number of Busy Banks Disable */
#define UOTGHS_HSTPIPIDR_FIFOCONC (0x1u << 14) /**< \brief (UOTGHS_HSTPIPIDR[10]) FIFO Control Disable */
#define UOTGHS_HSTPIPIDR_PDISHDMAC (0x1u << 16) /**< \brief (UOTGHS_HSTPIPIDR[10]) Pipe Interrupts Disable HDMA Request Disable */
#define UOTGHS_HSTPIPIDR_PFREEZEC (0x1u << 17) /**< \brief (UOTGHS_HSTPIPIDR[10]) Pipe Freeze Disable */
/* -------- UOTGHS_HSTPIPINRQ[10] : (UOTGHS Offset: 0x650) Host Pipe IN Request Register (n = 0) -------- */
#define UOTGHS_HSTPIPINRQ_INRQ_Pos 0
#define UOTGHS_HSTPIPINRQ_INRQ_Msk (0xffu << UOTGHS_HSTPIPINRQ_INRQ_Pos) /**< \brief (UOTGHS_HSTPIPINRQ[10]) IN Request Number before Freeze */
#define UOTGHS_HSTPIPINRQ_INRQ(value) ((UOTGHS_HSTPIPINRQ_INRQ_Msk & ((value) << UOTGHS_HSTPIPINRQ_INRQ_Pos)))
#define UOTGHS_HSTPIPINRQ_INMODE (0x1u << 8) /**< \brief (UOTGHS_HSTPIPINRQ[10]) IN Request Mode */
/* -------- UOTGHS_HSTPIPERR[10] : (UOTGHS Offset: 0x680) Host Pipe Error Register (n = 0) -------- */
#define UOTGHS_HSTPIPERR_DATATGL (0x1u << 0) /**< \brief (UOTGHS_HSTPIPERR[10]) Data Toggle Error */
#define UOTGHS_HSTPIPERR_DATAPID (0x1u << 1) /**< \brief (UOTGHS_HSTPIPERR[10]) Data PID Error */
#define UOTGHS_HSTPIPERR_PID (0x1u << 2) /**< \brief (UOTGHS_HSTPIPERR[10]) PID Error */
#define UOTGHS_HSTPIPERR_TIMEOUT (0x1u << 3) /**< \brief (UOTGHS_HSTPIPERR[10]) Time-Out Error */
#define UOTGHS_HSTPIPERR_CRC16 (0x1u << 4) /**< \brief (UOTGHS_HSTPIPERR[10]) CRC16 Error */
#define UOTGHS_HSTPIPERR_COUNTER_Pos 5
#define UOTGHS_HSTPIPERR_COUNTER_Msk (0x3u << UOTGHS_HSTPIPERR_COUNTER_Pos) /**< \brief (UOTGHS_HSTPIPERR[10]) Error Counter */
#define UOTGHS_HSTPIPERR_COUNTER(value) ((UOTGHS_HSTPIPERR_COUNTER_Msk & ((value) << UOTGHS_HSTPIPERR_COUNTER_Pos)))
/* -------- UOTGHS_HSTDMANXTDSC : (UOTGHS Offset: N/A) Host DMA Channel Next Descriptor Address Register -------- */
#define UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD_Pos 0
#define UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD_Msk (0xffffffffu << UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD_Pos) /**< \brief (UOTGHS_HSTDMANXTDSC) Next Descriptor Address */
#define UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD(value) ((UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD_Msk & ((value) << UOTGHS_HSTDMANXTDSC_NXT_DSC_ADD_Pos)))
/* -------- UOTGHS_HSTDMAADDRESS : (UOTGHS Offset: N/A) Host DMA Channel Address Register -------- */
#define UOTGHS_HSTDMAADDRESS_BUFF_ADD_Pos 0
#define UOTGHS_HSTDMAADDRESS_BUFF_ADD_Msk (0xffffffffu << UOTGHS_HSTDMAADDRESS_BUFF_ADD_Pos) /**< \brief (UOTGHS_HSTDMAADDRESS) Buffer Address */
#define UOTGHS_HSTDMAADDRESS_BUFF_ADD(value) ((UOTGHS_HSTDMAADDRESS_BUFF_ADD_Msk & ((value) << UOTGHS_HSTDMAADDRESS_BUFF_ADD_Pos)))
/* -------- UOTGHS_HSTDMACONTROL : (UOTGHS Offset: N/A) Host DMA Channel Control Register -------- */
#define UOTGHS_HSTDMACONTROL_CHANN_ENB (0x1u << 0) /**< \brief (UOTGHS_HSTDMACONTROL) Channel Enable Command */
#define UOTGHS_HSTDMACONTROL_LDNXT_DSC (0x1u << 1) /**< \brief (UOTGHS_HSTDMACONTROL) Load Next Channel Transfer Descriptor Enable Command */
#define UOTGHS_HSTDMACONTROL_END_TR_EN (0x1u << 2) /**< \brief (UOTGHS_HSTDMACONTROL) End of Transfer Enable (Control) */
#define UOTGHS_HSTDMACONTROL_END_B_EN (0x1u << 3) /**< \brief (UOTGHS_HSTDMACONTROL) End of Buffer Enable Control */
#define UOTGHS_HSTDMACONTROL_END_TR_IT (0x1u << 4) /**< \brief (UOTGHS_HSTDMACONTROL) End of Transfer Interrupt Enable */
#define UOTGHS_HSTDMACONTROL_END_BUFFIT (0x1u << 5) /**< \brief (UOTGHS_HSTDMACONTROL) End of Buffer Interrupt Enable */
#define UOTGHS_HSTDMACONTROL_DESC_LD_IT (0x1u << 6) /**< \brief (UOTGHS_HSTDMACONTROL) Descriptor Loaded Interrupt Enable */
#define UOTGHS_HSTDMACONTROL_BURST_LCK (0x1u << 7) /**< \brief (UOTGHS_HSTDMACONTROL) Burst Lock Enable */
#define UOTGHS_HSTDMACONTROL_BUFF_LENGTH_Pos 16
#define UOTGHS_HSTDMACONTROL_BUFF_LENGTH_Msk (0xffffu << UOTGHS_HSTDMACONTROL_BUFF_LENGTH_Pos) /**< \brief (UOTGHS_HSTDMACONTROL) Buffer Byte Length (Write-only) */
#define UOTGHS_HSTDMACONTROL_BUFF_LENGTH(value) ((UOTGHS_HSTDMACONTROL_BUFF_LENGTH_Msk & ((value) << UOTGHS_HSTDMACONTROL_BUFF_LENGTH_Pos)))
/* -------- UOTGHS_HSTDMASTATUS : (UOTGHS Offset: N/A) Host DMA Channel Status Register -------- */
#define UOTGHS_HSTDMASTATUS_CHANN_ENB (0x1u << 0) /**< \brief (UOTGHS_HSTDMASTATUS) Channel Enable Status */
#define UOTGHS_HSTDMASTATUS_CHANN_ACT (0x1u << 1) /**< \brief (UOTGHS_HSTDMASTATUS) Channel Active Status */
#define UOTGHS_HSTDMASTATUS_END_TR_ST (0x1u << 4) /**< \brief (UOTGHS_HSTDMASTATUS) End of Channel Transfer Status */
#define UOTGHS_HSTDMASTATUS_END_BF_ST (0x1u << 5) /**< \brief (UOTGHS_HSTDMASTATUS) End of Channel Buffer Status */
#define UOTGHS_HSTDMASTATUS_DESC_LDST (0x1u << 6) /**< \brief (UOTGHS_HSTDMASTATUS) Descriptor Loaded Status */
#define UOTGHS_HSTDMASTATUS_BUFF_COUNT_Pos 16
#define UOTGHS_HSTDMASTATUS_BUFF_COUNT_Msk (0xffffu << UOTGHS_HSTDMASTATUS_BUFF_COUNT_Pos) /**< \brief (UOTGHS_HSTDMASTATUS) Buffer Byte Count */
#define UOTGHS_HSTDMASTATUS_BUFF_COUNT(value) ((UOTGHS_HSTDMASTATUS_BUFF_COUNT_Msk & ((value) << UOTGHS_HSTDMASTATUS_BUFF_COUNT_Pos)))
/* -------- UOTGHS_CTRL : (UOTGHS Offset: 0x0800) General Control Register -------- */
#define UOTGHS_CTRL_IDTE (0x1u << 0) /**< \brief (UOTGHS_CTRL) ID Transition Interrupt Enable */
#define UOTGHS_CTRL_VBUSTE (0x1u << 1) /**< \brief (UOTGHS_CTRL) VBus Transition Interrupt Enable */
#define UOTGHS_CTRL_SRPE (0x1u << 2) /**< \brief (UOTGHS_CTRL) SRP Interrupt Enable */
#define UOTGHS_CTRL_VBERRE (0x1u << 3) /**< \brief (UOTGHS_CTRL) VBus Error Interrupt Enable */
#define UOTGHS_CTRL_BCERRE (0x1u << 4) /**< \brief (UOTGHS_CTRL) B-Connection Error Interrupt Enable */
#define UOTGHS_CTRL_ROLEEXE (0x1u << 5) /**< \brief (UOTGHS_CTRL) Role Exchange Interrupt Enable */
#define UOTGHS_CTRL_HNPERRE (0x1u << 6) /**< \brief (UOTGHS_CTRL) HNP Error Interrupt Enable */
#define UOTGHS_CTRL_STOE (0x1u << 7) /**< \brief (UOTGHS_CTRL) Suspend Time-Out Interrupt Enable */
#define UOTGHS_CTRL_VBUSHWC (0x1u << 8) /**< \brief (UOTGHS_CTRL) VBus Hardware Control */
#define UOTGHS_CTRL_SRPSEL (0x1u << 9) /**< \brief (UOTGHS_CTRL) SRP Selection */
#define UOTGHS_CTRL_SRPREQ (0x1u << 10) /**< \brief (UOTGHS_CTRL) SRP Request */
#define UOTGHS_CTRL_HNPREQ (0x1u << 11) /**< \brief (UOTGHS_CTRL) HNP Request */
#define UOTGHS_CTRL_OTGPADE (0x1u << 12) /**< \brief (UOTGHS_CTRL) OTG Pad Enable */
#define UOTGHS_CTRL_VBUSPO (0x1u << 13) /**< \brief (UOTGHS_CTRL) VBus Polarity Off */
#define UOTGHS_CTRL_FRZCLK (0x1u << 14) /**< \brief (UOTGHS_CTRL) Freeze USB Clock */
#define UOTGHS_CTRL_USBE (0x1u << 15) /**< \brief (UOTGHS_CTRL) UOTGHS Enable */
#define UOTGHS_CTRL_TIMVALUE_Pos 16
#define UOTGHS_CTRL_TIMVALUE_Msk (0x3u << UOTGHS_CTRL_TIMVALUE_Pos) /**< \brief (UOTGHS_CTRL) Timer Value */
#define UOTGHS_CTRL_TIMVALUE(value) ((UOTGHS_CTRL_TIMVALUE_Msk & ((value) << UOTGHS_CTRL_TIMVALUE_Pos)))
#define UOTGHS_CTRL_TIMPAGE_Pos 20
#define UOTGHS_CTRL_TIMPAGE_Msk (0x3u << UOTGHS_CTRL_TIMPAGE_Pos) /**< \brief (UOTGHS_CTRL) Timer Page */
#define UOTGHS_CTRL_TIMPAGE(value) ((UOTGHS_CTRL_TIMPAGE_Msk & ((value) << UOTGHS_CTRL_TIMPAGE_Pos)))
#define UOTGHS_CTRL_UNLOCK (0x1u << 22) /**< \brief (UOTGHS_CTRL) Timer Access Unlock */
#define UOTGHS_CTRL_UIDE (0x1u << 24) /**< \brief (UOTGHS_CTRL) UOTGID Pin Enable */
#define   UOTGHS_CTRL_UIDE_UIMOD (0x0u << 24) /**< \brief (UOTGHS_CTRL) The USB mode (device/host) is selected from the UIMOD bit. */
#define   UOTGHS_CTRL_UIDE_UOTGID (0x1u << 24) /**< \brief (UOTGHS_CTRL) The USB mode (device/host) is selected from the UOTGID input pin. */
#define UOTGHS_CTRL_UIMOD (0x1u << 25) /**< \brief (UOTGHS_CTRL) UOTGHS Mode */
#define   UOTGHS_CTRL_UIMOD_Host (0x0u << 25) /**< \brief (UOTGHS_CTRL) The module is in USB host mode. */
#define   UOTGHS_CTRL_UIMOD_Device (0x1u << 25) /**< \brief (UOTGHS_CTRL) The module is in USB device mode. */
/* -------- UOTGHS_SR : (UOTGHS Offset: 0x0804) General Status Register -------- */
#define UOTGHS_SR_IDTI (0x1u << 0) /**< \brief (UOTGHS_SR) ID Transition Interrupt */
#define UOTGHS_SR_VBUSTI (0x1u << 1) /**< \brief (UOTGHS_SR) VBus Transition Interrupt */
#define UOTGHS_SR_SRPI (0x1u << 2) /**< \brief (UOTGHS_SR) SRP Interrupt */
#define UOTGHS_SR_VBERRI (0x1u << 3) /**< \brief (UOTGHS_SR) VBus Error Interrupt */
#define UOTGHS_SR_BCERRI (0x1u << 4) /**< \brief (UOTGHS_SR) B-Connection Error Interrupt */
#define UOTGHS_SR_ROLEEXI (0x1u << 5) /**< \brief (UOTGHS_SR) Role Exchange Interrupt */
#define UOTGHS_SR_HNPERRI (0x1u << 6) /**< \brief (UOTGHS_SR) HNP Error Interrupt */
#define UOTGHS_SR_STOI (0x1u << 7) /**< \brief (UOTGHS_SR) Suspend Time-Out Interrupt */
#define UOTGHS_SR_VBUSRQ (0x1u << 9) /**< \brief (UOTGHS_SR) VBus Request */
#define UOTGHS_SR_ID (0x1u << 10) /**< \brief (UOTGHS_SR) UOTGID Pin State */
#define UOTGHS_SR_VBUS (0x1u << 11) /**< \brief (UOTGHS_SR) VBus Level */
#define UOTGHS_SR_SPEED_Pos 12
#define UOTGHS_SR_SPEED_Msk (0x3u << UOTGHS_SR_SPEED_Pos) /**< \brief (UOTGHS_SR) Speed Status */
#define   UOTGHS_SR_SPEED_FULL_SPEED (0x0u << 12) /**< \brief (UOTGHS_SR) Full-Speed mode */
#define   UOTGHS_SR_SPEED_HIGH_SPEED (0x1u << 12) /**< \brief (UOTGHS_SR) High-Speed mode */
#define   UOTGHS_SR_SPEED_LOW_SPEED (0x2u << 12) /**< \brief (UOTGHS_SR) Low-Speed mode */
#define UOTGHS_SR_CLKUSABLE (0x1u << 14) /**< \brief (UOTGHS_SR) UTMI Clock Usable */
/* -------- UOTGHS_SCR : (UOTGHS Offset: 0x0808) General Status Clear Register -------- */
#define UOTGHS_SCR_IDTIC (0x1u << 0) /**< \brief (UOTGHS_SCR) ID Transition Interrupt Clear */
#define UOTGHS_SCR_VBUSTIC (0x1u << 1) /**< \brief (UOTGHS_SCR) VBus Transition Interrupt Clear */
#define UOTGHS_SCR_SRPIC (0x1u << 2) /**< \brief (UOTGHS_SCR) SRP Interrupt Clear */
#define UOTGHS_SCR_VBERRIC (0x1u << 3) /**< \brief (UOTGHS_SCR) VBus Error Interrupt Clear */
#define UOTGHS_SCR_BCERRIC (0x1u << 4) /**< \brief (UOTGHS_SCR) B-Connection Error Interrupt Clear */
#define UOTGHS_SCR_ROLEEXIC (0x1u << 5) /**< \brief (UOTGHS_SCR) Role Exchange Interrupt Clear */
#define UOTGHS_SCR_HNPERRIC (0x1u << 6) /**< \brief (UOTGHS_SCR) HNP Error Interrupt Clear */
#define UOTGHS_SCR_STOIC (0x1u << 7) /**< \brief (UOTGHS_SCR) Suspend Time-Out Interrupt Clear */
#define UOTGHS_SCR_VBUSRQC (0x1u << 9) /**< \brief (UOTGHS_SCR) VBus Request Clear */
/* -------- UOTGHS_SFR : (UOTGHS Offset: 0x080C) General Status Set Register -------- */
#define UOTGHS_SFR_IDTIS (0x1u << 0) /**< \brief (UOTGHS_SFR) ID Transition Interrupt Set */
#define UOTGHS_SFR_VBUSTIS (0x1u << 1) /**< \brief (UOTGHS_SFR) VBus Transition Interrupt Set */
#define UOTGHS_SFR_SRPIS (0x1u << 2) /**< \brief (UOTGHS_SFR) SRP Interrupt Set */
#define UOTGHS_SFR_VBERRIS (0x1u << 3) /**< \brief (UOTGHS_SFR) VBus Error Interrupt Set */
#define UOTGHS_SFR_BCERRIS (0x1u << 4) /**< \brief (UOTGHS_SFR) B-Connection Error Interrupt Set */
#define UOTGHS_SFR_ROLEEXIS (0x1u << 5) /**< \brief (UOTGHS_SFR) Role Exchange Interrupt Set */
#define UOTGHS_SFR_HNPERRIS (0x1u << 6) /**< \brief (UOTGHS_SFR) HNP Error Interrupt Set */
#define UOTGHS_SFR_STOIS (0x1u << 7) /**< \brief (UOTGHS_SFR) Suspend Time-Out Interrupt Set */
#define UOTGHS_SFR_VBUSRQS (0x1u << 9) /**< \brief (UOTGHS_SFR) VBus Request Set */
/* -------- UOTGHS_FSM : (UOTGHS Offset: 0x082C) General Finite State Machine Register -------- */
#define UOTGHS_FSM_DRDSTATE_Pos 0
#define UOTGHS_FSM_DRDSTATE_Msk (0xfu << UOTGHS_FSM_DRDSTATE_Pos) /**< \brief (UOTGHS_FSM)  */
#define   UOTGHS_FSM_DRDSTATE_A_IDLESTATE (0x0u << 0) /**< \brief (UOTGHS_FSM) This is the start state for A-devices (when the ID pin is 0) */
#define   UOTGHS_FSM_DRDSTATE_A_WAIT_VRISE (0x1u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device waits for the voltage on VBus to rise above the A-device VBus Valid threshold (4.4 V). */
#define   UOTGHS_FSM_DRDSTATE_A_WAIT_BCON (0x2u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device waits for the B-device to signal a connection. */
#define   UOTGHS_FSM_DRDSTATE_A_HOST (0x3u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device that operates in Host mode is operational. */
#define   UOTGHS_FSM_DRDSTATE_A_SUSPEND (0x4u << 0) /**< \brief (UOTGHS_FSM) The A-device operating as a host is in the suspend mode. */
#define   UOTGHS_FSM_DRDSTATE_A_PERIPHERAL (0x5u << 0) /**< \brief (UOTGHS_FSM) The A-device operates as a peripheral. */
#define   UOTGHS_FSM_DRDSTATE_A_WAIT_VFALL (0x6u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device waits for the voltage on VBus to drop below the A-device Session Valid threshold (1.4 V). */
#define   UOTGHS_FSM_DRDSTATE_A_VBUS_ERR (0x7u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device waits for recovery of the over-current condition that caused it to enter this state. */
#define   UOTGHS_FSM_DRDSTATE_A_WAIT_DISCHARGE (0x8u << 0) /**< \brief (UOTGHS_FSM) In this state, the A-device waits for the data USB line to discharge (100 us). */
#define   UOTGHS_FSM_DRDSTATE_B_IDLE (0x9u << 0) /**< \brief (UOTGHS_FSM) This is the start state for B-device (when the ID pin is 1). */
#define   UOTGHS_FSM_DRDSTATE_B_PERIPHERAL (0xAu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device acts as the peripheral. */
#define   UOTGHS_FSM_DRDSTATE_B_WAIT_BEGIN_HNP (0xBu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device is in suspend mode and waits until 3 ms before initiating the HNP protocol if requested. */
#define   UOTGHS_FSM_DRDSTATE_B_WAIT_DISCHARGE (0xCu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device waits for the data USB line to discharge (100 us) before becoming Host. */
#define   UOTGHS_FSM_DRDSTATE_B_WAIT_ACON (0xDu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device waits for the A-device to signal a connect before becoming B-Host. */
#define   UOTGHS_FSM_DRDSTATE_B_HOST (0xEu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device acts as the Host. */
#define   UOTGHS_FSM_DRDSTATE_B_SRP_INIT (0xFu << 0) /**< \brief (UOTGHS_FSM) In this state, the B-device attempts to start a session using the SRP protocol. */
/*@}*/ /* end of group SAM3XA_UOTGHS */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Universal Synchronous Asynchronous Receiver Transmitter */
/* ============================================================================= */
/** \addtogroup SAM3XA_USART Universal Synchronous Asynchronous Receiver Transmitter */
/*@{*/

/** \brief Usart hardware registers */
typedef struct {
  WoReg US_CR;         /**< \brief (Usart Offset: 0x0000) Control Register */
  RwReg US_MR;         /**< \brief (Usart Offset: 0x0004) Mode Register */
  WoReg US_IER;        /**< \brief (Usart Offset: 0x0008) Interrupt Enable Register */
  WoReg US_IDR;        /**< \brief (Usart Offset: 0x000C) Interrupt Disable Register */
  RoReg US_IMR;        /**< \brief (Usart Offset: 0x0010) Interrupt Mask Register */
  RoReg US_CSR;        /**< \brief (Usart Offset: 0x0014) Channel Status Register */
  RoReg US_RHR;        /**< \brief (Usart Offset: 0x0018) Receiver Holding Register */
  WoReg US_THR;        /**< \brief (Usart Offset: 0x001C) Transmitter Holding Register */
  RwReg US_BRGR;       /**< \brief (Usart Offset: 0x0020) Baud Rate Generator Register */
  RwReg US_RTOR;       /**< \brief (Usart Offset: 0x0024) Receiver Time-out Register */
  RwReg US_TTGR;       /**< \brief (Usart Offset: 0x0028) Transmitter Timeguard Register */
  RoReg Reserved1[5];
  RwReg US_FIDI;       /**< \brief (Usart Offset: 0x0040) FI DI Ratio Register */
  RoReg US_NER;        /**< \brief (Usart Offset: 0x0044) Number of Errors Register */
  RoReg Reserved2[1];
  RwReg US_IF;         /**< \brief (Usart Offset: 0x004C) IrDA Filter Register */
  RwReg US_MAN;        /**< \brief (Usart Offset: 0x0050) Manchester Encoder Decoder Register */
  RwReg US_LINMR;      /**< \brief (Usart Offset: 0x0054) LIN Mode Register */
  RwReg US_LINIR;      /**< \brief (Usart Offset: 0x0058) LIN Identifier Register */
  RoReg Reserved3[34];
  RwReg US_WPMR;       /**< \brief (Usart Offset: 0xE4) Write Protect Mode Register */
  RoReg US_WPSR;       /**< \brief (Usart Offset: 0xE8) Write Protect Status Register */
  RoReg Reserved4[5];
  RwReg US_RPR;        /**< \brief (Usart Offset: 0x100) Receive Pointer Register */
  RwReg US_RCR;        /**< \brief (Usart Offset: 0x104) Receive Counter Register */
  RwReg US_TPR;        /**< \brief (Usart Offset: 0x108) Transmit Pointer Register */
  RwReg US_TCR;        /**< \brief (Usart Offset: 0x10C) Transmit Counter Register */
  RwReg US_RNPR;       /**< \brief (Usart Offset: 0x110) Receive Next Pointer Register */
  RwReg US_RNCR;       /**< \brief (Usart Offset: 0x114) Receive Next Counter Register */
  RwReg US_TNPR;       /**< \brief (Usart Offset: 0x118) Transmit Next Pointer Register */
  RwReg US_TNCR;       /**< \brief (Usart Offset: 0x11C) Transmit Next Counter Register */
  WoReg US_PTCR;       /**< \brief (Usart Offset: 0x120) Transfer Control Register */
  RoReg US_PTSR;       /**< \brief (Usart Offset: 0x124) Transfer Status Register */
} Usart;
/* -------- US_CR : (USART Offset: 0x0000) Control Register -------- */
#define US_CR_RSTRX (0x1u << 2) /**< \brief (US_CR) Reset Receiver */
#define US_CR_RSTTX (0x1u << 3) /**< \brief (US_CR) Reset Transmitter */
#define US_CR_RXEN (0x1u << 4) /**< \brief (US_CR) Receiver Enable */
#define US_CR_RXDIS (0x1u << 5) /**< \brief (US_CR) Receiver Disable */
#define US_CR_TXEN (0x1u << 6) /**< \brief (US_CR) Transmitter Enable */
#define US_CR_TXDIS (0x1u << 7) /**< \brief (US_CR) Transmitter Disable */
#define US_CR_RSTSTA (0x1u << 8) /**< \brief (US_CR) Reset Status Bits */
#define US_CR_STTBRK (0x1u << 9) /**< \brief (US_CR) Start Break */
#define US_CR_STPBRK (0x1u << 10) /**< \brief (US_CR) Stop Break */
#define US_CR_STTTO (0x1u << 11) /**< \brief (US_CR) Start Time-out */
#define US_CR_SENDA (0x1u << 12) /**< \brief (US_CR) Send Address */
#define US_CR_RSTIT (0x1u << 13) /**< \brief (US_CR) Reset Iterations */
#define US_CR_RSTNACK (0x1u << 14) /**< \brief (US_CR) Reset Non Acknowledge */
#define US_CR_RETTO (0x1u << 15) /**< \brief (US_CR) Rearm Time-out */
#define US_CR_RTSEN (0x1u << 18) /**< \brief (US_CR) Request to Send Enable */
#define US_CR_FCS (0x1u << 18) /**< \brief (US_CR) Force SPI Chip Select */
#define US_CR_RTSDIS (0x1u << 19) /**< \brief (US_CR) Request to Send Disable */
#define US_CR_RCS (0x1u << 19) /**< \brief (US_CR) Release SPI Chip Select */
#define US_CR_LINABT (0x1u << 20) /**< \brief (US_CR) Abort LIN Transmission */
#define US_CR_LINWKUP (0x1u << 21) /**< \brief (US_CR) Send LIN Wakeup Signal */
/* -------- US_MR : (USART Offset: 0x0004) Mode Register -------- */
#define US_MR_USART_MODE_Pos 0
#define US_MR_USART_MODE_Msk (0xfu << US_MR_USART_MODE_Pos) /**< \brief (US_MR)  */
#define   US_MR_USART_MODE_NORMAL (0x0u << 0) /**< \brief (US_MR) Normal mode */
#define   US_MR_USART_MODE_RS485 (0x1u << 0) /**< \brief (US_MR) RS485 */
#define   US_MR_USART_MODE_HW_HANDSHAKING (0x2u << 0) /**< \brief (US_MR) Hardware Handshaking */
#define   US_MR_USART_MODE_IS07816_T_0 (0x4u << 0) /**< \brief (US_MR) IS07816 Protocol: T = 0 */
#define   US_MR_USART_MODE_IS07816_T_1 (0x6u << 0) /**< \brief (US_MR) IS07816 Protocol: T = 1 */
#define   US_MR_USART_MODE_IRDA (0x8u << 0) /**< \brief (US_MR) IrDA */
#define   US_MR_USART_MODE_LIN_MASTER (0xAu << 0) /**< \brief (US_MR) LIN Master */
#define   US_MR_USART_MODE_LIN_SLAVE (0xBu << 0) /**< \brief (US_MR) LIN Slave */
#define   US_MR_USART_MODE_SPI_MASTER (0xEu << 0) /**< \brief (US_MR) SPI Master */
#define   US_MR_USART_MODE_SPI_SLAVE (0xFu << 0) /**< \brief (US_MR) SPI Slave */
#define US_MR_USCLKS_Pos 4
#define US_MR_USCLKS_Msk (0x3u << US_MR_USCLKS_Pos) /**< \brief (US_MR) Clock Selection */
#define   US_MR_USCLKS_MCK (0x0u << 4) /**< \brief (US_MR) Master Clock MCK is selected */
#define   US_MR_USCLKS_DIV (0x1u << 4) /**< \brief (US_MR) Internal Clock Divided MCK/DIV (DIV=8) is selected */
#define   US_MR_USCLKS_SCK (0x3u << 4) /**< \brief (US_MR) Serial Clock SLK is selected */
#define US_MR_CHRL_Pos 6
#define US_MR_CHRL_Msk (0x3u << US_MR_CHRL_Pos) /**< \brief (US_MR) Character Length. */
#define   US_MR_CHRL_5_BIT (0x0u << 6) /**< \brief (US_MR) Character length is 5 bits */
#define   US_MR_CHRL_6_BIT (0x1u << 6) /**< \brief (US_MR) Character length is 6 bits */
#define   US_MR_CHRL_7_BIT (0x2u << 6) /**< \brief (US_MR) Character length is 7 bits */
#define   US_MR_CHRL_8_BIT (0x3u << 6) /**< \brief (US_MR) Character length is 8 bits */
#define US_MR_SYNC (0x1u << 8) /**< \brief (US_MR) Synchronous Mode Select */
#define US_MR_CPHA (0x1u << 8) /**< \brief (US_MR) SPI Clock Phase */
#define US_MR_PAR_Pos 9
#define US_MR_PAR_Msk (0x7u << US_MR_PAR_Pos) /**< \brief (US_MR) Parity Type */
#define   US_MR_PAR_EVEN (0x0u << 9) /**< \brief (US_MR) Even parity */
#define   US_MR_PAR_ODD (0x1u << 9) /**< \brief (US_MR) Odd parity */
#define   US_MR_PAR_SPACE (0x2u << 9) /**< \brief (US_MR) Parity forced to 0 (Space) */
#define   US_MR_PAR_MARK (0x3u << 9) /**< \brief (US_MR) Parity forced to 1 (Mark) */
#define   US_MR_PAR_NO (0x4u << 9) /**< \brief (US_MR) No parity */
#define   US_MR_PAR_MULTIDROP (0x6u << 9) /**< \brief (US_MR) Multidrop mode */
#define US_MR_NBSTOP_Pos 12
#define US_MR_NBSTOP_Msk (0x3u << US_MR_NBSTOP_Pos) /**< \brief (US_MR) Number of Stop Bits */
#define   US_MR_NBSTOP_1_BIT (0x0u << 12) /**< \brief (US_MR) 1 stop bit */
#define   US_MR_NBSTOP_1_5_BIT (0x1u << 12) /**< \brief (US_MR) 1.5 stop bit (SYNC = 0) or reserved (SYNC = 1) */
#define   US_MR_NBSTOP_2_BIT (0x2u << 12) /**< \brief (US_MR) 2 stop bits */
#define US_MR_CHMODE_Pos 14
#define US_MR_CHMODE_Msk (0x3u << US_MR_CHMODE_Pos) /**< \brief (US_MR) Channel Mode */
#define   US_MR_CHMODE_NORMAL (0x0u << 14) /**< \brief (US_MR) Normal Mode */
#define   US_MR_CHMODE_AUTOMATIC (0x1u << 14) /**< \brief (US_MR) Automatic Echo. Receiver input is connected to the TXD pin. */
#define   US_MR_CHMODE_LOCAL_LOOPBACK (0x2u << 14) /**< \brief (US_MR) Local Loopback. Transmitter output is connected to the Receiver Input. */
#define   US_MR_CHMODE_REMOTE_LOOPBACK (0x3u << 14) /**< \brief (US_MR) Remote Loopback. RXD pin is internally connected to the TXD pin. */
#define US_MR_MSBF (0x1u << 16) /**< \brief (US_MR) Bit Order */
#define US_MR_CPOL (0x1u << 16) /**< \brief (US_MR) SPI Clock Polarity */
#define US_MR_MODE9 (0x1u << 17) /**< \brief (US_MR) 9-bit Character Length */
#define US_MR_CLKO (0x1u << 18) /**< \brief (US_MR) Clock Output Select */
#define US_MR_OVER (0x1u << 19) /**< \brief (US_MR) Oversampling Mode */
#define US_MR_INACK (0x1u << 20) /**< \brief (US_MR) Inhibit Non Acknowledge */
#define US_MR_DSNACK (0x1u << 21) /**< \brief (US_MR) Disable Successive NACK */
#define US_MR_VAR_SYNC (0x1u << 22) /**< \brief (US_MR) Variable Synchronization of Command/Data Sync Start Frame Delimiter */
#define US_MR_INVDATA (0x1u << 23) /**< \brief (US_MR) INverted Data */
#define US_MR_MAX_ITERATION_Pos 24
#define US_MR_MAX_ITERATION_Msk (0x7u << US_MR_MAX_ITERATION_Pos) /**< \brief (US_MR)  */
#define US_MR_MAX_ITERATION(value) ((US_MR_MAX_ITERATION_Msk & ((value) << US_MR_MAX_ITERATION_Pos)))
#define US_MR_FILTER (0x1u << 28) /**< \brief (US_MR) Infrared Receive Line Filter */
#define US_MR_MAN (0x1u << 29) /**< \brief (US_MR) Manchester Encoder/Decoder Enable */
#define US_MR_MODSYNC (0x1u << 30) /**< \brief (US_MR) Manchester Synchronization Mode */
#define US_MR_ONEBIT (0x1u << 31) /**< \brief (US_MR) Start Frame Delimiter Selector */
/* -------- US_IER : (USART Offset: 0x0008) Interrupt Enable Register -------- */
#define US_IER_RXRDY (0x1u << 0) /**< \brief (US_IER) RXRDY Interrupt Enable */
#define US_IER_TXRDY (0x1u << 1) /**< \brief (US_IER) TXRDY Interrupt Enable */
#define US_IER_RXBRK (0x1u << 2) /**< \brief (US_IER) Receiver Break Interrupt Enable */
#define US_IER_ENDRX (0x1u << 3) /**< \brief (US_IER) End of Receive Transfer Interrupt Enable */
#define US_IER_ENDTX (0x1u << 4) /**< \brief (US_IER) End of Transmit Interrupt Enable */
#define US_IER_OVRE (0x1u << 5) /**< \brief (US_IER) Overrun Error Interrupt Enable */
#define US_IER_FRAME (0x1u << 6) /**< \brief (US_IER) Framing Error Interrupt Enable */
#define US_IER_PARE (0x1u << 7) /**< \brief (US_IER) Parity Error Interrupt Enable */
#define US_IER_TIMEOUT (0x1u << 8) /**< \brief (US_IER) Time-out Interrupt Enable */
#define US_IER_TXEMPTY (0x1u << 9) /**< \brief (US_IER) TXEMPTY Interrupt Enable */
#define US_IER_ITER (0x1u << 10) /**< \brief (US_IER) Max number of Repetitions Reached */
#define US_IER_UNRE (0x1u << 10) /**< \brief (US_IER) SPI Underrun Error */
#define US_IER_TXBUFE (0x1u << 11) /**< \brief (US_IER) Buffer Empty Interrupt Enable */
#define US_IER_RXBUFF (0x1u << 12) /**< \brief (US_IER) Buffer Full Interrupt Enable */
#define US_IER_NACK (0x1u << 13) /**< \brief (US_IER) Non Acknowledge Interrupt Enable */
#define US_IER_LINBK (0x1u << 13) /**< \brief (US_IER) LIN Break Sent or LIN Break Received Interrupt Enable */
#define US_IER_LINID (0x1u << 14) /**< \brief (US_IER) LIN Identifier Sent or LIN Identifier Received Interrupt Enable */
#define US_IER_LINTC (0x1u << 15) /**< \brief (US_IER) LIN Transfer Completed Interrupt Enable */
#define US_IER_CTSIC (0x1u << 19) /**< \brief (US_IER) Clear to Send Input Change Interrupt Enable */
#define US_IER_MANE (0x1u << 24) /**< \brief (US_IER) Manchester Error Interrupt Enable */
#define US_IER_LINBE (0x1u << 25) /**< \brief (US_IER) LIN Bus Error Interrupt Enable */
#define US_IER_LINISFE (0x1u << 26) /**< \brief (US_IER) LIN Inconsistent Synch Field Error Interrupt Enable */
#define US_IER_LINIPE (0x1u << 27) /**< \brief (US_IER) LIN Identifier Parity Interrupt Enable */
#define US_IER_LINCE (0x1u << 28) /**< \brief (US_IER) LIN Checksum Error Interrupt Enable */
#define US_IER_LINSNRE (0x1u << 29) /**< \brief (US_IER) LIN Slave Not Responding Error Interrupt Enable */
/* -------- US_IDR : (USART Offset: 0x000C) Interrupt Disable Register -------- */
#define US_IDR_RXRDY (0x1u << 0) /**< \brief (US_IDR) RXRDY Interrupt Disable */
#define US_IDR_TXRDY (0x1u << 1) /**< \brief (US_IDR) TXRDY Interrupt Disable */
#define US_IDR_RXBRK (0x1u << 2) /**< \brief (US_IDR) Receiver Break Interrupt Disable */
#define US_IDR_ENDRX (0x1u << 3) /**< \brief (US_IDR) End of Receive Transfer Interrupt Disable */
#define US_IDR_ENDTX (0x1u << 4) /**< \brief (US_IDR) End of Transmit Interrupt Disable */
#define US_IDR_OVRE (0x1u << 5) /**< \brief (US_IDR) Overrun Error Interrupt Disable */
#define US_IDR_FRAME (0x1u << 6) /**< \brief (US_IDR) Framing Error Interrupt Disable */
#define US_IDR_PARE (0x1u << 7) /**< \brief (US_IDR) Parity Error Interrupt Disable */
#define US_IDR_TIMEOUT (0x1u << 8) /**< \brief (US_IDR) Time-out Interrupt Disable */
#define US_IDR_TXEMPTY (0x1u << 9) /**< \brief (US_IDR) TXEMPTY Interrupt Disable */
#define US_IDR_ITER (0x1u << 10) /**< \brief (US_IDR) Max number of Repetitions Reached Disable */
#define US_IDR_UNRE (0x1u << 10) /**< \brief (US_IDR) SPI Underrun Error Disable */
#define US_IDR_TXBUFE (0x1u << 11) /**< \brief (US_IDR) Buffer Empty Interrupt Disable */
#define US_IDR_RXBUFF (0x1u << 12) /**< \brief (US_IDR) Buffer Full Interrupt Disable */
#define US_IDR_NACK (0x1u << 13) /**< \brief (US_IDR) Non Acknowledge Interrupt Disable */
#define US_IDR_LINBK (0x1u << 13) /**< \brief (US_IDR) LIN Break Sent or LIN Break Received Interrupt Disable */
#define US_IDR_LINID (0x1u << 14) /**< \brief (US_IDR) LIN Identifier Sent or LIN Identifier Received Interrupt Disable */
#define US_IDR_LINTC (0x1u << 15) /**< \brief (US_IDR) LIN Transfer Completed Interrupt Disable */
#define US_IDR_CTSIC (0x1u << 19) /**< \brief (US_IDR) Clear to Send Input Change Interrupt Disable */
#define US_IDR_MANE (0x1u << 24) /**< \brief (US_IDR) Manchester Error Interrupt Disable */
#define US_IDR_LINBE (0x1u << 25) /**< \brief (US_IDR) LIN Bus Error Interrupt Disable */
#define US_IDR_LINISFE (0x1u << 26) /**< \brief (US_IDR) LIN Inconsistent Synch Field Error Interrupt Disable */
#define US_IDR_LINIPE (0x1u << 27) /**< \brief (US_IDR) LIN Identifier Parity Interrupt Disable */
#define US_IDR_LINCE (0x1u << 28) /**< \brief (US_IDR) LIN Checksum Error Interrupt Disable */
#define US_IDR_LINSNRE (0x1u << 29) /**< \brief (US_IDR) LIN Slave Not Responding Error Interrupt Disable */
/* -------- US_IMR : (USART Offset: 0x0010) Interrupt Mask Register -------- */
#define US_IMR_RXRDY (0x1u << 0) /**< \brief (US_IMR) RXRDY Interrupt Mask */
#define US_IMR_TXRDY (0x1u << 1) /**< \brief (US_IMR) TXRDY Interrupt Mask */
#define US_IMR_RXBRK (0x1u << 2) /**< \brief (US_IMR) Receiver Break Interrupt Mask */
#define US_IMR_ENDRX (0x1u << 3) /**< \brief (US_IMR) End of Receive Transfer Interrupt Mask */
#define US_IMR_ENDTX (0x1u << 4) /**< \brief (US_IMR) End of Transmit Interrupt Mask */
#define US_IMR_OVRE (0x1u << 5) /**< \brief (US_IMR) Overrun Error Interrupt Mask */
#define US_IMR_FRAME (0x1u << 6) /**< \brief (US_IMR) Framing Error Interrupt Mask */
#define US_IMR_PARE (0x1u << 7) /**< \brief (US_IMR) Parity Error Interrupt Mask */
#define US_IMR_TIMEOUT (0x1u << 8) /**< \brief (US_IMR) Time-out Interrupt Mask */
#define US_IMR_TXEMPTY (0x1u << 9) /**< \brief (US_IMR) TXEMPTY Interrupt Mask */
#define US_IMR_ITER (0x1u << 10) /**< \brief (US_IMR) Max number of Repetitions Reached Mask */
#define US_IMR_UNRE (0x1u << 10) /**< \brief (US_IMR) SPI Underrun Error Mask */
#define US_IMR_TXBUFE (0x1u << 11) /**< \brief (US_IMR) Buffer Empty Interrupt Mask */
#define US_IMR_RXBUFF (0x1u << 12) /**< \brief (US_IMR) Buffer Full Interrupt Mask */
#define US_IMR_NACK (0x1u << 13) /**< \brief (US_IMR) Non Acknowledge Interrupt Mask */
#define US_IMR_LINBK (0x1u << 13) /**< \brief (US_IMR) LIN Break Sent or LIN Break Received Interrupt Mask */
#define US_IMR_LINID (0x1u << 14) /**< \brief (US_IMR) LIN Identifier Sent or LIN Identifier Received Interrupt Mask */
#define US_IMR_LINTC (0x1u << 15) /**< \brief (US_IMR) LIN Transfer Completed Interrupt Mask */
#define US_IMR_CTSIC (0x1u << 19) /**< \brief (US_IMR) Clear to Send Input Change Interrupt Mask */
#define US_IMR_MANE (0x1u << 24) /**< \brief (US_IMR) Manchester Error Interrupt Mask */
#define US_IMR_LINBE (0x1u << 25) /**< \brief (US_IMR) LIN Bus Error Interrupt Mask */
#define US_IMR_LINISFE (0x1u << 26) /**< \brief (US_IMR) LIN Inconsistent Synch Field Error Interrupt Mask */
#define US_IMR_LINIPE (0x1u << 27) /**< \brief (US_IMR) LIN Identifier Parity Interrupt Mask */
#define US_IMR_LINCE (0x1u << 28) /**< \brief (US_IMR) LIN Checksum Error Interrupt Mask */
#define US_IMR_LINSNRE (0x1u << 29) /**< \brief (US_IMR) LIN Slave Not Responding Error Interrupt Mask */
/* -------- US_CSR : (USART Offset: 0x0014) Channel Status Register -------- */
#define US_CSR_RXRDY (0x1u << 0) /**< \brief (US_CSR) Receiver Ready */
#define US_CSR_TXRDY (0x1u << 1) /**< \brief (US_CSR) Transmitter Ready */
#define US_CSR_RXBRK (0x1u << 2) /**< \brief (US_CSR) Break Received/End of Break */
#define US_CSR_ENDRX (0x1u << 3) /**< \brief (US_CSR) End of Receiver Transfer */
#define US_CSR_ENDTX (0x1u << 4) /**< \brief (US_CSR) End of Transmitter Transfer */
#define US_CSR_OVRE (0x1u << 5) /**< \brief (US_CSR) Overrun Error */
#define US_CSR_FRAME (0x1u << 6) /**< \brief (US_CSR) Framing Error */
#define US_CSR_PARE (0x1u << 7) /**< \brief (US_CSR) Parity Error */
#define US_CSR_TIMEOUT (0x1u << 8) /**< \brief (US_CSR) Receiver Time-out */
#define US_CSR_TXEMPTY (0x1u << 9) /**< \brief (US_CSR) Transmitter Empty */
#define US_CSR_ITER (0x1u << 10) /**< \brief (US_CSR) Max number of Repetitions Reached */
#define US_CSR_UNRE (0x1u << 10) /**< \brief (US_CSR) SPI Underrun Error */
#define US_CSR_TXBUFE (0x1u << 11) /**< \brief (US_CSR) Transmission Buffer Empty */
#define US_CSR_RXBUFF (0x1u << 12) /**< \brief (US_CSR) Reception Buffer Full */
#define US_CSR_NACK (0x1u << 13) /**< \brief (US_CSR) Non Acknowledge Interrupt */
#define US_CSR_LINBK (0x1u << 13) /**< \brief (US_CSR) LIN Break Sent or LIN Break Received */
#define US_CSR_LINID (0x1u << 14) /**< \brief (US_CSR) LIN Identifier Sent or LIN Identifier Received */
#define US_CSR_LINTC (0x1u << 15) /**< \brief (US_CSR) LIN Transfer Completed */
#define US_CSR_CTSIC (0x1u << 19) /**< \brief (US_CSR) Clear to Send Input Change Flag */
#define US_CSR_CTS (0x1u << 23) /**< \brief (US_CSR) Image of CTS Input */
#define US_CSR_LINBLS (0x1u << 23) /**< \brief (US_CSR) LIN Bus Line Status */
#define US_CSR_MANERR (0x1u << 24) /**< \brief (US_CSR) Manchester Error */
#define US_CSR_LINBE (0x1u << 25) /**< \brief (US_CSR) LIN Bit Error */
#define US_CSR_LINISFE (0x1u << 26) /**< \brief (US_CSR) LIN Inconsistent Synch Field Error */
#define US_CSR_LINIPE (0x1u << 27) /**< \brief (US_CSR) LIN Identifier Parity Error */
#define US_CSR_LINCE (0x1u << 28) /**< \brief (US_CSR) LIN Checksum Error */
#define US_CSR_LINSNRE (0x1u << 29) /**< \brief (US_CSR) LIN Slave Not Responding Error */
/* -------- US_RHR : (USART Offset: 0x0018) Receiver Holding Register -------- */
#define US_RHR_RXCHR_Pos 0
#define US_RHR_RXCHR_Msk (0x1ffu << US_RHR_RXCHR_Pos) /**< \brief (US_RHR) Received Character */
#define US_RHR_RXSYNH (0x1u << 15) /**< \brief (US_RHR) Received Sync */
/* -------- US_THR : (USART Offset: 0x001C) Transmitter Holding Register -------- */
#define US_THR_TXCHR_Pos 0
#define US_THR_TXCHR_Msk (0x1ffu << US_THR_TXCHR_Pos) /**< \brief (US_THR) Character to be Transmitted */
#define US_THR_TXCHR(value) ((US_THR_TXCHR_Msk & ((value) << US_THR_TXCHR_Pos)))
#define US_THR_TXSYNH (0x1u << 15) /**< \brief (US_THR) Sync Field to be transmitted */
/* -------- US_BRGR : (USART Offset: 0x0020) Baud Rate Generator Register -------- */
#define US_BRGR_CD_Pos 0
#define US_BRGR_CD_Msk (0xffffu << US_BRGR_CD_Pos) /**< \brief (US_BRGR) Clock Divider */
#define US_BRGR_CD(value) ((US_BRGR_CD_Msk & ((value) << US_BRGR_CD_Pos)))
#define US_BRGR_FP_Pos 16
#define US_BRGR_FP_Msk (0x7u << US_BRGR_FP_Pos) /**< \brief (US_BRGR) Fractional Part */
#define US_BRGR_FP(value) ((US_BRGR_FP_Msk & ((value) << US_BRGR_FP_Pos)))
/* -------- US_RTOR : (USART Offset: 0x0024) Receiver Time-out Register -------- */
#define US_RTOR_TO_Pos 0
#define US_RTOR_TO_Msk (0x1ffffu << US_RTOR_TO_Pos) /**< \brief (US_RTOR) Time-out Value */
#define US_RTOR_TO(value) ((US_RTOR_TO_Msk & ((value) << US_RTOR_TO_Pos)))
/* -------- US_TTGR : (USART Offset: 0x0028) Transmitter Timeguard Register -------- */
#define US_TTGR_TG_Pos 0
#define US_TTGR_TG_Msk (0xffu << US_TTGR_TG_Pos) /**< \brief (US_TTGR) Timeguard Value */
#define US_TTGR_TG(value) ((US_TTGR_TG_Msk & ((value) << US_TTGR_TG_Pos)))
/* -------- US_FIDI : (USART Offset: 0x0040) FI DI Ratio Register -------- */
#define US_FIDI_FI_DI_RATIO_Pos 0
#define US_FIDI_FI_DI_RATIO_Msk (0x7ffu << US_FIDI_FI_DI_RATIO_Pos) /**< \brief (US_FIDI) FI Over DI Ratio Value */
#define US_FIDI_FI_DI_RATIO(value) ((US_FIDI_FI_DI_RATIO_Msk & ((value) << US_FIDI_FI_DI_RATIO_Pos)))
/* -------- US_NER : (USART Offset: 0x0044) Number of Errors Register -------- */
#define US_NER_NB_ERRORS_Pos 0
#define US_NER_NB_ERRORS_Msk (0xffu << US_NER_NB_ERRORS_Pos) /**< \brief (US_NER) Number of Errors */
/* -------- US_IF : (USART Offset: 0x004C) IrDA Filter Register -------- */
#define US_IF_IRDA_FILTER_Pos 0
#define US_IF_IRDA_FILTER_Msk (0xffu << US_IF_IRDA_FILTER_Pos) /**< \brief (US_IF) IrDA Filter */
#define US_IF_IRDA_FILTER(value) ((US_IF_IRDA_FILTER_Msk & ((value) << US_IF_IRDA_FILTER_Pos)))
/* -------- US_MAN : (USART Offset: 0x0050) Manchester Encoder Decoder Register -------- */
#define US_MAN_TX_PL_Pos 0
#define US_MAN_TX_PL_Msk (0xfu << US_MAN_TX_PL_Pos) /**< \brief (US_MAN) Transmitter Preamble Length */
#define US_MAN_TX_PL(value) ((US_MAN_TX_PL_Msk & ((value) << US_MAN_TX_PL_Pos)))
#define US_MAN_TX_PP_Pos 8
#define US_MAN_TX_PP_Msk (0x3u << US_MAN_TX_PP_Pos) /**< \brief (US_MAN) Transmitter Preamble Pattern */
#define   US_MAN_TX_PP_ALL_ONE (0x0u << 8) /**< \brief (US_MAN) The preamble is composed of '1's */
#define   US_MAN_TX_PP_ALL_ZERO (0x1u << 8) /**< \brief (US_MAN) The preamble is composed of '0's */
#define   US_MAN_TX_PP_ZERO_ONE (0x2u << 8) /**< \brief (US_MAN) The preamble is composed of '01's */
#define   US_MAN_TX_PP_ONE_ZERO (0x3u << 8) /**< \brief (US_MAN) The preamble is composed of '10's */
#define US_MAN_TX_MPOL (0x1u << 12) /**< \brief (US_MAN) Transmitter Manchester Polarity */
#define US_MAN_RX_PL_Pos 16
#define US_MAN_RX_PL_Msk (0xfu << US_MAN_RX_PL_Pos) /**< \brief (US_MAN) Receiver Preamble Length */
#define US_MAN_RX_PL(value) ((US_MAN_RX_PL_Msk & ((value) << US_MAN_RX_PL_Pos)))
#define US_MAN_RX_PP_Pos 24
#define US_MAN_RX_PP_Msk (0x3u << US_MAN_RX_PP_Pos) /**< \brief (US_MAN) Receiver Preamble Pattern detected */
#define   US_MAN_RX_PP_ALL_ONE (0x0u << 24) /**< \brief (US_MAN) The preamble is composed of '1's */
#define   US_MAN_RX_PP_ALL_ZERO (0x1u << 24) /**< \brief (US_MAN) The preamble is composed of '0's */
#define   US_MAN_RX_PP_ZERO_ONE (0x2u << 24) /**< \brief (US_MAN) The preamble is composed of '01's */
#define   US_MAN_RX_PP_ONE_ZERO (0x3u << 24) /**< \brief (US_MAN) The preamble is composed of '10's */
#define US_MAN_RX_MPOL (0x1u << 28) /**< \brief (US_MAN) Receiver Manchester Polarity */
#define US_MAN_STUCKTO1 (0x1u << 29) /**< \brief (US_MAN)  */
#define US_MAN_DRIFT (0x1u << 30) /**< \brief (US_MAN) Drift compensation */
/* -------- US_LINMR : (USART Offset: 0x0054) LIN Mode Register -------- */
#define US_LINMR_NACT_Pos 0
#define US_LINMR_NACT_Msk (0x3u << US_LINMR_NACT_Pos) /**< \brief (US_LINMR) LIN Node Action */
#define   US_LINMR_NACT_PUBLISH (0x0u << 0) /**< \brief (US_LINMR) The USART transmits the response. */
#define   US_LINMR_NACT_SUBSCRIBE (0x1u << 0) /**< \brief (US_LINMR) The USART receives the response. */
#define   US_LINMR_NACT_IGNORE (0x2u << 0) /**< \brief (US_LINMR) The USART does not transmit and does not receive the response. */
#define US_LINMR_PARDIS (0x1u << 2) /**< \brief (US_LINMR) Parity Disable */
#define US_LINMR_CHKDIS (0x1u << 3) /**< \brief (US_LINMR) Checksum Disable */
#define US_LINMR_CHKTYP (0x1u << 4) /**< \brief (US_LINMR) Checksum Type */
#define US_LINMR_DLM (0x1u << 5) /**< \brief (US_LINMR) Data Length Mode */
#define US_LINMR_FSDIS (0x1u << 6) /**< \brief (US_LINMR) Frame Slot Mode Disable */
#define US_LINMR_WKUPTYP (0x1u << 7) /**< \brief (US_LINMR) Wakeup Signal Type */
#define US_LINMR_DLC_Pos 8
#define US_LINMR_DLC_Msk (0xffu << US_LINMR_DLC_Pos) /**< \brief (US_LINMR) Data Length Control */
#define US_LINMR_DLC(value) ((US_LINMR_DLC_Msk & ((value) << US_LINMR_DLC_Pos)))
#define US_LINMR_PDCM (0x1u << 16) /**< \brief (US_LINMR) PDC Mode */
/* -------- US_LINIR : (USART Offset: 0x0058) LIN Identifier Register -------- */
#define US_LINIR_IDCHR_Pos 0
#define US_LINIR_IDCHR_Msk (0xffu << US_LINIR_IDCHR_Pos) /**< \brief (US_LINIR) Identifier Character */
#define US_LINIR_IDCHR(value) ((US_LINIR_IDCHR_Msk & ((value) << US_LINIR_IDCHR_Pos)))
/* -------- US_WPMR : (USART Offset: 0xE4) Write Protect Mode Register -------- */
#define US_WPMR_WPEN (0x1u << 0) /**< \brief (US_WPMR) Write Protect Enable */
#define US_WPMR_WPKEY_Pos 8
#define US_WPMR_WPKEY_Msk (0xffffffu << US_WPMR_WPKEY_Pos) /**< \brief (US_WPMR) Write Protect KEY */
#define US_WPMR_WPKEY(value) ((US_WPMR_WPKEY_Msk & ((value) << US_WPMR_WPKEY_Pos)))
/* -------- US_WPSR : (USART Offset: 0xE8) Write Protect Status Register -------- */
#define US_WPSR_WPVS (0x1u << 0) /**< \brief (US_WPSR) Write Protect Violation Status */
#define US_WPSR_WPVSRC_Pos 8
#define US_WPSR_WPVSRC_Msk (0xffffu << US_WPSR_WPVSRC_Pos) /**< \brief (US_WPSR) Write Protect Violation Source */
/* -------- US_RPR : (USART Offset: 0x100) Receive Pointer Register -------- */
#define US_RPR_RXPTR_Pos 0
#define US_RPR_RXPTR_Msk (0xffffffffu << US_RPR_RXPTR_Pos) /**< \brief (US_RPR) Receive Pointer Register */
#define US_RPR_RXPTR(value) ((US_RPR_RXPTR_Msk & ((value) << US_RPR_RXPTR_Pos)))
/* -------- US_RCR : (USART Offset: 0x104) Receive Counter Register -------- */
#define US_RCR_RXCTR_Pos 0
#define US_RCR_RXCTR_Msk (0xffffu << US_RCR_RXCTR_Pos) /**< \brief (US_RCR) Receive Counter Register */
#define US_RCR_RXCTR(value) ((US_RCR_RXCTR_Msk & ((value) << US_RCR_RXCTR_Pos)))
/* -------- US_TPR : (USART Offset: 0x108) Transmit Pointer Register -------- */
#define US_TPR_TXPTR_Pos 0
#define US_TPR_TXPTR_Msk (0xffffffffu << US_TPR_TXPTR_Pos) /**< \brief (US_TPR) Transmit Counter Register */
#define US_TPR_TXPTR(value) ((US_TPR_TXPTR_Msk & ((value) << US_TPR_TXPTR_Pos)))
/* -------- US_TCR : (USART Offset: 0x10C) Transmit Counter Register -------- */
#define US_TCR_TXCTR_Pos 0
#define US_TCR_TXCTR_Msk (0xffffu << US_TCR_TXCTR_Pos) /**< \brief (US_TCR) Transmit Counter Register */
#define US_TCR_TXCTR(value) ((US_TCR_TXCTR_Msk & ((value) << US_TCR_TXCTR_Pos)))
/* -------- US_RNPR : (USART Offset: 0x110) Receive Next Pointer Register -------- */
#define US_RNPR_RXNPTR_Pos 0
#define US_RNPR_RXNPTR_Msk (0xffffffffu << US_RNPR_RXNPTR_Pos) /**< \brief (US_RNPR) Receive Next Pointer */
#define US_RNPR_RXNPTR(value) ((US_RNPR_RXNPTR_Msk & ((value) << US_RNPR_RXNPTR_Pos)))
/* -------- US_RNCR : (USART Offset: 0x114) Receive Next Counter Register -------- */
#define US_RNCR_RXNCTR_Pos 0
#define US_RNCR_RXNCTR_Msk (0xffffu << US_RNCR_RXNCTR_Pos) /**< \brief (US_RNCR) Receive Next Counter */
#define US_RNCR_RXNCTR(value) ((US_RNCR_RXNCTR_Msk & ((value) << US_RNCR_RXNCTR_Pos)))
/* -------- US_TNPR : (USART Offset: 0x118) Transmit Next Pointer Register -------- */
#define US_TNPR_TXNPTR_Pos 0
#define US_TNPR_TXNPTR_Msk (0xffffffffu << US_TNPR_TXNPTR_Pos) /**< \brief (US_TNPR) Transmit Next Pointer */
#define US_TNPR_TXNPTR(value) ((US_TNPR_TXNPTR_Msk & ((value) << US_TNPR_TXNPTR_Pos)))
/* -------- US_TNCR : (USART Offset: 0x11C) Transmit Next Counter Register -------- */
#define US_TNCR_TXNCTR_Pos 0
#define US_TNCR_TXNCTR_Msk (0xffffu << US_TNCR_TXNCTR_Pos) /**< \brief (US_TNCR) Transmit Counter Next */
#define US_TNCR_TXNCTR(value) ((US_TNCR_TXNCTR_Msk & ((value) << US_TNCR_TXNCTR_Pos)))
/* -------- US_PTCR : (USART Offset: 0x120) Transfer Control Register -------- */
#define US_PTCR_RXTEN (0x1u << 0) /**< \brief (US_PTCR) Receiver Transfer Enable */
#define US_PTCR_RXTDIS (0x1u << 1) /**< \brief (US_PTCR) Receiver Transfer Disable */
#define US_PTCR_TXTEN (0x1u << 8) /**< \brief (US_PTCR) Transmitter Transfer Enable */
#define US_PTCR_TXTDIS (0x1u << 9) /**< \brief (US_PTCR) Transmitter Transfer Disable */
/* -------- US_PTSR : (USART Offset: 0x124) Transfer Status Register -------- */
#define US_PTSR_RXTEN (0x1u << 0) /**< \brief (US_PTSR) Receiver Transfer Enable */
#define US_PTSR_TXTEN (0x1u << 8) /**< \brief (US_PTSR) Transmitter Transfer Enable */
/*@}*/ /* end of group SAM3XA_USART */

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Watchdog Timer */
/* ============================================================================= */
/** \addtogroup SAM3XA_WDT Watchdog Timer */
/*@{*/

/** \brief Wdt hardware registers */
typedef struct {
  WoReg WDT_CR; /**< \brief (Wdt Offset: 0x00) Control Register */
  RwReg WDT_MR; /**< \brief (Wdt Offset: 0x04) Mode Register */
  RoReg WDT_SR; /**< \brief (Wdt Offset: 0x08) Status Register */
} Wdt;
/* -------- WDT_CR : (WDT Offset: 0x00) Control Register -------- */
#define WDT_CR_WDRSTT (0x1u << 0) /**< \brief (WDT_CR) Watchdog Restart */
#define WDT_CR_KEY_Pos 24
#define WDT_CR_KEY_Msk (0xffu << WDT_CR_KEY_Pos) /**< \brief (WDT_CR) Password */
#define WDT_CR_KEY(value) ((WDT_CR_KEY_Msk & ((value) << WDT_CR_KEY_Pos)))
/* -------- WDT_MR : (WDT Offset: 0x04) Mode Register -------- */
#define WDT_MR_WDV_Pos 0
#define WDT_MR_WDV_Msk (0xfffu << WDT_MR_WDV_Pos) /**< \brief (WDT_MR) Watchdog Counter Value */
#define WDT_MR_WDV(value) ((WDT_MR_WDV_Msk & ((value) << WDT_MR_WDV_Pos)))
#define WDT_MR_WDFIEN (0x1u << 12) /**< \brief (WDT_MR) Watchdog Fault Interrupt Enable */
#define WDT_MR_WDRSTEN (0x1u << 13) /**< \brief (WDT_MR) Watchdog Reset Enable */
#define WDT_MR_WDRPROC (0x1u << 14) /**< \brief (WDT_MR) Watchdog Reset Processor */
#define WDT_MR_WDDIS (0x1u << 15) /**< \brief (WDT_MR) Watchdog Disable */
#define WDT_MR_WDD_Pos 16
#define WDT_MR_WDD_Msk (0xfffu << WDT_MR_WDD_Pos) /**< \brief (WDT_MR) Watchdog Delta Value */
#define WDT_MR_WDD(value) ((WDT_MR_WDD_Msk & ((value) << WDT_MR_WDD_Pos)))
#define WDT_MR_WDDBGHLT (0x1u << 28) /**< \brief (WDT_MR) Watchdog Debug Halt */
#define WDT_MR_WDIDLEHLT (0x1u << 29) /**< \brief (WDT_MR) Watchdog Idle Halt */
/* -------- WDT_SR : (WDT Offset: 0x08) Status Register -------- */
#define WDT_SR_WDUNF (0x1u << 0) /**< \brief (WDT_SR) Watchdog Underflow */
#define WDT_SR_WDERR (0x1u << 1) /**< \brief (WDT_SR) Watchdog Error */
/*@}*/ /* end of group SAM3XA_WDT */

/*@}*/

/* ************************************************************************** */
/*   REGISTER ACCESS DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_reg Registers Access Definitions */
/*@{*/

#define REG_ADC_CR      (*(WoReg*)0x400C0000U) /**< \brief (ADC) Control Register */
#define REG_ADC_MR      (*(RwReg*)0x400C0004U) /**< \brief (ADC) Mode Register */
#define REG_ADC_SEQR1   (*(RwReg*)0x400C0008U) /**< \brief (ADC) Channel Sequence Register 1 */
#define REG_ADC_SEQR2   (*(RwReg*)0x400C000CU) /**< \brief (ADC) Channel Sequence Register 2 */
#define REG_ADC_CHER    (*(WoReg*)0x400C0010U) /**< \brief (ADC) Channel Enable Register */
#define REG_ADC_CHDR    (*(WoReg*)0x400C0014U) /**< \brief (ADC) Channel Disable Register */
#define REG_ADC_CHSR    (*(RoReg*)0x400C0018U) /**< \brief (ADC) Channel Status Register */
#define REG_ADC_LCDR    (*(RoReg*)0x400C0020U) /**< \brief (ADC) Last Converted Data Register */
#define REG_ADC_IER     (*(WoReg*)0x400C0024U) /**< \brief (ADC) Interrupt Enable Register */
#define REG_ADC_IDR     (*(WoReg*)0x400C0028U) /**< \brief (ADC) Interrupt Disable Register */
#define REG_ADC_IMR     (*(RoReg*)0x400C002CU) /**< \brief (ADC) Interrupt Mask Register */
#define REG_ADC_ISR     (*(RoReg*)0x400C0030U) /**< \brief (ADC) Interrupt Status Register */
#define REG_ADC_OVER    (*(RoReg*)0x400C003CU) /**< \brief (ADC) Overrun Status Register */
#define REG_ADC_EMR     (*(RwReg*)0x400C0040U) /**< \brief (ADC) Extended Mode Register */
#define REG_ADC_CWR     (*(RwReg*)0x400C0044U) /**< \brief (ADC) Compare Window Register */
#define REG_ADC_CGR     (*(RwReg*)0x400C0048U) /**< \brief (ADC) Channel Gain Register */
#define REG_ADC_COR     (*(RwReg*)0x400C004CU) /**< \brief (ADC) Channel Offset Register */
#define REG_ADC_CDR     (*(RoReg*)0x400C0050U) /**< \brief (ADC) Channel Data Register */
#define REG_ADC_ACR     (*(RwReg*)0x400C0094U) /**< \brief (ADC) Analog Control Register */
#define REG_ADC_WPMR    (*(RwReg*)0x400C00E4U) /**< \brief (ADC) Write Protect Mode Register */
#define REG_ADC_WPSR    (*(RoReg*)0x400C00E8U) /**< \brief (ADC) Write Protect Status Register */
#define REG_ADC_RPR     (*(RwReg*)0x400C0100U) /**< \brief (ADC) Receive Pointer Register */
#define REG_ADC_RCR     (*(RwReg*)0x400C0104U) /**< \brief (ADC) Receive Counter Register */
#define REG_ADC_RNPR    (*(RwReg*)0x400C0110U) /**< \brief (ADC) Receive Next Pointer Register */
#define REG_ADC_RNCR    (*(RwReg*)0x400C0114U) /**< \brief (ADC) Receive Next Counter Register */
#define REG_ADC_PTCR    (*(WoReg*)0x400C0120U) /**< \brief (ADC) Transfer Control Register */
#define REG_ADC_PTSR    (*(RoReg*)0x400C0124U) /**< \brief (ADC) Transfer Status Register */

#define REG_CAN0_MR      (*(RwReg*)0x400B4000U) /**< \brief (CAN0) Mode Register */
#define REG_CAN0_IER     (*(WoReg*)0x400B4004U) /**< \brief (CAN0) Interrupt Enable Register */
#define REG_CAN0_IDR     (*(WoReg*)0x400B4008U) /**< \brief (CAN0) Interrupt Disable Register */
#define REG_CAN0_IMR     (*(RoReg*)0x400B400CU) /**< \brief (CAN0) Interrupt Mask Register */
#define REG_CAN0_SR      (*(RoReg*)0x400B4010U) /**< \brief (CAN0) Status Register */
#define REG_CAN0_BR      (*(RwReg*)0x400B4014U) /**< \brief (CAN0) Baudrate Register */
#define REG_CAN0_TIM     (*(RoReg*)0x400B4018U) /**< \brief (CAN0) Timer Register */
#define REG_CAN0_TIMESTP (*(RoReg*)0x400B401CU) /**< \brief (CAN0) Timestamp Register */
#define REG_CAN0_ECR     (*(RoReg*)0x400B4020U) /**< \brief (CAN0) Error Counter Register */
#define REG_CAN0_TCR     (*(WoReg*)0x400B4024U) /**< \brief (CAN0) Transfer Command Register */
#define REG_CAN0_ACR     (*(WoReg*)0x400B4028U) /**< \brief (CAN0) Abort Command Register */
#define REG_CAN0_WPMR    (*(RwReg*)0x400B40E4U) /**< \brief (CAN0) Write Protect Mode Register */
#define REG_CAN0_WPSR    (*(RoReg*)0x400B40E8U) /**< \brief (CAN0) Write Protect Status Register */
#define REG_CAN0_MMR0    (*(RwReg*)0x400B4200U) /**< \brief (CAN0) Mailbox Mode Register (MB = 0) */
#define REG_CAN0_MAM0    (*(RwReg*)0x400B4204U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 0) */
#define REG_CAN0_MID0    (*(RwReg*)0x400B4208U) /**< \brief (CAN0) Mailbox ID Register (MB = 0) */
#define REG_CAN0_MFID0   (*(RoReg*)0x400B420CU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 0) */
#define REG_CAN0_MSR0    (*(RoReg*)0x400B4210U) /**< \brief (CAN0) Mailbox Status Register (MB = 0) */
#define REG_CAN0_MDL0    (*(RwReg*)0x400B4214U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 0) */
#define REG_CAN0_MDH0    (*(RwReg*)0x400B4218U) /**< \brief (CAN0) Mailbox Data High Register (MB = 0) */
#define REG_CAN0_MCR0    (*(WoReg*)0x400B421CU) /**< \brief (CAN0) Mailbox Control Register (MB = 0) */
#define REG_CAN0_MMR1    (*(RwReg*)0x400B4220U) /**< \brief (CAN0) Mailbox Mode Register (MB = 1) */
#define REG_CAN0_MAM1    (*(RwReg*)0x400B4224U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 1) */
#define REG_CAN0_MID1    (*(RwReg*)0x400B4228U) /**< \brief (CAN0) Mailbox ID Register (MB = 1) */
#define REG_CAN0_MFID1   (*(RoReg*)0x400B422CU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 1) */
#define REG_CAN0_MSR1    (*(RoReg*)0x400B4230U) /**< \brief (CAN0) Mailbox Status Register (MB = 1) */
#define REG_CAN0_MDL1    (*(RwReg*)0x400B4234U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 1) */
#define REG_CAN0_MDH1    (*(RwReg*)0x400B4238U) /**< \brief (CAN0) Mailbox Data High Register (MB = 1) */
#define REG_CAN0_MCR1    (*(WoReg*)0x400B423CU) /**< \brief (CAN0) Mailbox Control Register (MB = 1) */
#define REG_CAN0_MMR2    (*(RwReg*)0x400B4240U) /**< \brief (CAN0) Mailbox Mode Register (MB = 2) */
#define REG_CAN0_MAM2    (*(RwReg*)0x400B4244U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 2) */
#define REG_CAN0_MID2    (*(RwReg*)0x400B4248U) /**< \brief (CAN0) Mailbox ID Register (MB = 2) */
#define REG_CAN0_MFID2   (*(RoReg*)0x400B424CU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 2) */
#define REG_CAN0_MSR2    (*(RoReg*)0x400B4250U) /**< \brief (CAN0) Mailbox Status Register (MB = 2) */
#define REG_CAN0_MDL2    (*(RwReg*)0x400B4254U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 2) */
#define REG_CAN0_MDH2    (*(RwReg*)0x400B4258U) /**< \brief (CAN0) Mailbox Data High Register (MB = 2) */
#define REG_CAN0_MCR2    (*(WoReg*)0x400B425CU) /**< \brief (CAN0) Mailbox Control Register (MB = 2) */
#define REG_CAN0_MMR3    (*(RwReg*)0x400B4260U) /**< \brief (CAN0) Mailbox Mode Register (MB = 3) */
#define REG_CAN0_MAM3    (*(RwReg*)0x400B4264U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 3) */
#define REG_CAN0_MID3    (*(RwReg*)0x400B4268U) /**< \brief (CAN0) Mailbox ID Register (MB = 3) */
#define REG_CAN0_MFID3   (*(RoReg*)0x400B426CU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 3) */
#define REG_CAN0_MSR3    (*(RoReg*)0x400B4270U) /**< \brief (CAN0) Mailbox Status Register (MB = 3) */
#define REG_CAN0_MDL3    (*(RwReg*)0x400B4274U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 3) */
#define REG_CAN0_MDH3    (*(RwReg*)0x400B4278U) /**< \brief (CAN0) Mailbox Data High Register (MB = 3) */
#define REG_CAN0_MCR3    (*(WoReg*)0x400B427CU) /**< \brief (CAN0) Mailbox Control Register (MB = 3) */
#define REG_CAN0_MMR4    (*(RwReg*)0x400B4280U) /**< \brief (CAN0) Mailbox Mode Register (MB = 4) */
#define REG_CAN0_MAM4    (*(RwReg*)0x400B4284U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 4) */
#define REG_CAN0_MID4    (*(RwReg*)0x400B4288U) /**< \brief (CAN0) Mailbox ID Register (MB = 4) */
#define REG_CAN0_MFID4   (*(RoReg*)0x400B428CU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 4) */
#define REG_CAN0_MSR4    (*(RoReg*)0x400B4290U) /**< \brief (CAN0) Mailbox Status Register (MB = 4) */
#define REG_CAN0_MDL4    (*(RwReg*)0x400B4294U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 4) */
#define REG_CAN0_MDH4    (*(RwReg*)0x400B4298U) /**< \brief (CAN0) Mailbox Data High Register (MB = 4) */
#define REG_CAN0_MCR4    (*(WoReg*)0x400B429CU) /**< \brief (CAN0) Mailbox Control Register (MB = 4) */
#define REG_CAN0_MMR5    (*(RwReg*)0x400B42A0U) /**< \brief (CAN0) Mailbox Mode Register (MB = 5) */
#define REG_CAN0_MAM5    (*(RwReg*)0x400B42A4U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 5) */
#define REG_CAN0_MID5    (*(RwReg*)0x400B42A8U) /**< \brief (CAN0) Mailbox ID Register (MB = 5) */
#define REG_CAN0_MFID5   (*(RoReg*)0x400B42ACU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 5) */
#define REG_CAN0_MSR5    (*(RoReg*)0x400B42B0U) /**< \brief (CAN0) Mailbox Status Register (MB = 5) */
#define REG_CAN0_MDL5    (*(RwReg*)0x400B42B4U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 5) */
#define REG_CAN0_MDH5    (*(RwReg*)0x400B42B8U) /**< \brief (CAN0) Mailbox Data High Register (MB = 5) */
#define REG_CAN0_MCR5    (*(WoReg*)0x400B42BCU) /**< \brief (CAN0) Mailbox Control Register (MB = 5) */
#define REG_CAN0_MMR6    (*(RwReg*)0x400B42C0U) /**< \brief (CAN0) Mailbox Mode Register (MB = 6) */
#define REG_CAN0_MAM6    (*(RwReg*)0x400B42C4U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 6) */
#define REG_CAN0_MID6    (*(RwReg*)0x400B42C8U) /**< \brief (CAN0) Mailbox ID Register (MB = 6) */
#define REG_CAN0_MFID6   (*(RoReg*)0x400B42CCU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 6) */
#define REG_CAN0_MSR6    (*(RoReg*)0x400B42D0U) /**< \brief (CAN0) Mailbox Status Register (MB = 6) */
#define REG_CAN0_MDL6    (*(RwReg*)0x400B42D4U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 6) */
#define REG_CAN0_MDH6    (*(RwReg*)0x400B42D8U) /**< \brief (CAN0) Mailbox Data High Register (MB = 6) */
#define REG_CAN0_MCR6    (*(WoReg*)0x400B42DCU) /**< \brief (CAN0) Mailbox Control Register (MB = 6) */
#define REG_CAN0_MMR7    (*(RwReg*)0x400B42E0U) /**< \brief (CAN0) Mailbox Mode Register (MB = 7) */
#define REG_CAN0_MAM7    (*(RwReg*)0x400B42E4U) /**< \brief (CAN0) Mailbox Acceptance Mask Register (MB = 7) */
#define REG_CAN0_MID7    (*(RwReg*)0x400B42E8U) /**< \brief (CAN0) Mailbox ID Register (MB = 7) */
#define REG_CAN0_MFID7   (*(RoReg*)0x400B42ECU) /**< \brief (CAN0) Mailbox Family ID Register (MB = 7) */
#define REG_CAN0_MSR7    (*(RoReg*)0x400B42F0U) /**< \brief (CAN0) Mailbox Status Register (MB = 7) */
#define REG_CAN0_MDL7    (*(RwReg*)0x400B42F4U) /**< \brief (CAN0) Mailbox Data Low Register (MB = 7) */
#define REG_CAN0_MDH7    (*(RwReg*)0x400B42F8U) /**< \brief (CAN0) Mailbox Data High Register (MB = 7) */
#define REG_CAN0_MCR7    (*(WoReg*)0x400B42FCU) /**< \brief (CAN0) Mailbox Control Register (MB = 7) */

#define REG_CAN1_MR      (*(RwReg*)0x400B8000U) /**< \brief (CAN1) Mode Register */
#define REG_CAN1_IER     (*(WoReg*)0x400B8004U) /**< \brief (CAN1) Interrupt Enable Register */
#define REG_CAN1_IDR     (*(WoReg*)0x400B8008U) /**< \brief (CAN1) Interrupt Disable Register */
#define REG_CAN1_IMR     (*(RoReg*)0x400B800CU) /**< \brief (CAN1) Interrupt Mask Register */
#define REG_CAN1_SR      (*(RoReg*)0x400B8010U) /**< \brief (CAN1) Status Register */
#define REG_CAN1_BR      (*(RwReg*)0x400B8014U) /**< \brief (CAN1) Baudrate Register */
#define REG_CAN1_TIM     (*(RoReg*)0x400B8018U) /**< \brief (CAN1) Timer Register */
#define REG_CAN1_TIMESTP (*(RoReg*)0x400B801CU) /**< \brief (CAN1) Timestamp Register */
#define REG_CAN1_ECR     (*(RoReg*)0x400B8020U) /**< \brief (CAN1) Error Counter Register */
#define REG_CAN1_TCR     (*(WoReg*)0x400B8024U) /**< \brief (CAN1) Transfer Command Register */
#define REG_CAN1_ACR     (*(WoReg*)0x400B8028U) /**< \brief (CAN1) Abort Command Register */
#define REG_CAN1_WPMR    (*(RwReg*)0x400B80E4U) /**< \brief (CAN1) Write Protect Mode Register */
#define REG_CAN1_WPSR    (*(RoReg*)0x400B80E8U) /**< \brief (CAN1) Write Protect Status Register */
#define REG_CAN1_MMR0    (*(RwReg*)0x400B8200U) /**< \brief (CAN1) Mailbox Mode Register (MB = 0) */
#define REG_CAN1_MAM0    (*(RwReg*)0x400B8204U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 0) */
#define REG_CAN1_MID0    (*(RwReg*)0x400B8208U) /**< \brief (CAN1) Mailbox ID Register (MB = 0) */
#define REG_CAN1_MFID0   (*(RoReg*)0x400B820CU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 0) */
#define REG_CAN1_MSR0    (*(RoReg*)0x400B8210U) /**< \brief (CAN1) Mailbox Status Register (MB = 0) */
#define REG_CAN1_MDL0    (*(RwReg*)0x400B8214U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 0) */
#define REG_CAN1_MDH0    (*(RwReg*)0x400B8218U) /**< \brief (CAN1) Mailbox Data High Register (MB = 0) */
#define REG_CAN1_MCR0    (*(WoReg*)0x400B821CU) /**< \brief (CAN1) Mailbox Control Register (MB = 0) */
#define REG_CAN1_MMR1    (*(RwReg*)0x400B8220U) /**< \brief (CAN1) Mailbox Mode Register (MB = 1) */
#define REG_CAN1_MAM1    (*(RwReg*)0x400B8224U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 1) */
#define REG_CAN1_MID1    (*(RwReg*)0x400B8228U) /**< \brief (CAN1) Mailbox ID Register (MB = 1) */
#define REG_CAN1_MFID1   (*(RoReg*)0x400B822CU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 1) */
#define REG_CAN1_MSR1    (*(RoReg*)0x400B8230U) /**< \brief (CAN1) Mailbox Status Register (MB = 1) */
#define REG_CAN1_MDL1    (*(RwReg*)0x400B8234U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 1) */
#define REG_CAN1_MDH1    (*(RwReg*)0x400B8238U) /**< \brief (CAN1) Mailbox Data High Register (MB = 1) */
#define REG_CAN1_MCR1    (*(WoReg*)0x400B823CU) /**< \brief (CAN1) Mailbox Control Register (MB = 1) */
#define REG_CAN1_MMR2    (*(RwReg*)0x400B8240U) /**< \brief (CAN1) Mailbox Mode Register (MB = 2) */
#define REG_CAN1_MAM2    (*(RwReg*)0x400B8244U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 2) */
#define REG_CAN1_MID2    (*(RwReg*)0x400B8248U) /**< \brief (CAN1) Mailbox ID Register (MB = 2) */
#define REG_CAN1_MFID2   (*(RoReg*)0x400B824CU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 2) */
#define REG_CAN1_MSR2    (*(RoReg*)0x400B8250U) /**< \brief (CAN1) Mailbox Status Register (MB = 2) */
#define REG_CAN1_MDL2    (*(RwReg*)0x400B8254U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 2) */
#define REG_CAN1_MDH2    (*(RwReg*)0x400B8258U) /**< \brief (CAN1) Mailbox Data High Register (MB = 2) */
#define REG_CAN1_MCR2    (*(WoReg*)0x400B825CU) /**< \brief (CAN1) Mailbox Control Register (MB = 2) */
#define REG_CAN1_MMR3    (*(RwReg*)0x400B8260U) /**< \brief (CAN1) Mailbox Mode Register (MB = 3) */
#define REG_CAN1_MAM3    (*(RwReg*)0x400B8264U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 3) */
#define REG_CAN1_MID3    (*(RwReg*)0x400B8268U) /**< \brief (CAN1) Mailbox ID Register (MB = 3) */
#define REG_CAN1_MFID3   (*(RoReg*)0x400B826CU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 3) */
#define REG_CAN1_MSR3    (*(RoReg*)0x400B8270U) /**< \brief (CAN1) Mailbox Status Register (MB = 3) */
#define REG_CAN1_MDL3    (*(RwReg*)0x400B8274U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 3) */
#define REG_CAN1_MDH3    (*(RwReg*)0x400B8278U) /**< \brief (CAN1) Mailbox Data High Register (MB = 3) */
#define REG_CAN1_MCR3    (*(WoReg*)0x400B827CU) /**< \brief (CAN1) Mailbox Control Register (MB = 3) */
#define REG_CAN1_MMR4    (*(RwReg*)0x400B8280U) /**< \brief (CAN1) Mailbox Mode Register (MB = 4) */
#define REG_CAN1_MAM4    (*(RwReg*)0x400B8284U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 4) */
#define REG_CAN1_MID4    (*(RwReg*)0x400B8288U) /**< \brief (CAN1) Mailbox ID Register (MB = 4) */
#define REG_CAN1_MFID4   (*(RoReg*)0x400B828CU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 4) */
#define REG_CAN1_MSR4    (*(RoReg*)0x400B8290U) /**< \brief (CAN1) Mailbox Status Register (MB = 4) */
#define REG_CAN1_MDL4    (*(RwReg*)0x400B8294U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 4) */
#define REG_CAN1_MDH4    (*(RwReg*)0x400B8298U) /**< \brief (CAN1) Mailbox Data High Register (MB = 4) */
#define REG_CAN1_MCR4    (*(WoReg*)0x400B829CU) /**< \brief (CAN1) Mailbox Control Register (MB = 4) */
#define REG_CAN1_MMR5    (*(RwReg*)0x400B82A0U) /**< \brief (CAN1) Mailbox Mode Register (MB = 5) */
#define REG_CAN1_MAM5    (*(RwReg*)0x400B82A4U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 5) */
#define REG_CAN1_MID5    (*(RwReg*)0x400B82A8U) /**< \brief (CAN1) Mailbox ID Register (MB = 5) */
#define REG_CAN1_MFID5   (*(RoReg*)0x400B82ACU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 5) */
#define REG_CAN1_MSR5    (*(RoReg*)0x400B82B0U) /**< \brief (CAN1) Mailbox Status Register (MB = 5) */
#define REG_CAN1_MDL5    (*(RwReg*)0x400B82B4U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 5) */
#define REG_CAN1_MDH5    (*(RwReg*)0x400B82B8U) /**< \brief (CAN1) Mailbox Data High Register (MB = 5) */
#define REG_CAN1_MCR5    (*(WoReg*)0x400B82BCU) /**< \brief (CAN1) Mailbox Control Register (MB = 5) */
#define REG_CAN1_MMR6    (*(RwReg*)0x400B82C0U) /**< \brief (CAN1) Mailbox Mode Register (MB = 6) */
#define REG_CAN1_MAM6    (*(RwReg*)0x400B82C4U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 6) */
#define REG_CAN1_MID6    (*(RwReg*)0x400B82C8U) /**< \brief (CAN1) Mailbox ID Register (MB = 6) */
#define REG_CAN1_MFID6   (*(RoReg*)0x400B82CCU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 6) */
#define REG_CAN1_MSR6    (*(RoReg*)0x400B82D0U) /**< \brief (CAN1) Mailbox Status Register (MB = 6) */
#define REG_CAN1_MDL6    (*(RwReg*)0x400B82D4U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 6) */
#define REG_CAN1_MDH6    (*(RwReg*)0x400B82D8U) /**< \brief (CAN1) Mailbox Data High Register (MB = 6) */
#define REG_CAN1_MCR6    (*(WoReg*)0x400B82DCU) /**< \brief (CAN1) Mailbox Control Register (MB = 6) */
#define REG_CAN1_MMR7    (*(RwReg*)0x400B82E0U) /**< \brief (CAN1) Mailbox Mode Register (MB = 7) */
#define REG_CAN1_MAM7    (*(RwReg*)0x400B82E4U) /**< \brief (CAN1) Mailbox Acceptance Mask Register (MB = 7) */
#define REG_CAN1_MID7    (*(RwReg*)0x400B82E8U) /**< \brief (CAN1) Mailbox ID Register (MB = 7) */
#define REG_CAN1_MFID7   (*(RoReg*)0x400B82ECU) /**< \brief (CAN1) Mailbox Family ID Register (MB = 7) */
#define REG_CAN1_MSR7    (*(RoReg*)0x400B82F0U) /**< \brief (CAN1) Mailbox Status Register (MB = 7) */
#define REG_CAN1_MDL7    (*(RwReg*)0x400B82F4U) /**< \brief (CAN1) Mailbox Data Low Register (MB = 7) */
#define REG_CAN1_MDH7    (*(RwReg*)0x400B82F8U) /**< \brief (CAN1) Mailbox Data High Register (MB = 7) */
#define REG_CAN1_MCR7    (*(WoReg*)0x400B82FCU) /**< \brief (CAN1) Mailbox Control Register (MB = 7) */

#define REG_CHIPID_CIDR (*(RoReg*)0x400E0940U) /**< \brief (CHIPID) Chip ID Register */
#define REG_CHIPID_EXID (*(RoReg*)0x400E0944U) /**< \brief (CHIPID) Chip ID Extension Register */

#define REG_DACC_CR   (*(WoReg*)0x400C8000U) /**< \brief (DACC) Control Register */
#define REG_DACC_MR   (*(RwReg*)0x400C8004U) /**< \brief (DACC) Mode Register */
#define REG_DACC_CHER (*(WoReg*)0x400C8010U) /**< \brief (DACC) Channel Enable Register */
#define REG_DACC_CHDR (*(WoReg*)0x400C8014U) /**< \brief (DACC) Channel Disable Register */
#define REG_DACC_CHSR (*(RoReg*)0x400C8018U) /**< \brief (DACC) Channel Status Register */
#define REG_DACC_CDR  (*(WoReg*)0x400C8020U) /**< \brief (DACC) Conversion Data Register */
#define REG_DACC_IER  (*(WoReg*)0x400C8024U) /**< \brief (DACC) Interrupt Enable Register */
#define REG_DACC_IDR  (*(WoReg*)0x400C8028U) /**< \brief (DACC) Interrupt Disable Register */
#define REG_DACC_IMR  (*(RoReg*)0x400C802CU) /**< \brief (DACC) Interrupt Mask Register */
#define REG_DACC_ISR  (*(RoReg*)0x400C8030U) /**< \brief (DACC) Interrupt Status Register */
#define REG_DACC_ACR  (*(RwReg*)0x400C8094U) /**< \brief (DACC) Analog Current Register */
#define REG_DACC_WPMR (*(RwReg*)0x400C80E4U) /**< \brief (DACC) Write Protect Mode register */
#define REG_DACC_WPSR (*(RoReg*)0x400C80E8U) /**< \brief (DACC) Write Protect Status register */
#define REG_DACC_TPR  (*(RwReg*)0x400C8108U) /**< \brief (DACC) Transmit Pointer Register */
#define REG_DACC_TCR  (*(RwReg*)0x400C810CU) /**< \brief (DACC) Transmit Counter Register */
#define REG_DACC_TNPR (*(RwReg*)0x400C8118U) /**< \brief (DACC) Transmit Next Pointer Register */
#define REG_DACC_TNCR (*(RwReg*)0x400C811CU) /**< \brief (DACC) Transmit Next Counter Register */
#define REG_DACC_PTCR (*(WoReg*)0x400C8120U) /**< \brief (DACC) Transfer Control Register */
#define REG_DACC_PTSR (*(RoReg*)0x400C8124U) /**< \brief (DACC) Transfer Status Register */

#define REG_DMAC_GCFG   (*(RwReg*)0x400C4000U) /**< \brief (DMAC) DMAC Global Configuration Register */
#define REG_DMAC_EN     (*(RwReg*)0x400C4004U) /**< \brief (DMAC) DMAC Enable Register */
#define REG_DMAC_SREQ   (*(RwReg*)0x400C4008U) /**< \brief (DMAC) DMAC Software Single Request Register */
#define REG_DMAC_CREQ   (*(RwReg*)0x400C400CU) /**< \brief (DMAC) DMAC Software Chunk Transfer Request Register */
#define REG_DMAC_LAST   (*(RwReg*)0x400C4010U) /**< \brief (DMAC) DMAC Software Last Transfer Flag Register */
#define REG_DMAC_EBCIER (*(WoReg*)0x400C4018U) /**< \brief (DMAC) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Enable register. */
#define REG_DMAC_EBCIDR (*(WoReg*)0x400C401CU) /**< \brief (DMAC) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer Transfer Completed Interrupt Disable register. */
#define REG_DMAC_EBCIMR (*(RoReg*)0x400C4020U) /**< \brief (DMAC) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Mask Register. */
#define REG_DMAC_EBCISR (*(RoReg*)0x400C4024U) /**< \brief (DMAC) DMAC Error, Chained Buffer Transfer Completed Interrupt and Buffer transfer completed Status Register. */
#define REG_DMAC_CHER   (*(WoReg*)0x400C4028U) /**< \brief (DMAC) DMAC Channel Handler Enable Register */
#define REG_DMAC_CHDR   (*(WoReg*)0x400C402CU) /**< \brief (DMAC) DMAC Channel Handler Disable Register */
#define REG_DMAC_CHSR   (*(RoReg*)0x400C4030U) /**< \brief (DMAC) DMAC Channel Handler Status Register */
#define REG_DMAC_SADDR0 (*(RwReg*)0x400C403CU) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 0) */
#define REG_DMAC_DADDR0 (*(RwReg*)0x400C4040U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 0) */
#define REG_DMAC_DSCR0  (*(RwReg*)0x400C4044U) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 0) */
#define REG_DMAC_CTRLA0 (*(RwReg*)0x400C4048U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 0) */
#define REG_DMAC_CTRLB0 (*(RwReg*)0x400C404CU) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 0) */
#define REG_DMAC_CFG0   (*(RwReg*)0x400C4050U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 0) */
#define REG_DMAC_SADDR1 (*(RwReg*)0x400C4064U) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 1) */
#define REG_DMAC_DADDR1 (*(RwReg*)0x400C4068U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 1) */
#define REG_DMAC_DSCR1  (*(RwReg*)0x400C406CU) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 1) */
#define REG_DMAC_CTRLA1 (*(RwReg*)0x400C4070U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 1) */
#define REG_DMAC_CTRLB1 (*(RwReg*)0x400C4074U) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 1) */
#define REG_DMAC_CFG1   (*(RwReg*)0x400C4078U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 1) */
#define REG_DMAC_SADDR2 (*(RwReg*)0x400C408CU) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 2) */
#define REG_DMAC_DADDR2 (*(RwReg*)0x400C4090U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 2) */
#define REG_DMAC_DSCR2  (*(RwReg*)0x400C4094U) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 2) */
#define REG_DMAC_CTRLA2 (*(RwReg*)0x400C4098U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 2) */
#define REG_DMAC_CTRLB2 (*(RwReg*)0x400C409CU) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 2) */
#define REG_DMAC_CFG2   (*(RwReg*)0x400C40A0U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 2) */
#define REG_DMAC_SADDR3 (*(RwReg*)0x400C40B4U) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 3) */
#define REG_DMAC_DADDR3 (*(RwReg*)0x400C40B8U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 3) */
#define REG_DMAC_DSCR3  (*(RwReg*)0x400C40BCU) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 3) */
#define REG_DMAC_CTRLA3 (*(RwReg*)0x400C40C0U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 3) */
#define REG_DMAC_CTRLB3 (*(RwReg*)0x400C40C4U) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 3) */
#define REG_DMAC_CFG3   (*(RwReg*)0x400C40C8U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 3) */
#define REG_DMAC_SADDR4 (*(RwReg*)0x400C40DCU) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 4) */
#define REG_DMAC_DADDR4 (*(RwReg*)0x400C40E0U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 4) */
#define REG_DMAC_DSCR4  (*(RwReg*)0x400C40E4U) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 4) */
#define REG_DMAC_CTRLA4 (*(RwReg*)0x400C40E8U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 4) */
#define REG_DMAC_CTRLB4 (*(RwReg*)0x400C40ECU) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 4) */
#define REG_DMAC_CFG4   (*(RwReg*)0x400C40F0U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 4) */
#define REG_DMAC_SADDR5 (*(RwReg*)0x400C4104U) /**< \brief (DMAC) DMAC Channel Source Address Register (ch_num = 5) */
#define REG_DMAC_DADDR5 (*(RwReg*)0x400C4108U) /**< \brief (DMAC) DMAC Channel Destination Address Register (ch_num = 5) */
#define REG_DMAC_DSCR5  (*(RwReg*)0x400C410CU) /**< \brief (DMAC) DMAC Channel Descriptor Address Register (ch_num = 5) */
#define REG_DMAC_CTRLA5 (*(RwReg*)0x400C4110U) /**< \brief (DMAC) DMAC Channel Control A Register (ch_num = 5) */
#define REG_DMAC_CTRLB5 (*(RwReg*)0x400C4114U) /**< \brief (DMAC) DMAC Channel Control B Register (ch_num = 5) */
#define REG_DMAC_CFG5   (*(RwReg*)0x400C4118U) /**< \brief (DMAC) DMAC Channel Configuration Register (ch_num = 5) */
#define REG_DMAC_WPMR   (*(RwReg*)0x400C41E4U) /**< \brief (DMAC) DMAC Write Protect Mode Register */
#define REG_DMAC_WPSR   (*(RoReg*)0x400C41E8U) /**< \brief (DMAC) DMAC Write Protect Status Register */

#define REG_EFC0_FMR  (*(RwReg*)0x400E0A00U) /**< \brief (EFC0) EEFC Flash Mode Register */
#define REG_EFC0_FCR  (*(WoReg*)0x400E0A04U) /**< \brief (EFC0) EEFC Flash Command Register */
#define REG_EFC0_FSR  (*(RoReg*)0x400E0A08U) /**< \brief (EFC0) EEFC Flash Status Register */
#define REG_EFC0_FRR  (*(RoReg*)0x400E0A0CU) /**< \brief (EFC0) EEFC Flash Result Register */

#define REG_EFC1_FMR  (*(RwReg*)0x400E0C00U) /**< \brief (EFC1) EEFC Flash Mode Register */
#define REG_EFC1_FCR  (*(WoReg*)0x400E0C04U) /**< \brief (EFC1) EEFC Flash Command Register */
#define REG_EFC1_FSR  (*(RoReg*)0x400E0C08U) /**< \brief (EFC1) EEFC Flash Status Register */
#define REG_EFC1_FRR  (*(RoReg*)0x400E0C0CU) /**< \brief (EFC1) EEFC Flash Result Register */

#define REG_EMAC_NCR   (*(RwReg*)0x400B0000U) /**< \brief (EMAC) Network Control Register */
#define REG_EMAC_NCFGR (*(RwReg*)0x400B0004U) /**< \brief (EMAC) Network Configuration Register */
#define REG_EMAC_NSR   (*(RoReg*)0x400B0008U) /**< \brief (EMAC) Network Status Register */
#define REG_EMAC_TSR   (*(RwReg*)0x400B0014U) /**< \brief (EMAC) Transmit Status Register */
#define REG_EMAC_RBQP  (*(RwReg*)0x400B0018U) /**< \brief (EMAC) Receive Buffer Queue Pointer Register */
#define REG_EMAC_TBQP  (*(RwReg*)0x400B001CU) /**< \brief (EMAC) Transmit Buffer Queue Pointer Register */
#define REG_EMAC_RSR   (*(RwReg*)0x400B0020U) /**< \brief (EMAC) Receive Status Register */
#define REG_EMAC_ISR   (*(RwReg*)0x400B0024U) /**< \brief (EMAC) Interrupt Status Register */
#define REG_EMAC_IER   (*(WoReg*)0x400B0028U) /**< \brief (EMAC) Interrupt Enable Register */
#define REG_EMAC_IDR   (*(WoReg*)0x400B002CU) /**< \brief (EMAC) Interrupt Disable Register */
#define REG_EMAC_IMR   (*(RoReg*)0x400B0030U) /**< \brief (EMAC) Interrupt Mask Register */
#define REG_EMAC_MAN   (*(RwReg*)0x400B0034U) /**< \brief (EMAC) Phy Maintenance Register */
#define REG_EMAC_PTR   (*(RwReg*)0x400B0038U) /**< \brief (EMAC) Pause Time Register */
#define REG_EMAC_PFR   (*(RwReg*)0x400B003CU) /**< \brief (EMAC) Pause Frames Received Register */
#define REG_EMAC_FTO   (*(RwReg*)0x400B0040U) /**< \brief (EMAC) Frames Transmitted Ok Register */
#define REG_EMAC_SCF   (*(RwReg*)0x400B0044U) /**< \brief (EMAC) Single Collision Frames Register */
#define REG_EMAC_MCF   (*(RwReg*)0x400B0048U) /**< \brief (EMAC) Multiple Collision Frames Register */
#define REG_EMAC_FRO   (*(RwReg*)0x400B004CU) /**< \brief (EMAC) Frames Received Ok Register */
#define REG_EMAC_FCSE  (*(RwReg*)0x400B0050U) /**< \brief (EMAC) Frame Check Sequence Errors Register */
#define REG_EMAC_ALE   (*(RwReg*)0x400B0054U) /**< \brief (EMAC) Alignment Errors Register */
#define REG_EMAC_DTF   (*(RwReg*)0x400B0058U) /**< \brief (EMAC) Deferred Transmission Frames Register */
#define REG_EMAC_LCOL  (*(RwReg*)0x400B005CU) /**< \brief (EMAC) Late Collisions Register */
#define REG_EMAC_ECOL  (*(RwReg*)0x400B0060U) /**< \brief (EMAC) Excessive Collisions Register */
#define REG_EMAC_TUND  (*(RwReg*)0x400B0064U) /**< \brief (EMAC) Transmit Underrun Errors Register */
#define REG_EMAC_CSE   (*(RwReg*)0x400B0068U) /**< \brief (EMAC) Carrier Sense Errors Register */
#define REG_EMAC_RRE   (*(RwReg*)0x400B006CU) /**< \brief (EMAC) Receive Resource Errors Register */
#define REG_EMAC_ROV   (*(RwReg*)0x400B0070U) /**< \brief (EMAC) Receive Overrun Errors Register */
#define REG_EMAC_RSE   (*(RwReg*)0x400B0074U) /**< \brief (EMAC) Receive Symbol Errors Register */
#define REG_EMAC_ELE   (*(RwReg*)0x400B0078U) /**< \brief (EMAC) Excessive Length Errors Register */
#define REG_EMAC_RJA   (*(RwReg*)0x400B007CU) /**< \brief (EMAC) Receive Jabbers Register */
#define REG_EMAC_USF   (*(RwReg*)0x400B0080U) /**< \brief (EMAC) Undersize Frames Register */
#define REG_EMAC_STE   (*(RwReg*)0x400B0084U) /**< \brief (EMAC) SQE Test Errors Register */
#define REG_EMAC_RLE   (*(RwReg*)0x400B0088U) /**< \brief (EMAC) Received Length Field Mismatch Register */
#define REG_EMAC_HRB   (*(RwReg*)0x400B0090U) /**< \brief (EMAC) Hash Register Bottom [31:0] Register */
#define REG_EMAC_HRT   (*(RwReg*)0x400B0094U) /**< \brief (EMAC) Hash Register Top [63:32] Register */
#define REG_EMAC_SA1B  (*(RwReg*)0x400B0098U) /**< \brief (EMAC) Specific Address 1 Bottom Register */
#define REG_EMAC_SA1T  (*(RwReg*)0x400B009CU) /**< \brief (EMAC) Specific Address 1 Top Register */
#define REG_EMAC_SA2B  (*(RwReg*)0x400B00A0U) /**< \brief (EMAC) Specific Address 2 Bottom Register */
#define REG_EMAC_SA2T  (*(RwReg*)0x400B00A4U) /**< \brief (EMAC) Specific Address 2 Top Register */
#define REG_EMAC_SA3B  (*(RwReg*)0x400B00A8U) /**< \brief (EMAC) Specific Address 3 Bottom Register */
#define REG_EMAC_SA3T  (*(RwReg*)0x400B00ACU) /**< \brief (EMAC) Specific Address 3 Top Register */
#define REG_EMAC_SA4B  (*(RwReg*)0x400B00B0U) /**< \brief (EMAC) Specific Address 4 Bottom Register */
#define REG_EMAC_SA4T  (*(RwReg*)0x400B00B4U) /**< \brief (EMAC) Specific Address 4 Top Register */
#define REG_EMAC_TID   (*(RwReg*)0x400B00B8U) /**< \brief (EMAC) Type ID Checking Register */
#define REG_EMAC_USRIO (*(RwReg*)0x400B00C0U) /**< \brief (EMAC) User Input/Output Register */

#define REG_GPBR_GPBR   (*(RwReg*)0x400E1A90U) /**< \brief (GPBR) General Purpose Backup Register */

#define REG_HSMCI_CR        (*(WoReg*)0x40000000U) /**< \brief (HSMCI) Control Register */
#define REG_HSMCI_MR        (*(RwReg*)0x40000004U) /**< \brief (HSMCI) Mode Register */
#define REG_HSMCI_DTOR      (*(RwReg*)0x40000008U) /**< \brief (HSMCI) Data Timeout Register */
#define REG_HSMCI_SDCR      (*(RwReg*)0x4000000CU) /**< \brief (HSMCI) SD/SDIO Card Register */
#define REG_HSMCI_ARGR      (*(RwReg*)0x40000010U) /**< \brief (HSMCI) Argument Register */
#define REG_HSMCI_CMDR      (*(WoReg*)0x40000014U) /**< \brief (HSMCI) Command Register */
#define REG_HSMCI_BLKR      (*(RwReg*)0x40000018U) /**< \brief (HSMCI) Block Register */
#define REG_HSMCI_CSTOR     (*(RwReg*)0x4000001CU) /**< \brief (HSMCI) Completion Signal Timeout Register */
#define REG_HSMCI_RSPR      (*(RoReg*)0x40000020U) /**< \brief (HSMCI) Response Register */
#define REG_HSMCI_RDR       (*(RoReg*)0x40000030U) /**< \brief (HSMCI) Receive Data Register */
#define REG_HSMCI_TDR       (*(WoReg*)0x40000034U) /**< \brief (HSMCI) Transmit Data Register */
#define REG_HSMCI_SR        (*(RoReg*)0x40000040U) /**< \brief (HSMCI) Status Register */
#define REG_HSMCI_IER       (*(WoReg*)0x40000044U) /**< \brief (HSMCI) Interrupt Enable Register */
#define REG_HSMCI_IDR       (*(WoReg*)0x40000048U) /**< \brief (HSMCI) Interrupt Disable Register */
#define REG_HSMCI_IMR       (*(RoReg*)0x4000004CU) /**< \brief (HSMCI) Interrupt Mask Register */
#define REG_HSMCI_DMA       (*(RwReg*)0x40000050U) /**< \brief (HSMCI) DMA Configuration Register */
#define REG_HSMCI_CFG       (*(RwReg*)0x40000054U) /**< \brief (HSMCI) Configuration Register */
#define REG_HSMCI_WPMR      (*(RwReg*)0x400000E4U) /**< \brief (HSMCI) Write Protection Mode Register */
#define REG_HSMCI_WPSR      (*(RoReg*)0x400000E8U) /**< \brief (HSMCI) Write Protection Status Register */
#define REG_HSMCI_FIFO      (*(RwReg*)0x40000200U) /**< \brief (HSMCI) FIFO Memory Aperture0 */

#define REG_MATRIX_MCFG    (*(RwReg*)0x400E0400U) /**< \brief (MATRIX) Master Configuration Register */
#define REG_MATRIX_SCFG    (*(RwReg*)0x400E0440U) /**< \brief (MATRIX) Slave Configuration Register */
#define REG_MATRIX_PRAS0   (*(RwReg*)0x400E0480U) /**< \brief (MATRIX) Priority Register A for Slave 0 */
#define REG_MATRIX_PRAS1   (*(RwReg*)0x400E0488U) /**< \brief (MATRIX) Priority Register A for Slave 1 */
#define REG_MATRIX_PRAS2   (*(RwReg*)0x400E0490U) /**< \brief (MATRIX) Priority Register A for Slave 2 */
#define REG_MATRIX_PRAS3   (*(RwReg*)0x400E0498U) /**< \brief (MATRIX) Priority Register A for Slave 3 */
#define REG_MATRIX_PRAS4   (*(RwReg*)0x400E04A0U) /**< \brief (MATRIX) Priority Register A for Slave 4 */
#define REG_MATRIX_PRAS5   (*(RwReg*)0x400E04A8U) /**< \brief (MATRIX) Priority Register A for Slave 5 */
#define REG_MATRIX_PRAS6   (*(RwReg*)0x400E04B0U) /**< \brief (MATRIX) Priority Register A for Slave 6 */
#define REG_MATRIX_PRAS7   (*(RwReg*)0x400E04B8U) /**< \brief (MATRIX) Priority Register A for Slave 7 */
#define REG_MATRIX_PRAS8   (*(RwReg*)0x400E04C0U) /**< \brief (MATRIX) Priority Register A for Slave 8 */
#define REG_MATRIX_MRCR    (*(RwReg*)0x400E0500U) /**< \brief (MATRIX) Master Remap Control Register */
#define REG_CCFG_SYSIO     (*(RwReg*)0x400E0514U) /**< \brief (MATRIX) System I/O Configuration register */
#define REG_MATRIX_WPMR    (*(RwReg*)0x400E05E4U) /**< \brief (MATRIX) Write Protect Mode Register */
#define REG_MATRIX_WPSR    (*(RoReg*)0x400E05E8U) /**< \brief (MATRIX) Write Protect Status Register */

#define REG_PIOA_PER    (*(WoReg*)0x400E0E00U) /**< \brief (PIOA) PIO Enable Register */
#define REG_PIOA_PDR    (*(WoReg*)0x400E0E04U) /**< \brief (PIOA) PIO Disable Register */
#define REG_PIOA_PSR    (*(RoReg*)0x400E0E08U) /**< \brief (PIOA) PIO Status Register */
#define REG_PIOA_OER    (*(WoReg*)0x400E0E10U) /**< \brief (PIOA) Output Enable Register */
#define REG_PIOA_ODR    (*(WoReg*)0x400E0E14U) /**< \brief (PIOA) Output Disable Register */
#define REG_PIOA_OSR    (*(RoReg*)0x400E0E18U) /**< \brief (PIOA) Output Status Register */
#define REG_PIOA_IFER   (*(WoReg*)0x400E0E20U) /**< \brief (PIOA) Glitch Input Filter Enable Register */
#define REG_PIOA_IFDR   (*(WoReg*)0x400E0E24U) /**< \brief (PIOA) Glitch Input Filter Disable Register */
#define REG_PIOA_IFSR   (*(RoReg*)0x400E0E28U) /**< \brief (PIOA) Glitch Input Filter Status Register */
#define REG_PIOA_SODR   (*(WoReg*)0x400E0E30U) /**< \brief (PIOA) Set Output Data Register */
#define REG_PIOA_CODR   (*(WoReg*)0x400E0E34U) /**< \brief (PIOA) Clear Output Data Register */
#define REG_PIOA_ODSR   (*(RwReg*)0x400E0E38U) /**< \brief (PIOA) Output Data Status Register */
#define REG_PIOA_PDSR   (*(RoReg*)0x400E0E3CU) /**< \brief (PIOA) Pin Data Status Register */
#define REG_PIOA_IER    (*(WoReg*)0x400E0E40U) /**< \brief (PIOA) Interrupt Enable Register */
#define REG_PIOA_IDR    (*(WoReg*)0x400E0E44U) /**< \brief (PIOA) Interrupt Disable Register */
#define REG_PIOA_IMR    (*(RoReg*)0x400E0E48U) /**< \brief (PIOA) Interrupt Mask Register */
#define REG_PIOA_ISR    (*(RoReg*)0x400E0E4CU) /**< \brief (PIOA) Interrupt Status Register */
#define REG_PIOA_MDER   (*(WoReg*)0x400E0E50U) /**< \brief (PIOA) Multi-driver Enable Register */
#define REG_PIOA_MDDR   (*(WoReg*)0x400E0E54U) /**< \brief (PIOA) Multi-driver Disable Register */
#define REG_PIOA_MDSR   (*(RoReg*)0x400E0E58U) /**< \brief (PIOA) Multi-driver Status Register */
#define REG_PIOA_PUDR   (*(WoReg*)0x400E0E60U) /**< \brief (PIOA) Pull-up Disable Register */
#define REG_PIOA_PUER   (*(WoReg*)0x400E0E64U) /**< \brief (PIOA) Pull-up Enable Register */
#define REG_PIOA_PUSR   (*(RoReg*)0x400E0E68U) /**< \brief (PIOA) Pad Pull-up Status Register */
#define REG_PIOA_ABSR   (*(RwReg*)0x400E0E70U) /**< \brief (PIOA) Peripheral AB Select Register */
#define REG_PIOA_SCIFSR (*(WoReg*)0x400E0E80U) /**< \brief (PIOA) System Clock Glitch Input Filter Select Register */
#define REG_PIOA_DIFSR  (*(WoReg*)0x400E0E84U) /**< \brief (PIOA) Debouncing Input Filter Select Register */
#define REG_PIOA_IFDGSR (*(RoReg*)0x400E0E88U) /**< \brief (PIOA) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOA_SCDR   (*(RwReg*)0x400E0E8CU) /**< \brief (PIOA) Slow Clock Divider Debouncing Register */
#define REG_PIOA_OWER   (*(WoReg*)0x400E0EA0U) /**< \brief (PIOA) Output Write Enable */
#define REG_PIOA_OWDR   (*(WoReg*)0x400E0EA4U) /**< \brief (PIOA) Output Write Disable */
#define REG_PIOA_OWSR   (*(RoReg*)0x400E0EA8U) /**< \brief (PIOA) Output Write Status Register */
#define REG_PIOA_AIMER  (*(WoReg*)0x400E0EB0U) /**< \brief (PIOA) Additional Interrupt Modes Enable Register */
#define REG_PIOA_AIMDR  (*(WoReg*)0x400E0EB4U) /**< \brief (PIOA) Additional Interrupt Modes Disables Register */
#define REG_PIOA_AIMMR  (*(RoReg*)0x400E0EB8U) /**< \brief (PIOA) Additional Interrupt Modes Mask Register */
#define REG_PIOA_ESR    (*(WoReg*)0x400E0EC0U) /**< \brief (PIOA) Edge Select Register */
#define REG_PIOA_LSR    (*(WoReg*)0x400E0EC4U) /**< \brief (PIOA) Level Select Register */
#define REG_PIOA_ELSR   (*(RoReg*)0x400E0EC8U) /**< \brief (PIOA) Edge/Level Status Register */
#define REG_PIOA_FELLSR (*(WoReg*)0x400E0ED0U) /**< \brief (PIOA) Falling Edge/Low Level Select Register */
#define REG_PIOA_REHLSR (*(WoReg*)0x400E0ED4U) /**< \brief (PIOA) Rising Edge/ High Level Select Register */
#define REG_PIOA_FRLHSR (*(RoReg*)0x400E0ED8U) /**< \brief (PIOA) Fall/Rise - Low/High Status Register */
#define REG_PIOA_LOCKSR (*(RoReg*)0x400E0EE0U) /**< \brief (PIOA) Lock Status */
#define REG_PIOA_WPMR   (*(RwReg*)0x400E0EE4U) /**< \brief (PIOA) Write Protect Mode Register */
#define REG_PIOA_WPSR   (*(RoReg*)0x400E0EE8U) /**< \brief (PIOA) Write Protect Status Register */

#define REG_PIOB_PER    (*(WoReg*)0x400E1000U) /**< \brief (PIOB) PIO Enable Register */
#define REG_PIOB_PDR    (*(WoReg*)0x400E1004U) /**< \brief (PIOB) PIO Disable Register */
#define REG_PIOB_PSR    (*(RoReg*)0x400E1008U) /**< \brief (PIOB) PIO Status Register */
#define REG_PIOB_OER    (*(WoReg*)0x400E1010U) /**< \brief (PIOB) Output Enable Register */
#define REG_PIOB_ODR    (*(WoReg*)0x400E1014U) /**< \brief (PIOB) Output Disable Register */
#define REG_PIOB_OSR    (*(RoReg*)0x400E1018U) /**< \brief (PIOB) Output Status Register */
#define REG_PIOB_IFER   (*(WoReg*)0x400E1020U) /**< \brief (PIOB) Glitch Input Filter Enable Register */
#define REG_PIOB_IFDR   (*(WoReg*)0x400E1024U) /**< \brief (PIOB) Glitch Input Filter Disable Register */
#define REG_PIOB_IFSR   (*(RoReg*)0x400E1028U) /**< \brief (PIOB) Glitch Input Filter Status Register */
#define REG_PIOB_SODR   (*(WoReg*)0x400E1030U) /**< \brief (PIOB) Set Output Data Register */
#define REG_PIOB_CODR   (*(WoReg*)0x400E1034U) /**< \brief (PIOB) Clear Output Data Register */
#define REG_PIOB_ODSR   (*(RwReg*)0x400E1038U) /**< \brief (PIOB) Output Data Status Register */
#define REG_PIOB_PDSR   (*(RoReg*)0x400E103CU) /**< \brief (PIOB) Pin Data Status Register */
#define REG_PIOB_IER    (*(WoReg*)0x400E1040U) /**< \brief (PIOB) Interrupt Enable Register */
#define REG_PIOB_IDR    (*(WoReg*)0x400E1044U) /**< \brief (PIOB) Interrupt Disable Register */
#define REG_PIOB_IMR    (*(RoReg*)0x400E1048U) /**< \brief (PIOB) Interrupt Mask Register */
#define REG_PIOB_ISR    (*(RoReg*)0x400E104CU) /**< \brief (PIOB) Interrupt Status Register */
#define REG_PIOB_MDER   (*(WoReg*)0x400E1050U) /**< \brief (PIOB) Multi-driver Enable Register */
#define REG_PIOB_MDDR   (*(WoReg*)0x400E1054U) /**< \brief (PIOB) Multi-driver Disable Register */
#define REG_PIOB_MDSR   (*(RoReg*)0x400E1058U) /**< \brief (PIOB) Multi-driver Status Register */
#define REG_PIOB_PUDR   (*(WoReg*)0x400E1060U) /**< \brief (PIOB) Pull-up Disable Register */
#define REG_PIOB_PUER   (*(WoReg*)0x400E1064U) /**< \brief (PIOB) Pull-up Enable Register */
#define REG_PIOB_PUSR   (*(RoReg*)0x400E1068U) /**< \brief (PIOB) Pad Pull-up Status Register */
#define REG_PIOB_ABSR   (*(RwReg*)0x400E1070U) /**< \brief (PIOB) Peripheral AB Select Register */
#define REG_PIOB_SCIFSR (*(WoReg*)0x400E1080U) /**< \brief (PIOB) System Clock Glitch Input Filter Select Register */
#define REG_PIOB_DIFSR  (*(WoReg*)0x400E1084U) /**< \brief (PIOB) Debouncing Input Filter Select Register */
#define REG_PIOB_IFDGSR (*(RoReg*)0x400E1088U) /**< \brief (PIOB) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOB_SCDR   (*(RwReg*)0x400E108CU) /**< \brief (PIOB) Slow Clock Divider Debouncing Register */
#define REG_PIOB_OWER   (*(WoReg*)0x400E10A0U) /**< \brief (PIOB) Output Write Enable */
#define REG_PIOB_OWDR   (*(WoReg*)0x400E10A4U) /**< \brief (PIOB) Output Write Disable */
#define REG_PIOB_OWSR   (*(RoReg*)0x400E10A8U) /**< \brief (PIOB) Output Write Status Register */
#define REG_PIOB_AIMER  (*(WoReg*)0x400E10B0U) /**< \brief (PIOB) Additional Interrupt Modes Enable Register */
#define REG_PIOB_AIMDR  (*(WoReg*)0x400E10B4U) /**< \brief (PIOB) Additional Interrupt Modes Disables Register */
#define REG_PIOB_AIMMR  (*(RoReg*)0x400E10B8U) /**< \brief (PIOB) Additional Interrupt Modes Mask Register */
#define REG_PIOB_ESR    (*(WoReg*)0x400E10C0U) /**< \brief (PIOB) Edge Select Register */
#define REG_PIOB_LSR    (*(WoReg*)0x400E10C4U) /**< \brief (PIOB) Level Select Register */
#define REG_PIOB_ELSR   (*(RoReg*)0x400E10C8U) /**< \brief (PIOB) Edge/Level Status Register */
#define REG_PIOB_FELLSR (*(WoReg*)0x400E10D0U) /**< \brief (PIOB) Falling Edge/Low Level Select Register */
#define REG_PIOB_REHLSR (*(WoReg*)0x400E10D4U) /**< \brief (PIOB) Rising Edge/ High Level Select Register */
#define REG_PIOB_FRLHSR (*(RoReg*)0x400E10D8U) /**< \brief (PIOB) Fall/Rise - Low/High Status Register */
#define REG_PIOB_LOCKSR (*(RoReg*)0x400E10E0U) /**< \brief (PIOB) Lock Status */
#define REG_PIOB_WPMR   (*(RwReg*)0x400E10E4U) /**< \brief (PIOB) Write Protect Mode Register */
#define REG_PIOB_WPSR   (*(RoReg*)0x400E10E8U) /**< \brief (PIOB) Write Protect Status Register */

#define REG_PIOC_PER    (*(WoReg*)0x400E1200U) /**< \brief (PIOC) PIO Enable Register */
#define REG_PIOC_PDR    (*(WoReg*)0x400E1204U) /**< \brief (PIOC) PIO Disable Register */
#define REG_PIOC_PSR    (*(RoReg*)0x400E1208U) /**< \brief (PIOC) PIO Status Register */
#define REG_PIOC_OER    (*(WoReg*)0x400E1210U) /**< \brief (PIOC) Output Enable Register */
#define REG_PIOC_ODR    (*(WoReg*)0x400E1214U) /**< \brief (PIOC) Output Disable Register */
#define REG_PIOC_OSR    (*(RoReg*)0x400E1218U) /**< \brief (PIOC) Output Status Register */
#define REG_PIOC_IFER   (*(WoReg*)0x400E1220U) /**< \brief (PIOC) Glitch Input Filter Enable Register */
#define REG_PIOC_IFDR   (*(WoReg*)0x400E1224U) /**< \brief (PIOC) Glitch Input Filter Disable Register */
#define REG_PIOC_IFSR   (*(RoReg*)0x400E1228U) /**< \brief (PIOC) Glitch Input Filter Status Register */
#define REG_PIOC_SODR   (*(WoReg*)0x400E1230U) /**< \brief (PIOC) Set Output Data Register */
#define REG_PIOC_CODR   (*(WoReg*)0x400E1234U) /**< \brief (PIOC) Clear Output Data Register */
#define REG_PIOC_ODSR   (*(RwReg*)0x400E1238U) /**< \brief (PIOC) Output Data Status Register */
#define REG_PIOC_PDSR   (*(RoReg*)0x400E123CU) /**< \brief (PIOC) Pin Data Status Register */
#define REG_PIOC_IER    (*(WoReg*)0x400E1240U) /**< \brief (PIOC) Interrupt Enable Register */
#define REG_PIOC_IDR    (*(WoReg*)0x400E1244U) /**< \brief (PIOC) Interrupt Disable Register */
#define REG_PIOC_IMR    (*(RoReg*)0x400E1248U) /**< \brief (PIOC) Interrupt Mask Register */
#define REG_PIOC_ISR    (*(RoReg*)0x400E124CU) /**< \brief (PIOC) Interrupt Status Register */
#define REG_PIOC_MDER   (*(WoReg*)0x400E1250U) /**< \brief (PIOC) Multi-driver Enable Register */
#define REG_PIOC_MDDR   (*(WoReg*)0x400E1254U) /**< \brief (PIOC) Multi-driver Disable Register */
#define REG_PIOC_MDSR   (*(RoReg*)0x400E1258U) /**< \brief (PIOC) Multi-driver Status Register */
#define REG_PIOC_PUDR   (*(WoReg*)0x400E1260U) /**< \brief (PIOC) Pull-up Disable Register */
#define REG_PIOC_PUER   (*(WoReg*)0x400E1264U) /**< \brief (PIOC) Pull-up Enable Register */
#define REG_PIOC_PUSR   (*(RoReg*)0x400E1268U) /**< \brief (PIOC) Pad Pull-up Status Register */
#define REG_PIOC_ABSR   (*(RwReg*)0x400E1270U) /**< \brief (PIOC) Peripheral AB Select Register */
#define REG_PIOC_SCIFSR (*(WoReg*)0x400E1280U) /**< \brief (PIOC) System Clock Glitch Input Filter Select Register */
#define REG_PIOC_DIFSR  (*(WoReg*)0x400E1284U) /**< \brief (PIOC) Debouncing Input Filter Select Register */
#define REG_PIOC_IFDGSR (*(RoReg*)0x400E1288U) /**< \brief (PIOC) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOC_SCDR   (*(RwReg*)0x400E128CU) /**< \brief (PIOC) Slow Clock Divider Debouncing Register */
#define REG_PIOC_OWER   (*(WoReg*)0x400E12A0U) /**< \brief (PIOC) Output Write Enable */
#define REG_PIOC_OWDR   (*(WoReg*)0x400E12A4U) /**< \brief (PIOC) Output Write Disable */
#define REG_PIOC_OWSR   (*(RoReg*)0x400E12A8U) /**< \brief (PIOC) Output Write Status Register */
#define REG_PIOC_AIMER  (*(WoReg*)0x400E12B0U) /**< \brief (PIOC) Additional Interrupt Modes Enable Register */
#define REG_PIOC_AIMDR  (*(WoReg*)0x400E12B4U) /**< \brief (PIOC) Additional Interrupt Modes Disables Register */
#define REG_PIOC_AIMMR  (*(RoReg*)0x400E12B8U) /**< \brief (PIOC) Additional Interrupt Modes Mask Register */
#define REG_PIOC_ESR    (*(WoReg*)0x400E12C0U) /**< \brief (PIOC) Edge Select Register */
#define REG_PIOC_LSR    (*(WoReg*)0x400E12C4U) /**< \brief (PIOC) Level Select Register */
#define REG_PIOC_ELSR   (*(RoReg*)0x400E12C8U) /**< \brief (PIOC) Edge/Level Status Register */
#define REG_PIOC_FELLSR (*(WoReg*)0x400E12D0U) /**< \brief (PIOC) Falling Edge/Low Level Select Register */
#define REG_PIOC_REHLSR (*(WoReg*)0x400E12D4U) /**< \brief (PIOC) Rising Edge/ High Level Select Register */
#define REG_PIOC_FRLHSR (*(RoReg*)0x400E12D8U) /**< \brief (PIOC) Fall/Rise - Low/High Status Register */
#define REG_PIOC_LOCKSR (*(RoReg*)0x400E12E0U) /**< \brief (PIOC) Lock Status */
#define REG_PIOC_WPMR   (*(RwReg*)0x400E12E4U) /**< \brief (PIOC) Write Protect Mode Register */
#define REG_PIOC_WPSR   (*(RoReg*)0x400E12E8U) /**< \brief (PIOC) Write Protect Status Register */

#define REG_PIOD_PER    (*(WoReg*)0x400E1400U) /**< \brief (PIOD) PIO Enable Register */
#define REG_PIOD_PDR    (*(WoReg*)0x400E1404U) /**< \brief (PIOD) PIO Disable Register */
#define REG_PIOD_PSR    (*(RoReg*)0x400E1408U) /**< \brief (PIOD) PIO Status Register */
#define REG_PIOD_OER    (*(WoReg*)0x400E1410U) /**< \brief (PIOD) Output Enable Register */
#define REG_PIOD_ODR    (*(WoReg*)0x400E1414U) /**< \brief (PIOD) Output Disable Register */
#define REG_PIOD_OSR    (*(RoReg*)0x400E1418U) /**< \brief (PIOD) Output Status Register */
#define REG_PIOD_IFER   (*(WoReg*)0x400E1420U) /**< \brief (PIOD) Glitch Input Filter Enable Register */
#define REG_PIOD_IFDR   (*(WoReg*)0x400E1424U) /**< \brief (PIOD) Glitch Input Filter Disable Register */
#define REG_PIOD_IFSR   (*(RoReg*)0x400E1428U) /**< \brief (PIOD) Glitch Input Filter Status Register */
#define REG_PIOD_SODR   (*(WoReg*)0x400E1430U) /**< \brief (PIOD) Set Output Data Register */
#define REG_PIOD_CODR   (*(WoReg*)0x400E1434U) /**< \brief (PIOD) Clear Output Data Register */
#define REG_PIOD_ODSR   (*(RwReg*)0x400E1438U) /**< \brief (PIOD) Output Data Status Register */
#define REG_PIOD_PDSR   (*(RoReg*)0x400E143CU) /**< \brief (PIOD) Pin Data Status Register */
#define REG_PIOD_IER    (*(WoReg*)0x400E1440U) /**< \brief (PIOD) Interrupt Enable Register */
#define REG_PIOD_IDR    (*(WoReg*)0x400E1444U) /**< \brief (PIOD) Interrupt Disable Register */
#define REG_PIOD_IMR    (*(RoReg*)0x400E1448U) /**< \brief (PIOD) Interrupt Mask Register */
#define REG_PIOD_ISR    (*(RoReg*)0x400E144CU) /**< \brief (PIOD) Interrupt Status Register */
#define REG_PIOD_MDER   (*(WoReg*)0x400E1450U) /**< \brief (PIOD) Multi-driver Enable Register */
#define REG_PIOD_MDDR   (*(WoReg*)0x400E1454U) /**< \brief (PIOD) Multi-driver Disable Register */
#define REG_PIOD_MDSR   (*(RoReg*)0x400E1458U) /**< \brief (PIOD) Multi-driver Status Register */
#define REG_PIOD_PUDR   (*(WoReg*)0x400E1460U) /**< \brief (PIOD) Pull-up Disable Register */
#define REG_PIOD_PUER   (*(WoReg*)0x400E1464U) /**< \brief (PIOD) Pull-up Enable Register */
#define REG_PIOD_PUSR   (*(RoReg*)0x400E1468U) /**< \brief (PIOD) Pad Pull-up Status Register */
#define REG_PIOD_ABSR   (*(RwReg*)0x400E1470U) /**< \brief (PIOD) Peripheral AB Select Register */
#define REG_PIOD_SCIFSR (*(WoReg*)0x400E1480U) /**< \brief (PIOD) System Clock Glitch Input Filter Select Register */
#define REG_PIOD_DIFSR  (*(WoReg*)0x400E1484U) /**< \brief (PIOD) Debouncing Input Filter Select Register */
#define REG_PIOD_IFDGSR (*(RoReg*)0x400E1488U) /**< \brief (PIOD) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOD_SCDR   (*(RwReg*)0x400E148CU) /**< \brief (PIOD) Slow Clock Divider Debouncing Register */
#define REG_PIOD_OWER   (*(WoReg*)0x400E14A0U) /**< \brief (PIOD) Output Write Enable */
#define REG_PIOD_OWDR   (*(WoReg*)0x400E14A4U) /**< \brief (PIOD) Output Write Disable */
#define REG_PIOD_OWSR   (*(RoReg*)0x400E14A8U) /**< \brief (PIOD) Output Write Status Register */
#define REG_PIOD_AIMER  (*(WoReg*)0x400E14B0U) /**< \brief (PIOD) Additional Interrupt Modes Enable Register */
#define REG_PIOD_AIMDR  (*(WoReg*)0x400E14B4U) /**< \brief (PIOD) Additional Interrupt Modes Disables Register */
#define REG_PIOD_AIMMR  (*(RoReg*)0x400E14B8U) /**< \brief (PIOD) Additional Interrupt Modes Mask Register */
#define REG_PIOD_ESR    (*(WoReg*)0x400E14C0U) /**< \brief (PIOD) Edge Select Register */
#define REG_PIOD_LSR    (*(WoReg*)0x400E14C4U) /**< \brief (PIOD) Level Select Register */
#define REG_PIOD_ELSR   (*(RoReg*)0x400E14C8U) /**< \brief (PIOD) Edge/Level Status Register */
#define REG_PIOD_FELLSR (*(WoReg*)0x400E14D0U) /**< \brief (PIOD) Falling Edge/Low Level Select Register */
#define REG_PIOD_REHLSR (*(WoReg*)0x400E14D4U) /**< \brief (PIOD) Rising Edge/ High Level Select Register */
#define REG_PIOD_FRLHSR (*(RoReg*)0x400E14D8U) /**< \brief (PIOD) Fall/Rise - Low/High Status Register */
#define REG_PIOD_LOCKSR (*(RoReg*)0x400E14E0U) /**< \brief (PIOD) Lock Status */
#define REG_PIOD_WPMR   (*(RwReg*)0x400E14E4U) /**< \brief (PIOD) Write Protect Mode Register */
#define REG_PIOD_WPSR   (*(RoReg*)0x400E14E8U) /**< \brief (PIOD) Write Protect Status Register */

#define REG_PIOE_PER    (*(WoReg*)0x400E1600U) /**< \brief (PIOE) PIO Enable Register */
#define REG_PIOE_PDR    (*(WoReg*)0x400E1604U) /**< \brief (PIOE) PIO Disable Register */
#define REG_PIOE_PSR    (*(RoReg*)0x400E1608U) /**< \brief (PIOE) PIO Status Register */
#define REG_PIOE_OER    (*(WoReg*)0x400E1610U) /**< \brief (PIOE) Output Enable Register */
#define REG_PIOE_ODR    (*(WoReg*)0x400E1614U) /**< \brief (PIOE) Output Disable Register */
#define REG_PIOE_OSR    (*(RoReg*)0x400E1618U) /**< \brief (PIOE) Output Status Register */
#define REG_PIOE_IFER   (*(WoReg*)0x400E1620U) /**< \brief (PIOE) Glitch Input Filter Enable Register */
#define REG_PIOE_IFDR   (*(WoReg*)0x400E1624U) /**< \brief (PIOE) Glitch Input Filter Disable Register */
#define REG_PIOE_IFSR   (*(RoReg*)0x400E1628U) /**< \brief (PIOE) Glitch Input Filter Status Register */
#define REG_PIOE_SODR   (*(WoReg*)0x400E1630U) /**< \brief (PIOE) Set Output Data Register */
#define REG_PIOE_CODR   (*(WoReg*)0x400E1634U) /**< \brief (PIOE) Clear Output Data Register */
#define REG_PIOE_ODSR   (*(RwReg*)0x400E1638U) /**< \brief (PIOE) Output Data Status Register */
#define REG_PIOE_PDSR   (*(RoReg*)0x400E163CU) /**< \brief (PIOE) Pin Data Status Register */
#define REG_PIOE_IER    (*(WoReg*)0x400E1640U) /**< \brief (PIOE) Interrupt Enable Register */
#define REG_PIOE_IDR    (*(WoReg*)0x400E1644U) /**< \brief (PIOE) Interrupt Disable Register */
#define REG_PIOE_IMR    (*(RoReg*)0x400E1648U) /**< \brief (PIOE) Interrupt Mask Register */
#define REG_PIOE_ISR    (*(RoReg*)0x400E164CU) /**< \brief (PIOE) Interrupt Status Register */
#define REG_PIOE_MDER   (*(WoReg*)0x400E1650U) /**< \brief (PIOE) Multi-driver Enable Register */
#define REG_PIOE_MDDR   (*(WoReg*)0x400E1654U) /**< \brief (PIOE) Multi-driver Disable Register */
#define REG_PIOE_MDSR   (*(RoReg*)0x400E1658U) /**< \brief (PIOE) Multi-driver Status Register */
#define REG_PIOE_PUDR   (*(WoReg*)0x400E1660U) /**< \brief (PIOE) Pull-up Disable Register */
#define REG_PIOE_PUER   (*(WoReg*)0x400E1664U) /**< \brief (PIOE) Pull-up Enable Register */
#define REG_PIOE_PUSR   (*(RoReg*)0x400E1668U) /**< \brief (PIOE) Pad Pull-up Status Register */
#define REG_PIOE_ABSR   (*(RwReg*)0x400E1670U) /**< \brief (PIOE) Peripheral AB Select Register */
#define REG_PIOE_SCIFSR (*(WoReg*)0x400E1680U) /**< \brief (PIOE) System Clock Glitch Input Filter Select Register */
#define REG_PIOE_DIFSR  (*(WoReg*)0x400E1684U) /**< \brief (PIOE) Debouncing Input Filter Select Register */
#define REG_PIOE_IFDGSR (*(RoReg*)0x400E1688U) /**< \brief (PIOE) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOE_SCDR   (*(RwReg*)0x400E168CU) /**< \brief (PIOE) Slow Clock Divider Debouncing Register */
#define REG_PIOE_OWER   (*(WoReg*)0x400E16A0U) /**< \brief (PIOE) Output Write Enable */
#define REG_PIOE_OWDR   (*(WoReg*)0x400E16A4U) /**< \brief (PIOE) Output Write Disable */
#define REG_PIOE_OWSR   (*(RoReg*)0x400E16A8U) /**< \brief (PIOE) Output Write Status Register */
#define REG_PIOE_AIMER  (*(WoReg*)0x400E16B0U) /**< \brief (PIOE) Additional Interrupt Modes Enable Register */
#define REG_PIOE_AIMDR  (*(WoReg*)0x400E16B4U) /**< \brief (PIOE) Additional Interrupt Modes Disables Register */
#define REG_PIOE_AIMMR  (*(RoReg*)0x400E16B8U) /**< \brief (PIOE) Additional Interrupt Modes Mask Register */
#define REG_PIOE_ESR    (*(WoReg*)0x400E16C0U) /**< \brief (PIOE) Edge Select Register */
#define REG_PIOE_LSR    (*(WoReg*)0x400E16C4U) /**< \brief (PIOE) Level Select Register */
#define REG_PIOE_ELSR   (*(RoReg*)0x400E16C8U) /**< \brief (PIOE) Edge/Level Status Register */
#define REG_PIOE_FELLSR (*(WoReg*)0x400E16D0U) /**< \brief (PIOE) Falling Edge/Low Level Select Register */
#define REG_PIOE_REHLSR (*(WoReg*)0x400E16D4U) /**< \brief (PIOE) Rising Edge/ High Level Select Register */
#define REG_PIOE_FRLHSR (*(RoReg*)0x400E16D8U) /**< \brief (PIOE) Fall/Rise - Low/High Status Register */
#define REG_PIOE_LOCKSR (*(RoReg*)0x400E16E0U) /**< \brief (PIOE) Lock Status */
#define REG_PIOE_WPMR   (*(RwReg*)0x400E16E4U) /**< \brief (PIOE) Write Protect Mode Register */
#define REG_PIOE_WPSR   (*(RoReg*)0x400E16E8U) /**< \brief (PIOE) Write Protect Status Register */

#define REG_PIOF_PER    (*(WoReg*)0x400E1800U) /**< \brief (PIOF) PIO Enable Register */
#define REG_PIOF_PDR    (*(WoReg*)0x400E1804U) /**< \brief (PIOF) PIO Disable Register */
#define REG_PIOF_PSR    (*(RoReg*)0x400E1808U) /**< \brief (PIOF) PIO Status Register */
#define REG_PIOF_OER    (*(WoReg*)0x400E1810U) /**< \brief (PIOF) Output Enable Register */
#define REG_PIOF_ODR    (*(WoReg*)0x400E1814U) /**< \brief (PIOF) Output Disable Register */
#define REG_PIOF_OSR    (*(RoReg*)0x400E1818U) /**< \brief (PIOF) Output Status Register */
#define REG_PIOF_IFER   (*(WoReg*)0x400E1820U) /**< \brief (PIOF) Glitch Input Filter Enable Register */
#define REG_PIOF_IFDR   (*(WoReg*)0x400E1824U) /**< \brief (PIOF) Glitch Input Filter Disable Register */
#define REG_PIOF_IFSR   (*(RoReg*)0x400E1828U) /**< \brief (PIOF) Glitch Input Filter Status Register */
#define REG_PIOF_SODR   (*(WoReg*)0x400E1830U) /**< \brief (PIOF) Set Output Data Register */
#define REG_PIOF_CODR   (*(WoReg*)0x400E1834U) /**< \brief (PIOF) Clear Output Data Register */
#define REG_PIOF_ODSR   (*(RwReg*)0x400E1838U) /**< \brief (PIOF) Output Data Status Register */
#define REG_PIOF_PDSR   (*(RoReg*)0x400E183CU) /**< \brief (PIOF) Pin Data Status Register */
#define REG_PIOF_IER    (*(WoReg*)0x400E1840U) /**< \brief (PIOF) Interrupt Enable Register */
#define REG_PIOF_IDR    (*(WoReg*)0x400E1844U) /**< \brief (PIOF) Interrupt Disable Register */
#define REG_PIOF_IMR    (*(RoReg*)0x400E1848U) /**< \brief (PIOF) Interrupt Mask Register */
#define REG_PIOF_ISR    (*(RoReg*)0x400E184CU) /**< \brief (PIOF) Interrupt Status Register */
#define REG_PIOF_MDER   (*(WoReg*)0x400E1850U) /**< \brief (PIOF) Multi-driver Enable Register */
#define REG_PIOF_MDDR   (*(WoReg*)0x400E1854U) /**< \brief (PIOF) Multi-driver Disable Register */
#define REG_PIOF_MDSR   (*(RoReg*)0x400E1858U) /**< \brief (PIOF) Multi-driver Status Register */
#define REG_PIOF_PUDR   (*(WoReg*)0x400E1860U) /**< \brief (PIOF) Pull-up Disable Register */
#define REG_PIOF_PUER   (*(WoReg*)0x400E1864U) /**< \brief (PIOF) Pull-up Enable Register */
#define REG_PIOF_PUSR   (*(RoReg*)0x400E1868U) /**< \brief (PIOF) Pad Pull-up Status Register */
#define REG_PIOF_ABSR   (*(RwReg*)0x400E1870U) /**< \brief (PIOF) Peripheral AB Select Register */
#define REG_PIOF_SCIFSR (*(WoReg*)0x400E1880U) /**< \brief (PIOF) System Clock Glitch Input Filter Select Register */
#define REG_PIOF_DIFSR  (*(WoReg*)0x400E1884U) /**< \brief (PIOF) Debouncing Input Filter Select Register */
#define REG_PIOF_IFDGSR (*(RoReg*)0x400E1888U) /**< \brief (PIOF) Glitch or Debouncing Input Filter Clock Selection Status Register */
#define REG_PIOF_SCDR   (*(RwReg*)0x400E188CU) /**< \brief (PIOF) Slow Clock Divider Debouncing Register */
#define REG_PIOF_OWER   (*(WoReg*)0x400E18A0U) /**< \brief (PIOF) Output Write Enable */
#define REG_PIOF_OWDR   (*(WoReg*)0x400E18A4U) /**< \brief (PIOF) Output Write Disable */
#define REG_PIOF_OWSR   (*(RoReg*)0x400E18A8U) /**< \brief (PIOF) Output Write Status Register */
#define REG_PIOF_AIMER  (*(WoReg*)0x400E18B0U) /**< \brief (PIOF) Additional Interrupt Modes Enable Register */
#define REG_PIOF_AIMDR  (*(WoReg*)0x400E18B4U) /**< \brief (PIOF) Additional Interrupt Modes Disables Register */
#define REG_PIOF_AIMMR  (*(RoReg*)0x400E18B8U) /**< \brief (PIOF) Additional Interrupt Modes Mask Register */
#define REG_PIOF_ESR    (*(WoReg*)0x400E18C0U) /**< \brief (PIOF) Edge Select Register */
#define REG_PIOF_LSR    (*(WoReg*)0x400E18C4U) /**< \brief (PIOF) Level Select Register */
#define REG_PIOF_ELSR   (*(RoReg*)0x400E18C8U) /**< \brief (PIOF) Edge/Level Status Register */
#define REG_PIOF_FELLSR (*(WoReg*)0x400E18D0U) /**< \brief (PIOF) Falling Edge/Low Level Select Register */
#define REG_PIOF_REHLSR (*(WoReg*)0x400E18D4U) /**< \brief (PIOF) Rising Edge/ High Level Select Register */
#define REG_PIOF_FRLHSR (*(RoReg*)0x400E18D8U) /**< \brief (PIOF) Fall/Rise - Low/High Status Register */
#define REG_PIOF_LOCKSR (*(RoReg*)0x400E18E0U) /**< \brief (PIOF) Lock Status */
#define REG_PIOF_WPMR   (*(RwReg*)0x400E18E4U) /**< \brief (PIOF) Write Protect Mode Register */
#define REG_PIOF_WPSR   (*(RoReg*)0x400E18E8U) /**< \brief (PIOF) Write Protect Status Register */

#define REG_PMC_SCER   (*(WoReg*)0x400E0600U) /**< \brief (PMC) System Clock Enable Register */
#define REG_PMC_SCDR   (*(WoReg*)0x400E0604U) /**< \brief (PMC) System Clock Disable Register */
#define REG_PMC_SCSR   (*(RoReg*)0x400E0608U) /**< \brief (PMC) System Clock Status Register */
#define REG_PMC_PCER0  (*(WoReg*)0x400E0610U) /**< \brief (PMC) Peripheral Clock Enable Register 0 */
#define REG_PMC_PCDR0  (*(WoReg*)0x400E0614U) /**< \brief (PMC) Peripheral Clock Disable Register 0 */
#define REG_PMC_PCSR0  (*(RoReg*)0x400E0618U) /**< \brief (PMC) Peripheral Clock Status Register 0 */
#define REG_CKGR_UCKR  (*(RwReg*)0x400E061CU) /**< \brief (PMC) UTMI Clock Register */
#define REG_CKGR_MOR   (*(RwReg*)0x400E0620U) /**< \brief (PMC) Main Oscillator Register */
#define REG_CKGR_MCFR  (*(RoReg*)0x400E0624U) /**< \brief (PMC) Main Clock Frequency Register */
#define REG_CKGR_PLLAR (*(RwReg*)0x400E0628U) /**< \brief (PMC) PLLA Register */
#define REG_PMC_MCKR   (*(RwReg*)0x400E0630U) /**< \brief (PMC) Master Clock Register */
#define REG_PMC_USB    (*(RwReg*)0x400E0638U) /**< \brief (PMC) USB Clock Register */
#define REG_PMC_PCK    (*(RwReg*)0x400E0640U) /**< \brief (PMC) Programmable Clock 0 Register */
#define REG_PMC_IER    (*(WoReg*)0x400E0660U) /**< \brief (PMC) Interrupt Enable Register */
#define REG_PMC_IDR    (*(WoReg*)0x400E0664U) /**< \brief (PMC) Interrupt Disable Register */
#define REG_PMC_SR     (*(RoReg*)0x400E0668U) /**< \brief (PMC) Status Register */
#define REG_PMC_IMR    (*(RoReg*)0x400E066CU) /**< \brief (PMC) Interrupt Mask Register */
#define REG_PMC_FSMR   (*(RwReg*)0x400E0670U) /**< \brief (PMC) Fast Startup Mode Register */
#define REG_PMC_FSPR   (*(RwReg*)0x400E0674U) /**< \brief (PMC) Fast Startup Polarity Register */
#define REG_PMC_FOCR   (*(WoReg*)0x400E0678U) /**< \brief (PMC) Fault Output Clear Register */
#define REG_PMC_WPMR   (*(RwReg*)0x400E06E4U) /**< \brief (PMC) Write Protect Mode Register */
#define REG_PMC_WPSR   (*(RoReg*)0x400E06E8U) /**< \brief (PMC) Write Protect Status Register */
#define REG_PMC_PCER1  (*(WoReg*)0x400E0700U) /**< \brief (PMC) Peripheral Clock Enable Register 1 */
#define REG_PMC_PCDR1  (*(WoReg*)0x400E0704U) /**< \brief (PMC) Peripheral Clock Disable Register 1 */
#define REG_PMC_PCSR1  (*(RoReg*)0x400E0708U) /**< \brief (PMC) Peripheral Clock Status Register 1 */
#define REG_PMC_PCR    (*(RwReg*)0x400E070CU) /**< \brief (PMC) Peripheral Control Register */

#define REG_PWM_CLK      (*(RwReg*)0x40094000U) /**< \brief (PWM) PWM Clock Register */
#define REG_PWM_ENA      (*(WoReg*)0x40094004U) /**< \brief (PWM) PWM Enable Register */
#define REG_PWM_DIS      (*(WoReg*)0x40094008U) /**< \brief (PWM) PWM Disable Register */
#define REG_PWM_SR       (*(RoReg*)0x4009400CU) /**< \brief (PWM) PWM Status Register */
#define REG_PWM_IER1     (*(WoReg*)0x40094010U) /**< \brief (PWM) PWM Interrupt Enable Register 1 */
#define REG_PWM_IDR1     (*(WoReg*)0x40094014U) /**< \brief (PWM) PWM Interrupt Disable Register 1 */
#define REG_PWM_IMR1     (*(RoReg*)0x40094018U) /**< \brief (PWM) PWM Interrupt Mask Register 1 */
#define REG_PWM_ISR1     (*(RoReg*)0x4009401CU) /**< \brief (PWM) PWM Interrupt Status Register 1 */
#define REG_PWM_SCM      (*(RwReg*)0x40094020U) /**< \brief (PWM) PWM Sync Channels Mode Register */
#define REG_PWM_SCUC     (*(RwReg*)0x40094028U) /**< \brief (PWM) PWM Sync Channels Update Control Register */
#define REG_PWM_SCUP     (*(RwReg*)0x4009402CU) /**< \brief (PWM) PWM Sync Channels Update Period Register */
#define REG_PWM_SCUPUPD  (*(WoReg*)0x40094030U) /**< \brief (PWM) PWM Sync Channels Update Period Update Register */
#define REG_PWM_IER2     (*(WoReg*)0x40094034U) /**< \brief (PWM) PWM Interrupt Enable Register 2 */
#define REG_PWM_IDR2     (*(WoReg*)0x40094038U) /**< \brief (PWM) PWM Interrupt Disable Register 2 */
#define REG_PWM_IMR2     (*(RoReg*)0x4009403CU) /**< \brief (PWM) PWM Interrupt Mask Register 2 */
#define REG_PWM_ISR2     (*(RoReg*)0x40094040U) /**< \brief (PWM) PWM Interrupt Status Register 2 */
#define REG_PWM_OOV      (*(RwReg*)0x40094044U) /**< \brief (PWM) PWM Output Override Value Register */
#define REG_PWM_OS       (*(RwReg*)0x40094048U) /**< \brief (PWM) PWM Output Selection Register */
#define REG_PWM_OSS      (*(WoReg*)0x4009404CU) /**< \brief (PWM) PWM Output Selection Set Register */
#define REG_PWM_OSC      (*(WoReg*)0x40094050U) /**< \brief (PWM) PWM Output Selection Clear Register */
#define REG_PWM_OSSUPD   (*(WoReg*)0x40094054U) /**< \brief (PWM) PWM Output Selection Set Update Register */
#define REG_PWM_OSCUPD   (*(WoReg*)0x40094058U) /**< \brief (PWM) PWM Output Selection Clear Update Register */
#define REG_PWM_FMR      (*(RwReg*)0x4009405CU) /**< \brief (PWM) PWM Fault Mode Register */
#define REG_PWM_FSR      (*(RoReg*)0x40094060U) /**< \brief (PWM) PWM Fault Status Register */
#define REG_PWM_FCR      (*(WoReg*)0x40094064U) /**< \brief (PWM) PWM Fault Clear Register */
#define REG_PWM_FPV      (*(RwReg*)0x40094068U) /**< \brief (PWM) PWM Fault Protection Value Register */
#define REG_PWM_FPE1     (*(RwReg*)0x4009406CU) /**< \brief (PWM) PWM Fault Protection Enable Register 1 */
#define REG_PWM_FPE2     (*(RwReg*)0x40094070U) /**< \brief (PWM) PWM Fault Protection Enable Register 2 */
#define REG_PWM_ELMR     (*(RwReg*)0x4009407CU) /**< \brief (PWM) PWM Event Line 0 Mode Register */
#define REG_PWM_SMMR     (*(RwReg*)0x400940B0U) /**< \brief (PWM) PWM Stepper Motor Mode Register */
#define REG_PWM_WPCR     (*(WoReg*)0x400940E4U) /**< \brief (PWM) PWM Write Protect Control Register */
#define REG_PWM_WPSR     (*(RoReg*)0x400940E8U) /**< \brief (PWM) PWM Write Protect Status Register */
#define REG_PWM_TPR      (*(RwReg*)0x40094108U) /**< \brief (PWM) Transmit Pointer Register */
#define REG_PWM_TCR      (*(RwReg*)0x4009410CU) /**< \brief (PWM) Transmit Counter Register */
#define REG_PWM_TNPR     (*(RwReg*)0x40094118U) /**< \brief (PWM) Transmit Next Pointer Register */
#define REG_PWM_TNCR     (*(RwReg*)0x4009411CU) /**< \brief (PWM) Transmit Next Counter Register */
#define REG_PWM_PTCR     (*(WoReg*)0x40094120U) /**< \brief (PWM) Transfer Control Register */
#define REG_PWM_PTSR     (*(RoReg*)0x40094124U) /**< \brief (PWM) Transfer Status Register */
#define REG_PWM_CMPV0    (*(RwReg*)0x40094130U) /**< \brief (PWM) PWM Comparison 0 Value Register */
#define REG_PWM_CMPVUPD0 (*(WoReg*)0x40094134U) /**< \brief (PWM) PWM Comparison 0 Value Update Register */
#define REG_PWM_CMPM0    (*(RwReg*)0x40094138U) /**< \brief (PWM) PWM Comparison 0 Mode Register */
#define REG_PWM_CMPMUPD0 (*(WoReg*)0x4009413CU) /**< \brief (PWM) PWM Comparison 0 Mode Update Register */
#define REG_PWM_CMPV1    (*(RwReg*)0x40094140U) /**< \brief (PWM) PWM Comparison 1 Value Register */
#define REG_PWM_CMPVUPD1 (*(WoReg*)0x40094144U) /**< \brief (PWM) PWM Comparison 1 Value Update Register */
#define REG_PWM_CMPM1    (*(RwReg*)0x40094148U) /**< \brief (PWM) PWM Comparison 1 Mode Register */
#define REG_PWM_CMPMUPD1 (*(WoReg*)0x4009414CU) /**< \brief (PWM) PWM Comparison 1 Mode Update Register */
#define REG_PWM_CMPV2    (*(RwReg*)0x40094150U) /**< \brief (PWM) PWM Comparison 2 Value Register */
#define REG_PWM_CMPVUPD2 (*(WoReg*)0x40094154U) /**< \brief (PWM) PWM Comparison 2 Value Update Register */
#define REG_PWM_CMPM2    (*(RwReg*)0x40094158U) /**< \brief (PWM) PWM Comparison 2 Mode Register */
#define REG_PWM_CMPMUPD2 (*(WoReg*)0x4009415CU) /**< \brief (PWM) PWM Comparison 2 Mode Update Register */
#define REG_PWM_CMPV3    (*(RwReg*)0x40094160U) /**< \brief (PWM) PWM Comparison 3 Value Register */
#define REG_PWM_CMPVUPD3 (*(WoReg*)0x40094164U) /**< \brief (PWM) PWM Comparison 3 Value Update Register */
#define REG_PWM_CMPM3    (*(RwReg*)0x40094168U) /**< \brief (PWM) PWM Comparison 3 Mode Register */
#define REG_PWM_CMPMUPD3 (*(WoReg*)0x4009416CU) /**< \brief (PWM) PWM Comparison 3 Mode Update Register */
#define REG_PWM_CMPV4    (*(RwReg*)0x40094170U) /**< \brief (PWM) PWM Comparison 4 Value Register */
#define REG_PWM_CMPVUPD4 (*(WoReg*)0x40094174U) /**< \brief (PWM) PWM Comparison 4 Value Update Register */
#define REG_PWM_CMPM4    (*(RwReg*)0x40094178U) /**< \brief (PWM) PWM Comparison 4 Mode Register */
#define REG_PWM_CMPMUPD4 (*(WoReg*)0x4009417CU) /**< \brief (PWM) PWM Comparison 4 Mode Update Register */
#define REG_PWM_CMPV5    (*(RwReg*)0x40094180U) /**< \brief (PWM) PWM Comparison 5 Value Register */
#define REG_PWM_CMPVUPD5 (*(WoReg*)0x40094184U) /**< \brief (PWM) PWM Comparison 5 Value Update Register */
#define REG_PWM_CMPM5    (*(RwReg*)0x40094188U) /**< \brief (PWM) PWM Comparison 5 Mode Register */
#define REG_PWM_CMPMUPD5 (*(WoReg*)0x4009418CU) /**< \brief (PWM) PWM Comparison 5 Mode Update Register */
#define REG_PWM_CMPV6    (*(RwReg*)0x40094190U) /**< \brief (PWM) PWM Comparison 6 Value Register */
#define REG_PWM_CMPVUPD6 (*(WoReg*)0x40094194U) /**< \brief (PWM) PWM Comparison 6 Value Update Register */
#define REG_PWM_CMPM6    (*(RwReg*)0x40094198U) /**< \brief (PWM) PWM Comparison 6 Mode Register */
#define REG_PWM_CMPMUPD6 (*(WoReg*)0x4009419CU) /**< \brief (PWM) PWM Comparison 6 Mode Update Register */
#define REG_PWM_CMPV7    (*(RwReg*)0x400941A0U) /**< \brief (PWM) PWM Comparison 7 Value Register */
#define REG_PWM_CMPVUPD7 (*(WoReg*)0x400941A4U) /**< \brief (PWM) PWM Comparison 7 Value Update Register */
#define REG_PWM_CMPM7    (*(RwReg*)0x400941A8U) /**< \brief (PWM) PWM Comparison 7 Mode Register */
#define REG_PWM_CMPMUPD7 (*(WoReg*)0x400941ACU) /**< \brief (PWM) PWM Comparison 7 Mode Update Register */
#define REG_PWM_CMR0     (*(RwReg*)0x40094200U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 0) */
#define REG_PWM_CDTY0    (*(RwReg*)0x40094204U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 0) */
#define REG_PWM_CDTYUPD0 (*(WoReg*)0x40094208U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 0) */
#define REG_PWM_CPRD0    (*(RwReg*)0x4009420CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 0) */
#define REG_PWM_CPRDUPD0 (*(WoReg*)0x40094210U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 0) */
#define REG_PWM_CCNT0    (*(RoReg*)0x40094214U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 0) */
#define REG_PWM_DT0      (*(RwReg*)0x40094218U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 0) */
#define REG_PWM_DTUPD0   (*(WoReg*)0x4009421CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 0) */
#define REG_PWM_CMR1     (*(RwReg*)0x40094220U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 1) */
#define REG_PWM_CDTY1    (*(RwReg*)0x40094224U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 1) */
#define REG_PWM_CDTYUPD1 (*(WoReg*)0x40094228U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 1) */
#define REG_PWM_CPRD1    (*(RwReg*)0x4009422CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 1) */
#define REG_PWM_CPRDUPD1 (*(WoReg*)0x40094230U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 1) */
#define REG_PWM_CCNT1    (*(RoReg*)0x40094234U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 1) */
#define REG_PWM_DT1      (*(RwReg*)0x40094238U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 1) */
#define REG_PWM_DTUPD1   (*(WoReg*)0x4009423CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 1) */
#define REG_PWM_CMR2     (*(RwReg*)0x40094240U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 2) */
#define REG_PWM_CDTY2    (*(RwReg*)0x40094244U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 2) */
#define REG_PWM_CDTYUPD2 (*(WoReg*)0x40094248U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 2) */
#define REG_PWM_CPRD2    (*(RwReg*)0x4009424CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 2) */
#define REG_PWM_CPRDUPD2 (*(WoReg*)0x40094250U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 2) */
#define REG_PWM_CCNT2    (*(RoReg*)0x40094254U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 2) */
#define REG_PWM_DT2      (*(RwReg*)0x40094258U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 2) */
#define REG_PWM_DTUPD2   (*(WoReg*)0x4009425CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 2) */
#define REG_PWM_CMR3     (*(RwReg*)0x40094260U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 3) */
#define REG_PWM_CDTY3    (*(RwReg*)0x40094264U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 3) */
#define REG_PWM_CDTYUPD3 (*(WoReg*)0x40094268U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 3) */
#define REG_PWM_CPRD3    (*(RwReg*)0x4009426CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 3) */
#define REG_PWM_CPRDUPD3 (*(WoReg*)0x40094270U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 3) */
#define REG_PWM_CCNT3    (*(RoReg*)0x40094274U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 3) */
#define REG_PWM_DT3      (*(RwReg*)0x40094278U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 3) */
#define REG_PWM_DTUPD3   (*(WoReg*)0x4009427CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 3) */
#define REG_PWM_CMR4     (*(RwReg*)0x40094280U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 4) */
#define REG_PWM_CDTY4    (*(RwReg*)0x40094284U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 4) */
#define REG_PWM_CDTYUPD4 (*(WoReg*)0x40094288U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 4) */
#define REG_PWM_CPRD4    (*(RwReg*)0x4009428CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 4) */
#define REG_PWM_CPRDUPD4 (*(WoReg*)0x40094290U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 4) */
#define REG_PWM_CCNT4    (*(RoReg*)0x40094294U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 4) */
#define REG_PWM_DT4      (*(RwReg*)0x40094298U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 4) */
#define REG_PWM_DTUPD4   (*(WoReg*)0x4009429CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 4) */
#define REG_PWM_CMR5     (*(RwReg*)0x400942A0U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 5) */
#define REG_PWM_CDTY5    (*(RwReg*)0x400942A4U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 5) */
#define REG_PWM_CDTYUPD5 (*(WoReg*)0x400942A8U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 5) */
#define REG_PWM_CPRD5    (*(RwReg*)0x400942ACU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 5) */
#define REG_PWM_CPRDUPD5 (*(WoReg*)0x400942B0U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 5) */
#define REG_PWM_CCNT5    (*(RoReg*)0x400942B4U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 5) */
#define REG_PWM_DT5      (*(RwReg*)0x400942B8U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 5) */
#define REG_PWM_DTUPD5   (*(WoReg*)0x400942BCU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 5) */
#define REG_PWM_CMR6     (*(RwReg*)0x400942C0U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 6) */
#define REG_PWM_CDTY6    (*(RwReg*)0x400942C4U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 6) */
#define REG_PWM_CDTYUPD6 (*(WoReg*)0x400942C8U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 6) */
#define REG_PWM_CPRD6    (*(RwReg*)0x400942CCU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 6) */
#define REG_PWM_CPRDUPD6 (*(WoReg*)0x400942D0U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 6) */
#define REG_PWM_CCNT6    (*(RoReg*)0x400942D4U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 6) */
#define REG_PWM_DT6      (*(RwReg*)0x400942D8U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 6) */
#define REG_PWM_DTUPD6   (*(WoReg*)0x400942DCU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 6) */
#define REG_PWM_CMR7     (*(RwReg*)0x400942E0U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 7) */
#define REG_PWM_CDTY7    (*(RwReg*)0x400942E4U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 7) */
#define REG_PWM_CDTYUPD7 (*(WoReg*)0x400942E8U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 7) */
#define REG_PWM_CPRD7    (*(RwReg*)0x400942ECU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 7) */
#define REG_PWM_CPRDUPD7 (*(WoReg*)0x400942F0U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 7) */
#define REG_PWM_CCNT7    (*(RoReg*)0x400942F4U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 7) */
#define REG_PWM_DT7      (*(RwReg*)0x400942F8U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 7) */
#define REG_PWM_DTUPD7   (*(WoReg*)0x400942FCU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 7) */

#define REG_RSTC_CR (*(WoReg*)0x400E1A00U) /**< \brief (RSTC) Control Register */
#define REG_RSTC_SR (*(RoReg*)0x400E1A04U) /**< \brief (RSTC) Status Register */
#define REG_RSTC_MR (*(RwReg*)0x400E1A08U) /**< \brief (RSTC) Mode Register */

#define REG_RTC_CR     (*(RwReg*)0x400E1A60U) /**< \brief (RTC) Control Register */
#define REG_RTC_MR     (*(RwReg*)0x400E1A64U) /**< \brief (RTC) Mode Register */
#define REG_RTC_TIMR   (*(RwReg*)0x400E1A68U) /**< \brief (RTC) Time Register */
#define REG_RTC_CALR   (*(RwReg*)0x400E1A6CU) /**< \brief (RTC) Calendar Register */
#define REG_RTC_TIMALR (*(RwReg*)0x400E1A70U) /**< \brief (RTC) Time Alarm Register */
#define REG_RTC_CALALR (*(RwReg*)0x400E1A74U) /**< \brief (RTC) Calendar Alarm Register */
#define REG_RTC_SR     (*(RoReg*)0x400E1A78U) /**< \brief (RTC) Status Register */
#define REG_RTC_SCCR   (*(WoReg*)0x400E1A7CU) /**< \brief (RTC) Status Clear Command Register */
#define REG_RTC_IER    (*(WoReg*)0x400E1A80U) /**< \brief (RTC) Interrupt Enable Register */
#define REG_RTC_IDR    (*(WoReg*)0x400E1A84U) /**< \brief (RTC) Interrupt Disable Register */
#define REG_RTC_IMR    (*(RoReg*)0x400E1A88U) /**< \brief (RTC) Interrupt Mask Register */
#define REG_RTC_VER    (*(RoReg*)0x400E1A8CU) /**< \brief (RTC) Valid Entry Register */
#define REG_RTC_WPMR   (*(RwReg*)0x400E1B44U) /**< \brief (RTC) Write Protect Mode Register */

#define REG_RTT_MR (*(RwReg*)0x400E1A30U) /**< \brief (RTT) Mode Register */
#define REG_RTT_AR (*(RwReg*)0x400E1A34U) /**< \brief (RTT) Alarm Register */
#define REG_RTT_VR (*(RoReg*)0x400E1A38U) /**< \brief (RTT) Value Register */
#define REG_RTT_SR (*(RoReg*)0x400E1A3CU) /**< \brief (RTT) Status Register */

#define REG_SDRAMC_MR   (*(RwReg*)0x400E0200U) /**< \brief (SDRAMC) SDRAMC Mode Register */
#define REG_SDRAMC_TR   (*(RwReg*)0x400E0204U) /**< \brief (SDRAMC) SDRAMC Refresh Timer Register */
#define REG_SDRAMC_CR   (*(RwReg*)0x400E0208U) /**< \brief (SDRAMC) SDRAMC Configuration Register */
#define REG_SDRAMC_LPR  (*(RwReg*)0x400E0210U) /**< \brief (SDRAMC) SDRAMC Low Power Register */
#define REG_SDRAMC_IER  (*(WoReg*)0x400E0214U) /**< \brief (SDRAMC) SDRAMC Interrupt Enable Register */
#define REG_SDRAMC_IDR  (*(WoReg*)0x400E0218U) /**< \brief (SDRAMC) SDRAMC Interrupt Disable Register */
#define REG_SDRAMC_IMR  (*(RoReg*)0x400E021CU) /**< \brief (SDRAMC) SDRAMC Interrupt Mask Register */
#define REG_SDRAMC_ISR  (*(RoReg*)0x400E0220U) /**< \brief (SDRAMC) SDRAMC Interrupt Status Register */
#define REG_SDRAMC_MDR  (*(RwReg*)0x400E0224U) /**< \brief (SDRAMC) SDRAMC Memory Device Register */
#define REG_SDRAMC_CR1  (*(RwReg*)0x400E0228U) /**< \brief (SDRAMC) SDRAMC Configuration Register 1 */
#define REG_SDRAMC_OCMS (*(RwReg*)0x400E022CU) /**< \brief (SDRAMC) SDRAMC OCMS Register 1 */

#define REG_SMC_CFG      (*(RwReg*)0x400E0000U) /**< \brief (SMC) SMC NFC Configuration Register */
#define REG_SMC_CTRL     (*(WoReg*)0x400E0004U) /**< \brief (SMC) SMC NFC Control Register */
#define REG_SMC_SR       (*(RoReg*)0x400E0008U) /**< \brief (SMC) SMC NFC Status Register */
#define REG_SMC_IER      (*(WoReg*)0x400E000CU) /**< \brief (SMC) SMC NFC Interrupt Enable Register */
#define REG_SMC_IDR      (*(WoReg*)0x400E0010U) /**< \brief (SMC) SMC NFC Interrupt Disable Register */
#define REG_SMC_IMR      (*(RoReg*)0x400E0014U) /**< \brief (SMC) SMC NFC Interrupt Mask Register */
#define REG_SMC_ADDR     (*(RwReg*)0x400E0018U) /**< \brief (SMC) SMC NFC Address Cycle Zero Register */
#define REG_SMC_BANK     (*(RwReg*)0x400E001CU) /**< \brief (SMC) SMC Bank Address Register */
#define REG_SMC_ECC_CTRL (*(WoReg*)0x400E0020U) /**< \brief (SMC) SMC ECC Control Register */
#define REG_SMC_ECC_MD   (*(RwReg*)0x400E0024U) /**< \brief (SMC) SMC ECC Mode Register */
#define REG_SMC_ECC_SR1  (*(RoReg*)0x400E0028U) /**< \brief (SMC) SMC ECC Status 1 Register */
#define REG_SMC_ECC_PR0  (*(RoReg*)0x400E002CU) /**< \brief (SMC) SMC ECC Parity 0 Register */
#define REG_SMC_ECC_PR1  (*(RoReg*)0x400E0030U) /**< \brief (SMC) SMC ECC parity 1 Register */
#define REG_SMC_ECC_SR2  (*(RoReg*)0x400E0034U) /**< \brief (SMC) SMC ECC status 2 Register */
#define REG_SMC_ECC_PR2  (*(RoReg*)0x400E0038U) /**< \brief (SMC) SMC ECC parity 2 Register */
#define REG_SMC_ECC_PR3  (*(RoReg*)0x400E003CU) /**< \brief (SMC) SMC ECC parity 3 Register */
#define REG_SMC_ECC_PR4  (*(RoReg*)0x400E0040U) /**< \brief (SMC) SMC ECC parity 4 Register */
#define REG_SMC_ECC_PR5  (*(RoReg*)0x400E0044U) /**< \brief (SMC) SMC ECC parity 5 Register */
#define REG_SMC_ECC_PR6  (*(RoReg*)0x400E0048U) /**< \brief (SMC) SMC ECC parity 6 Register */
#define REG_SMC_ECC_PR7  (*(RoReg*)0x400E004CU) /**< \brief (SMC) SMC ECC parity 7 Register */
#define REG_SMC_ECC_PR8  (*(RoReg*)0x400E0050U) /**< \brief (SMC) SMC ECC parity 8 Register */
#define REG_SMC_ECC_PR9  (*(RoReg*)0x400E0054U) /**< \brief (SMC) SMC ECC parity 9 Register */
#define REG_SMC_ECC_PR10 (*(RoReg*)0x400E0058U) /**< \brief (SMC) SMC ECC parity 10 Register */
#define REG_SMC_ECC_PR11 (*(RoReg*)0x400E005CU) /**< \brief (SMC) SMC ECC parity 11 Register */
#define REG_SMC_ECC_PR12 (*(RoReg*)0x400E0060U) /**< \brief (SMC) SMC ECC parity 12 Register */
#define REG_SMC_ECC_PR13 (*(RoReg*)0x400E0064U) /**< \brief (SMC) SMC ECC parity 13 Register */
#define REG_SMC_ECC_PR14 (*(RoReg*)0x400E0068U) /**< \brief (SMC) SMC ECC parity 14 Register */
#define REG_SMC_ECC_PR15 (*(RoReg*)0x400E006CU) /**< \brief (SMC) SMC ECC parity 15 Register */
#define REG_SMC_SETUP0   (*(RwReg*)0x400E0070U) /**< \brief (SMC) SMC Setup Register (CS_number = 0) */
#define REG_SMC_PULSE0   (*(RwReg*)0x400E0074U) /**< \brief (SMC) SMC Pulse Register (CS_number = 0) */
#define REG_SMC_CYCLE0   (*(RwReg*)0x400E0078U) /**< \brief (SMC) SMC Cycle Register (CS_number = 0) */
#define REG_SMC_TIMINGS0 (*(RwReg*)0x400E007CU) /**< \brief (SMC) SMC Timings Register (CS_number = 0) */
#define REG_SMC_MODE0    (*(RwReg*)0x400E0080U) /**< \brief (SMC) SMC Mode Register (CS_number = 0) */
#define REG_SMC_SETUP1   (*(RwReg*)0x400E0084U) /**< \brief (SMC) SMC Setup Register (CS_number = 1) */
#define REG_SMC_PULSE1   (*(RwReg*)0x400E0088U) /**< \brief (SMC) SMC Pulse Register (CS_number = 1) */
#define REG_SMC_CYCLE1   (*(RwReg*)0x400E008CU) /**< \brief (SMC) SMC Cycle Register (CS_number = 1) */
#define REG_SMC_TIMINGS1 (*(RwReg*)0x400E0090U) /**< \brief (SMC) SMC Timings Register (CS_number = 1) */
#define REG_SMC_MODE1    (*(RwReg*)0x400E0094U) /**< \brief (SMC) SMC Mode Register (CS_number = 1) */
#define REG_SMC_SETUP2   (*(RwReg*)0x400E0098U) /**< \brief (SMC) SMC Setup Register (CS_number = 2) */
#define REG_SMC_PULSE2   (*(RwReg*)0x400E009CU) /**< \brief (SMC) SMC Pulse Register (CS_number = 2) */
#define REG_SMC_CYCLE2   (*(RwReg*)0x400E00A0U) /**< \brief (SMC) SMC Cycle Register (CS_number = 2) */
#define REG_SMC_TIMINGS2 (*(RwReg*)0x400E00A4U) /**< \brief (SMC) SMC Timings Register (CS_number = 2) */
#define REG_SMC_MODE2    (*(RwReg*)0x400E00A8U) /**< \brief (SMC) SMC Mode Register (CS_number = 2) */
#define REG_SMC_SETUP3   (*(RwReg*)0x400E00ACU) /**< \brief (SMC) SMC Setup Register (CS_number = 3) */
#define REG_SMC_PULSE3   (*(RwReg*)0x400E00B0U) /**< \brief (SMC) SMC Pulse Register (CS_number = 3) */
#define REG_SMC_CYCLE3   (*(RwReg*)0x400E00B4U) /**< \brief (SMC) SMC Cycle Register (CS_number = 3) */
#define REG_SMC_TIMINGS3 (*(RwReg*)0x400E00B8U) /**< \brief (SMC) SMC Timings Register (CS_number = 3) */
#define REG_SMC_MODE3    (*(RwReg*)0x400E00BCU) /**< \brief (SMC) SMC Mode Register (CS_number = 3) */
#define REG_SMC_SETUP4   (*(RwReg*)0x400E00C0U) /**< \brief (SMC) SMC Setup Register (CS_number = 4) */
#define REG_SMC_PULSE4   (*(RwReg*)0x400E00C4U) /**< \brief (SMC) SMC Pulse Register (CS_number = 4) */
#define REG_SMC_CYCLE4   (*(RwReg*)0x400E00C8U) /**< \brief (SMC) SMC Cycle Register (CS_number = 4) */
#define REG_SMC_TIMINGS4 (*(RwReg*)0x400E00CCU) /**< \brief (SMC) SMC Timings Register (CS_number = 4) */
#define REG_SMC_MODE4    (*(RwReg*)0x400E00D0U) /**< \brief (SMC) SMC Mode Register (CS_number = 4) */
#define REG_SMC_SETUP5   (*(RwReg*)0x400E00D4U) /**< \brief (SMC) SMC Setup Register (CS_number = 5) */
#define REG_SMC_PULSE5   (*(RwReg*)0x400E00D8U) /**< \brief (SMC) SMC Pulse Register (CS_number = 5) */
#define REG_SMC_CYCLE5   (*(RwReg*)0x400E00DCU) /**< \brief (SMC) SMC Cycle Register (CS_number = 5) */
#define REG_SMC_TIMINGS5 (*(RwReg*)0x400E00E0U) /**< \brief (SMC) SMC Timings Register (CS_number = 5) */
#define REG_SMC_MODE5    (*(RwReg*)0x400E00E4U) /**< \brief (SMC) SMC Mode Register (CS_number = 5) */
#define REG_SMC_SETUP6   (*(RwReg*)0x400E00E8U) /**< \brief (SMC) SMC Setup Register (CS_number = 6) */
#define REG_SMC_PULSE6   (*(RwReg*)0x400E00ECU) /**< \brief (SMC) SMC Pulse Register (CS_number = 6) */
#define REG_SMC_CYCLE6   (*(RwReg*)0x400E00F0U) /**< \brief (SMC) SMC Cycle Register (CS_number = 6) */
#define REG_SMC_TIMINGS6 (*(RwReg*)0x400E00F4U) /**< \brief (SMC) SMC Timings Register (CS_number = 6) */
#define REG_SMC_MODE6    (*(RwReg*)0x400E00F8U) /**< \brief (SMC) SMC Mode Register (CS_number = 6) */
#define REG_SMC_SETUP7   (*(RwReg*)0x400E00FCU) /**< \brief (SMC) SMC Setup Register (CS_number = 7) */
#define REG_SMC_PULSE7   (*(RwReg*)0x400E0100U) /**< \brief (SMC) SMC Pulse Register (CS_number = 7) */
#define REG_SMC_CYCLE7   (*(RwReg*)0x400E0104U) /**< \brief (SMC) SMC Cycle Register (CS_number = 7) */
#define REG_SMC_TIMINGS7 (*(RwReg*)0x400E0108U) /**< \brief (SMC) SMC Timings Register (CS_number = 7) */
#define REG_SMC_MODE7    (*(RwReg*)0x400E010CU) /**< \brief (SMC) SMC Mode Register (CS_number = 7) */
#define REG_SMC_OCMS     (*(RwReg*)0x400E0110U) /**< \brief (SMC) SMC OCMS Register */
#define REG_SMC_KEY1     (*(WoReg*)0x400E0114U) /**< \brief (SMC) SMC OCMS KEY1 Register */
#define REG_SMC_KEY2     (*(WoReg*)0x400E0118U) /**< \brief (SMC) SMC OCMS KEY2 Register */
#define REG_SMC_WPCR     (*(WoReg*)0x400E01E4U) /**< \brief (SMC) Write Protection Control Register */
#define REG_SMC_WPSR     (*(RoReg*)0x400E01E8U) /**< \brief (SMC) Write Protection Status Register */

#define REG_SPI0_CR     (*(WoReg*)0x40008000U) /**< \brief (SPI0) Control Register */
#define REG_SPI0_MR     (*(RwReg*)0x40008004U) /**< \brief (SPI0) Mode Register */
#define REG_SPI0_RDR    (*(RoReg*)0x40008008U) /**< \brief (SPI0) Receive Data Register */
#define REG_SPI0_TDR    (*(WoReg*)0x4000800CU) /**< \brief (SPI0) Transmit Data Register */
#define REG_SPI0_SR     (*(RoReg*)0x40008010U) /**< \brief (SPI0) Status Register */
#define REG_SPI0_IER    (*(WoReg*)0x40008014U) /**< \brief (SPI0) Interrupt Enable Register */
#define REG_SPI0_IDR    (*(WoReg*)0x40008018U) /**< \brief (SPI0) Interrupt Disable Register */
#define REG_SPI0_IMR    (*(RoReg*)0x4000801CU) /**< \brief (SPI0) Interrupt Mask Register */
#define REG_SPI0_CSR    (*(RwReg*)0x40008030U) /**< \brief (SPI0) Chip Select Register */
#define REG_SPI0_WPMR   (*(RwReg*)0x400080E4U) /**< \brief (SPI0) Write Protection Control Register */
#define REG_SPI0_WPSR   (*(RoReg*)0x400080E8U) /**< \brief (SPI0) Write Protection Status Register */

#define REG_SPI1_CR     (*(WoReg*)0x4000C000U) /**< \brief (SPI1) Control Register */
#define REG_SPI1_MR     (*(RwReg*)0x4000C004U) /**< \brief (SPI1) Mode Register */
#define REG_SPI1_RDR    (*(RoReg*)0x4000C008U) /**< \brief (SPI1) Receive Data Register */
#define REG_SPI1_TDR    (*(WoReg*)0x4000C00CU) /**< \brief (SPI1) Transmit Data Register */
#define REG_SPI1_SR     (*(RoReg*)0x4000C010U) /**< \brief (SPI1) Status Register */
#define REG_SPI1_IER    (*(WoReg*)0x4000C014U) /**< \brief (SPI1) Interrupt Enable Register */
#define REG_SPI1_IDR    (*(WoReg*)0x4000C018U) /**< \brief (SPI1) Interrupt Disable Register */
#define REG_SPI1_IMR    (*(RoReg*)0x4000C01CU) /**< \brief (SPI1) Interrupt Mask Register */
#define REG_SPI1_CSR    (*(RwReg*)0x4000C030U) /**< \brief (SPI1) Chip Select Register */
#define REG_SPI1_WPMR   (*(RwReg*)0x4000C0E4U) /**< \brief (SPI1) Write Protection Control Register */
#define REG_SPI1_WPSR   (*(RoReg*)0x4000C0E8U) /**< \brief (SPI1) Write Protection Status Register */

#define REG_SSC_CR   (*(WoReg*)0x40004000U) /**< \brief (SSC) Control Register */
#define REG_SSC_CMR  (*(RwReg*)0x40004004U) /**< \brief (SSC) Clock Mode Register */
#define REG_SSC_RCMR (*(RwReg*)0x40004010U) /**< \brief (SSC) Receive Clock Mode Register */
#define REG_SSC_RFMR (*(RwReg*)0x40004014U) /**< \brief (SSC) Receive Frame Mode Register */
#define REG_SSC_TCMR (*(RwReg*)0x40004018U) /**< \brief (SSC) Transmit Clock Mode Register */
#define REG_SSC_TFMR (*(RwReg*)0x4000401CU) /**< \brief (SSC) Transmit Frame Mode Register */
#define REG_SSC_RHR  (*(RoReg*)0x40004020U) /**< \brief (SSC) Receive Holding Register */
#define REG_SSC_THR  (*(WoReg*)0x40004024U) /**< \brief (SSC) Transmit Holding Register */
#define REG_SSC_RSHR (*(RoReg*)0x40004030U) /**< \brief (SSC) Receive Sync. Holding Register */
#define REG_SSC_TSHR (*(RwReg*)0x40004034U) /**< \brief (SSC) Transmit Sync. Holding Register */
#define REG_SSC_RC0R (*(RwReg*)0x40004038U) /**< \brief (SSC) Receive Compare 0 Register */
#define REG_SSC_RC1R (*(RwReg*)0x4000403CU) /**< \brief (SSC) Receive Compare 1 Register */
#define REG_SSC_SR   (*(RoReg*)0x40004040U) /**< \brief (SSC) Status Register */
#define REG_SSC_IER  (*(WoReg*)0x40004044U) /**< \brief (SSC) Interrupt Enable Register */
#define REG_SSC_IDR  (*(WoReg*)0x40004048U) /**< \brief (SSC) Interrupt Disable Register */
#define REG_SSC_IMR  (*(RoReg*)0x4000404CU) /**< \brief (SSC) Interrupt Mask Register */
#define REG_SSC_WPMR (*(RwReg*)0x400040E4U) /**< \brief (SSC) Write Protect Mode Register */
#define REG_SSC_WPSR (*(RoReg*)0x400040E8U) /**< \brief (SSC) Write Protect Status Register */

#define REG_SUPC_CR   (*(WoReg*)0x400E1A10U) /**< \brief (SUPC) Supply Controller Control Register */
#define REG_SUPC_SMMR (*(RwReg*)0x400E1A14U) /**< \brief (SUPC) Supply Controller Supply Monitor Mode Register */
#define REG_SUPC_MR   (*(RwReg*)0x400E1A18U) /**< \brief (SUPC) Supply Controller Mode Register */
#define REG_SUPC_WUMR (*(RwReg*)0x400E1A1CU) /**< \brief (SUPC) Supply Controller Wake Up Mode Register */
#define REG_SUPC_WUIR (*(RwReg*)0x400E1A20U) /**< \brief (SUPC) Supply Controller Wake Up Inputs Register */
#define REG_SUPC_SR   (*(RoReg*)0x400E1A24U) /**< \brief (SUPC) Supply Controller Status Register */

#define REG_TC0_CCR0  (*(WoReg*)0x40080000U) /**< \brief (TC0) Channel Control Register (channel = 0) */
#define REG_TC0_CMR0  (*(RwReg*)0x40080004U) /**< \brief (TC0) Channel Mode Register (channel = 0) */
#define REG_TC0_SMMR0 (*(RwReg*)0x40080008U) /**< \brief (TC0) Stepper Motor Mode Register (channel = 0) */
#define REG_TC0_CV0   (*(RoReg*)0x40080010U) /**< \brief (TC0) Counter Value (channel = 0) */
#define REG_TC0_RA0   (*(RwReg*)0x40080014U) /**< \brief (TC0) Register A (channel = 0) */
#define REG_TC0_RB0   (*(RwReg*)0x40080018U) /**< \brief (TC0) Register B (channel = 0) */
#define REG_TC0_RC0   (*(RwReg*)0x4008001CU) /**< \brief (TC0) Register C (channel = 0) */
#define REG_TC0_SR0   (*(RoReg*)0x40080020U) /**< \brief (TC0) Status Register (channel = 0) */
#define REG_TC0_IER0  (*(WoReg*)0x40080024U) /**< \brief (TC0) Interrupt Enable Register (channel = 0) */
#define REG_TC0_IDR0  (*(WoReg*)0x40080028U) /**< \brief (TC0) Interrupt Disable Register (channel = 0) */
#define REG_TC0_IMR0  (*(RoReg*)0x4008002CU) /**< \brief (TC0) Interrupt Mask Register (channel = 0) */
#define REG_TC0_CCR1  (*(WoReg*)0x40080040U) /**< \brief (TC0) Channel Control Register (channel = 1) */
#define REG_TC0_CMR1  (*(RwReg*)0x40080044U) /**< \brief (TC0) Channel Mode Register (channel = 1) */
#define REG_TC0_SMMR1 (*(RwReg*)0x40080048U) /**< \brief (TC0) Stepper Motor Mode Register (channel = 1) */
#define REG_TC0_CV1   (*(RoReg*)0x40080050U) /**< \brief (TC0) Counter Value (channel = 1) */
#define REG_TC0_RA1   (*(RwReg*)0x40080054U) /**< \brief (TC0) Register A (channel = 1) */
#define REG_TC0_RB1   (*(RwReg*)0x40080058U) /**< \brief (TC0) Register B (channel = 1) */
#define REG_TC0_RC1   (*(RwReg*)0x4008005CU) /**< \brief (TC0) Register C (channel = 1) */
#define REG_TC0_SR1   (*(RoReg*)0x40080060U) /**< \brief (TC0) Status Register (channel = 1) */
#define REG_TC0_IER1  (*(WoReg*)0x40080064U) /**< \brief (TC0) Interrupt Enable Register (channel = 1) */
#define REG_TC0_IDR1  (*(WoReg*)0x40080068U) /**< \brief (TC0) Interrupt Disable Register (channel = 1) */
#define REG_TC0_IMR1  (*(RoReg*)0x4008006CU) /**< \brief (TC0) Interrupt Mask Register (channel = 1) */
#define REG_TC0_CCR2  (*(WoReg*)0x40080080U) /**< \brief (TC0) Channel Control Register (channel = 2) */
#define REG_TC0_CMR2  (*(RwReg*)0x40080084U) /**< \brief (TC0) Channel Mode Register (channel = 2) */
#define REG_TC0_SMMR2 (*(RwReg*)0x40080088U) /**< \brief (TC0) Stepper Motor Mode Register (channel = 2) */
#define REG_TC0_CV2   (*(RoReg*)0x40080090U) /**< \brief (TC0) Counter Value (channel = 2) */
#define REG_TC0_RA2   (*(RwReg*)0x40080094U) /**< \brief (TC0) Register A (channel = 2) */
#define REG_TC0_RB2   (*(RwReg*)0x40080098U) /**< \brief (TC0) Register B (channel = 2) */
#define REG_TC0_RC2   (*(RwReg*)0x4008009CU) /**< \brief (TC0) Register C (channel = 2) */
#define REG_TC0_SR2   (*(RoReg*)0x400800A0U) /**< \brief (TC0) Status Register (channel = 2) */
#define REG_TC0_IER2  (*(WoReg*)0x400800A4U) /**< \brief (TC0) Interrupt Enable Register (channel = 2) */
#define REG_TC0_IDR2  (*(WoReg*)0x400800A8U) /**< \brief (TC0) Interrupt Disable Register (channel = 2) */
#define REG_TC0_IMR2  (*(RoReg*)0x400800ACU) /**< \brief (TC0) Interrupt Mask Register (channel = 2) */
#define REG_TC0_BCR   (*(WoReg*)0x400800C0U) /**< \brief (TC0) Block Control Register */
#define REG_TC0_BMR   (*(RwReg*)0x400800C4U) /**< \brief (TC0) Block Mode Register */
#define REG_TC0_QIER  (*(WoReg*)0x400800C8U) /**< \brief (TC0) QDEC Interrupt Enable Register */
#define REG_TC0_QIDR  (*(WoReg*)0x400800CCU) /**< \brief (TC0) QDEC Interrupt Disable Register */
#define REG_TC0_QIMR  (*(RoReg*)0x400800D0U) /**< \brief (TC0) QDEC Interrupt Mask Register */
#define REG_TC0_QISR  (*(RoReg*)0x400800D4U) /**< \brief (TC0) QDEC Interrupt Status Register */
#define REG_TC0_FMR   (*(RwReg*)0x400800D8U) /**< \brief (TC0) Fault Mode Register */
#define REG_TC0_WPMR  (*(RwReg*)0x400800E4U) /**< \brief (TC0) Write Protect Mode Register */

#define REG_TC1_CCR0  (*(WoReg*)0x40084000U) /**< \brief (TC1) Channel Control Register (channel = 0) */
#define REG_TC1_CMR0  (*(RwReg*)0x40084004U) /**< \brief (TC1) Channel Mode Register (channel = 0) */
#define REG_TC1_SMMR0 (*(RwReg*)0x40084008U) /**< \brief (TC1) Stepper Motor Mode Register (channel = 0) */
#define REG_TC1_CV0   (*(RoReg*)0x40084010U) /**< \brief (TC1) Counter Value (channel = 0) */
#define REG_TC1_RA0   (*(RwReg*)0x40084014U) /**< \brief (TC1) Register A (channel = 0) */
#define REG_TC1_RB0   (*(RwReg*)0x40084018U) /**< \brief (TC1) Register B (channel = 0) */
#define REG_TC1_RC0   (*(RwReg*)0x4008401CU) /**< \brief (TC1) Register C (channel = 0) */
#define REG_TC1_SR0   (*(RoReg*)0x40084020U) /**< \brief (TC1) Status Register (channel = 0) */
#define REG_TC1_IER0  (*(WoReg*)0x40084024U) /**< \brief (TC1) Interrupt Enable Register (channel = 0) */
#define REG_TC1_IDR0  (*(WoReg*)0x40084028U) /**< \brief (TC1) Interrupt Disable Register (channel = 0) */
#define REG_TC1_IMR0  (*(RoReg*)0x4008402CU) /**< \brief (TC1) Interrupt Mask Register (channel = 0) */
#define REG_TC1_CCR1  (*(WoReg*)0x40084040U) /**< \brief (TC1) Channel Control Register (channel = 1) */
#define REG_TC1_CMR1  (*(RwReg*)0x40084044U) /**< \brief (TC1) Channel Mode Register (channel = 1) */
#define REG_TC1_SMMR1 (*(RwReg*)0x40084048U) /**< \brief (TC1) Stepper Motor Mode Register (channel = 1) */
#define REG_TC1_CV1   (*(RoReg*)0x40084050U) /**< \brief (TC1) Counter Value (channel = 1) */
#define REG_TC1_RA1   (*(RwReg*)0x40084054U) /**< \brief (TC1) Register A (channel = 1) */
#define REG_TC1_RB1   (*(RwReg*)0x40084058U) /**< \brief (TC1) Register B (channel = 1) */
#define REG_TC1_RC1   (*(RwReg*)0x4008405CU) /**< \brief (TC1) Register C (channel = 1) */
#define REG_TC1_SR1   (*(RoReg*)0x40084060U) /**< \brief (TC1) Status Register (channel = 1) */
#define REG_TC1_IER1  (*(WoReg*)0x40084064U) /**< \brief (TC1) Interrupt Enable Register (channel = 1) */
#define REG_TC1_IDR1  (*(WoReg*)0x40084068U) /**< \brief (TC1) Interrupt Disable Register (channel = 1) */
#define REG_TC1_IMR1  (*(RoReg*)0x4008406CU) /**< \brief (TC1) Interrupt Mask Register (channel = 1) */
#define REG_TC1_CCR2  (*(WoReg*)0x40084080U) /**< \brief (TC1) Channel Control Register (channel = 2) */
#define REG_TC1_CMR2  (*(RwReg*)0x40084084U) /**< \brief (TC1) Channel Mode Register (channel = 2) */
#define REG_TC1_SMMR2 (*(RwReg*)0x40084088U) /**< \brief (TC1) Stepper Motor Mode Register (channel = 2) */
#define REG_TC1_CV2   (*(RoReg*)0x40084090U) /**< \brief (TC1) Counter Value (channel = 2) */
#define REG_TC1_RA2   (*(RwReg*)0x40084094U) /**< \brief (TC1) Register A (channel = 2) */
#define REG_TC1_RB2   (*(RwReg*)0x40084098U) /**< \brief (TC1) Register B (channel = 2) */
#define REG_TC1_RC2   (*(RwReg*)0x4008409CU) /**< \brief (TC1) Register C (channel = 2) */
#define REG_TC1_SR2   (*(RoReg*)0x400840A0U) /**< \brief (TC1) Status Register (channel = 2) */
#define REG_TC1_IER2  (*(WoReg*)0x400840A4U) /**< \brief (TC1) Interrupt Enable Register (channel = 2) */
#define REG_TC1_IDR2  (*(WoReg*)0x400840A8U) /**< \brief (TC1) Interrupt Disable Register (channel = 2) */
#define REG_TC1_IMR2  (*(RoReg*)0x400840ACU) /**< \brief (TC1) Interrupt Mask Register (channel = 2) */
#define REG_TC1_BCR   (*(WoReg*)0x400840C0U) /**< \brief (TC1) Block Control Register */
#define REG_TC1_BMR   (*(RwReg*)0x400840C4U) /**< \brief (TC1) Block Mode Register */
#define REG_TC1_QIER  (*(WoReg*)0x400840C8U) /**< \brief (TC1) QDEC Interrupt Enable Register */
#define REG_TC1_QIDR  (*(WoReg*)0x400840CCU) /**< \brief (TC1) QDEC Interrupt Disable Register */
#define REG_TC1_QIMR  (*(RoReg*)0x400840D0U) /**< \brief (TC1) QDEC Interrupt Mask Register */
#define REG_TC1_QISR  (*(RoReg*)0x400840D4U) /**< \brief (TC1) QDEC Interrupt Status Register */
#define REG_TC1_FMR   (*(RwReg*)0x400840D8U) /**< \brief (TC1) Fault Mode Register */
#define REG_TC1_WPMR  (*(RwReg*)0x400840E4U) /**< \brief (TC1) Write Protect Mode Register */

#define REG_TC2_CCR0  (*(WoReg*)0x40088000U) /**< \brief (TC2) Channel Control Register (channel = 0) */
#define REG_TC2_CMR0  (*(RwReg*)0x40088004U) /**< \brief (TC2) Channel Mode Register (channel = 0) */
#define REG_TC2_SMMR0 (*(RwReg*)0x40088008U) /**< \brief (TC2) Stepper Motor Mode Register (channel = 0) */
#define REG_TC2_CV0   (*(RoReg*)0x40088010U) /**< \brief (TC2) Counter Value (channel = 0) */
#define REG_TC2_RA0   (*(RwReg*)0x40088014U) /**< \brief (TC2) Register A (channel = 0) */
#define REG_TC2_RB0   (*(RwReg*)0x40088018U) /**< \brief (TC2) Register B (channel = 0) */
#define REG_TC2_RC0   (*(RwReg*)0x4008801CU) /**< \brief (TC2) Register C (channel = 0) */
#define REG_TC2_SR0   (*(RoReg*)0x40088020U) /**< \brief (TC2) Status Register (channel = 0) */
#define REG_TC2_IER0  (*(WoReg*)0x40088024U) /**< \brief (TC2) Interrupt Enable Register (channel = 0) */
#define REG_TC2_IDR0  (*(WoReg*)0x40088028U) /**< \brief (TC2) Interrupt Disable Register (channel = 0) */
#define REG_TC2_IMR0  (*(RoReg*)0x4008802CU) /**< \brief (TC2) Interrupt Mask Register (channel = 0) */
#define REG_TC2_CCR1  (*(WoReg*)0x40088040U) /**< \brief (TC2) Channel Control Register (channel = 1) */
#define REG_TC2_CMR1  (*(RwReg*)0x40088044U) /**< \brief (TC2) Channel Mode Register (channel = 1) */
#define REG_TC2_SMMR1 (*(RwReg*)0x40088048U) /**< \brief (TC2) Stepper Motor Mode Register (channel = 1) */
#define REG_TC2_CV1   (*(RoReg*)0x40088050U) /**< \brief (TC2) Counter Value (channel = 1) */
#define REG_TC2_RA1   (*(RwReg*)0x40088054U) /**< \brief (TC2) Register A (channel = 1) */
#define REG_TC2_RB1   (*(RwReg*)0x40088058U) /**< \brief (TC2) Register B (channel = 1) */
#define REG_TC2_RC1   (*(RwReg*)0x4008805CU) /**< \brief (TC2) Register C (channel = 1) */
#define REG_TC2_SR1   (*(RoReg*)0x40088060U) /**< \brief (TC2) Status Register (channel = 1) */
#define REG_TC2_IER1  (*(WoReg*)0x40088064U) /**< \brief (TC2) Interrupt Enable Register (channel = 1) */
#define REG_TC2_IDR1  (*(WoReg*)0x40088068U) /**< \brief (TC2) Interrupt Disable Register (channel = 1) */
#define REG_TC2_IMR1  (*(RoReg*)0x4008806CU) /**< \brief (TC2) Interrupt Mask Register (channel = 1) */
#define REG_TC2_CCR2  (*(WoReg*)0x40088080U) /**< \brief (TC2) Channel Control Register (channel = 2) */
#define REG_TC2_CMR2  (*(RwReg*)0x40088084U) /**< \brief (TC2) Channel Mode Register (channel = 2) */
#define REG_TC2_SMMR2 (*(RwReg*)0x40088088U) /**< \brief (TC2) Stepper Motor Mode Register (channel = 2) */
#define REG_TC2_CV2   (*(RoReg*)0x40088090U) /**< \brief (TC2) Counter Value (channel = 2) */
#define REG_TC2_RA2   (*(RwReg*)0x40088094U) /**< \brief (TC2) Register A (channel = 2) */
#define REG_TC2_RB2   (*(RwReg*)0x40088098U) /**< \brief (TC2) Register B (channel = 2) */
#define REG_TC2_RC2   (*(RwReg*)0x4008809CU) /**< \brief (TC2) Register C (channel = 2) */
#define REG_TC2_SR2   (*(RoReg*)0x400880A0U) /**< \brief (TC2) Status Register (channel = 2) */
#define REG_TC2_IER2  (*(WoReg*)0x400880A4U) /**< \brief (TC2) Interrupt Enable Register (channel = 2) */
#define REG_TC2_IDR2  (*(WoReg*)0x400880A8U) /**< \brief (TC2) Interrupt Disable Register (channel = 2) */
#define REG_TC2_IMR2  (*(RoReg*)0x400880ACU) /**< \brief (TC2) Interrupt Mask Register (channel = 2) */
#define REG_TC2_BCR   (*(WoReg*)0x400880C0U) /**< \brief (TC2) Block Control Register */
#define REG_TC2_BMR   (*(RwReg*)0x400880C4U) /**< \brief (TC2) Block Mode Register */
#define REG_TC2_QIER  (*(WoReg*)0x400880C8U) /**< \brief (TC2) QDEC Interrupt Enable Register */
#define REG_TC2_QIDR  (*(WoReg*)0x400880CCU) /**< \brief (TC2) QDEC Interrupt Disable Register */
#define REG_TC2_QIMR  (*(RoReg*)0x400880D0U) /**< \brief (TC2) QDEC Interrupt Mask Register */
#define REG_TC2_QISR  (*(RoReg*)0x400880D4U) /**< \brief (TC2) QDEC Interrupt Status Register */
#define REG_TC2_FMR   (*(RwReg*)0x400880D8U) /**< \brief (TC2) Fault Mode Register */
#define REG_TC2_WPMR  (*(RwReg*)0x400880E4U) /**< \brief (TC2) Write Protect Mode Register */

#define REG_TRNG_CR    (*(WoReg*)0x400BC000U) /**< \brief (TRNG) Control Register */
#define REG_TRNG_IER   (*(WoReg*)0x400BC010U) /**< \brief (TRNG) Interrupt Enable Register */
#define REG_TRNG_IDR   (*(WoReg*)0x400BC014U) /**< \brief (TRNG) Interrupt Disable Register */
#define REG_TRNG_IMR   (*(RoReg*)0x400BC018U) /**< \brief (TRNG) Interrupt Mask Register */
#define REG_TRNG_ISR   (*(RoReg*)0x400BC01CU) /**< \brief (TRNG) Interrupt Status Register */
#define REG_TRNG_ODATA (*(RoReg*)0x400BC050U) /**< \brief (TRNG) Output Data Register */

#define REG_TWI0_CR   (*(WoReg*)0x4008C000U) /**< \brief (TWI0) Control Register */
#define REG_TWI0_MMR  (*(RwReg*)0x4008C004U) /**< \brief (TWI0) Master Mode Register */
#define REG_TWI0_SMR  (*(RwReg*)0x4008C008U) /**< \brief (TWI0) Slave Mode Register */
#define REG_TWI0_IADR (*(RwReg*)0x4008C00CU) /**< \brief (TWI0) Internal Address Register */
#define REG_TWI0_CWGR (*(RwReg*)0x4008C010U) /**< \brief (TWI0) Clock Waveform Generator Register */
#define REG_TWI0_SR   (*(RoReg*)0x4008C020U) /**< \brief (TWI0) Status Register */
#define REG_TWI0_IER  (*(WoReg*)0x4008C024U) /**< \brief (TWI0) Interrupt Enable Register */
#define REG_TWI0_IDR  (*(WoReg*)0x4008C028U) /**< \brief (TWI0) Interrupt Disable Register */
#define REG_TWI0_IMR  (*(RoReg*)0x4008C02CU) /**< \brief (TWI0) Interrupt Mask Register */
#define REG_TWI0_RHR  (*(RoReg*)0x4008C030U) /**< \brief (TWI0) Receive Holding Register */
#define REG_TWI0_THR  (*(WoReg*)0x4008C034U) /**< \brief (TWI0) Transmit Holding Register */
#define REG_TWI0_RPR  (*(RwReg*)0x4008C100U) /**< \brief (TWI0) Receive Pointer Register */
#define REG_TWI0_RCR  (*(RwReg*)0x4008C104U) /**< \brief (TWI0) Receive Counter Register */
#define REG_TWI0_TPR  (*(RwReg*)0x4008C108U) /**< \brief (TWI0) Transmit Pointer Register */
#define REG_TWI0_TCR  (*(RwReg*)0x4008C10CU) /**< \brief (TWI0) Transmit Counter Register */
#define REG_TWI0_RNPR (*(RwReg*)0x4008C110U) /**< \brief (TWI0) Receive Next Pointer Register */
#define REG_TWI0_RNCR (*(RwReg*)0x4008C114U) /**< \brief (TWI0) Receive Next Counter Register */
#define REG_TWI0_TNPR (*(RwReg*)0x4008C118U) /**< \brief (TWI0) Transmit Next Pointer Register */
#define REG_TWI0_TNCR (*(RwReg*)0x4008C11CU) /**< \brief (TWI0) Transmit Next Counter Register */
#define REG_TWI0_PTCR (*(WoReg*)0x4008C120U) /**< \brief (TWI0) Transfer Control Register */
#define REG_TWI0_PTSR (*(RoReg*)0x4008C124U) /**< \brief (TWI0) Transfer Status Register */

#define REG_TWI1_CR   (*(WoReg*)0x40090000U) /**< \brief (TWI1) Control Register */
#define REG_TWI1_MMR  (*(RwReg*)0x40090004U) /**< \brief (TWI1) Master Mode Register */
#define REG_TWI1_SMR  (*(RwReg*)0x40090008U) /**< \brief (TWI1) Slave Mode Register */
#define REG_TWI1_IADR (*(RwReg*)0x4009000CU) /**< \brief (TWI1) Internal Address Register */
#define REG_TWI1_CWGR (*(RwReg*)0x40090010U) /**< \brief (TWI1) Clock Waveform Generator Register */
#define REG_TWI1_SR   (*(RoReg*)0x40090020U) /**< \brief (TWI1) Status Register */
#define REG_TWI1_IER  (*(WoReg*)0x40090024U) /**< \brief (TWI1) Interrupt Enable Register */
#define REG_TWI1_IDR  (*(WoReg*)0x40090028U) /**< \brief (TWI1) Interrupt Disable Register */
#define REG_TWI1_IMR  (*(RoReg*)0x4009002CU) /**< \brief (TWI1) Interrupt Mask Register */
#define REG_TWI1_RHR  (*(RoReg*)0x40090030U) /**< \brief (TWI1) Receive Holding Register */
#define REG_TWI1_THR  (*(WoReg*)0x40090034U) /**< \brief (TWI1) Transmit Holding Register */
#define REG_TWI1_RPR  (*(RwReg*)0x40090100U) /**< \brief (TWI1) Receive Pointer Register */
#define REG_TWI1_RCR  (*(RwReg*)0x40090104U) /**< \brief (TWI1) Receive Counter Register */
#define REG_TWI1_TPR  (*(RwReg*)0x40090108U) /**< \brief (TWI1) Transmit Pointer Register */
#define REG_TWI1_TCR  (*(RwReg*)0x4009010CU) /**< \brief (TWI1) Transmit Counter Register */
#define REG_TWI1_RNPR (*(RwReg*)0x40090110U) /**< \brief (TWI1) Receive Next Pointer Register */
#define REG_TWI1_RNCR (*(RwReg*)0x40090114U) /**< \brief (TWI1) Receive Next Counter Register */
#define REG_TWI1_TNPR (*(RwReg*)0x40090118U) /**< \brief (TWI1) Transmit Next Pointer Register */
#define REG_TWI1_TNCR (*(RwReg*)0x4009011CU) /**< \brief (TWI1) Transmit Next Counter Register */
#define REG_TWI1_PTCR (*(WoReg*)0x40090120U) /**< \brief (TWI1) Transfer Control Register */
#define REG_TWI1_PTSR (*(RoReg*)0x40090124U) /**< \brief (TWI1) Transfer Status Register */

#define REG_UART_CR   (*(WoReg*)0x400E0800U) /**< \brief (UART) Control Register */
#define REG_UART_MR   (*(RwReg*)0x400E0804U) /**< \brief (UART) Mode Register */
#define REG_UART_IER  (*(WoReg*)0x400E0808U) /**< \brief (UART) Interrupt Enable Register */
#define REG_UART_IDR  (*(WoReg*)0x400E080CU) /**< \brief (UART) Interrupt Disable Register */
#define REG_UART_IMR  (*(RoReg*)0x400E0810U) /**< \brief (UART) Interrupt Mask Register */
#define REG_UART_SR   (*(RoReg*)0x400E0814U) /**< \brief (UART) Status Register */
#define REG_UART_RHR  (*(RoReg*)0x400E0818U) /**< \brief (UART) Receive Holding Register */
#define REG_UART_THR  (*(WoReg*)0x400E081CU) /**< \brief (UART) Transmit Holding Register */
#define REG_UART_BRGR (*(RwReg*)0x400E0820U) /**< \brief (UART) Baud Rate Generator Register */
#define REG_UART_RPR  (*(RwReg*)0x400E0900U) /**< \brief (UART) Receive Pointer Register */
#define REG_UART_RCR  (*(RwReg*)0x400E0904U) /**< \brief (UART) Receive Counter Register */
#define REG_UART_TPR  (*(RwReg*)0x400E0908U) /**< \brief (UART) Transmit Pointer Register */
#define REG_UART_TCR  (*(RwReg*)0x400E090CU) /**< \brief (UART) Transmit Counter Register */
#define REG_UART_RNPR (*(RwReg*)0x400E0910U) /**< \brief (UART) Receive Next Pointer Register */
#define REG_UART_RNCR (*(RwReg*)0x400E0914U) /**< \brief (UART) Receive Next Counter Register */
#define REG_UART_TNPR (*(RwReg*)0x400E0918U) /**< \brief (UART) Transmit Next Pointer Register */
#define REG_UART_TNCR (*(RwReg*)0x400E091CU) /**< \brief (UART) Transmit Next Counter Register */
#define REG_UART_PTCR (*(WoReg*)0x400E0920U) /**< \brief (UART) Transfer Control Register */
#define REG_UART_PTSR (*(RoReg*)0x400E0924U) /**< \brief (UART) Transfer Status Register */

#define REG_UOTGHS_DEVCTRL        (*(RwReg*)0x400AC000U) /**< \brief (UOTGHS) Device General Control Register */
#define REG_UOTGHS_DEVISR         (*(RoReg*)0x400AC004U) /**< \brief (UOTGHS) Device Global Interrupt Status Register */
#define REG_UOTGHS_DEVICR         (*(WoReg*)0x400AC008U) /**< \brief (UOTGHS) Device Global Interrupt Clear Register */
#define REG_UOTGHS_DEVIFR         (*(WoReg*)0x400AC00CU) /**< \brief (UOTGHS) Device Global Interrupt Set Register */
#define REG_UOTGHS_DEVIMR         (*(RoReg*)0x400AC010U) /**< \brief (UOTGHS) Device Global Interrupt Mask Register */
#define REG_UOTGHS_DEVIDR         (*(WoReg*)0x400AC014U) /**< \brief (UOTGHS) Device Global Interrupt Disable Register */
#define REG_UOTGHS_DEVIER         (*(WoReg*)0x400AC018U) /**< \brief (UOTGHS) Device Global Interrupt Enable Register */
#define REG_UOTGHS_DEVEPT         (*(RwReg*)0x400AC01CU) /**< \brief (UOTGHS) Device Endpoint Register */
#define REG_UOTGHS_DEVFNUM        (*(RoReg*)0x400AC020U) /**< \brief (UOTGHS) Device Frame Number Register */
#define REG_UOTGHS_DEVEPTCFG      (*(RwReg*)0x400AC100U) /**< \brief (UOTGHS) Device Endpoint Configuration Register (n = 0) */
#define REG_UOTGHS_DEVEPTISR      (*(RoReg*)0x400AC130U) /**< \brief (UOTGHS) Device Endpoint Status Register (n = 0) */
#define REG_UOTGHS_DEVEPTICR      (*(WoReg*)0x400AC160U) /**< \brief (UOTGHS) Device Endpoint Clear Register (n = 0) */
#define REG_UOTGHS_DEVEPTIFR      (*(WoReg*)0x400AC190U) /**< \brief (UOTGHS) Device Endpoint Set Register (n = 0) */
#define REG_UOTGHS_DEVEPTIMR      (*(RoReg*)0x400AC1C0U) /**< \brief (UOTGHS) Device Endpoint Mask Register (n = 0) */
#define REG_UOTGHS_DEVEPTIER      (*(WoReg*)0x400AC1F0U) /**< \brief (UOTGHS) Device Endpoint Enable Register (n = 0) */
#define REG_UOTGHS_DEVEPTIDR      (*(WoReg*)0x400AC220U) /**< \brief (UOTGHS) Device Endpoint Disable Register (n = 0) */
#define REG_UOTGHS_DEVDMANXTDSC1  (*(RwReg*)0x400AC310U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 1) */
#define REG_UOTGHS_DEVDMAADDRESS1 (*(RwReg*)0x400AC314U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 1) */
#define REG_UOTGHS_DEVDMACONTROL1 (*(RwReg*)0x400AC318U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 1) */
#define REG_UOTGHS_DEVDMASTATUS1  (*(RwReg*)0x400AC31CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 1) */
#define REG_UOTGHS_DEVDMANXTDSC2  (*(RwReg*)0x400AC320U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 2) */
#define REG_UOTGHS_DEVDMAADDRESS2 (*(RwReg*)0x400AC324U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 2) */
#define REG_UOTGHS_DEVDMACONTROL2 (*(RwReg*)0x400AC328U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 2) */
#define REG_UOTGHS_DEVDMASTATUS2  (*(RwReg*)0x400AC32CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 2) */
#define REG_UOTGHS_DEVDMANXTDSC3  (*(RwReg*)0x400AC330U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 3) */
#define REG_UOTGHS_DEVDMAADDRESS3 (*(RwReg*)0x400AC334U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 3) */
#define REG_UOTGHS_DEVDMACONTROL3 (*(RwReg*)0x400AC338U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 3) */
#define REG_UOTGHS_DEVDMASTATUS3  (*(RwReg*)0x400AC33CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 3) */
#define REG_UOTGHS_DEVDMANXTDSC4  (*(RwReg*)0x400AC340U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 4) */
#define REG_UOTGHS_DEVDMAADDRESS4 (*(RwReg*)0x400AC344U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 4) */
#define REG_UOTGHS_DEVDMACONTROL4 (*(RwReg*)0x400AC348U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 4) */
#define REG_UOTGHS_DEVDMASTATUS4  (*(RwReg*)0x400AC34CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 4) */
#define REG_UOTGHS_DEVDMANXTDSC5  (*(RwReg*)0x400AC350U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 5) */
#define REG_UOTGHS_DEVDMAADDRESS5 (*(RwReg*)0x400AC354U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 5) */
#define REG_UOTGHS_DEVDMACONTROL5 (*(RwReg*)0x400AC358U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 5) */
#define REG_UOTGHS_DEVDMASTATUS5  (*(RwReg*)0x400AC35CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 5) */
#define REG_UOTGHS_DEVDMANXTDSC6  (*(RwReg*)0x400AC360U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 6) */
#define REG_UOTGHS_DEVDMAADDRESS6 (*(RwReg*)0x400AC364U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 6) */
#define REG_UOTGHS_DEVDMACONTROL6 (*(RwReg*)0x400AC368U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 6) */
#define REG_UOTGHS_DEVDMASTATUS6  (*(RwReg*)0x400AC36CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 6) */
#define REG_UOTGHS_DEVDMANXTDSC7  (*(RwReg*)0x400AC370U) /**< \brief (UOTGHS) Device DMA Channel Next Descriptor Address Register (n = 7) */
#define REG_UOTGHS_DEVDMAADDRESS7 (*(RwReg*)0x400AC374U) /**< \brief (UOTGHS) Device DMA Channel Address Register (n = 7) */
#define REG_UOTGHS_DEVDMACONTROL7 (*(RwReg*)0x400AC378U) /**< \brief (UOTGHS) Device DMA Channel Control Register (n = 7) */
#define REG_UOTGHS_DEVDMASTATUS7  (*(RwReg*)0x400AC37CU) /**< \brief (UOTGHS) Device DMA Channel Status Register (n = 7) */
#define REG_UOTGHS_HSTCTRL        (*(RwReg*)0x400AC400U) /**< \brief (UOTGHS) Host General Control Register */
#define REG_UOTGHS_HSTISR         (*(RoReg*)0x400AC404U) /**< \brief (UOTGHS) Host Global Interrupt Status Register */
#define REG_UOTGHS_HSTICR         (*(WoReg*)0x400AC408U) /**< \brief (UOTGHS) Host Global Interrupt Clear Register */
#define REG_UOTGHS_HSTIFR         (*(WoReg*)0x400AC40CU) /**< \brief (UOTGHS) Host Global Interrupt Set Register */
#define REG_UOTGHS_HSTIMR         (*(RoReg*)0x400AC410U) /**< \brief (UOTGHS) Host Global Interrupt Mask Register */
#define REG_UOTGHS_HSTIDR         (*(WoReg*)0x400AC414U) /**< \brief (UOTGHS) Host Global Interrupt Disable Register */
#define REG_UOTGHS_HSTIER         (*(WoReg*)0x400AC418U) /**< \brief (UOTGHS) Host Global Interrupt Enable Register */
#define REG_UOTGHS_HSTPIP         (*(RwReg*)0x400AC41CU) /**< \brief (UOTGHS) Host Pipe Register */
#define REG_UOTGHS_HSTFNUM        (*(RwReg*)0x400AC420U) /**< \brief (UOTGHS) Host Frame Number Register */
#define REG_UOTGHS_HSTADDR1       (*(RwReg*)0x400AC424U) /**< \brief (UOTGHS) Host Address 1 Register */
#define REG_UOTGHS_HSTADDR2       (*(RwReg*)0x400AC428U) /**< \brief (UOTGHS) Host Address 2 Register */
#define REG_UOTGHS_HSTADDR3       (*(RwReg*)0x400AC42CU) /**< \brief (UOTGHS) Host Address 3 Register */
#define REG_UOTGHS_HSTPIPCFG      (*(RwReg*)0x400AC500U) /**< \brief (UOTGHS) Host Pipe Configuration Register (n = 0) */
#define REG_UOTGHS_HSTPIPISR      (*(RoReg*)0x400AC530U) /**< \brief (UOTGHS) Host Pipe Status Register (n = 0) */
#define REG_UOTGHS_HSTPIPICR      (*(WoReg*)0x400AC560U) /**< \brief (UOTGHS) Host Pipe Clear Register (n = 0) */
#define REG_UOTGHS_HSTPIPIFR      (*(WoReg*)0x400AC590U) /**< \brief (UOTGHS) Host Pipe Set Register (n = 0) */
#define REG_UOTGHS_HSTPIPIMR      (*(RoReg*)0x400AC5C0U) /**< \brief (UOTGHS) Host Pipe Mask Register (n = 0) */
#define REG_UOTGHS_HSTPIPIER      (*(WoReg*)0x400AC5F0U) /**< \brief (UOTGHS) Host Pipe Enable Register (n = 0) */
#define REG_UOTGHS_HSTPIPIDR      (*(WoReg*)0x400AC620U) /**< \brief (UOTGHS) Host Pipe Disable Register (n = 0) */
#define REG_UOTGHS_HSTPIPINRQ     (*(RwReg*)0x400AC650U) /**< \brief (UOTGHS) Host Pipe IN Request Register (n = 0) */
#define REG_UOTGHS_HSTPIPERR      (*(RwReg*)0x400AC680U) /**< \brief (UOTGHS) Host Pipe Error Register (n = 0) */
#define REG_UOTGHS_HSTDMANXTDSC1  (*(RwReg*)0x400AC710U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 1) */
#define REG_UOTGHS_HSTDMAADDRESS1 (*(RwReg*)0x400AC714U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 1) */
#define REG_UOTGHS_HSTDMACONTROL1 (*(RwReg*)0x400AC718U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 1) */
#define REG_UOTGHS_HSTDMASTATUS1  (*(RwReg*)0x400AC71CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 1) */
#define REG_UOTGHS_HSTDMANXTDSC2  (*(RwReg*)0x400AC720U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 2) */
#define REG_UOTGHS_HSTDMAADDRESS2 (*(RwReg*)0x400AC724U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 2) */
#define REG_UOTGHS_HSTDMACONTROL2 (*(RwReg*)0x400AC728U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 2) */
#define REG_UOTGHS_HSTDMASTATUS2  (*(RwReg*)0x400AC72CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 2) */
#define REG_UOTGHS_HSTDMANXTDSC3  (*(RwReg*)0x400AC730U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 3) */
#define REG_UOTGHS_HSTDMAADDRESS3 (*(RwReg*)0x400AC734U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 3) */
#define REG_UOTGHS_HSTDMACONTROL3 (*(RwReg*)0x400AC738U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 3) */
#define REG_UOTGHS_HSTDMASTATUS3  (*(RwReg*)0x400AC73CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 3) */
#define REG_UOTGHS_HSTDMANXTDSC4  (*(RwReg*)0x400AC740U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 4) */
#define REG_UOTGHS_HSTDMAADDRESS4 (*(RwReg*)0x400AC744U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 4) */
#define REG_UOTGHS_HSTDMACONTROL4 (*(RwReg*)0x400AC748U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 4) */
#define REG_UOTGHS_HSTDMASTATUS4  (*(RwReg*)0x400AC74CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 4) */
#define REG_UOTGHS_HSTDMANXTDSC5  (*(RwReg*)0x400AC750U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 5) */
#define REG_UOTGHS_HSTDMAADDRESS5 (*(RwReg*)0x400AC754U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 5) */
#define REG_UOTGHS_HSTDMACONTROL5 (*(RwReg*)0x400AC758U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 5) */
#define REG_UOTGHS_HSTDMASTATUS5  (*(RwReg*)0x400AC75CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 5) */
#define REG_UOTGHS_HSTDMANXTDSC6  (*(RwReg*)0x400AC760U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 6) */
#define REG_UOTGHS_HSTDMAADDRESS6 (*(RwReg*)0x400AC764U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 6) */
#define REG_UOTGHS_HSTDMACONTROL6 (*(RwReg*)0x400AC768U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 6) */
#define REG_UOTGHS_HSTDMASTATUS6  (*(RwReg*)0x400AC76CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 6) */
#define REG_UOTGHS_HSTDMANXTDSC7  (*(RwReg*)0x400AC770U) /**< \brief (UOTGHS) Host DMA Channel Next Descriptor Address Register (n = 7) */
#define REG_UOTGHS_HSTDMAADDRESS7 (*(RwReg*)0x400AC774U) /**< \brief (UOTGHS) Host DMA Channel Address Register (n = 7) */
#define REG_UOTGHS_HSTDMACONTROL7 (*(RwReg*)0x400AC778U) /**< \brief (UOTGHS) Host DMA Channel Control Register (n = 7) */
#define REG_UOTGHS_HSTDMASTATUS7  (*(RwReg*)0x400AC77CU) /**< \brief (UOTGHS) Host DMA Channel Status Register (n = 7) */
#define REG_UOTGHS_CTRL           (*(RwReg*)0x400AC800U) /**< \brief (UOTGHS) General Control Register */
#define REG_UOTGHS_SR             (*(RoReg*)0x400AC804U) /**< \brief (UOTGHS) General Status Register */
#define REG_UOTGHS_SCR            (*(WoReg*)0x400AC808U) /**< \brief (UOTGHS) General Status Clear Register */
#define REG_UOTGHS_SFR            (*(WoReg*)0x400AC80CU) /**< \brief (UOTGHS) General Status Set Register */
#define REG_UOTGHS_FSM            (*(RoReg*)0x400AC82CU) /**< \brief (UOTGHS) General Finite State Machine Register */

#define REG_USART0_CR (*(WoReg*)0x40098000U) /**< \brief (USART0) Control Register */
#define REG_USART0_MR (*(RwReg*)0x40098004U) /**< \brief (USART0) Mode Register */
#define REG_USART0_IER (*(WoReg*)0x40098008U) /**< \brief (USART0) Interrupt Enable Register */
#define REG_USART0_IDR (*(WoReg*)0x4009800CU) /**< \brief (USART0) Interrupt Disable Register */
#define REG_USART0_IMR (*(RoReg*)0x40098010U) /**< \brief (USART0) Interrupt Mask Register */
#define REG_USART0_CSR (*(RoReg*)0x40098014U) /**< \brief (USART0) Channel Status Register */
#define REG_USART0_RHR (*(RoReg*)0x40098018U) /**< \brief (USART0) Receiver Holding Register */
#define REG_USART0_THR (*(WoReg*)0x4009801CU) /**< \brief (USART0) Transmitter Holding Register */
#define REG_USART0_BRGR (*(RwReg*)0x40098020U) /**< \brief (USART0) Baud Rate Generator Register */
#define REG_USART0_RTOR (*(RwReg*)0x40098024U) /**< \brief (USART0) Receiver Time-out Register */
#define REG_USART0_TTGR (*(RwReg*)0x40098028U) /**< \brief (USART0) Transmitter Timeguard Register */
#define REG_USART0_FIDI (*(RwReg*)0x40098040U) /**< \brief (USART0) FI DI Ratio Register */
#define REG_USART0_NER (*(RoReg*)0x40098044U) /**< \brief (USART0) Number of Errors Register */
#define REG_USART0_IF (*(RwReg*)0x4009804CU) /**< \brief (USART0) IrDA Filter Register */
#define REG_USART0_MAN (*(RwReg*)0x40098050U) /**< \brief (USART0) Manchester Encoder Decoder Register */
#define REG_USART0_LINMR (*(RwReg*)0x40098054U) /**< \brief (USART0) LIN Mode Register */
#define REG_USART0_LINIR (*(RwReg*)0x40098058U) /**< \brief (USART0) LIN Identifier Register */
#define REG_USART0_WPMR (*(RwReg*)0x400980E4U) /**< \brief (USART0) Write Protect Mode Register */
#define REG_USART0_WPSR (*(RoReg*)0x400980E8U) /**< \brief (USART0) Write Protect Status Register */
#define REG_USART0_RPR (*(RwReg*)0x40098100U) /**< \brief (USART0) Receive Pointer Register */
#define REG_USART0_RCR (*(RwReg*)0x40098104U) /**< \brief (USART0) Receive Counter Register */
#define REG_USART0_TPR (*(RwReg*)0x40098108U) /**< \brief (USART0) Transmit Pointer Register */
#define REG_USART0_TCR (*(RwReg*)0x4009810CU) /**< \brief (USART0) Transmit Counter Register */
#define REG_USART0_RNPR (*(RwReg*)0x40098110U) /**< \brief (USART0) Receive Next Pointer Register */
#define REG_USART0_RNCR (*(RwReg*)0x40098114U) /**< \brief (USART0) Receive Next Counter Register */
#define REG_USART0_TNPR (*(RwReg*)0x40098118U) /**< \brief (USART0) Transmit Next Pointer Register */
#define REG_USART0_TNCR (*(RwReg*)0x4009811CU) /**< \brief (USART0) Transmit Next Counter Register */
#define REG_USART0_PTCR (*(WoReg*)0x40098120U) /**< \brief (USART0) Transfer Control Register */
#define REG_USART0_PTSR (*(RoReg*)0x40098124U) /**< \brief (USART0) Transfer Status Register */

#define REG_USART1_CR (*(WoReg*)0x4009C000U) /**< \brief (USART1) Control Register */
#define REG_USART1_MR (*(RwReg*)0x4009C004U) /**< \brief (USART1) Mode Register */
#define REG_USART1_IER (*(WoReg*)0x4009C008U) /**< \brief (USART1) Interrupt Enable Register */
#define REG_USART1_IDR (*(WoReg*)0x4009C00CU) /**< \brief (USART1) Interrupt Disable Register */
#define REG_USART1_IMR (*(RoReg*)0x4009C010U) /**< \brief (USART1) Interrupt Mask Register */
#define REG_USART1_CSR (*(RoReg*)0x4009C014U) /**< \brief (USART1) Channel Status Register */
#define REG_USART1_RHR (*(RoReg*)0x4009C018U) /**< \brief (USART1) Receiver Holding Register */
#define REG_USART1_THR (*(WoReg*)0x4009C01CU) /**< \brief (USART1) Transmitter Holding Register */
#define REG_USART1_BRGR (*(RwReg*)0x4009C020U) /**< \brief (USART1) Baud Rate Generator Register */
#define REG_USART1_RTOR (*(RwReg*)0x4009C024U) /**< \brief (USART1) Receiver Time-out Register */
#define REG_USART1_TTGR (*(RwReg*)0x4009C028U) /**< \brief (USART1) Transmitter Timeguard Register */
#define REG_USART1_FIDI (*(RwReg*)0x4009C040U) /**< \brief (USART1) FI DI Ratio Register */
#define REG_USART1_NER (*(RoReg*)0x4009C044U) /**< \brief (USART1) Number of Errors Register */
#define REG_USART1_IF (*(RwReg*)0x4009C04CU) /**< \brief (USART1) IrDA Filter Register */
#define REG_USART1_MAN (*(RwReg*)0x4009C050U) /**< \brief (USART1) Manchester Encoder Decoder Register */
#define REG_USART1_LINMR (*(RwReg*)0x4009C054U) /**< \brief (USART1) LIN Mode Register */
#define REG_USART1_LINIR (*(RwReg*)0x4009C058U) /**< \brief (USART1) LIN Identifier Register */
#define REG_USART1_WPMR (*(RwReg*)0x4009C0E4U) /**< \brief (USART1) Write Protect Mode Register */
#define REG_USART1_WPSR (*(RoReg*)0x4009C0E8U) /**< \brief (USART1) Write Protect Status Register */
#define REG_USART1_RPR (*(RwReg*)0x4009C100U) /**< \brief (USART1) Receive Pointer Register */
#define REG_USART1_RCR (*(RwReg*)0x4009C104U) /**< \brief (USART1) Receive Counter Register */
#define REG_USART1_TPR (*(RwReg*)0x4009C108U) /**< \brief (USART1) Transmit Pointer Register */
#define REG_USART1_TCR (*(RwReg*)0x4009C10CU) /**< \brief (USART1) Transmit Counter Register */
#define REG_USART1_RNPR (*(RwReg*)0x4009C110U) /**< \brief (USART1) Receive Next Pointer Register */
#define REG_USART1_RNCR (*(RwReg*)0x4009C114U) /**< \brief (USART1) Receive Next Counter Register */
#define REG_USART1_TNPR (*(RwReg*)0x4009C118U) /**< \brief (USART1) Transmit Next Pointer Register */
#define REG_USART1_TNCR (*(RwReg*)0x4009C11CU) /**< \brief (USART1) Transmit Next Counter Register */
#define REG_USART1_PTCR (*(WoReg*)0x4009C120U) /**< \brief (USART1) Transfer Control Register */
#define REG_USART1_PTSR (*(RoReg*)0x4009C124U) /**< \brief (USART1) Transfer Status Register */

#define REG_USART2_CR (*(WoReg*)0x400A0000U) /**< \brief (USART2) Control Register */
#define REG_USART2_MR (*(RwReg*)0x400A0004U) /**< \brief (USART2) Mode Register */
#define REG_USART2_IER (*(WoReg*)0x400A0008U) /**< \brief (USART2) Interrupt Enable Register */
#define REG_USART2_IDR (*(WoReg*)0x400A000CU) /**< \brief (USART2) Interrupt Disable Register */
#define REG_USART2_IMR (*(RoReg*)0x400A0010U) /**< \brief (USART2) Interrupt Mask Register */
#define REG_USART2_CSR (*(RoReg*)0x400A0014U) /**< \brief (USART2) Channel Status Register */
#define REG_USART2_RHR (*(RoReg*)0x400A0018U) /**< \brief (USART2) Receiver Holding Register */
#define REG_USART2_THR (*(WoReg*)0x400A001CU) /**< \brief (USART2) Transmitter Holding Register */
#define REG_USART2_BRGR (*(RwReg*)0x400A0020U) /**< \brief (USART2) Baud Rate Generator Register */
#define REG_USART2_RTOR (*(RwReg*)0x400A0024U) /**< \brief (USART2) Receiver Time-out Register */
#define REG_USART2_TTGR (*(RwReg*)0x400A0028U) /**< \brief (USART2) Transmitter Timeguard Register */
#define REG_USART2_FIDI (*(RwReg*)0x400A0040U) /**< \brief (USART2) FI DI Ratio Register */
#define REG_USART2_NER (*(RoReg*)0x400A0044U) /**< \brief (USART2) Number of Errors Register */
#define REG_USART2_IF (*(RwReg*)0x400A004CU) /**< \brief (USART2) IrDA Filter Register */
#define REG_USART2_MAN (*(RwReg*)0x400A0050U) /**< \brief (USART2) Manchester Encoder Decoder Register */
#define REG_USART2_LINMR (*(RwReg*)0x400A0054U) /**< \brief (USART2) LIN Mode Register */
#define REG_USART2_LINIR (*(RwReg*)0x400A0058U) /**< \brief (USART2) LIN Identifier Register */
#define REG_USART2_WPMR (*(RwReg*)0x400A00E4U) /**< \brief (USART2) Write Protect Mode Register */
#define REG_USART2_WPSR (*(RoReg*)0x400A00E8U) /**< \brief (USART2) Write Protect Status Register */
#define REG_USART2_RPR (*(RwReg*)0x400A0100U) /**< \brief (USART2) Receive Pointer Register */
#define REG_USART2_RCR (*(RwReg*)0x400A0104U) /**< \brief (USART2) Receive Counter Register */
#define REG_USART2_TPR (*(RwReg*)0x400A0108U) /**< \brief (USART2) Transmit Pointer Register */
#define REG_USART2_TCR (*(RwReg*)0x400A010CU) /**< \brief (USART2) Transmit Counter Register */
#define REG_USART2_RNPR (*(RwReg*)0x400A0110U) /**< \brief (USART2) Receive Next Pointer Register */
#define REG_USART2_RNCR (*(RwReg*)0x400A0114U) /**< \brief (USART2) Receive Next Counter Register */
#define REG_USART2_TNPR (*(RwReg*)0x400A0118U) /**< \brief (USART2) Transmit Next Pointer Register */
#define REG_USART2_TNCR (*(RwReg*)0x400A011CU) /**< \brief (USART2) Transmit Next Counter Register */
#define REG_USART2_PTCR (*(WoReg*)0x400A0120U) /**< \brief (USART2) Transfer Control Register */
#define REG_USART2_PTSR (*(RoReg*)0x400A0124U) /**< \brief (USART2) Transfer Status Register */

#define REG_USART3_CR (*(WoReg*)0x400A4000U) /**< \brief (USART3) Control Register */
#define REG_USART3_MR (*(RwReg*)0x400A4004U) /**< \brief (USART3) Mode Register */
#define REG_USART3_IER (*(WoReg*)0x400A4008U) /**< \brief (USART3) Interrupt Enable Register */
#define REG_USART3_IDR (*(WoReg*)0x400A400CU) /**< \brief (USART3) Interrupt Disable Register */
#define REG_USART3_IMR (*(RoReg*)0x400A4010U) /**< \brief (USART3) Interrupt Mask Register */
#define REG_USART3_CSR (*(RoReg*)0x400A4014U) /**< \brief (USART3) Channel Status Register */
#define REG_USART3_RHR (*(RoReg*)0x400A4018U) /**< \brief (USART3) Receiver Holding Register */
#define REG_USART3_THR (*(WoReg*)0x400A401CU) /**< \brief (USART3) Transmitter Holding Register */
#define REG_USART3_BRGR (*(RwReg*)0x400A4020U) /**< \brief (USART3) Baud Rate Generator Register */
#define REG_USART3_RTOR (*(RwReg*)0x400A4024U) /**< \brief (USART3) Receiver Time-out Register */
#define REG_USART3_TTGR (*(RwReg*)0x400A4028U) /**< \brief (USART3) Transmitter Timeguard Register */
#define REG_USART3_FIDI (*(RwReg*)0x400A4040U) /**< \brief (USART3) FI DI Ratio Register */
#define REG_USART3_NER (*(RoReg*)0x400A4044U) /**< \brief (USART3) Number of Errors Register */
#define REG_USART3_IF (*(RwReg*)0x400A404CU) /**< \brief (USART3) IrDA Filter Register */
#define REG_USART3_MAN (*(RwReg*)0x400A4050U) /**< \brief (USART3) Manchester Encoder Decoder Register */
#define REG_USART3_LINMR (*(RwReg*)0x400A4054U) /**< \brief (USART3) LIN Mode Register */
#define REG_USART3_LINIR (*(RwReg*)0x400A4058U) /**< \brief (USART3) LIN Identifier Register */
#define REG_USART3_WPMR (*(RwReg*)0x400A40E4U) /**< \brief (USART3) Write Protect Mode Register */
#define REG_USART3_WPSR (*(RoReg*)0x400A40E8U) /**< \brief (USART3) Write Protect Status Register */
#define REG_USART3_RPR (*(RwReg*)0x400A4100U) /**< \brief (USART3) Receive Pointer Register */
#define REG_USART3_RCR (*(RwReg*)0x400A4104U) /**< \brief (USART3) Receive Counter Register */
#define REG_USART3_TPR (*(RwReg*)0x400A4108U) /**< \brief (USART3) Transmit Pointer Register */
#define REG_USART3_TCR (*(RwReg*)0x400A410CU) /**< \brief (USART3) Transmit Counter Register */
#define REG_USART3_RNPR (*(RwReg*)0x400A4110U) /**< \brief (USART3) Receive Next Pointer Register */
#define REG_USART3_RNCR (*(RwReg*)0x400A4114U) /**< \brief (USART3) Receive Next Counter Register */
#define REG_USART3_TNPR (*(RwReg*)0x400A4118U) /**< \brief (USART3) Transmit Next Pointer Register */
#define REG_USART3_TNCR (*(RwReg*)0x400A411CU) /**< \brief (USART3) Transmit Next Counter Register */
#define REG_USART3_PTCR (*(WoReg*)0x400A4120U) /**< \brief (USART3) Transfer Control Register */
#define REG_USART3_PTSR (*(RoReg*)0x400A4124U) /**< \brief (USART3) Transfer Status Register */

#define REG_WDT_CR (*(WoReg*)0x400E1A50U) /**< \brief (WDT) Control Register */
#define REG_WDT_MR (*(RwReg*)0x400E1A54U) /**< \brief (WDT) Mode Register */
#define REG_WDT_SR (*(RoReg*)0x400E1A58U) /**< \brief (WDT) Status Register */
/*@}*/

/* ************************************************************************** */
/*   PERIPHERAL ID DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_id Peripheral Ids Definitions */
/*@{*/

#define ID_SUPC   ( 0) /**< \brief Supply Controller (SUPC) */
#define ID_RSTC   ( 1) /**< \brief Reset Controller (RSTC) */
#define ID_RTC    ( 2) /**< \brief Real Time Clock (RTC) */
#define ID_RTT    ( 3) /**< \brief Real Time Timer (RTT) */
#define ID_WDT    ( 4) /**< \brief Watchdog Timer (WDT) */
#define ID_PMC    ( 5) /**< \brief Power Management Controller (PMC) */
#define ID_EFC0   ( 6) /**< \brief Enhanced Flash Controller 0 (EFC0) */
#define ID_EFC1   ( 7) /**< \brief Enhanced Flash Controller 1 (EFC1) */
#define ID_UART   ( 8) /**< \brief Universal Asynchronous Receiver Transceiver (UART) */
#define ID_SMC    ( 9) /**< \brief Static Memory Controller (SMC) */
#define ID_PIOA   (11) /**< \brief Parallel I/O Controller A, (PIOA) */
#define ID_PIOB   (12) /**< \brief Parallel I/O Controller B (PIOB) */
#define ID_PIOC   (13) /**< \brief Parallel I/O Controller C (PIOC) */
#define ID_PIOD   (14) /**< \brief Parallel I/O Controller D (PIOD) */
#define ID_USART0 (17) /**< \brief USART 0 (USART0) */
#define ID_USART1 (18) /**< \brief USART 1 (USART1) */
#define ID_USART2 (19) /**< \brief USART 2 (USART2) */
#define ID_USART3 (20) /**< \brief USART 3 (USART3) */
#define ID_HSMCI  (21) /**< \brief Multimedia Card Interface (HSMCI) */
#define ID_TWI0   (22) /**< \brief Two-Wire Interface 0 (TWI0) */
#define ID_TWI1   (23) /**< \brief Two-Wire Interface 1 (TWI1) */
#define ID_SPI0   (24) /**< \brief Serial Peripheral Interface (SPI0) */
#define ID_SSC    (26) /**< \brief Synchronous Serial Controller (SSC) */
#define ID_TC0    (27) /**< \brief Timer Counter 0 (TC0) */
#define ID_TC1    (28) /**< \brief Timer Counter 1 (TC1) */
#define ID_TC2    (29) /**< \brief Timer Counter 2 (TC2) */
#define ID_TC3    (30) /**< \brief Timer Counter 3 (TC3) */
#define ID_TC4    (31) /**< \brief Timer Counter 4 (TC4) */
#define ID_TC5    (32) /**< \brief Timer Counter 5 (TC5) */
#define ID_TC6    (33) /**< \brief Timer Counter 6 (TC6) */
#define ID_TC7    (34) /**< \brief Timer Counter 7 (TC7) */
#define ID_TC8    (35) /**< \brief Timer Counter 8 (TC8) */
#define ID_PWM    (36) /**< \brief Pulse Width Modulation Controller (PWM) */
#define ID_ADC    (37) /**< \brief ADC Controller (ADC) */
#define ID_DACC   (38) /**< \brief DAC Controller (DACC) */
#define ID_DMAC   (39) /**< \brief DMA Controller (DMAC) */
#define ID_UOTGHS (40) /**< \brief USB OTG High Speed (UOTGHS) */
#define ID_TRNG   (41) /**< \brief True Random Number Generator (TRNG) */
#define ID_EMAC   (42) /**< \brief Ethernet MAC (EMAC) */
#define ID_CAN0   (43) /**< \brief CAN Controller 0 (CAN0) */
#define ID_CAN1   (44) /**< \brief CAN Controller 1 (CAN1) */

#define ID_PERIPH_COUNT (45) /**< \brief Number of peripheral IDs */
/*@}*/

/* ************************************************************************** */
/*   BASE ADDRESS DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_base Peripheral Base Address Definitions */
/*@{*/

#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#define HSMCI      (0x40000000U) /**< \brief (HSMCI     ) Base Address */
#define SSC        (0x40004000U) /**< \brief (SSC       ) Base Address */
#define SPI0       (0x40008000U) /**< \brief (SPI0      ) Base Address */
#define TC0        (0x40080000U) /**< \brief (TC0       ) Base Address */
#define TC1        (0x40084000U) /**< \brief (TC1       ) Base Address */
#define TC2        (0x40088000U) /**< \brief (TC2       ) Base Address */
#define TWI0       (0x4008C000U) /**< \brief (TWI0      ) Base Address */
#define PDC_TWI0   (0x4008C100U) /**< \brief (PDC_TWI0  ) Base Address */
#define TWI1       (0x40090000U) /**< \brief (TWI1      ) Base Address */
#define PDC_TWI1   (0x40090100U) /**< \brief (PDC_TWI1  ) Base Address */
#define PWM        (0x40094000U) /**< \brief (PWM       ) Base Address */
#define PDC_PWM    (0x40094100U) /**< \brief (PDC_PWM   ) Base Address */
#define USART0     (0x40098000U) /**< \brief (USART0    ) Base Address */
#define PDC_USART0 (0x40098100U) /**< \brief (PDC_USART0) Base Address */
#define USART1     (0x4009C000U) /**< \brief (USART1    ) Base Address */
#define PDC_USART1 (0x4009C100U) /**< \brief (PDC_USART1) Base Address */
#define USART2     (0x400A0000U) /**< \brief (USART2    ) Base Address */
#define PDC_USART2 (0x400A0100U) /**< \brief (PDC_USART2) Base Address */
#define USART3     (0x400A4000U) /**< \brief (USART3    ) Base Address */
#define PDC_USART3 (0x400A4100U) /**< \brief (PDC_USART3) Base Address */
#define UOTGHS     (0x400AC000U) /**< \brief (UOTGHS    ) Base Address */
#define EMAC       (0x400B0000U) /**< \brief (EMAC      ) Base Address */
#define CAN0       (0x400B4000U) /**< \brief (CAN0      ) Base Address */
#define CAN1       (0x400B8000U) /**< \brief (CAN1      ) Base Address */
#define TRNG       (0x400BC000U) /**< \brief (TRNG      ) Base Address */
#define ADC        (0x400C0000U) /**< \brief (ADC       ) Base Address */
#define PDC_ADC    (0x400C0100U) /**< \brief (PDC_ADC   ) Base Address */
#define DMAC       (0x400C4000U) /**< \brief (DMAC      ) Base Address */
#define DACC       (0x400C8000U) /**< \brief (DACC      ) Base Address */
#define PDC_DACC   (0x400C8100U) /**< \brief (PDC_DACC  ) Base Address */
#define SMC        (0x400E0000U) /**< \brief (SMC       ) Base Address */
#define MATRIX     (0x400E0400U) /**< \brief (MATRIX    ) Base Address */
#define PMC        (0x400E0600U) /**< \brief (PMC       ) Base Address */
#define UART       (0x400E0800U) /**< \brief (UART      ) Base Address */
#define PDC_UART   (0x400E0900U) /**< \brief (PDC_UART  ) Base Address */
#define CHIPID     (0x400E0940U) /**< \brief (CHIPID    ) Base Address */
#define EFC0       (0x400E0A00U) /**< \brief (EFC0      ) Base Address */
#define EFC1       (0x400E0C00U) /**< \brief (EFC1      ) Base Address */
#define PIOA       (0x400E0E00U) /**< \brief (PIOA      ) Base Address */
#define PIOB       (0x400E1000U) /**< \brief (PIOB      ) Base Address */
#define PIOC       (0x400E1200U) /**< \brief (PIOC      ) Base Address */
#define PIOD       (0x400E1400U) /**< \brief (PIOD      ) Base Address */
#define RSTC       (0x400E1A00U) /**< \brief (RSTC      ) Base Address */
#define SUPC       (0x400E1A10U) /**< \brief (SUPC      ) Base Address */
#define RTT        (0x400E1A30U) /**< \brief (RTT       ) Base Address */
#define WDT        (0x400E1A50U) /**< \brief (WDT       ) Base Address */
#define RTC        (0x400E1A60U) /**< \brief (RTC       ) Base Address */
#define GPBR       (0x400E1A90U) /**< \brief (GPBR      ) Base Address */
#else
#define HSMCI      ((Hsmci  *)0x40000000U) /**< \brief (HSMCI     ) Base Address */
#define SSC        ((Ssc    *)0x40004000U) /**< \brief (SSC       ) Base Address */
#define SPI0       ((Spi    *)0x40008000U) /**< \brief (SPI0      ) Base Address */
#define TC0        ((Tc     *)0x40080000U) /**< \brief (TC0       ) Base Address */
#define TC1        ((Tc     *)0x40084000U) /**< \brief (TC1       ) Base Address */
#define TC2        ((Tc     *)0x40088000U) /**< \brief (TC2       ) Base Address */
#define TWI0       ((Twi    *)0x4008C000U) /**< \brief (TWI0      ) Base Address */
#define PDC_TWI0   ((Pdc    *)0x4008C100U) /**< \brief (PDC_TWI0  ) Base Address */
#define TWI1       ((Twi    *)0x40090000U) /**< \brief (TWI1      ) Base Address */
#define PDC_TWI1   ((Pdc    *)0x40090100U) /**< \brief (PDC_TWI1  ) Base Address */
#define PWM        ((Pwm    *)0x40094000U) /**< \brief (PWM       ) Base Address */
#define PDC_PWM    ((Pdc    *)0x40094100U) /**< \brief (PDC_PWM   ) Base Address */
#define USART0     ((Usart  *)0x40098000U) /**< \brief (USART0    ) Base Address */
#define PDC_USART0 ((Pdc    *)0x40098100U) /**< \brief (PDC_USART0) Base Address */
#define USART1     ((Usart  *)0x4009C000U) /**< \brief (USART1    ) Base Address */
#define PDC_USART1 ((Pdc    *)0x4009C100U) /**< \brief (PDC_USART1) Base Address */
#define USART2     ((Usart  *)0x400A0000U) /**< \brief (USART2    ) Base Address */
#define PDC_USART2 ((Pdc    *)0x400A0100U) /**< \brief (PDC_USART2) Base Address */
#define USART3     ((Usart  *)0x400A4000U) /**< \brief (USART3    ) Base Address */
#define PDC_USART3 ((Pdc    *)0x400A4100U) /**< \brief (PDC_USART3) Base Address */
#define UOTGHS     ((Uotghs *)0x400AC000U) /**< \brief (UOTGHS    ) Base Address */
#define EMAC       ((Emac   *)0x400B0000U) /**< \brief (EMAC      ) Base Address */
#define CAN0       ((Can    *)0x400B4000U) /**< \brief (CAN0      ) Base Address */
#define CAN1       ((Can    *)0x400B8000U) /**< \brief (CAN1      ) Base Address */
#define TRNG       ((Trng   *)0x400BC000U) /**< \brief (TRNG      ) Base Address */
#define ADC        ((Adc    *)0x400C0000U) /**< \brief (ADC       ) Base Address */
#define PDC_ADC    ((Pdc    *)0x400C0100U) /**< \brief (PDC_ADC   ) Base Address */
#define DMAC       ((Dmac   *)0x400C4000U) /**< \brief (DMAC      ) Base Address */
#define DACC       ((Dacc   *)0x400C8000U) /**< \brief (DACC      ) Base Address */
#define PDC_DACC   ((Pdc    *)0x400C8100U) /**< \brief (PDC_DACC  ) Base Address */
#define SMC        ((Smc    *)0x400E0000U) /**< \brief (SMC       ) Base Address */
#define MATRIX     ((Matrix *)0x400E0400U) /**< \brief (MATRIX    ) Base Address */
#define PMC        ((Pmc    *)0x400E0600U) /**< \brief (PMC       ) Base Address */
#define UART       ((Uart   *)0x400E0800U) /**< \brief (UART      ) Base Address */
#define PDC_UART   ((Pdc    *)0x400E0900U) /**< \brief (PDC_UART  ) Base Address */
#define CHIPID     ((Chipid *)0x400E0940U) /**< \brief (CHIPID    ) Base Address */
#define EFC0       ((Efc    *)0x400E0A00U) /**< \brief (EFC0      ) Base Address */
#define EFC1       ((Efc    *)0x400E0C00U) /**< \brief (EFC1      ) Base Address */
#define PIOA       ((Pio    *)0x400E0E00U) /**< \brief (PIOA      ) Base Address */
#define PIOB       ((Pio    *)0x400E1000U) /**< \brief (PIOB      ) Base Address */
#define PIOC       ((Pio    *)0x400E1200U) /**< \brief (PIOC      ) Base Address */
#define PIOD       ((Pio    *)0x400E1400U) /**< \brief (PIOD      ) Base Address */
#define RSTC       ((Rstc   *)0x400E1A00U) /**< \brief (RSTC      ) Base Address */
#define SUPC       ((Supc   *)0x400E1A10U) /**< \brief (SUPC      ) Base Address */
#define RTT        ((Rtt    *)0x400E1A30U) /**< \brief (RTT       ) Base Address */
#define WDT        ((Wdt    *)0x400E1A50U) /**< \brief (WDT       ) Base Address */
#define RTC        ((Rtc    *)0x400E1A60U) /**< \brief (RTC       ) Base Address */
#define GPBR       ((Gpbr   *)0x400E1A90U) /**< \brief (GPBR      ) Base Address */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */
/*@}*/

/* ************************************************************************** */
/*   PIO DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */
/** \addtogroup SAM3X8E_pio Peripheral Pio Definitions */
/*@{*/

//#include "pio/pio_sam3x8e.h"
/*@}*/

/* ************************************************************************** */
/*   MEMORY MAPPING DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */

#define IFLASH0_SIZE             (0x40000u)
#define IFLASH0_PAGE_SIZE        (256u)
#define IFLASH0_LOCK_REGION_SIZE (16384u)
#define IFLASH0_NB_OF_PAGES      (1024u)
#define IFLASH1_SIZE             (0x40000u)
#define IFLASH1_PAGE_SIZE        (256u)
#define IFLASH1_LOCK_REGION_SIZE (16384u)
#define IFLASH1_NB_OF_PAGES      (1024u)
#define IRAM0_SIZE               (0x10000u)
#define IRAM1_SIZE               (0x8000u)
#define NFCRAM_SIZE              (0x1000u)
#define IFLASH_SIZE              (IFLASH0_SIZE+IFLASH1_SIZE)
#define IRAM_SIZE                (IRAM0_SIZE+IRAM1_SIZE)

#define IFLASH0_ADDR    (0x00080000u) /**< Internal Flash 0 base address */
#if defined IFLASH0_SIZE
#define IFLASH1_ADDR    (IFLASH0_ADDR+IFLASH0_SIZE) /**< Internal Flash 1 base address */
#endif
#define IROM_ADDR       (0x00100000u) /**< Internal ROM base address */
#define IRAM0_ADDR      (0x20000000u) /**< Internal RAM 0 base address */
#define IRAM1_ADDR      (0x20080000u) /**< Internal RAM 1 base address */
#define NFC_RAM_ADDR    (0x20100000u) /**< NAND Flash Controller RAM base address */
#define UOTGHS_RAM_ADDR (0x20180000u) /**< USB On-The-Go Interface RAM base address */
#define EBI_CS0_ADDR    (0x60000000u) /**< EBI Chip Select 0 base address */
#define EBI_CS1_ADDR    (0x61000000u) /**< EBI Chip Select 1 base address */
#define EBI_CS2_ADDR    (0x62000000u) /**< EBI Chip Select 2 base address */
#define EBI_CS3_ADDR    (0x63000000u) /**< EBI Chip Select 3 base address */
#define EBI_CS4_ADDR    (0x64000000u) /**< EBI Chip Select 4 base address */
#define EBI_CS5_ADDR    (0x65000000u) /**< EBI Chip Select 5 base address */
#define EBI_CS6_ADDR    (0x66000000u) /**< EBI Chip Select 6 base address */
#define EBI_CS7_ADDR    (0x67000000u) /**< EBI Chip Select 7 base address */

/* ************************************************************************** */
/*   ELECTRICAL DEFINITIONS FOR SAM3X8E */
/* ************************************************************************** */

/* Device characteristics */
#define CHIP_FREQ_SLCK_RC_MIN           (20000UL)
#define CHIP_FREQ_SLCK_RC               (32000UL)
#define CHIP_FREQ_SLCK_RC_MAX           (44000UL)
#define CHIP_FREQ_MAINCK_RC_4MHZ        (4000000UL)
#define CHIP_FREQ_MAINCK_RC_8MHZ        (8000000UL)
#define CHIP_FREQ_MAINCK_RC_12MHZ       (12000000UL)
#define CHIP_FREQ_CPU_MAX               (84000000UL)
#define CHIP_FREQ_XTAL_32K              (32768UL)
#define CHIP_FREQ_XTAL_12M              (12000000UL)

/* Embedded Flash Write Wait State */
#define CHIP_FLASH_WRITE_WAIT_STATE     (6U)

/* Embedded Flash Read Wait State (VDDCORE set at 1.65V) */
#define CHIP_FREQ_FWS_0                 (22500000UL) /**< \brief Maximum operating frequency when FWS is 0 */
#define CHIP_FREQ_FWS_1                 (34000000UL) /**< \brief Maximum operating frequency when FWS is 1 */
#define CHIP_FREQ_FWS_2                 (53000000UL) /**< \brief Maximum operating frequency when FWS is 2 */
#define CHIP_FREQ_FWS_3                 (78000000UL) /**< \brief Maximum operating frequency when FWS is 3 */


#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* _SAM3X8E_ */
