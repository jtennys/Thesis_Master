;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: RX_TIMEOUT.inc
;;   Version: 2.6, Updated on 2009/7/10 at 10:46:29
;;  Generated by PSoC Designer 5.0.985.0
;;
;;  DESCRIPTION: Assembler declarations for the Timer16 user module interface
;;               for the 22/24/27/29xxx PSoC family of devices
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2004. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"


;--------------------------------------------------
; Constants for RX_TIMEOUT API's.
;--------------------------------------------------

RX_TIMEOUT_CONTROL_REG_START_BIT:          equ 0x01    ; Control register start bit
RX_TIMEOUT_INT_REG:                        equ 0x0e1
RX_TIMEOUT_INT_MASK:                       equ 0x02


;--------------------------------------------------
; Constants for RX_TIMEOUT user defined values
;--------------------------------------------------

RX_TIMEOUT_PERIOD:                         equ 0xf0
RX_TIMEOUT_COMPARE_VALUE:                  equ 0x0


;--------------------------------------------------
; Register Address Constants for RX_TIMEOUT
;--------------------------------------------------

RX_TIMEOUT_COUNTER_LSB_REG:            equ 0x20   ; DR0 Counter register
RX_TIMEOUT_COUNTER_MSB_REG:            equ 0x24
RX_TIMEOUT_PERIOD_LSB_REG:             equ 0x21   ; DR1 Period register
RX_TIMEOUT_PERIOD_MSB_REG:             equ 0x25
RX_TIMEOUT_COMPARE_LSB_REG:            equ 0x22   ; DR2 CompareValue register
RX_TIMEOUT_COMPARE_MSB_REG:            equ 0x26
RX_TIMEOUT_CONTROL_LSB_REG:            equ 0x23   ; Control register
RX_TIMEOUT_CONTROL_MSB_REG:            equ 0x27
RX_TIMEOUT_FUNC_LSB_REG:               equ 0x20   ; Function register
RX_TIMEOUT_FUNC_MSB_REG:               equ 0x24
RX_TIMEOUT_INPUT_LSB_REG:              equ 0x21   ; Input register
RX_TIMEOUT_INPUT_MSB_REG:              equ 0x25
RX_TIMEOUT_OUTPUT_LSB_REG:             equ 0x22   ; Output register
RX_TIMEOUT_OUTPUT_MSB_REG:             equ 0x26


;--------------------------------------------------
; RX_TIMEOUT Macro 'Functions'
;--------------------------------------------------

   macro RX_TIMEOUT_Start_M
   or    reg[RX_TIMEOUT_CONTROL_LSB_REG],  RX_TIMEOUT_CONTROL_REG_START_BIT
   endm

   macro RX_TIMEOUT_Stop_M
   and   reg[RX_TIMEOUT_CONTROL_LSB_REG], ~RX_TIMEOUT_CONTROL_REG_START_BIT
   endm

   macro RX_TIMEOUT_EnableInt_M
   M8C_EnableIntMask RX_TIMEOUT_INT_REG, RX_TIMEOUT_INT_MASK
   endm

   macro RX_TIMEOUT_DisableInt_M
   M8C_DisableIntMask RX_TIMEOUT_INT_REG, RX_TIMEOUT_INT_MASK
   endm


; end of file RX_TIMEOUT.inc