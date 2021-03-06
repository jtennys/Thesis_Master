;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: TX_REPEATER_23.inc
;;   Version: 3.3, Updated on 2009/7/10 at 10:46:51
;;  Generated by PSoC Designer 5.0.985.0
;;
;;  DESCRIPTION:  Assembler declarations for the TX8 user module for the
;;                22/24/25/26/27xxx PSoC family of devices.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"

;---------------------------------------------------
; Constants for TX_REPEATER_23 API's
;---------------------------------------------------

;mask value for global int reg bit for TX instance
TX_REPEATER_23_bINT_MASK:        equ 08h
;TX8 interrupt address
TX_REPEATER_23_INT_REG:          equ 0e1h

; Old equate, will be removed in future release, Do not use.
bTX_REPEATER_23_INT_MASK:        equ 08h


;------------------------------------
;  Transmitter Parity masks
;------------------------------------
TX_REPEATER_23_PARITY_NONE:       equ   00h
TX_REPEATER_23_PARITY_EVEN:       equ   02h
TX_REPEATER_23_PARITY_ODD:        equ   06h

;------------------------------------
;  Transmitter Status Register masks
;------------------------------------
TX_REPEATER_23_TX_COMPLETE:        equ   20h
TX_REPEATER_23_TX_BUFFER_EMPTY:    equ   10h


TX_REPEATER_23_INT_MODE_TX_REG_EMPTY:        equ 0x00
TX_REPEATER_23_INT_MODE_TX_COMPLETE:         equ 0x01

; Do not use! old style equates.  Will be moved
; in future versions.
TX8_PARITY_NONE:           equ   00h
TX8_PARITY_EVEN:           equ   02h
TX8_PARITY_ODD:            equ   06h
TX8_TX_COMPLETE:           equ   20h
TX8_TX_BUFFER_EMPTY:       equ   10h

;----------------------------------------------------
;  Register Address Constants for TX_REPEATER_23
;----------------------------------------------------
TX_REPEATER_23_CONTROL_REG: equ 2fh              ; Control register
TX_REPEATER_23_TX_SHIFT_REG:    equ 2ch             ; TX Shift Register register
TX_REPEATER_23_TX_BUFFER_REG:   equ 2dh             ; TX Buffer Register
TX_REPEATER_23_FUNC_REG:    equ 2ch              ; Function register
TX_REPEATER_23_INPUT_REG:   equ 2dh              ; Input register
TX_REPEATER_23_OUTPUT_REG:  equ 2eh              ; Output register

; end of file TX_REPEATER_23.inc
