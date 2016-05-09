;----------------------------------------------------------------------------------
;	FILE:			HVPSFB_VMC-ISR.ASM
;
;	Description:	Voltage mode control of a phase shifted full bridge (ISR)
;
;	Version: 		1.0
;
;   Target:  		TMS320F2802x(PiccoloA) 
;
;	Type: 			Device dependent
;
;----------------------------------------------------------------------------------
;  Copyright Texas Instruments © 2004-2011                                			
;----------------------------------------------------------------------------------
;  Revision History:
;----------------------------------------------------------------------------------
;  Date	     | Description
;----------------------------------------------------------------------------------
;  04 May 2011 - Voltage Mode Controlled Phase Shifted Full bridge (HN)
;----------------------------------------------------------------------------------

	;include C header file - sets INCR_BUILD (used in conditional builds)
	.cdecls C,NOLIST, "HVPSFB-Settings.h"
	.cdecls C,NOLIST, "PeripheralHeaderIncludes.h"
	
	.include "ADCDRV_4ch.asm"	
	.include "PWMDRV_PSFB_VMC_SR.asm"
	.include "CNTL_2P2Z.asm" 

;===============================================================
;//===============================================================
;// Interrupt Framework options
;//---------------------------------------------------------------
EPWMn_DPL_ISR	.set	1	; for EPWM triggered ISR
ADC_DPL_ISR		.set	0	; for ADC triggered ISR

;// If EPWMn_ISR = 1, then choose which module
EPWM1			.set	1	; EPWM1 provides ISR trigger
EPWM2			.set	0	; EPWM2 provides ISR trigger
EPWM3			.set	0	; EPWM3 provides ISR trigger
EPWM4			.set	0	; EPWM4 provides ISR trigger
EPWM5			.set	0	; EPWM5 provides ISR trigger
EPWM6			.set	0	; EPWM6 provides ISR trigger
NET_DEBUG		.set	0	; do not change this

;**********************************************************************************
; Declare Public functions for External Reference
;**********************************************************************************
		; label to DP initialisation function
				.def _DPL_Init	
		; label to DP ISR Run function
				.def _DPL_ISR

;**********************************************************************************
; Variable declaration
;**********************************************************************************	
tsPtr			.usect "Net_terminals",2

; All Terminal modules initially point to the ZeroNet to ensure a known
; start state. Pad extra locations to accomodate unwanted ADC results.
ZeroNet			.usect "Net_terminals",8,1,1
DummyNet	 	.usect "Net_terminals",7,1,1
IsrClkCtr		.usect "Net_terminals",1

				.text
;---------------------------------------------------------
; ISR Initialisation
;---------------------------------------------------------
_DPL_Init:

; Clear the ZeroNet
	MOVL 	XAR2,#ZeroNet
	RPT		#7 ; 8 times
	||MOV	*XAR2++, #0

	.if(INCR_BUILD = 1)
			ADCDRV_4CH_INIT 		 1,2,3,4
			PWMDRV_PSFB_VMC_SR_INIT  1,2,4		; ePWMs 1,2 and 4	
	.endif ; INCR_BUILD = 1
	;---------------------------------------------------------
	;---------------------------------------------------------
	.if(INCR_BUILD = 2)
			ADCDRV_4CH_INIT 		 1,2,3,4
			PWMDRV_PSFB_VMC_SR_INIT  1,2,4		; ePWMs 1,2 and 4	
			CNTL_2P2Z_INIT				1		; Voltage Controller 	
	.endif ; INCR_BUILD = 2 

	;---------------------------------------------------------
			LRETR

			.sect "ramfuncs"
;---------------------------------------------------------
; ISR Run
;---------------------------------------------------------
_DPL_ISR:	;CONTEXT_SAVE
		; full context save - push any unprotected registers onto stack
		PUSH  	AR1H:AR0H
		PUSH  	XAR2
		PUSH  	XAR3
		PUSH  	XAR4
		PUSH  	XAR5
		PUSH  	XAR6
		PUSH  	XAR7
		PUSH  	XT
		SPM   	0          				; set C28 mode
		CLRC  	AMODE       
		CLRC  	PAGE0,OVM 
;		CLRC	INTM					; clear interrupt mask - comment if ISR non-nestable
;-----------------------------------------------------------------------------------------

; call DP library modules
		ZAPA
;===================================
	;---------------------------------------------------------
	.if(INCR_BUILD = 1)

		ADCDRV_4ch			 1,2,3,4	; Read ADC results		 

		PWMDRV_PSFB_VMC_SR   1,2,4		; Run the PWM driver

	  ; ADC Sample Point Calculation
		MOVW 	DP,#_EPwm2Regs.TBPHS				
		MOV 	AL,@_EPwm2Regs.TBPHS.half.TBPHS
		ADD 	AL,@_EPwm2Regs.TBPRD
		LSR		AL,#1					; AL = (TBPRD2 + TBPHS2)/2
		MOV		@_EPwm2Regs.CMPB, AL	; CMPB2 = (TBPRD2 + TBPHS2)/2

	.endif ; INCR_BUILD = 1
	;---------------------------------------------------------

	;---------------------------------------------------------
	.if(INCR_BUILD = 2)

		ADCDRV_4ch			 1,2,3,4	; Read ADC results		 

		.ref	_No_2p2z
		MOVW	DP, #(_No_2p2z)
		MOV		AL, @(_No_2p2z)		
		CMPB 	AL,#0x1				
		B		SKIP_2P2Z, EQ			; If equal - coefficients are being changed in the slower loop --> don't execute 2P2Z

		CNTL_2P2Z			 1			; Voltage Controller 

SKIP_2P2Z:
		PWMDRV_PSFB_VMC_SR   1,2,4		; Run the PWM driver

	  ; ADC Sample Point Calculation
		MOVW 	DP,#_EPwm2Regs.TBPHS				
		MOV 	AL,@_EPwm2Regs.TBPHS.half.TBPHS
		ADD 	AL,@_EPwm2Regs.TBPRD
		LSR		AL,#1					; AL = (TBPRD2 + TBPHS2)/2
		MOV		@_EPwm2Regs.CMPB, AL	; CMPB2 = (TBPRD2 + TBPHS2)/2

	.endif ; INCR_BUILD = 2
	;---------------------------------------------------------
;===================================
EXIT_ISR:
		ZAPA
;-----------------------------------------------------------------------------------------
; Interrupt management before exit

	.if(EPWMn_DPL_ISR=1)

	.if(EPWM1)
		MOVW 	DP,#_EPwm1Regs.ETCLR
		MOV 	@_EPwm1Regs.ETCLR,#0x01			; Clear EPWM1 Int flag
	.endif ; EPWM1

	.if(EPWM2)
		MOVW 	DP,#_EPwm2Regs.ETCLR
		MOV 	@_EPwm2Regs.ETCLR,#0x01			; Clear EPWM2 Int flag
	.endif ; EPWM2

	.if(EPWM3)
		MOVW 	DP,#_EPwm3Regs.ETCLR
		MOV 	@_EPwm3Regs.ETCLR,#0x01			; Clear EPWM3 Int flag
	.endif ; EPWM3

	.if(EPWM4)
		MOVW 	DP,#_EPwm4Regs.ETCLR
		MOV 	@_EPwm4Regs.ETCLR,#0x01			; Clear EPWM4 Int flag
	.endif ; EPWM4

	.if(EPWM5)
		MOVW 	DP,#_EPwm5Regs.ETCLR
		MOV 	@_EPwm5Regs.ETCLR,#0x01			; Clear EPWM5 Int flag
	.endif ; EPWM5

	.if(EPWM6)
		MOVW 	DP,#_EPwm6Regs.ETCLR
		MOV 	@_EPwm6Regs.ETCLR,#0x01			; Clear EPWM6 Int flag
	.endif ; EPWM6

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 3
		MOV 	@_PieCtrlRegs.PIEACK, #0x4
	.endif ; EPWMn_ISR


	.if(ADC_DPL_ISR=1)
	; Case where ISR is triggered by ADC 
		MOVW 	DP,#ADCST>>6
		MOV 	@ADCST,#0x010					; Clear INT SEQ1 Int flag

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 1
		MOV 	@_PieCtrlRegs.PIEACK,0x1
	.endif ; ADC_ISR 

	
;-----------------------------------------------------------------------------------------
; full context restore
;		SETC	INTM							; set INTM to protect context restore
		POP   	XT
		POP   	XAR7
		POP   	XAR6
		POP   	XAR5
		POP   	XAR4
		POP   	XAR3
		POP   	XAR2
		POP   	AR1H:AR0H
		IRET									; return from interrupt
		.end

