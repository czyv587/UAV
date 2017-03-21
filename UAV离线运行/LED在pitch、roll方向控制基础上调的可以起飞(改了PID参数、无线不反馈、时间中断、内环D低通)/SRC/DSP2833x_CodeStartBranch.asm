;// TI File $Revision: /main/1 $
;// Checkin $Date: August 18, 2006   13:45:55 $
;//###########################################################################
;//
;// FILE:  DSP2833x_CodeStartBranch.asm	
;//
;// TITLE: Branch for redirecting code execution after boot. 
;//
;// For these examples, code_start is the first code that is executed after
;// exiting the boot ROM code. 
;//
;// The codestart section in the linker cmd file is used to physically place
;// this code at the correct memory location.  This section should be placed 
;// at the location the BOOT ROM will re-direct the code to.  For example, 
;// for boot to FLASH this code will be located at 0x3f7ff6. 
;//
;// In addition, the example DSP2833x projects are setup such that the codegen
;// entry point is also set to the code_start label.  This is done by linker 
;// option -e in the project build options.  When the debugger loads the code,
;// it will automatically set the PC to the "entry point" address indicated by
;// the -e linker option.  In this case the debugger is simply assigning the PC, 
;// it is not the same as a full reset of the device. 
;// 
;// The compiler may warn that the entry point for the project is other then
;//  _c_init00.  _c_init00 is the C environment setup and is run before 
;// main() is entered. The code_start code will re-direct the execution 
;// to _c_init00 and thus there is no worry and this warning can be ignored. 
;// 
;//###########################################################################
;// $TI Release: DSP2833x Header Files V1.01 $
;// $Release Date: September 26, 2007 $
;//###########################################################################

    .ref _c_int00
	.global copy_sections
	.global _cinit_loadstart, _cinit_runstart, _cinit_size
	.global _const_loadstart, _const_runstart, _const_size
	.global _econst_loadstart, _econst_runstart, _econst_size
	.global _pinit_loadstart, _pinit_runstart, _pinit_size
	.global _switch_loadstart, _switch_runstart, _switch_size
	.global _text_loadstart, _text_runstart, _text_size
***********************************************************************

WD_DISABLE	.set	1		;set to 1 to disable WD, else set to 0

    .ref _c_int00
    .global code_start

***********************************************************************
* Function: codestart section
*
* Description: Branch to code starting point
***********************************************************************

    .sect "codestart"

code_start:
    .if WD_DISABLE == 1
        LB wd_disable       ;Branch to watchdog disable code
    .else
        LB copy_sections         ;Branch to start of boot.asm in RTS library
    .endif

;end codestart section


***********************************************************************
* Function: wd_disable
*
* Description: Disables the watchdog timer
***********************************************************************
    .if WD_DISABLE == 1

    .sect "wddisable"
wd_disable:
    SETC OBJMODE        ;Set OBJMODE for 28x object code
    EALLOW              ;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6  ;Set data page for WDCR register
    MOV @7029h, #0068h  ;Set WDDIS bit in WDCR to disable WD
    EDIS                ;Disable EALLOW protected register access
    LB copy_sections         ;Branch to start of boot.asm in RTS library

    .endif

;end wd_disable

***********************************************************************
* Function: copy_sections
*
* Description: Copies initialized sections from flash to ram
***********************************************************************

	.sect "copy_sections"

copy_sections:

	MOVL XAR5,#_const_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_const_loadstart			; Store Load Starting Address in XAR6
    MOVL XAR7,#_const_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

	MOVL XAR5,#_econst_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_econst_loadstart		; Store Load Starting Address in XAR6
    MOVL XAR7,#_econst_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

	MOVL XAR5,#_pinit_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_pinit_loadstart			; Store Load Starting Address in XAR6
    MOVL XAR7,#_pinit_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

	MOVL XAR5,#_switch_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_switch_loadstart		; Store Load Starting Address in XAR6
    MOVL XAR7,#_switch_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

	MOVL XAR5,#_text_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_text_loadstart			; Store Load Starting Address in XAR6
    MOVL XAR7,#_text_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

   	MOVL XAR5,#_cinit_size				; Store Section Size in XAR5
	MOVL ACC,@XAR5						; Move Section Size to ACC
	MOVL XAR6,#_cinit_loadstart			; Store Load Starting Address in XAR6
    MOVL XAR7,#_cinit_runstart			; Store Run Address in XAR7
    LCR  copy							; Branch to Copy

    LB _c_int00				 			; Branch to start of boot.asm in RTS library

copy:
	B return,EQ							; Return if ACC is Zero (No section to copy)

    RPT AL								; Copy Section From Load Address to
    || PWRITE  *XAR7, *XAR6++			; Run Address

return:
	LRETR								; Return

	.end


	.end
	
;//===========================================================================
;// End of file.
;//===========================================================================
