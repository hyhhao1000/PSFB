/*==================================================================================*/
/*	User specific Linker command file for running from FLASH						*/
/*==================================================================================*/
/*	FILE:			F2806_FLASH_DPFW.CMD                                            */
/*                                                                                  */
/*	Description:	Linker command file for User custom sections targetted to run   */
/*					from FLASH.  			                                        */
/*                                                                                  */
/*  Target:  		TMS320F2802x or TMS320F2802x                               */
/*                                                                                  */
/*	Version: 		1.00                                							*/
/*                                                                                  */
/*----------------------------------------------------------------------------------*/
/*  Copyright Texas Instruments © 2007                                			    */	
/*----------------------------------------------------------------------------------*/
/*  Revision History:                                                               */
/*----------------------------------------------------------------------------------*/
/*  Date	  | Description                                                         */
/*----------------------------------------------------------------------------------*/
/*  08/30/07  | Release 1.0  		New release.                                    */
/*----------------------------------------------------------------------------------*/
 MEMORY
{
PAGE 0:
   /* For this example, H0 is split between PAGE 0 and PAGE 1 */  
	BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
/*	progRAM		: origin = 0x008000, length = 0x000600*/
	progRAM		: origin = 0x008000, length = 0x000300		

	OTP         : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */
	FLASHD      : origin = 0x3F0000, length = 0x002000     /* on-chip FLASH */
	FLASHC      : origin = 0x3F2000, length = 0x002000     /* on-chip FLASH */
	FLASHA      : origin = 0x3F6000, length = 0x001F80     /* on-chip FLASH */
	CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
	BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
	CSM_PWL     : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */
	
	IQTABLES    : origin = 0x3FE000, length = 0x000B50     /* IQ Math Table in Boot */
	IQTABLES2   : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Table in Boot */
	IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA     /* IQ Math Table in Boot */

	BOOTROM     : origin = 0x3FF27C, length = 0x000D44     /* Boot ROM */
	RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
	VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

PAGE 1 : 

   /* For this example, H0 is split between PAGE 0 and PAGE 1 */

	RAMM0       : origin = 0x000050, length = 0x0003B0
	RAMM1       : origin = 0x000400, length = 0x000400
/*	dataRAM		: origin = 0x008600, length = 0x000a00*/
	dataRAM		: origin = 0x008300, length = 0x000D00
	FLASHB      : origin = 0x3F4000, length = 0x002000
}
 
 
SECTIONS
{
   /* Allocate program areas: */
   .cinit              	: > FLASHA,     PAGE = 0
   .pinit              	: > FLASHA,     PAGE = 0
   .text               	: > FLASHA,     PAGE = 0

   codestart           : > BEGIN       PAGE = 0
   ramfuncs            : LOAD = FLASHA, 
                         RUN = progRAM, 
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         PAGE = 0

   csmpasswds          : > CSM_PWL     PAGE = 0
   csm_rsvd            : > CSM_RSVD    PAGE = 0
   
   /* Allocate uninitalized data sections: */
   .stack           : > RAMM1,      PAGE = 1
   .ebss            : > dataRAM,    PAGE = 1
   .esysmem         : > RAMM1,    PAGE = 1

   /* Initalized sections go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   .econst             : > FLASHA      PAGE = 0
   .switch             : > FLASHA      PAGE = 0      

   /* Allocate IQ math areas: */
   IQmath              : > FLASHA      PAGE = 0                  /* Math Code */
   IQmathTables        : > IQTABLES    PAGE = 0, TYPE = NOLOAD   /* Math Tables In ROM */

   .reset              : > RESET,      PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS     PAGE = 0, TYPE = DSECT

   DLOG	 	           : > dataRAM,    PAGE = 1
}
    
    
SECTIONS
{
	/*************       DPLIB Sections C28x      ************************/
	/* ADCDRV_1ch section */
	ADCDRV_1ch_Section				: > dataRAM				PAGE = 1
	
	/* ADCDRV_4ch section */
	ADCDRV_4ch_Section				: > dataRAM				PAGE = 1

	/* ADCDRV_8ch section */
	ADCDRV_8ch_Section				: > dataRAM				PAGE = 1

	/* CNTL_2P2Z section */
	CNTL_2P2Z_Section				: > dataRAM				PAGE = 1
	CNTL_2P2Z_InternalData			: > dataRAM				PAGE = 1
	CNTL_2P2Z_Coef					: > dataRAM				PAGE = 1
	
	/* CNTL_3P3Z section */
	CNTL_3P3Z_Section				: > dataRAM				PAGE = 1
	CNTL_3P3Z_InternalData			: > dataRAM				PAGE = 1
	CNTL_3P3Z_Coef					: > dataRAM				PAGE = 1

	/*DLOG_4CH section */
	DLOG_4CH_Section				: > dataRAM				PAGE = 1
	DLOG_BUFF						: > dataRAM				PAGE = 1
	
	/*MATH_EMAVG section */
	MATH_EMAVG_Section				: > dataRAM				PAGE = 1
	
	/*PFC_ICMD section*/
	PFC_ICMD_Section				: > dataRAM				PAGE = 1
	
	/*PFC_INVSQR section*/
	PFC_INVSQR_Section				: > dataRAM				PAGE = 1

	/* DACDRV_RAMP section */
	DACDRV_RAMP_Section				: > dataRAM				PAGE = 1
	
	/* PWMDRV_1ch driver section */
	PWMDRV_1ch_Section				: > dataRAM				PAGE = 1
	
	/* PWMDRV_1chHiRes driver section */
	PWMDRV_1chHiRes_Section			: > dataRAM				PAGE = 1
	
	/* PWMDRV_PFC2PhiL driver section */
	PWMDRV_PFC2PhiL_Section			: > dataRAM				PAGE = 1
	
 	/* PWMDRV_PSFB driver section */
	PWMDRV_PSFB_Section				: > dataRAM				PAGE = 1
	
	/* PWMDRV_DualUpDwnCnt driver section */
	PWMDRV_DualUpDwnCnt_Section		: > dataRAM				PAGE = 1
	
	/* PWMDRV_ComplPairDB driver section */
	PWMDRV_ComplPairDB_Section		: > dataRAM				PAGE = 1
	
	/* ZeroNet_Section  */
	ZeroNet_Section					: > dataRAM				PAGE = 1

	/* Net_terminals_Section  */
	Net_terminals:	> dataRAM,PAGE = 1

}



 
