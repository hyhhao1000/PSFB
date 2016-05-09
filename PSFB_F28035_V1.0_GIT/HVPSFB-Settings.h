//----------------------------------------------------------------------------------
//	FILE:			HVPSFB-Settings.h
//
//	Description:	Project settings file for HVPSFB_VMC project
//
//	Version: 		1.0
//
//  Target:  		TMS320F2802x(PiccoloA) 
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments �2004-2011
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 04 May 2011 - Voltage Mode Controlled Phase Shifted Full bridge (HN)
//----------------------------------------------------------------------------------
// DECRIPTION: 	This file contains the definitions for the project, and is 
//				linked to both X-Main.c and X-ISR.asm (where X is the project
//				name).  When editing this file please "Rebuild All".
//
//###########################################################################

#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

//**************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//**************************************************************
//===============================================================
// Incremental Build options for System check-out
//---------------------------------------------------------------
#define INCR_BUILD  1		//1 - Open loop PSFB drive + ADC feedback
							//2 - Closed loop: Full PSFB in VMC mode
//===============================================================
// System Settings
//---------------------------------------------------------------
#define	NumChannels	4	// ADC Number of Channels
#define HistorySize 8	// Number of samples averaged for use in GUI

#endif

