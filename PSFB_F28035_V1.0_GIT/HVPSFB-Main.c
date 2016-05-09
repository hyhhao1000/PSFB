//----------------------------------------------------------------------------------
//	FILE:			HVPSFB-Main.C
//
//	Description:	电压控制移相全桥28V1.5kW

//	Version: 		1.0
//
//  Target:  		TMS320F28035(PiccoloA)
//
//----------------------------------------------------------------------------------
//  Copyright
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 2016.3.28 - Voltage Mode Controlled Phase Shifted Full bridge (HN)
//----------------------------------------------------------------------------------
//
// PLEASE READ - Useful notes about this Project

// Although this project is made up of several files, the most important ones are:
//	 "{ProjectName}-Main.C"	- this file
//		- Application Initialization, Peripheral config,
//		- Application management
//		- Slower background code loops and Task scheduling
//	 "{ProjectName}-DevInit_F28xxx.C
//		- Device Initialization, e.g. Clock, PLL, WD, GPIO mapping
//		- Peripheral clock enables
//		- DevInit file will differ per each F28xxx device series, e.g. F280x, F2833x,
//	 "{ProjectName}-DPL-ISR.asm
//		- Assembly level library Macros and any cycle critical functions are found here
//	 "{ProjectName}-Settings.h"
//		- Global defines (settings) project selections are found here
//		- This file is referenced by both C and ASM files.

// Code is made up of sections, e.g. "FUNCTION PROTOTYPES", "VARIABLE DECLARATIONS" ,..etc
//	each section has FRAMEWORK and USER areas.
//  FRAMEWORK areas provide useful ready made "infrastructure" code which for the most part
//	does not need modification, e.g. Task scheduling, ISR call, GUI interface support,...etc
//  USER areas have functional example code which can be modified by USER to fit their appl.
//
// Code can be compiled with various build options (Incremental Builds IBx), these
//  options are selected in file "{ProjectName}-Settings.h".  Note: "Rebuild All" compile
//  tool bar button must be used if this file is modified.
//----------------------------------------------------------------------------------

#include "HVPSFB-Settings.h"
#include "PeripheralHeaderIncludes.h"
#include "DSP2803x_EPWM_defines.h"

#include "DPlib.h"
#include "IQmathLib.h"
//#define FLASH   //使用RAM调试时去掉本条语句
//GIT TSET
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void DeviceInit(void);
#ifdef FLASH
	void InitFlash();
#endif
void MemCopy();
void DeviceInit(void);
void SCIA_Init();
void SerialHostComms();
void InitFlash();
//---------------------------------------------------------------

// System Defines
//---------------------------------------------------------------
#define	PWM_PRD			(60000/100)			// Period count = 600 corresponding to 100 KHz @ 60 MHz (Up count mode)

volatile struct EPWM_REGS *ePWM[] =
 				  { &EPwm1Regs,			//intentional: (ePWM[0] not used)
				  	&EPwm1Regs,
					&EPwm2Regs,
					&EPwm3Regs,
					&EPwm4Regs,
					&EPwm5Regs,
					&EPwm6Regs,
					#if (DSP2803x_DEVICE_H || DSP2804x_DEVICE_H)
					&EPwm7Regs,
					#if (DSP2804x_DEVICE_H)
					&EPwm8Regs
					#endif
					#endif
				  };

// Used to indirectly access all Comparator modules
volatile struct COMP_REGS *Comp[] =
 				  { &Comp1Regs,			//intentional: (Comp[0] not used)
					&Comp1Regs,
					&Comp2Regs,
					#if (DSP2803x_DEVICE_H)
					&Comp3Regs
					#endif
				  };


//-------------------------------- DPLIB --------------------------------------------
extern void PWMDRV_PSFB_VMC_SR_CNF(int16 n, int16 period, int16 SR_Enable, int16 Comp1_Prot);
extern void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);
extern void DacDrvCnf(int16 n, int16 DACval, int16 DACsrc, int16 RAMPsrc, int16 Slope_initial);


// -------------------------------- FRAMEWORK --------------------------------------
// State Machine function prototypes
//----------------------------------------------------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3
void A4(void);	//state A4

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3
void B4(void);	//state B4

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3
void C4(void);	//state C4

// Variable declarations
void (*Alpha_State_Ptr)(void);				// Base States pointer
void (*A_Task_Ptr)(void);					// State pointer A branch
void (*B_Task_Ptr)(void);					// State pointer B branch
void (*C_Task_Ptr)(void);					// State pointer C branch

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------- FRAMEWORK --------------------------------------

int16 VTimer0[4];							// Virtual Timers slaved of CPU Timer 0 (A events)
int16 VTimer1[4]; 							// Virtual Timers slaved of CPU Timer 1 (B events)
int16 VTimer2[4]; 							// Virtual Timers slaved of CPU Timer 2 (C events)
int16 SerialCommsTimer;
int16 CommsOKflg;

extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

// Used for ADC Configuration  ADC配置
int ChSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //通道选择
int ACQPS[16] = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8}; //序列选择
int	TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//触发选择

// ---------------------------------- USER -----------------------------------------
// ---------------------------- DPLIB Net Pointers ---------------------------------
//DPLIB网络指针
// Declare net pointers that are used to connect the DP Lib Macros  here

// PWMDRV_PSFB_VMC_SR
extern volatile long *PWMDRV_PSFB_Phase1;
extern volatile int16 *PWMDRV_PSFB_DbAtoP1, *PWMDRV_PSFB_DbPtoA1;  //死区定义

// ADCDRV_4ch    4通道ADC外设驱动指针变量
extern volatile long *ADCDRV_4ch_RltPtrA;
extern volatile long *ADCDRV_4ch_RltPtrB;
extern volatile long *ADCDRV_4ch_RltPtrC;
extern volatile long *ADCDRV_4ch_RltPtrD;
//2极点2零点宏模块指针变量
extern volatile long *CNTL_2P2Z_Ref1, *CNTL_2P2Z_Out1, *CNTL_2P2Z_Fdbk1;
extern volatile long *CNTL_2P2Z_Ref2, *CNTL_2P2Z_Out2, *CNTL_2P2Z_Fdbk2;
extern volatile long *CNTL_2P2Z_Coef1, *CNTL_2P2Z_Coef2;

// ---------------------------- DPLIB Variables ---------------------------------
//数字电源库变量
//定义网络变量用于DPLib中的宏定义模块
// Declare the net variables being used by the DP Lib Macro here


volatile long Adc_VavgBus[5];  								// Used as consecutive addresses for ADC Vout conversion results
#pragma DATA_SECTION(Adc_VavgBus, "ADCDRV_4ch_Section");	// Output terminal 1

volatile long Adc_Ifb; //原边电流反馈
volatile long Adc_Vfbin;//输入电压反馈
volatile long Adc_Iout;//输出电流采样
volatile long Vref = 0;										// FB Set Voltage 输出电压参考信号
volatile long Iref = 0;										// FB Current Loop Command 电流反馈参考信号
volatile long Vfb_slew_temp = 0;							// Temp variable: used only if implementing
															// slew rate control in the slower state machine
volatile long VfbSetSlewed = 2093568;						// Slewed set point for the FB voltage loop - start from 4V
volatile long VfbSlewRate = 25600;							// FB Slew rate adjustment

volatile long phase = 0;									// FB phase command
int16 dbAtoP_leg = 20, dbPtoA_leg = 20;						// Dead band adjust for the two legs
int16 SR_mode = 2;											//同步整流模式
int16 No_2p2z = 0;											// Used to disable 2P2Z execution when control loop coefficients are being changed
int16 range = 0, auto_DB = 1;								// Used to adjust DB based on load conditions
int16 input_good = 0, start_flag = 0;						// Used for soft-start ad shut-down软启动和关断

int16 sub_adj = 0;											// Used to compensate for offsets in Iout reading

volatile long Adc_Vfbout = 0, Adc_Ifb = 0;
volatile long Adc_Vfbin = 0, Adc_Iout = 0;

int16 Iout_prev = 0, Iout_diff = 0;							// Used for fast dead-band adjustment for big load transients

//int16 Auto_Run = 1;
int16 Auto_Run = 0;

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct1, "CNTL_2P2Z_Coef");
#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct2, "CNTL_2P2Z_Coef");
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct2;
long Pgain, Igain, Dgain, Dmax;

//过流保护电平设置
//int16 Ipri_trip = 12089;
int    Ipri_trip = 29789;   	//30A 保护
//int16 Ipri_trip = 0;									// Programmable overcurrent shut-down level via on-chip comparator and DAC
//PDI参数设置
int16 Pgain_Gui = 648, Igain_Gui = 96, Dgain_Gui = 0;		// PID gains for the voltage loop (Values can be controlled from CCS or GUI)
//2P2Z系数设置
int16 b2_Gui=0, b1_Gui=-3468, b0_Gui=3933, a2_Gui=0, a1_Gui=1024, a0_Gui; // 2P2Z coefficients for the voltage loop (Values can be controlled directly from CCS or from GUI by poles and zeroes placement)

int16 pid2p2z_Gui= 1, coeff_change = 1;						// Flag for switching between PID and poles and zeroes based coeffiefients, Flag to indicate change of coefficients

// System Flags
//错误标志   输出过流或输出欠压
int16	FaultFlg;											// Fault flag set on over current or output undervoltage conditions

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - CCS WatchWindow / GUI support
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------- FRAMEWORK --------------------------------------

//GUI support variables
// sets a limit on the amount of external GUI controls - increase as necessary
int16 	*varSetTxtList[64];					//64 textbox controlled variables
int16 	*varSetBtnList[16];					//16 button controlled variables
int16 	*varSetSldrList[16];				//16 slider controlled variables
int16 	*varGetList[16];					//16 variables sendable to GUI
int16 	*arrayGetList[16];					//16 arrays sendable to GUI

// ---------------------------------- USER -----------------------------------------
// Monitor ("Get")							// Display as:
int16	Gui_Vfbin;							// Q5
int16	Gui_Ifb;							// Q12
int16	Gui_Vfbout;							// Q10
int16	Gui_Iout;							// Q7
int16   Gui_Pout = 0; 						// Q6

int16	Gui_VfbSet = 2*1024;				// Q10

int16	Gui_IfbSet = 614;					// Q12 -- Build 1 Only - 0.15A

//Scaling Constants (values found via spreadsheet)
int16	K_Vfbin;							// Q15
int16	K_Ifb;								// Q15
int16	K_Vfbout;							// Q15

int16	K_Iout;								// Q15

int16	iK_Ifb;								// Q14
int16	iK_Vfbout;							// Q14

// Variables for background support only (no need to access)
int16	i;									// common use incrementer
int16	HistPtr, temp_Scratch;
int16	temp_ChNum, temp_Iout;

int16	Vset[NumChannels+1];				// Per Unit (Q15)
int16	Vmargin[NumChannels+1];				// Per Unit (Q15)

// History arrays are used for Running Average calculation (boxcar filter)
// Used for CCS display and GUI only, not part of control loop processing
int16	Hist_Vfbin[HistorySize];
int16	Hist_Ifb[HistorySize];
int16	Hist_Vfbout[HistorySize];

int16	Hist_Iout[HistorySize];

void main(void)
{
//=================================================================================
//	INITIALISATION - General
//=================================================================================

//-------------------------------- FRAMEWORK --------------------------------------
	DeviceInit();									// Device Life support & GPIO
	SCIA_Init();  									// Initalize the Serial Comms A peripheral

// Only used if running from FLASH
#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
      MemCopy((Uint16*)&RamfuncsLoadStart, (Uint16*)&RamfuncsLoadEnd, (Uint16*)&RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();									// Call the flash wrapper init function
#endif //(FLASH)

// Timing sync for background loops
// Timer period definitions found in PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec2;					// A tasks 任务A时间2ms
	CpuTimer1Regs.PRD.all =  mSec5;					// B tasks 任务B时间5ms
	CpuTimer2Regs.PRD.all =  mSec0_5; 				// C tasks 任务C时间0.5ms

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

	VTimer0[0] = 0;
	VTimer1[0] = 0;
	VTimer1[1] = 0;
	VTimer2[0] = 0;
	CommsOKflg = 0;

	HistPtr = 0;

// ---------------------------------- USER -----------------------------------------
//配置缩放常数
//AD转换  AD采集的数据*缩放常数得到对应实际数据 Q15格式
//Configure Scaling Constants

	K_Vfbin = 27287;								//0.8327 in Q15 hyh MAX 426V
	K_Ifb = 16896;									//0.5156 in Q15 hyh MAX 33A
	K_Vfbout = 30720;								//0.9375 in Q15 hyh MAX 55V
	K_Iout = 30720; 								// 0.9375 in Q15 hyh MAX 40A
	iK_Ifb = 31775;									// 1.9394 in Q14 hyh
	iK_Vfbout = 17476;								// 1.0667 in Q14 hyh
	//K_Vfbin = 21555;								// 0.6578 in Q15 (see excel spreadsheet)
	//K_Ifb = 27755;									// 0.8470 in Q15 (see excel spreadsheet)
	//K_Vfbout = 16411;								// 0.5008 in Q15 (see excel spreadsheet)
	//K_Iout = 17974; 								// 0.5485 in Q15 (see excel spreadsheet)
	//iK_Ifb = 19343;									// 1.1806 in Q14 (see excel spreadsheet)
	//iK_Vfbout = 32714;								// 1.9967 in Q14 (see excel spreadsheet)

//=================================================================================
//	INITIALISATION - GUI connections
//=================================================================================
// Use this section only if you plan to "Instrument" your application using the
// Microsoft C# freeware GUI Template provided by TI

	//"Set" variables
	//---------------------------------------
	// assign GUI variable Textboxes to desired "setable" parameter addresses
		varSetTxtList[0] = &SR_mode;			// Q0
		varSetTxtList[1] = &Ipri_trip;			// Q15
		varSetTxtList[2] = &dbAtoP_leg;			// Q15
		varSetTxtList[3] = &dbPtoA_leg;			// Q15
		varSetTxtList[4] = &sub_adj;			// Q15

		varSetTxtList[5] = &b0_Gui;				// I5Q10
		varSetTxtList[6] = &b1_Gui;				// I5Q10
		varSetTxtList[7] = &b2_Gui;				// I5Q10
		varSetTxtList[8] = &a0_Gui;				// I5Q10
		varSetTxtList[9] = &a1_Gui;				// I5Q10
		varSetTxtList[10] = &a2_Gui;			// I5Q10
		varSetTxtList[11] = &coeff_change;		// Q0

	// assign GUI Buttons to desired flag addresses
		varSetBtnList[0] = &auto_DB;
		varSetBtnList[1] = &pid2p2z_Gui;

		varSetSldrList[0] = &Gui_VfbSet;		// Q10

		varSetSldrList[1] = &Pgain_Gui;			// Q26/67108
		varSetSldrList[2] = &Igain_Gui;			// Q26/67108
		varSetSldrList[3] = &Dgain_Gui;			// Q26/67108

		varGetList[0] = &Gui_Vfbout;			// Q10
		varGetList[1] = &Gui_Vfbin;				// Q5
		varGetList[2] = &Gui_Ifb;				// Q12
		varGetList[3] = &Gui_Iout;				// Q7

		varGetList[4] = &FaultFlg;				// -
		varGetList[5] = &dbAtoP_leg;			// -
		varGetList[6] = &dbPtoA_leg;			// -

		varGetList[7] = &SR_mode;				// -
		varGetList[8] = &Gui_Pout;				// -

//---------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Incremental build options via Module connections to system Nets.
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

//==================================================================================
//	INCREMENTAL BUILD OPTIONS - NOTE: select via ProjectSettings.h
//==================================================================================
// ---------------------------------- USER -----------------------------------------
//==============================================================================
#if (INCR_BUILD == 1) // Open loop PSFB drive + ADC feedback
//=============================================================
#define		Vfb_outR	AdcResult.ADCRESULT1	//
#define		IfbR		AdcResult.ADCRESULT2	//
#define		Vfb_inR	 	AdcResult.ADCRESULT3	//

#define		IoutR	 	AdcResult.ADCRESULT4	//
//级联定序器通道选择
// Channel Selection for Cascaded Sequencer
	/*ChSel[0] = 0;				// A0 - O/P Voltage - Dummy
	ChSel[1] = 0;				// A0 - O/P Voltage
	ChSel[2] = 2;				// A2 - Transformer Primary Current
	ChSel[3] = 1;				// B1 - I/P Voltage
	ChSel[4] = 12;				// B4 - Iout2

	TrigSel[0] = 7;				// O/P Voltage sampling triggered by EPWM2 SOCA - Dummy
	TrigSel[1] = 7;				// O/P Voltage sampling triggered by EPWM2 SOCA
	TrigSel[2] = 7;				// Transformer Primary Current sampling triggered by EPWM2 SOCA
	TrigSel[3] = 7;				// I/P Voltage sampling triggered by EPWM2 SOCA
	TrigSel[4] = 7;				// Iout sampling triggered by EPWM2 SOCA*/
	ChSel[0] = 14;				// B6 - O/P Voltage - Dummy 输出直流电压，输出直流保护电压
	ChSel[1] = 14;				// B6 - O/P Voltage 输出直流电压，输出直流保护电压
	ChSel[2] = 2;				// A2 - Transformer Primary Current 原边变压器电流
	ChSel[3] = 0;				// A0 - I/P Voltage PFC输入，直流输入电压，保护电压
	ChSel[4] = 12;				// B4 - Iout2 输出电流
//AD采样触发条件，SOCA (Start of Conversion ADC),ADCSOCxCTL=7，EPWM2触发ADC
	TrigSel[0] = 7;				// O/P Voltage sampling triggered by EPWM2 SOCA - Dummy
	TrigSel[1] = 7;				// O/P Voltage sampling triggered by EPWM2 SOCA
	TrigSel[2] = 7;				// Transformer Primary Current sampling triggered by EPWM2 SOCA
	TrigSel[3] = 7;				// I/P Voltage sampling triggered by EPWM2 SOCA
	TrigSel[4] = 7;				// Iout sampling triggered by EPWM2 SOCA

	EALLOW;
	AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 4; // SOC0-3 are high priority
	EDIS;

	PWMDRV_PSFB_VMC_SR_CNF(1, PWM_PRD, 1, 1); 	// ePWM1 and ePWM2, Period=PWM_PRD,
												//SR_Enable=1 (ePWM4), Comp1_Prot=1

	ADC_SOC_CNF(ChSel,TrigSel,ACQPS, 16, 0);// ACQPS=8, No ADC channel triggers an interrupt IntChSel > 15,
											// Mode= Start/Stop (0)

	DPL_Init();								// ASM ISR init

// Lib Module connection to "nets"
//----------------------------------------
// ADC feedback connections
// ADC 4个通道采样对应数据
	ADCDRV_4ch_RltPtrA = &Adc_Vfbout;
	ADCDRV_4ch_RltPtrB = &Adc_Ifb;
	ADCDRV_4ch_RltPtrC = &Adc_Vfbin;
	ADCDRV_4ch_RltPtrD = &Adc_Iout;

// Connect the PWMDRV_PSFB_VMC_SR driver block
	PWMDRV_PSFB_Phase1  = &phase;			// Point to the phase net
	PWMDRV_PSFB_DbAtoP1 = &dbAtoP_leg;		// Point to the left leg dead band adjust
	PWMDRV_PSFB_DbPtoA1 = &dbPtoA_leg;		// Point to the right leg dead band adjust

	phase    = _IQ24(0.015625);				// Starting phase，初始化相位
	dbAtoP_leg  = 20;
	dbPtoA_leg  = 18;

#endif //  (INCR_BUILD == 1)

//==============================================================================
#if (INCR_BUILD == 2) // Closed loop: Full PSFB in VMC mode
//=============================================================
#define		Vfb_outR	AdcResult.ADCRESULT1	//
#define		IfbR		AdcResult.ADCRESULT2	//
#define		Vfb_inR	 	AdcResult.ADCRESULT3	//

#define		IoutR	 	AdcResult.ADCRESULT4	//

// Channel Selection for Cascaded Sequencer
	/*ChSel[0] = 0;				// A0 - O/P Voltage - Dummy
	ChSel[1] = 0;				// A0 - O/P Voltage
	ChSel[2] = 2;				// A2 - Transformer Primary Current
	ChSel[3] = 9;				// B1 - I/P Voltage

	ChSel[4] = 12;				// B4 - Iout2*/

	ChSel[0] = 14;				// B6 - O/P Voltage - Dummy 输出直流电压，输出直流保护电压
	ChSel[1] = 14;				// B6 - O/P Voltage 输出直流电压，输出直流保护电压
	ChSel[2] = 2;				// A2 - Transformer Primary Current 原边变压器电流
	ChSel[3] = 0;				// A0 - I/P Voltage PFC输入，直流输入电压，保护电压
	ChSel[4] = 12;				// B4 - Iout2 输出电流

	TrigSel[0] = 7;		// O/P Voltage sampling triggered by EPWM2 SOCA - Dummy
	TrigSel[1] = 7;		// O/P Voltage sampling triggered by EPWM2 SOCA
	TrigSel[2] = 7;		// Transformer Primary Current sampling triggered by EPWM2 SOCA
	TrigSel[3] = 7;		// I/P Voltage sampling triggered by EPWM2 SOCA

	TrigSel[4] = 7;		// Iout sampling triggered by EPWM2 SOCA

	EALLOW;
	AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 4; // SOC0-3 are high priority
	EDIS;

	PWMDRV_PSFB_VMC_SR_CNF(1, PWM_PRD, 1, 1); 	// ePWM1 and ePWM2, Period=PWM_PRD, SR_Enable=1 (ePWM4), Comp1_Prot=1

	ADC_SOC_CNF(ChSel,TrigSel,ACQPS, 16, 0);// ACQPS=8, No ADC channel triggers an interrupt IntChSel > 15,
											// Mode= Start/Stop (0)

	DPL_Init();								// ASM ISR init

// Lib Module connection to "nets"
//----------------------------------------
// ADC feedback connections
	ADCDRV_4ch_RltPtrA = &Adc_Vfbout;
	ADCDRV_4ch_RltPtrB = &Adc_Ifb;
	ADCDRV_4ch_RltPtrC = &Adc_Vfbin;
	ADCDRV_4ch_RltPtrD = &Adc_Iout;

// Connect the PWMDRV_PSFB_VMC_SR driver block
	PWMDRV_PSFB_Phase1  = &phase;			// Point to the phase net
	PWMDRV_PSFB_DbAtoP1 = &dbAtoP_leg;		// Point to the left leg dead band adjust
	PWMDRV_PSFB_DbPtoA1 = &dbPtoA_leg;		// Point to the right leg dead band adjust

//2P2Z connections for the Voltage Loop
	CNTL_2P2Z_Ref1 = &VfbSetSlewed;					// Slewed Voltage command
	CNTL_2P2Z_Out1 = &phase;						// Reference command to the current loop
	CNTL_2P2Z_Fdbk1 = &Adc_Vfbout;					// O/P Voltage feedback
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;	// point to first coeff.

// Coefficients for the Voltage Loop
// PID coefficients & Clamping (Q26)
	Dmax  = _IQ24(0.984375);
	Pgain = _IQ26(0.001953);
	Igain = _IQ26(0.031250);
	Dgain = _IQ26(0.0);

// Coefficient init	--- Coeeficient values in Q26
// Use IQ Maths to generate floating point values for the CLA
	CNTL_2P2Z_CoefStruct1.b2   = Dgain;                            	// B2
    CNTL_2P2Z_CoefStruct1.b1   = (Igain-Pgain-Dgain-Dgain);  		// B1
    CNTL_2P2Z_CoefStruct1.b0   = (Pgain + Igain + Dgain);      		// B0
    CNTL_2P2Z_CoefStruct1.a2   = 0.0;                              	// A2 = 0
    CNTL_2P2Z_CoefStruct1.a1   = _IQ26(1.0);                       	// A1 = 1
    CNTL_2P2Z_CoefStruct1.max  = Dmax;					  		  	//Clamp Hi
    CNTL_2P2Z_CoefStruct1.min  = _IQ24(0.0); 					  	//Clamp Min

	Vref 	 = 0;
	dbAtoP_leg  = 20;
	dbPtoA_leg  = 18;

#endif //  (INCR_BUILD == 2)

//==============================================================================
// Items common to all builds
//==============================================================================
// Configure Comparator1 and DAC for over current protection
// 数字比较器1  DAC 过压保护
	DacDrvCnf(1, Ipri_trip, 0, 2, 0);		// Comp1, DACval = Ipri_trip, DAC Source is DACval, Ramp Source = don't care, Slope = don't care
//OST TripZone one-shot 中断标志
	EPwm1Regs.TZCLR.bit.OST = 1;	// Clear any spurious trips
	EPwm2Regs.TZCLR.bit.OST = 1;	// Clear any spurious trips

	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;		// 0=GPIO,  1=EPWM1B,  2=SPISIMO-D,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;		// 0=GPIO,  1=EPWM2B,  2=SPISOMI-D,  3=Resv
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;		// 0=GPIO,  1=EPWM4A,  2=SYNCI,  3=SYNCO
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;		// 0=GPIO,  1=EPWM4B,  2=SPISTE-D,  3=ECAP2
//All enabled ePWM module clocks are started with the first rising edge of TBCLK aligned
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

//=================================================================================
//	INTERRUPT & ISR INITIALISATION (best to run this section after other initialisation)
//=================================================================================
//Also Set the appropriate # define's in the {ProjectName}-Settings.h
//to enable interrupt management in the ISR
//DPL_ISR汇编中断响应程序
	EALLOW;
	PieVectTable.EPWM1_INT = &DPL_ISR;			// Map Interrupt
	EDIS;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;			// PIE level enable, Grp3 / Int1
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;	// INT on ZRO event   EPWM1事件触发中断选择 CTR=ZERO
	EPwm1Regs.ETSEL.bit.INTEN = 1;				// Enable INT 使能事件中断
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;			// Generate INT every event (100 KHz) 每个事件都触发中断

// Enable Peripheral, global Ints and higher priority real-time debug events:
	IER |= M_INT3;
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

// Backgound Loop
	for(;;)
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

		// Fast dead-band adjustment for large load transients
		Iout_diff = IoutR - Iout_prev;
		if (Iout_diff>292 || Iout_diff<(-292)) //12-bit ADC result corresponding to a change of 10A on Iout ~ 292d
			C1();
	    Iout_prev = IoutR;
	}
} //END MAIN CODE


//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1) //计数器TIME0是否到0   1 到0   0未到0
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag 清楚计数器到0标志，写1清楚。写0无效

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0
		VTimer1[1]++;			// virtual timer 1, instance 1 (used by DSP280xx_SciCommsGui.c)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;	// Back to State A0
}


//=================================================================================
//	A - TASKS
//=================================================================================
//--------------------------------------------------------
void A1(void) // Control Coefficient re-calculations
//=====================================================================
{
// Fault management
	if ( (*ePWM[1]).TZFLG.bit.OST == 1 )  // Trip Zones One Shot Int flag 单次阻塞中断标志
	{
		FaultFlg = 1;
	}
	else FaultFlg = 0;

	Pgain 	= Pgain_Gui*67108;		// Q26
	Igain 	= Igain_Gui*67108;		// Q26
	Dgain 	= Dgain_Gui*67108;		// Q26

if (coeff_change == 1)//PID与POLE ZORO 系数变换标志
{
//	EPwm1Regs.ETSEL.bit.INTEN = 0;				// Disable INT
	No_2p2z = 1;	// Used to disable 2P2Z execution when coefficients are being changed

if (pid2p2z_Gui == 0)
{
// Voltage loop coefficient update
	CNTL_2P2Z_CoefStruct1.b2   = Dgain;                            	// B2
    CNTL_2P2Z_CoefStruct1.b1   = (Igain - Pgain - Dgain - Dgain);  	// B1
    CNTL_2P2Z_CoefStruct1.b0   = (Pgain + Igain + Dgain);      		// B0
    CNTL_2P2Z_CoefStruct1.a2   = 0.0;                              	// A2 = 0
    CNTL_2P2Z_CoefStruct1.a1   = _IQ26(1.0);                       	// A1 = 1
//    CNTL_2P2Z_CoefStruct1.max  =Dmax;					  	 		//Clamp Hi
//    CNTL_2P2Z_CoefStruct1.min  =_IQ24(0.0); 					  	//Clamp Min
}
else
{
	CNTL_2P2Z_CoefStruct1.b2 = b2_Gui*65536;						// B2 - I5Q10 scaled to I5Q26
	CNTL_2P2Z_CoefStruct1.b1 = b1_Gui*65536;						// B1
	CNTL_2P2Z_CoefStruct1.b0 = b0_Gui*65536;						// B0
	CNTL_2P2Z_CoefStruct1.a2 = a2_Gui*65536;						// A2
	CNTL_2P2Z_CoefStruct1.a1 = a1_Gui*65536;						// A1
//    CNTL_2P2Z_CoefStruct1.max  =Dmax;					  	 		//Clamp Hi
//    CNTL_2P2Z_CoefStruct1.min  =_IQ24(0.0); 					  	//Clamp Min
}

//	EPwm1Regs.ETSEL.bit.INTEN = 1;				// Enable INT
	No_2p2z = 0;

	coeff_change = 0;
}

	EALLOW;
	Comp1Regs.DACVAL.bit.DACVAL = ((Ipri_trip)>>5);	// DAC Value is in Q10
	EDIS;

	//-------------------
	A_Task_Ptr = &A2;
	//-------------------
}

//=====================================================================
void A2(void)  // Slew Rate,  SCI GUI
//-----------------------------------------------------------------
{
// This is an example code for implementing the slew rate control in
// a slower state machine instead of implementing it in the ISR.
// VfbSlewRate should be set as a positive value
Vfb_slew_temp = Vref - VfbSetSlewed;

if (Vfb_slew_temp >= VfbSlewRate) // Positive Command
{
	VfbSetSlewed = VfbSetSlewed + VfbSlewRate;
}
else
	{
	if ((-1)*(Vfb_slew_temp) >= VfbSlewRate) // Negative Command
		{
		VfbSetSlewed = VfbSetSlewed - VfbSlewRate;
		}
	}

	SerialHostComms();

	//-------------------
	A_Task_Ptr = &A1;	// To make task A3 active, change &A1 to &A3
	//-------------------
}
/*
//=======================================================================
void A3(void) // SPARE (not active)
//=======================================================================
{

	//-----------------
	A_Task_Ptr = &A1;	// To make task A4 active, change &A1 to &A4
	//-----------------
}

//=====================================================================
void A4(void) //  SPARE (not active)
//=====================================================================
{

	//-----------------
	A_Task_Ptr = &A1;	// After Task A4, start over with task A1
	//-----------------
}*/


//%%%%%%%%%%%%%%%    B-Tasks:   %%%%%%%%%%%%%%%%%%%%%%%%%
//=====================================================================
void B1(void) // Voltage and Current Dashboard measurements
//=====================================================================
{
// Voltage measurement calculated by:
//	Gui_Vfbin = VfbinAvg * K_Vfbin, where VfbinAvg = sum of 8 Vfb_inR samples
//	Gui_Vfbout = VfboutAvg * K_Vfbout, where VfboutAvg = sum of 8 Vfb_outR samples

	HistPtr++;
	if (HistPtr >= 8)	HistPtr = 0;

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------
	Hist_Vfbin[HistPtr] = Vfb_inR; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vfbin[i]; // Q12 * 8 = Q15
	Gui_Vfbin = ( (long) temp_Scratch * (long) K_Vfbin ) >> 15; // (Q15 * Q15)>>15 = Q15

// BoxCar Averages - Input Raw samples into History arrays
//----------------------------------------------------------------
	Hist_Vfbout[HistPtr] = Vfb_outR; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Vfbout[i]; // Q12 * 8 = Q15
	Gui_Vfbout = ( (long) temp_Scratch * (long) K_Vfbout ) >> 15; // (Q15 * Q15)>>15 = Q15

// Voltage Meas
//----------------------------------------------------------------
// view following variables in Watch Window as:
//		Gui_Vfbin = Q5
//		Gui_Vfbout = Q10

// Current measurement calculated by:
//	Gui_Ifb = IfbAvg * K_Ifb, where IfbAvg = sum of 8 IfbR samples
//BoxCar Averages - Input Raw samples into History arrays

	Hist_Ifb[HistPtr] = IfbR; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Ifb[i]; // Q12 * 8 = Q15
	Gui_Ifb = ( (long) temp_Scratch * (long) K_Ifb ) >> 15; // (Q15 * Q15)>>15 = Q15


	Hist_Iout[HistPtr] = IoutR; // Raw ADC result (Q12)

	temp_Scratch=0;
	for(i=0; i<HistorySize; i++)	temp_Scratch = temp_Scratch + Hist_Iout[i]; // Q12 * 8 = Q15
	temp_Scratch = ( ((long) temp_Scratch * (long) K_Iout ) >> 15) - sub_adj; // (Q15 * Q15)>>15 = Q15 (Offset Adjust)
	if (temp_Scratch < 0) temp_Scratch = 0;
	Gui_Iout = temp_Scratch;

// Current Meas
//----------------------------------------------------------------
// view following variables in Watch Window as:
//		Gui_Ifb = Q12

// Voltage setting calculated by:
// Vref = Gui_VfbSet * iK_Vfbout, where iK_Vfbout = 1/K_Vfbout (i.e. inverse K_Vfbout)
// view and set following variable in Watch Window as:
//		Gui_VfbSet = Q10 (Used as Q15 below)

if (Gui_VfbSet > 250)	// If command is greater than a certain minimum
	Vref = ( (long) Gui_VfbSet * (long) iK_Vfbout ) >> 5; // (Q15 * Q14) >> 5 = Q24
else					// If FB is disabled, start from a small value
	Vref = 250;			// Re-initialise to a small command

	B_Task_Ptr = &B1;
	//-----------------
}
/*
//=====================================================================
void B2(void) // SPARE
//=====================================================================
{
	//-----------------
	B_Task_Ptr = &B1;
	//-----------------
}

//=====================================================================
void B3(void) // SPARE (not active)
//=====================================================================
{
	//-----------------
	B_Task_Ptr = &B1;
	//-----------------
}

//=====================================================================
void B4(void) //  SPARE (not active)
//=====================================================================
{

	//-----------------
	B_Task_Ptr = &B1;
	//-----------------
}
*/


//%%%%%%%%%%%%%%%    C-Tasks:   %%%%%%%%%%%%%%%%%%%%%%%%%
//=====================================================================
//=====================================================================
void C1(void)
//=====================================================================
{
 if (auto_DB == 1)
 {
  switch (range)
	{
	case 0:
			if (Gui_Iout < 640) 				    // 5A
			{
/*				if (dbAtoP_leg<36) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>36) dbAtoP_leg--;
				}

				if (dbPtoA_leg<44) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>44) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 36;
				dbPtoA_leg = 44;
				range = 0;
			}
			else  range = 1;
			break;

	case 1:
			if (Gui_Iout > 512 && Gui_Iout < 1024) // 4A & 8A
			{
/*				if (dbAtoP_leg<32) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>32) dbAtoP_leg--;
				}

				if (dbPtoA_leg<42) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>42) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 32;
				dbPtoA_leg = 42;
				range = 1;
			}
			else
			{
				if (Gui_Iout > 1024) range = 2;
				else range = 0;
			}
			break;

	case 2:
			if (Gui_Iout > 896 && Gui_Iout < 1408) // 7A & 11A
			{
/*				if (dbAtoP_leg<27) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>27) dbAtoP_leg--;
				}

				if (dbPtoA_leg<22) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>22) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 27;
				dbPtoA_leg = 22;
				range = 2;
			}
			else
			{
				if (Gui_Iout > 1408) range = 3;
				else range = 0;

			}
			break;

	case 3:
			if (Gui_Iout > 1280 && Gui_Iout < 1792)  // 10A & 14A
			{
/*				if (dbAtoP_leg<22) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>22) dbAtoP_leg--;
				}

				if (dbPtoA_leg<22) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>22) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 22;
				dbPtoA_leg = 22;
				range = 3;
			}
			else
			{
				if (Gui_Iout > 1792) range = 4;
				else range = 0;
			}
			break;

	case 4:
			if (Gui_Iout > 1664 && Gui_Iout < 2176)  // 13A & 17A
			{
/*				if (dbAtoP_leg<20) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>20) dbAtoP_leg--;
				}

				if (dbPtoA_leg<21) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>21) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 20;
				dbPtoA_leg = 21;
				range = 4;
			}
			else
			{
				if (Gui_Iout > 2176) range = 5;
				else range = 0;
			}
			break;


	case 5:
			if (Gui_Iout > 2048 && Gui_Iout < 2560)  // 16A & 20A
			{
/*				if (dbAtoP_leg<20) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>20) dbAtoP_leg--;
				}

				if (dbPtoA_leg<18) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>18) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 20;
				dbPtoA_leg = 18;
				range = 5;
			}
			else
			{
				if (Gui_Iout > 2560) range = 6;
				else range = 0;
			}
			break;


	case 6:
			if (Gui_Iout > 2432 && Gui_Iout < 2944)  // 19A & 23A
			{
/*				if (dbAtoP_leg<18) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>18) dbAtoP_leg--;
				}

				if (dbPtoA_leg<18) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>18) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 18;
				dbPtoA_leg = 18;
				range = 6;
			}
			else
			{
				if (Gui_Iout > 2944) range = 7;
				else range = 0;
			}
			break;


	case 7:
			if (Gui_Iout > 2816 && Gui_Iout < 3328)  // 22A & 26A
			{
/*				if (dbAtoP_leg<18) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>18) dbAtoP_leg--;
				}

				if (dbPtoA_leg<14) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>14) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 18;
				dbPtoA_leg = 14;
				range = 7;
			}
			else
			{
				if (Gui_Iout > 3328) range = 8;
				else range = 0;
			}
			break;




	case 8:
			if (Gui_Iout > 3200 && Gui_Iout < 4096)  // 25A & 32A
			{
/*				if (dbAtoP_leg<16) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>16) dbAtoP_leg--;
				}

				if (dbPtoA_leg<14) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>14) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 16;
				dbPtoA_leg = 14;
				range = 8;
			}
			else
			{
				if (Gui_Iout > 4096) range = 9;
				else range = 0;
			}
			break;


	case 9:
			if (Gui_Iout > 3968 && Gui_Iout < 4864)  // 31A & 38A
			{
/*				if (dbAtoP_leg<15) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>15) dbAtoP_leg--;
				}

				if (dbPtoA_leg<13) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>13) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 15;
				dbPtoA_leg = 13;
				range = 9;
			}
			else
			{
				if (Gui_Iout > 4864) range = 10;
				else range = 0;
			}
			break;

	case 10:
			if (Gui_Iout > 4736 && Gui_Iout < 5632)  // 37A & 44A
			{
/*				if (dbAtoP_leg<14) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>14) dbAtoP_leg--;
				}

				if (dbPtoA_leg<12) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>12) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 14;
				dbPtoA_leg = 12;
				range = 10;
			}
			else
			{
				if (Gui_Iout > 5632) range = 11;
				else range = 0;
			}
			break;


	case 11:
			if (Gui_Iout > 5504) 					   // 43A
			{
/*				if (dbAtoP_leg<12) dbAtoP_leg++;
				else
				{
					if (dbAtoP_leg>12) dbAtoP_leg--;
				}

				if (dbPtoA_leg<12) dbPtoA_leg++;
				else
				{
					if (dbPtoA_leg>12) dbPtoA_leg--;
				}*/

				dbAtoP_leg  = 12;
				dbPtoA_leg = 12;
				range = 11;
			}
			else range = 0;
			break;
	}

 }

// Following code allows changing SR mode directly from watch view or GUI
  switch (SR_mode)
	{
	case 0:
			EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;	// Req'd when coming from Mode 1
			EPwm4Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;	// Req'd when coming from Mode 2

			EPwm4Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;	// Req'd when coming from Mode 1
			EPwm4Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;	// Req'd when coming from Mode 2

			break;

	case 1:
			EPwm4Regs.AQCTLA.bit.CBU = AQ_CLEAR; 		// Req'd when coming from Mode 0 or 2
			EPwm4Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;	// Req'd when coming from Mode 2
			EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;			// Req'd when coming from Mode 0

			EPwm4Regs.AQCTLB.bit.CAD = AQ_CLEAR;		// Req'd when coming from Mode 0 or 2
			EPwm4Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;	// Req'd when coming from Mode 2
			EPwm4Regs.AQCTLB.bit.PRD = AQ_SET;			// Req'd when coming from Mode 0

			break;

	case 2:
			EPwm4Regs.AQCTLA.bit.CBU = AQ_NO_ACTION; 	// Req'd when coming from Mode 1
			EPwm4Regs.AQCTLA.bit.CAD = AQ_SET; 			// Req'd when coming from Mode 1
			EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;			// Req'd when coming from Mode 0

			EPwm4Regs.AQCTLB.bit.CAD = AQ_NO_ACTION;	// Req'd when coming from Mode 0 or 2
			EPwm4Regs.AQCTLB.bit.CBU = AQ_SET; 			// Req'd when coming from Mode 1
			EPwm4Regs.AQCTLB.bit.PRD = AQ_SET;			// Req'd when coming from Mode 0

			break;
	}

if(Auto_Run == 1)
	//-----------------
	C_Task_Ptr = &C2;
	//-----------------
else
	//-----------------
	C_Task_Ptr = &C1;
	//-----------------

}

//=====================================================================
void C2(void) //  SPARE (not active)
//=====================================================================
{

if (Gui_Vfbin > 11200 && Gui_Vfbin < 13440)  	// 350<Vfbin<420
{
	if (input_good >= 100)						// Input has been good for 100*(C2 task execution rate)
	{
		if (start_flag == 0)
		{
			VfbSetSlewed = 2093568; 			// Start ramping up from 2V (Q24)
			Gui_VfbSet = 12.2*1024;				// Desired Output * 2^10
			start_flag = 1;
		}
	}
	else
	{
		input_good++;
	}

}
else
{
	input_good = 0;
	Gui_VfbSet = 2*1024;	//2V
	Dmax = _IQ24(0.25);
	CNTL_2P2Z_CoefStruct1.max = Dmax;
	start_flag = 0;
}

Gui_Pout =  ((long)(Gui_Iout) * (long)(Gui_Vfbout))>>11;	// Q7*Q10>>11. Gui_Pout in Q6

if (start_flag == 1)
{
		if (Dmax < _IQ24(0.984375))
		{
			Dmax = Dmax + _IQ24(0.001);
			CNTL_2P2Z_CoefStruct1.max = Dmax;
		}
		else
		{
			Dmax = _IQ24(0.984375);
			CNTL_2P2Z_CoefStruct1.max = Dmax;
		}
}
else 	// start_flag = 0. FB is disabled, re-intialise Dmax to a low value
	{
		Dmax = _IQ24(0.25);
		CNTL_2P2Z_CoefStruct1.max = Dmax;
	}

	//-----------------
	C_Task_Ptr = &C1;
	//-----------------
}

/*
//=====================================================================
void C4(void) //  SPARE (not active)
//=====================================================================
{
	//-----------------
	C_Task_Ptr = &C1;
	//-----------------
}*/


