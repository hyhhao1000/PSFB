//============================================================================
//============================================================================
//
// FILE:	SciCommsGui.c
//
// TITLE:	GP Comms kernel as an interface to external GUI
// 
// Version: 20 Feb 2008 - Release 1 - Internal Release (BRL)
//============================================================================
//============================================================================
#include "PeripheralHeaderIncludes.h"

#define	PktSize				6
#define	CmdNumber			16
#define	MAX_CMD_NUM			16


// Function prototypes for Command RECEIVE State machine
// ------------------------------------------------------------
void GetCmdByte(void);
void EchoCmdByte(void);
void GetSizeByte(void);
void EchoSizeByte(void);
void GetDataByte(void);
void EchoDataByte(void);
void PackWord(void);
void PackArray(void);
void CmdInterpreter(void);

// Function prototypes for Command Interpreter and dispatcher
// ------------------------------------------------------------
void LifePulseTsk(void);		// 0
void TextSet(void);				// 1
void ButtonSet(void);			// 2
void SliderSet(void);			// 3
void VariableGet(void);			// 4
void ArrayGet(void);			// 5
void DataGet(void);				// 6
void SpareTsk07(void);			// 7
void SpareTsk08(void);			// 8
void SpareTsk09(void);			// 9
void SpareTsk10(void);			// 10
void SpareTsk11(void);			// 11
void SpareTsk12(void);			// 12

void SendData(void);

// Variable declarations
void (*RcvTaskPointer)(void); 			// State pointer for Command Packet Receive
void (*CmdDispatcher[CmdNumber])(void);	// Array of pointers to Function (i.e. tasks)

extern int *varSetTxtList[];
extern int *varSetBtnList[];
extern int *varSetSldrList[];
extern int *varGetList[];
extern int *arrayGetList[];
extern int *dataGetList[];

extern int16 CommsOKflg, VTimer0[1];

Uint16	LowByteFlag, SendTaskPtr;
Uint16	RxChar, RxWord;
Uint16	CmdPacket[PktSize];
Uint16	TaskDoneFlag, j, NumWords, wordsLeftToGet;

Uint16 dataOut;
int16 *memDataPtr;

int16  RcvTskPtrShdw;	// for debug

int16 	delayer;

int16	MemGetPtr;
Uint32	MemGetAddress;
int16	MemGetAmount;

Uint32 Temp;

int skipval = 1;

/*//long temp;
int* DLOG_point = (int16*)0xB000;
int* DLOG_point_vac = (int16*)0xB400; */


void SCIA_Init()
{    
// Note: Assumes Clocks to SCIA are turned on in InitSysCtrl()
// Note: Assumes GPIO pins for SCIA are configured to Primary function    

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback 
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK, 
                                   // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003; 
	SciaRegs.SCICTL2.bit.TXINTENA =0;
	SciaRegs.SCICTL2.bit.RXBKINTENA =0;
    SciaRegs.SCIHBAUD    =0x0000;

	//#ifdef DSP2833x_DEVICE_H
	//SciaRegs.SCILBAUD = 0x0079;		// 79h = 38.4Kbaud @ LSPCLK = 150/4 MHz
	//SciaRegs.SCILBAUD = 0x0050;			// 50h = 57.6Kbaud @ LSPCLK = 150 /4 MHz
	//SciaRegs.SCILBAUD = 0x0028;		// 28h = 115.2Kbaud @ LSPCLK = 150/4 MHz
	//#else
	//SciaRegs.SCILBAUD  =0x00A2;		// A2h = 19.2Kbaud @ LSPCLK = 100/4 MHz
	//SciaRegs.SCILBAUD  =0x0050;		// 50h = 38.4Kbaud @ LSPCLK = 100/4 MHz
//	SciaRegs.SCILBAUD = 0x0035;			// 35h = 57.6Kbaud @ LSPCLK = 100/4 MHz
	//SciaRegs.SCILBAUD = 0x001B;		// 1Bh = 115.2Kbaud @ LSPCLK = 100/4 MHz
//	#endif

	SciaRegs.SCILBAUD = 0x0020;			// 35h = 57.6Kbaud @ LSPCLK = 100/4 MHz

	SciaRegs.SCICTL1.all =0x0023;		// Relinquish SCI from Reset 

    //SciaRegs.SCIFFTX.all=0xE040;		// ENable FIFO enhancement
    SciaRegs.SCIFFTX.all=0x8040;		// DISable FIFO enhancement
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;
    SciaRegs.SCIPRI.bit.SOFT=0x0;
    SciaRegs.SCIPRI.bit.FREE=0x1;

	RcvTaskPointer = &GetCmdByte;			// Initialize the CmdPacket Rcv Handler state machine ptr
	RcvTskPtrShdw = 1;						// DEBUG
	SendTaskPtr = 0;						// Init to 1st state
	LowByteFlag = 1;						// Start with LSB during Byte-to-Word packing

	dataOut = 0;
	*memDataPtr = 0;

	RcvTskPtrShdw = 0;	// for debug

	delayer = 0;

	MemGetPtr = 0;
	MemGetAddress = 0x00000000;
	MemGetAmount = 0;

	// clear Command Packet
	for (j=0; j<PktSize; j++)
	{
		CmdPacket[j] = 0x0;
	}

	j=0;
	
// init all dispatch Tasks                   
 	CmdDispatcher[0] = LifePulseTsk;             
 	CmdDispatcher[1] = TextSet;                  
 	CmdDispatcher[2] = ButtonSet;                
 	CmdDispatcher[3] = SliderSet;                
 	CmdDispatcher[4] = VariableGet;              
 	CmdDispatcher[5] = ArrayGet;                 
 	CmdDispatcher[6] = DataGet;                  
 	CmdDispatcher[7] = SpareTsk07;               
 	CmdDispatcher[8] = SpareTsk08;               
 	CmdDispatcher[9] = SpareTsk09;               
 	CmdDispatcher[10] = SpareTsk10;              
 	CmdDispatcher[11] = SpareTsk11;              
 	CmdDispatcher[12] = SpareTsk12;              
                                               
}

//===========================================================
// Host Command RECEIVE and DISPATCH State Machine
//===========================================================

//======== SM Entry Point =================
void SerialHostComms()
{		
	(*RcvTaskPointer)();	// Call routine pointed to by state pointer
}
//=========================================

void GetCmdByte(void) // Task 1
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoCmdByte;		// point to next state
		VTimer0[1] = 0;
		//RcvTskPtrShdw = 2;				// DEBUG
		EchoCmdByte();
	}                                                                      

	else if (SciaRegs.SCIRXST.bit.BRKDT == 1 || VTimer0[1] > 2000) //2 ms timeout
	{	
	// If break detected or serialport times out, reset SCI
		//--- Needed by some serialports when code is run with an emulator  
 		SciaRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback             
  	                                 		// No parity,8 char bits,           
  	                                  		// async mode, idle-line protocol   
 		SciaRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK,      
  	                                  		// Disable RX ERR, SLEEP, TXWAKE    
 		SciaRegs.SCICTL2.all =0x0000;                                                                                
                                                                              
 		SciaRegs.SCICTL1.all =0x0023;		// Relinquish SCI from Reset        

		asm(" RPT#8 || NOP");
		//---

		SendTaskPtr = 0;					// Init to 1st state	
		VTimer0[1] = 0;
								                                      
		//CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// go back and wait for new CMD                       
	}
}

void EchoCmdByte(void) // Task 2
{
    if(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
    	CmdPacket[0] = RxChar;
		RcvTaskPointer = &GetSizeByte;
		//RcvTskPtrShdw = 3;					// DEBUG
		//RcvTaskPointer = &GetCmdByte;		// Un-comment for simple echo test
		VTimer0[1] = 0;						// Reset Time-out timer
	}

}

void GetSizeByte(void) // Task 3
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = SciaRegs.SCIRXBUF.all;

		RcvTaskPointer = &EchoSizeByte;		// point to next state
		//RcvTskPtrShdw = 4;					// DEBUG
		EchoSizeByte();
	}

	else if (VTimer0[1] > 500)				// 500*4mS = 2 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		VTimer0[1] = 0;
	}
}

void EchoSizeByte(void) // Task 4
{
    if(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
    	CmdPacket[1] = RxChar;
		RcvTaskPointer = &GetDataByte;
		//RcvTskPtrShdw = 5;				// DEBUG
		//RcvTaskPointer = &GetCmdByte;		// Un-comment for Test
		VTimer0[1] = 0;						// Reset Time-out timer
    }
}

void GetDataByte(void) // Task 5
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)	// check if a char has been received
	{
		RxChar = SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoDataByte;		// point to next state
		//RcvTskPtrShdw = 6;				// DEBUG
		EchoDataByte();				
	}

	else if (VTimer0[1] > 500)				// 500*4mS = 2 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		VTimer0[1] = 0;
	}
	
}

void EchoDataByte(void) // Task 6
{
    if(SciaRegs.SCICTL2.bit.TXRDY == 1)		// is TXBUF empty ?, i.e. TXRDY = 1
    {
    	SciaRegs.SCITXBUF=RxChar;			// if yes, echo back the received char
		RcvTaskPointer = &PackWord;
		//RcvTskPtrShdw = 7;				// DEBUG
    }
}

void PackWord(void) // expects LSB first then MSB // Task 7
{
	if(LowByteFlag == 1)
	{ 
		RxWord = RxChar;
		LowByteFlag = 0;
		RcvTaskPointer = &GetDataByte;
		//RcvTskPtrShdw = 5;					// DEBUG
		GetDataByte();
	}
	else
	{
		RxWord = RxWord | (RxChar<<8);
		LowByteFlag = 1;
		CmdPacket[2] = RxWord;	// store data in packet
		RcvTaskPointer = &CmdInterpreter;
		//RcvTskPtrShdw = 8;					// DEBUG
		TaskDoneFlag = 0; 		// indicate new task underway	
	}
}

void CmdInterpreter(void) // Task 8
{
	if (TaskDoneFlag == 0)
	{
	 	(*CmdDispatcher[ CmdPacket[0] ] )();  	// dispatch Task					
	}

	// Incase Task never finishes 
	if (VTimer0[1] > 500)					// 500*4mS = 2 sec timeout
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;		// Abort, go back wait for new CMD
		VTimer0[1] = 0;
	}
	if (TaskDoneFlag == 1)
	{
		RcvTaskPointer = &GetCmdByte;
	 	//RcvTskPtrShdw = 1;   				// DEBUG
	}
}

//===========================================================
// Slave Tasks commanded by Host
//===========================================================
void LifePulseTsk(void) 	// CmdPacket[0] = 0
{
	if (CmdPacket[2]==0x0000 && CmdPacket[1]==0x00) 
	{
		#ifdef DSP2803x_DEVICE_H
		GpioDataRegs.GPBSET.bit.GPIO34=1;		//LED2-ON
		#else
		GpioDataRegs.GPBSET.bit.GPIO38=1;
		#endif
	}
	if (CmdPacket[2]==0x0001 && CmdPacket[1]==0x00) 
	{
		#ifdef DSP2803x_DEVICE_H
		GpioDataRegs.GPBCLEAR.bit.GPIO34=1;		//LED2-OFF
		#else
		GpioDataRegs.GPBCLEAR.bit.GPIO38=1;
		#endif
	}
	if (CmdPacket[2]==0x0002 && CmdPacket[1]==0x00)
	{
		#ifdef DSP2803x_DEVICE_H
		GpioDataRegs.GPBTOGGLE.bit.GPIO34=1;	//LED2-Toggle
		#else
		GpioDataRegs.GPBTOGGLE.bit.GPIO38=1;
		#endif
	}

	CommsOKflg = 1;
	VTimer0[1] = 0;
	TaskDoneFlag = 1;
}
//-----------------------------------------------------------
void TextSet(void) // CmdPacket[0] = 1
{	
	*varSetTxtList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void ButtonSet(void) 	// CmdPacket[0] = 2
{
	*varSetBtnList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void SliderSet(void) 	// CmdPacket[0] = 3
{
	*varSetSldrList[CmdPacket[1]] = CmdPacket[2];

	TaskDoneFlag = 1; // indicate Task execution is complete
}
//-----------------------------------------------------------
void VariableGet(void)		// CmdPacket[0] = 4
{
	SendData();
}
//-----------------------------------------------------------
//Send a Uint16 array one element at a time
void ArrayGet(void)			// CmdPacket[0] = 5
{
	SendData();	
}
//-----------------------------------------------------------
void DataGet(void)			// CmdPacket[0] = 6
{
	switch(MemGetPtr)
	{
		case 0:
			MemGetAddress = CmdPacket[2];
			MemGetPtr = 1;

			wordsLeftToGet = 1;
			SendTaskPtr = 1;
			TaskDoneFlag = 1;
			break;

		case 1:
			Temp = CmdPacket[2];
			MemGetAddress = MemGetAddress + (Temp<<16);
			memDataPtr = (int16*)MemGetAddress;
			dataOut = *memDataPtr;		
			SendData();		

			if(TaskDoneFlag == 1)
			{
				MemGetPtr = 0;
			}
			break;
		}

	//TaskDoneFlag = 1; 	// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk07(void)		// CmdPacket[0] = 7
{
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk08(void)		// CmdPacket[0] = 8
{
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk09(void)		// CmdPacket[0] = 9
{
	// DO NOTHING
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk10(void)		// CmdPacket[0] = 10
{
	// DO NOTHING
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk11(void)		// CmdPacket[0] = 11
{
	// DO NOTHING
	TaskDoneFlag = 1; 		// indicate Task execution is complete
}
//-----------------------------------------------------------
void SpareTsk12(void) 		// CmdPacket[0] = 12
{
	// DO NOTHING
		TaskDoneFlag = 1;
}
//-----------------------------------------------------------
void SendData(void)
{
	switch(SendTaskPtr)
	{
	case 0:  //initialization 
		if(CmdPacket[0] == 0x04)
		{
			memDataPtr = (int16 *) varGetList[CmdPacket[1]];
		}
		else
		{
			memDataPtr = (int16 *) arrayGetList[CmdPacket[1]];
		}
		
		dataOut = *memDataPtr;
		wordsLeftToGet = CmdPacket[2];
		//Note that case 0 rolls into case 1 (no break)

	case 1:  //send LSB
		if(wordsLeftToGet > 0)
		{
			if (SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				SciaRegs.SCITXBUF = dataOut & 0x000000FF;
				SendTaskPtr = 2;
			}
			else
			{
				TaskDoneFlag = TaskDoneFlag;
				break;
			}
		}
		else
		{
			SendTaskPtr = 0;
			TaskDoneFlag = 1;
			break;
		}		

	case 2: //send MSB
		if (SciaRegs.SCICTL2.bit.TXRDY == 1)
		{
			SciaRegs.SCITXBUF = (dataOut>>8 & 0x000000FF);

	if (CmdPacket[0] == 0x04)
			memDataPtr = memDataPtr + 1;
	else
		{
		  	memDataPtr = memDataPtr + skipval;
		}
			dataOut = *memDataPtr;
			wordsLeftToGet = wordsLeftToGet - 1;
			SendTaskPtr = 1;
		}
		break;
	}
}





















