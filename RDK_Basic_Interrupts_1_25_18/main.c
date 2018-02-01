/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/	
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "stdtypes.h"
#include "config.h"
#include "MtrCtrl.h"
#include "spi.h"
#include "util.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer 
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable
    
#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#define	stPressed	1
#define	stReleased	0

#define	cstMaxCnt	10 // number of consecutive reads required for
					   // the state of a button to be updated

struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button 
					// state
};

//PmodCLS instructions
static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0}; 
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0}; 
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0}; 

static	char szCursorPosRow1[] = {0x1B, '[', '1', ';', '0', 'H', 0}; 
static	char szCursorPosHome[] = {0x1B, '[', '0', ';', '0', 'H', 0}; 
/* ------------------------------------------------------------ */
/*				Global Variables				                */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;

unsigned int IC2Counter = 0;
unsigned int IC3Counter = 0;

int delta_time;

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);
void	Wait_ms(WORD ms);

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */
/***	Timer5Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
*/

// ISRs - edit and uncomment later

void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl6) _IC2_IntHandler(void) 
{
    static int time = 0;
    static int prev_time = 0;
	IFS0CLR	= ( 1 << 9 );	// clear interrupt flag for Input Capture 2
    
    //Reading the buffer
    
    
    while((IC2CON & (1 << 3)) == (1 << 3)) //use a define
    {
        time = (int) IC2BUF;
    }
    
    /*//Code routine to disable/enable interrupts and ISR
    INTDisableInterrupts();
    IEC0SET	= ( 0 << 9); // Sets IEC0 Bit 9 to 1 (IC2 interrupt enabled)
    IEC0SET	= ( 1 << 9); // Sets IEC0 Bit 9 to 1 (IC2 interrupt enabled)
    INTEnableInterrupts();*/
    
	IC2Counter++;
    
    prev_time = time;
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl6) _IC3_IntHandler(void) // ipl = interrupt priority level
{
    static int time = 0;
    static int prev_time = 0;
    
	IFS0CLR	= ( 1 << 13 );	// clear interrupt flag for Input Capture 3	
    
    //Reading the buffer
    
    while((IC3CON & (1 << 3)) == (1 << 3)) //use a define
    {
        time = (int) IC3BUF;
    }
    
    /*//Code routine to disable/enable interrupts and ISR
    INTDisableInterrupts();
    IEC0SET	= ( 0 << 13); // Sets IEC0 Bit 13 to 1 (IC3 interrupt enabled)    
    IEC0SET	= ( 1 << 13); // Sets IEC0 Bit 13 to 1 (IC3 interrupt enabled)
    INTEnableInterrupts();*/
    
	IC3Counter++;
    
    prev_time = time;
}

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{
	static	WORD tusLeds = 0;
	static int count = 0;
	mT5ClearIntFlag();
    count++;
        if(count > 10000)
        {
            count = 0;
            IC3Counter++;  
        }
        

    
	// Read the raw state of the button pins.
	btnBtn1.stCur = ( prtBtn1 & ( 1 << bnBtn1 ) ) ? stPressed : stReleased;
	btnBtn2.stCur = ( prtBtn2 & ( 1 << bnBtn2 ) ) ? stPressed : stReleased;
	
	//Read the raw state of the PmodBTN pins
	PmodBtn1.stCur = ( prtJE1 & ( 1 << bnJE1 ) ) ? stPressed : stReleased;
	PmodBtn2.stCur = ( prtJE2 & ( 1 << bnJE2 ) ) ? stPressed : stReleased;
	PmodBtn3.stCur = ( prtJE3 & ( 1 << bnJE3 ) ) ? stPressed : stReleased;
	PmodBtn4.stCur = ( prtJE4 & ( 1 << bnJE4 ) ) ? stPressed : stReleased;

	//Read the raw state of the PmodSWT pins
	PmodSwt1.stCur = ( prtJA1 & ( 1 << swtJA1 ) ) ? stPressed : stReleased;
	PmodSwt2.stCur = ( prtJA2 & ( 1 << swtJA2 ) ) ? stPressed : stReleased;
	PmodSwt3.stCur = ( prtJA3 & ( 1 << swtJA3 ) ) ? stPressed : stReleased;
	PmodSwt4.stCur = ( prtJA4 & ( 1 << swtJA4 ) ) ? stPressed : stReleased;

	// Update state counts.
	btnBtn1.cst = ( btnBtn1.stCur == btnBtn1.stPrev ) ? btnBtn1.cst + 1 : 0;
	btnBtn2.cst = ( btnBtn2.stCur == btnBtn2.stPrev ) ? btnBtn2.cst + 1 : 0;

	//Update state counts for PmodBTN
	PmodBtn1.cst = (PmodBtn1.stCur == PmodBtn1.stPrev) ? PmodBtn1.cst +1 : 0;
	PmodBtn2.cst = (PmodBtn2.stCur == PmodBtn2.stPrev) ? PmodBtn2.cst +1 : 0;
	PmodBtn3.cst = (PmodBtn3.stCur == PmodBtn3.stPrev) ? PmodBtn3.cst +1 : 0;
	PmodBtn4.cst = (PmodBtn4.stCur == PmodBtn4.stPrev) ? PmodBtn4.cst +1 : 0;

	//Update state counts for PmodSWT
	PmodSwt1.cst = (PmodSwt1.stCur == PmodSwt1.stPrev) ? PmodSwt1.cst +1 : 0;
	PmodSwt2.cst = (PmodSwt2.stCur == PmodSwt2.stPrev) ? PmodSwt2.cst +1 : 0;
	PmodSwt3.cst = (PmodSwt3.stCur == PmodSwt3.stPrev) ? PmodSwt3.cst +1 : 0;
	PmodSwt4.cst = (PmodSwt4.stCur == PmodSwt4.stPrev) ? PmodSwt4.cst +1 : 0;
	
	// Save the current state.
	btnBtn1.stPrev = btnBtn1.stCur;
	btnBtn2.stPrev = btnBtn2.stCur;

	// Save the current state for PmodBTN
	PmodBtn1.stPrev = PmodBtn1.stCur;
	PmodBtn2.stPrev = PmodBtn2.stCur;
	PmodBtn3.stPrev = PmodBtn3.stCur;
	PmodBtn4.stPrev = PmodBtn4.stCur;

	// Save the current state for PmodSWT
	PmodSwt1.stPrev = PmodSwt1.stCur;
	PmodSwt2.stPrev = PmodSwt2.stCur;
	PmodSwt3.stPrev = PmodSwt3.stCur;
	PmodSwt4.stPrev = PmodSwt4.stCur;
	
	// Update the state of button 1 if necessary.
	if ( cstMaxCnt == btnBtn1.cst ) {
		btnBtn1.stBtn = btnBtn1.stCur;
		btnBtn1.cst = 0;
	}
	
	// Update the state of button 2 if necessary.
	if ( cstMaxCnt == btnBtn2.cst ) {
		btnBtn2.stBtn = btnBtn2.stCur;
		btnBtn2.cst = 0;
	}

	//if statements for buttons

	// Update the state of PmodBTN1 if necessary.
	if ( cstMaxCnt == PmodBtn1.cst ) {
		PmodBtn1.stBtn = PmodBtn1.stCur;
		PmodBtn1.cst = 0;
	}
	
	// Update the state of PmodBTN2 if necessary.
	if ( cstMaxCnt == PmodBtn2.cst ) {
		PmodBtn2.stBtn = PmodBtn2.stCur;
		PmodBtn2.cst = 0;
	}

	// Update the state of PmodBTN3 if necessary.
	if ( cstMaxCnt == PmodBtn3.cst ) {
		PmodBtn3.stBtn = PmodBtn3.stCur;
		PmodBtn3.cst = 0;
	}

	// Update the state of PmodBTN4 if necessary.
	if ( cstMaxCnt == PmodBtn4.cst ) {
		PmodBtn4.stBtn = PmodBtn4.stCur;
		PmodBtn4.cst = 0;
	}

	//if statements for switches

	// Update the state of PmodSWT1 if necessary.
	if ( cstMaxCnt == PmodSwt1.cst ) {
		PmodSwt1.stBtn = PmodSwt1.stCur;
		PmodSwt1.cst = 0;
	}
	
	// Update the state of PmodSWT2 if necessary.
	if ( cstMaxCnt == PmodSwt2.cst ) {
		PmodSwt2.stBtn = PmodSwt2.stCur;
		PmodSwt2.cst = 0;
	}

	// Update the state of PmodSWT3 if necessary.
	if ( cstMaxCnt == PmodSwt3.cst ) {
		PmodSwt3.stBtn = PmodSwt3.stCur;
		PmodSwt3.cst = 0;
	}

	// Update the state of PmodSWT4 if necessary.
	if ( cstMaxCnt == PmodSwt4.cst ) {
		PmodSwt4.stBtn = PmodSwt4.stCur;
		PmodSwt4.cst = 0;
	}

}

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/

int main(void) {
    
    
	BYTE	stBtn1;
	BYTE	stBtn2;

	BYTE	stPmodBtn1;
	BYTE	stPmodBtn2;
	BYTE	stPmodBtn3;
	BYTE	stPmodBtn4;

	BYTE	stPmodSwt1;
	BYTE	stPmodSwt2;
	BYTE	stPmodSwt3;
	BYTE	stPmodSwt4;

    char bufftemp [50]; // used for information written to display
    
	DeviceInit();
	AppInit();

	//INTDisableInterrupts();
	DelayMs(500);
	

	//write to PmodCLS
    int n_2 = sprintf(bufftemp ,"IC2Count: %i", IC2Counter);
    
	SpiEnable();
	SpiPutBuff(szClearScreen, 3);
	DelayMs(4);
	SpiPutBuff(szBacklightOn, 4);
	DelayMs(4);
	SpiPutBuff(szCursorOff, 4);
	DelayMs(4);
	SpiPutBuff(bufftemp, n_2);
    //SpiPutBuff("IC2Count: %d", 9);
    int n_3 = sprintf(bufftemp ,"IC3Count: %i", IC3Counter);
	DelayMs(4);
	SpiPutBuff(szCursorPosRow1, 6);
	DelayMs(4);
	SpiPutBuff(bufftemp, n_3);
	DelayMs(2000);
	SpiDisable();

	prtLed1Set	= ( 1 << bnLed1 );
	//INTEnableInterrupts();
	while (fTrue)
	{		
        
        
        //write to PmodCLS
    
    n_2 = sprintf(bufftemp ,"IC2Count: %i", IC2Counter);
    
	SpiEnable();
	DelayMs(1);
    SpiPutBuff(szCursorPosHome, 6);
	SpiPutBuff(bufftemp, n_2);
    //SpiPutBuff("IC2Count: %d", 9);
    n_3 = sprintf(bufftemp ,"IC3Count: %i", IC3Counter);
	DelayMs(1);
	SpiPutBuff(szCursorPosRow1, 6);
	SpiPutBuff(bufftemp, n_3);
	SpiDisable();
        
        
		INTDisableInterrupts();
	
		//get data here
		stBtn1 = btnBtn1.stBtn;
		stBtn2 = btnBtn2.stBtn;

		stPmodBtn1 = PmodBtn1.stBtn;
		stPmodBtn2 = PmodBtn2.stBtn;
		stPmodBtn3 = PmodBtn3.stBtn;
		stPmodBtn4 = PmodBtn4.stBtn;

		stPmodSwt1 = PmodSwt1.stBtn;
		stPmodSwt2 = PmodSwt2.stBtn;
		stPmodSwt3 = PmodSwt3.stBtn;
		stPmodSwt4 = PmodSwt4.stBtn;

		INTEnableInterrupts();
		//configure OCR to go forward

		if(stPressed == stPmodBtn1){
			//start motor if button 2 pressed

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn2){
			//start left turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlBwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn3){
			//start right turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodBtn4){
			//start move backward

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlLeft();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt1){
			//make square to right

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();		// first turn
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();     // second turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();		// third turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt2){
			//make triangle to left

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00); //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();  	//first turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//second turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//third turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
		
		}else if(stPressed == stPmodSwt3){
			// Three point turn around

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdRight();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlBwdLeft();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();

		}else if(stPressed == stPmodSwt4){
			// dance
			
			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdLeft(); // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();		
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdLeft();  // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlLeft();     // spin
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
		}  //end if

	}  //end while
}  //end main

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes on chip peripheral devices to the default
**		state.
*/

void DeviceInit() {

    //Set IC2 and IC3 as inputs (they are on PORTD Pins 9/10 respectively)
    TRISDSET = (1 << 9) | (1 << 10);
    
	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );    // setting up Port D (the way we connected the hardware defines this))
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward (left needs to be a 0)
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward (right needs to be a 1)

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= ( 1 << 2 ) | ( 1 << 1 );	// pwm set up
	OC2R	= dtcMtrStopped; //sets speed to 0
	OC2RS	= dtcMtrStopped;

	// Configure Output Compare 3.
	//*OC3CON = ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm
	OC3CON  = ( 1 << 2 ) | ( 1 << 1 );	// pwm  No longer using timer 3 1/22/2018 KEC
	OC3R	= dtcMtrStopped;
	OC3RS	= dtcMtrStopped;

	// Configure Timer 2.
	TMR2	= 0;									// clear timer 2 count
	PR2		= 9999;

	// Configure Timer 3.
	//*TMR3	= 0;
	//*PR3		= 9999;

	// Start timers and output compare units.
	T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 )|(1 << TCKPS21);		// timer 2 prescale = 8
    // Bit 15 is the enable; setting TCKPS = [011] results in prescaler of 8
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	//*T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8
    
    IC3CONSET = (1 << 1) | (1 << 0); // Enable IC3/IC2 capture only on Rising Edge
    IC2CONSET = (1 << 1) | (1 << 0);

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 99; // period match every 100 us
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
    // Bits 4, 3, 2 set the priority to 7 (111)
    // Bits 1 and 0 set the sub to 3 (11)
    
    IPC3SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 6, sub 3
    IPC2SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 6, sub 3
    
	IFS0CLR = ( 1 << 20); // Sets IFS0 Bit 20 to 0 (no interrupt request)
	
    
    IFS0CLR = ( 1 << 13); // Clears IC3 interrupt status flag
    IFS0CLR = ( 1 << 9); // Clears IC2 interrupt status flag
    
    IEC0SET	= ( 1 << 13); // Sets IEC0 Bit 13 to 1 (IC3 interrupt enabled)
    IEC0SET	= ( 1 << 9); // Sets IEC0 Bit 9 to 1 (IC2 interrupt enabled)
    IEC0SET	= ( 1 << 20); // Sets IEC0 Bit 20 to 1 (timer 5 interrupt enabled)
	
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
    // Bit 5 enables 32-bit output compare; Bit 4 sets PWM fault
    
	//enable SPI
	SpiInit();
    
    IC2BUF = 75;
    
    IC3CONSET = (1 << 15); // Turn on IC3 and 2 ISRs
    IC2CONSET = (1 << 15);
    
	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
}

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/

void AppInit() {



}


/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of AVR Studio
**		and WinAVR GCC.  A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete. 
**
*/

void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 375; i ++){
			;;
		}
		delay -= 1;
	}
}

/************************************************************************/
