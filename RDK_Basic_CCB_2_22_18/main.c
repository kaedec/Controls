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
/*   01/25/18: Display functional, IC2 & IC3 functional. Display prints */
/*   counter indicating motion of each wheel                            */
/*   01/31/18: Both wheels can move a set number of revolutions and     */
/*   then stop; attempts to perform timing on the wheel                 */
/*   02/05/18: Implemented wheel timing and cleaned up the code         */
/*   02/06/18: Heavily cleaned up and commented code                    */
/*   02/14/18: Implemented Speed Control for Right wheel                */
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

// Timer prescalar bits
#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4

#define     revCounter          1575 // counting IC2/IC3 for 10ish revolutions
#define     initSpeedLeft       0 // Initialized speed for left wheel
#define     initSpeedRight      0 // Initialized speed for right wheel
#define     timer2MaxVal        9999 // Timer2 overflows at this value
#define     timer3MaxVal        49999 // Timer3 overflows at this value (50ms)

#define     wheelC              0.71886 // Circumference of the wheel in feet

#define     bufferNotEmpty      3 // Bit in ICxCON register for buffer not empty

#define     IC2IntFlag          9 // Bit in IFS0 register for IC2's interrupt flag
#define     IC3IntFlag          13 // Bit in IFS0 register for IC3's interrupt flag
#define     T3IntFlag           12 // Bit in IFS0 register for T3's interrupt flag

#define     IC2IntEnable        9 // Bit in IEC0 register for IC2's enable
#define     IC3IntEnable        13 // Bit in IEC0 register for IC3's enable
#define     T3IntEnable         12 // Bit in IEC0 register for T3's enable

//#define     alpha               0.1 // percent of new data point used in PID
//#define     beta                1-alpha
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

//unsigned int IC2OVCounter = 0;
unsigned int IC3OVCounter = 0;

float IC2Time = 0.0;
float IC3Time = 0.0;

unsigned int desired_time = 3500; // microseconds
float desired_spd = 0.75; // ft/s
float integral_error = 0.0;
float err = 0.0;

float hist0[500];
float hist1[500];
float hist2[500];
float hist3[500];
float hist4[500];
float hist5[500];


int full_error = 3500;

unsigned int desired_time2 = 3500; // microseconds
float desired_spd2 = 0.6; // ft/s
float integral_error2 = 0.0;
float err2 = 0.0;

int full_error2 = 3500;

float Kp2 = 2500.0;
float Ki2;
float Kd2;

float Kp = 2500.0;
float Ki;
float Kd;

//float alpha = 0.1; // percent of new data point used in PID
//float beta = 0.9; //1-alpha;

float distanceL = 0; // value for distance traveled by left wheel (IC2)
float distanceR = 0; // value for distance traveled by right wheel (IC3)
float speedL = 0; // speed of left wheel in fps
float speedR = 0; // speed of right wheel in fps

// Used for speed control in Timer5 ISR

float IC2_speed = 0.0;
float IC2_spd_avg = 0.0;

float IC3_speed = 0.0;
float IC3_spd_avg = 0.0;
/* written to in T3 ISR
 * read in IC2&IC3 ISRs
 * used for counting number of overflows of T3
 * 50ms per overflow
 */
unsigned int T3_OV_Count = 0;

int delta_time2 = 0;
int delta_time3 = 0;

// These take on the value of local variables so that they can be watched
int time2;
int time3;

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


// IC2 and IC3 are connected to the wheels
// ipl = interrupt priority level
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl6) _IC2_IntHandler(void)
{
    static int time = 0;
    static int prev_time = 0;
    int T3_OV_Count_Local;
    static int prev_T3_OV_Count_Local = 0;
    static float alpha = 0.1;
    static float beta = 0.9;
    
    T3_OV_Count_Local = T3_OV_Count; //make a copy of the global variable to fix its value for this ISR
    
	IFS0CLR	= ( 1 << IC2IntFlag ); // clear interrupt flag for Input Capture 2
    
    //Reading the buffer whenever it is not empty
    
    while((IC2CON & ( 1 << bufferNotEmpty )) == ( 1 << bufferNotEmpty ))
    {
        time = (int) (IC2BUF & 0x0000FFFF); // mask off the upper half of the buffer
        
        delta_time2 = time - prev_time; //keep track of delta time
        
        /* It is possible that an overflow occurred but the counter did not
         * increase; this statement catches such an occurrence
         */ 
        if ((delta_time2 <= 0)&&(T3_OV_Count_Local == prev_T3_OV_Count_Local))
        {
            delta_time2 += 50000;
            //IC2OVCounter++;
            T3_OV_Count_Local++;
        }
    }
    
	IC2Counter++;
    
   
    IC2Time = T3_OV_Count_Local*50000 + time; //new(er) time algorithm microseconds
    //IC2_speed = 5000.0/(float)delta_time2;
    if(delta_time2 > 500)
       IC2_speed = 5000.0/(float)delta_time2;
    else
       IC2_speed = 2.0;
    IC2_spd_avg = alpha*IC2_speed + beta*IC2_spd_avg;
    
    // Update state variables
    prev_time = time;
    time2 = time;
    prev_T3_OV_Count_Local = T3_OV_Count_Local;
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl6) _IC3_IntHandler(void)
{
    static int time = 0;
    static int prev_time = 0;
    int T3_OV_Count_Local;
    static int prev_T3_OV_Count_Local = 0;
    static float alpha = 0.1;
    static float beta = 0.9;
    
    T3_OV_Count_Local = T3_OV_Count; //make a copy of the global variable to fix its value for this ISR
    
	IFS0CLR	= ( 1 << IC3IntFlag );	// clear interrupt flag for Input Capture 3	
    
    //Reading the buffer whenever it is not empty
    
    while((IC3CON & ( 1 << bufferNotEmpty )) == ( 1 << bufferNotEmpty ))
    {
        time = (int) (IC3BUF & 0x0000FFFF); // mask off the upper half of the buffer
        
        /* It is possible that an overflow occurred but the counter did not
         * increase; this statement catches such an occurrence
         */ 
        delta_time3 = time - prev_time; //keep track of delta time
        if ((delta_time3 <= 0)&&(T3_OV_Count_Local == prev_T3_OV_Count_Local))
        {
            delta_time3 += 50000;
            IC3OVCounter++;
            T3_OV_Count_Local++;
        }
    }
     
    // For testing if we want to fill data vector once and hold execution in this ISR
     /*if (count>=499) 
      while(1)
      {
          count = count;
      }*/
    
	IC3Counter++;
    

    IC3Time = T3_OV_Count_Local*50000 + time; //new(er) time algorithm microseconds
    //IC3_speed = 5000.0/(float)delta_time3;
    if(delta_time3 > 500)
       IC3_speed = 5000.0/(float)delta_time3;
    else
       IC3_speed = 2.0;
    IC3_spd_avg = alpha*IC3_speed + beta*IC3_spd_avg;
    
    // Update state variables
    prev_time = time;
    time3 = time;
    prev_T3_OV_Count_Local = T3_OV_Count_Local;
}

void __ISR(_TIMER_3_VECTOR, ipl5) Timer3Handler(void)
{
    
    IFS0CLR = ( 1 << T3IntFlag ); //clear T3 interrupt flag
    T3_OV_Count++;
}

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{
	static	WORD tusLeds = 0;
	static int T5_count = 0;
	
    
    float temp_output;
    //float err = 0;
    static float prev_error = 0.0;
    
    float temp_output2;
    //float err2 = 0;
    static float prev_error2 = 0.0;
    static int index = 0;
    
    mT5ClearIntFlag();
    //full_error = desired_time; // full_error is equal to desired_time
    //Kp = 5000/full_error; // Kp*full_error = 50% of output range, output range = 10000 ms
    
    Ki2 = Kp2/10.0;
    Kd2 = Kp2/100.0;
    
    
    Ki = Kp/10.0;
    Kd = Kp/100.0;
    
/* ------------------------------------------------------------ */
/*				Right Wheel PID     							*/
/* ------------------------------------------------------------ */
    
    //error = desired_time - avg_meas_time; 
    err = desired_spd - IC3_spd_avg; 
    integral_error += err;
    
    if(integral_error > 25000/Ki) integral_error = 25000/Ki; // Bounds integral_error
    else if(integral_error < -25000/Ki) integral_error = -25000/Ki;
    
    //temp_output = 10000 - (Kp*err + Ki*integral_error + Kd*(err-prev_error)); //subtract from 10000 for time control
    temp_output = Kp*err + Ki*integral_error + Kd*(err-prev_error); //subtract from 10000 for time control
    
    hist0 [index] = IC3_spd_avg;
    hist1[index] = Kp*err;
    hist2[index] = Ki*integral_error;
    hist3[index] = Kd*(err-prev_error);
    hist4[index] = temp_output;
    
    if(temp_output > 9999) temp_output = 9999; // Bounds temp_output between 0 and 10000
    else if(temp_output < 800) temp_output = 800; // Prevent startup issue
    
/* ------------------------------------------------------------ */
/*				Left Wheel PID        							*/
/* ------------------------------------------------------------ */
    
     //error = desired_time - avg_meas_time; 
    err2 = desired_spd2 - IC2_spd_avg; 
    integral_error2 += err2;
    
    if(integral_error2 > 25000/Ki2) integral_error2 = 25000/Ki2; // Bounds integral_error
    else if(integral_error2 < -25000/Ki2) integral_error2 = -25000/Ki2;
    
    //temp_output = 10000 - (Kp*err + Ki*integral_error + Kd*(err-prev_error)); //subtract from 10000 for time control
    temp_output2 = Kp2*err2 + Ki2*integral_error2 + Kd2*(err2-prev_error2);
    
    hist5[index] = IC2_spd_avg;
    
    if(temp_output2 > 9999) temp_output2 = 9999; // Bounds temp_output between 0 and 10000
    else if(temp_output2 < 800) temp_output2 = 800; // Prevent startup issue
    
    
    // Startup testing to see if motors can be jump started and stay moving
    // Worked until T5_count reached its limit then stopped again
    /*if(T5_count < 500) temp_output += 2000.0;
    else 
    { temp_output = temp_output;}
    T5_count++;  // cheeky American*/
    
    
    OC2R = (HWORD)temp_output2;
    OC2RS = (HWORD)temp_output2;
    OC3R = (HWORD)temp_output;
    OC3RS = (HWORD)temp_output;
    
    // Update state variables
    prev_error2 = err2;
    prev_error = err;
    
    
    // Index increment and reset, while(1) used for startup testing, NEEDS TO BE REMOVED BEFORE RELEASE 
    index++;
    if(index > 499)  { index = 0; } //index = 0;
    
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

void __ISR (_OUTPUT_COMPARE_2_VECTOR, ipl6) OC2_IntHandler (void)
{
    IFS0CLR = (1 << 10); // Clears OC2 interrupt status flag
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
    //int n_2 = sprintf(bufftemp ,"IC2Count: %i", IC2Counter);
    int n_2 = sprintf(bufftemp , "Lspeed: %.4f", IC2_spd_avg);
    
	SpiEnable();
	SpiPutBuff(szClearScreen, 3);
	DelayMs(4);
	SpiPutBuff(szBacklightOn, 4);
	DelayMs(4);
	SpiPutBuff(szCursorOff, 4);
	DelayMs(4);
	SpiPutBuff(bufftemp, n_2);
    //SpiPutBuff("IC2Count: %d", 9);
    //int n_3 = sprintf(bufftemp ,"IC3Count: %i", IC3Counter);
	int n_3 = sprintf(bufftemp , "Rspeed: %.4f", IC3_spd_avg);
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
    
    //n_2 = sprintf(bufftemp ,"IC2Count: %i", IC2Counter);
    int n_2 = sprintf(bufftemp , "Lspeed: %.4f", IC2_spd_avg);
	SpiEnable();
	DelayMs(1);
    SpiPutBuff(szCursorPosHome, 6);
	SpiPutBuff(bufftemp, n_2);
    //SpiPutBuff("IC2Count: %d", 9);
    //n_3 = sprintf(bufftemp ,"IC3Count: %i", IC3Counter);
    int n_3 = sprintf(bufftemp , "Rspeed: %.4f", IC3_spd_avg);
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
        
        //Run wheels for revCounter pulses then stop
        /*if (IC2Counter >= revCounter)
        {
            OC2R = dtcMtrStopped;
            OC2RS = dtcMtrStopped;
        }
        
        if (IC3Counter >= revCounter)
        {
            OC3R = dtcMtrStopped;
            OC3RS = dtcMtrStopped;
        }*/
        
        distanceL = (IC2Counter/160)*wheelC;
        distanceR = (IC3Counter/160)*wheelC;
        
        speedL = (distanceL/((float) IC2Time))*1000000;
        speedR = (distanceR/((float) IC3Time))*1000000;
       
		//configure OCR to go forward

		/*if(stPressed == stPmodBtn1){
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
         */
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
    //TRISBSET = (1 << 2) | (1 << 3) | (1 <<4);
    
	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );    // setting up Port D (the way we connected the hardware defines this))
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward (left needs to be a 0)
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward (right needs to be a 1)
    
	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= ( 1 << 2 ) | ( 1 << 1 );	// pwm using T2
	OC2R	= initSpeedLeft;
	OC2RS	= initSpeedLeft;

	// Configure Output Compare 3 to drive the right motor.
	OC3CON  = ( 1 << 2 ) | ( 1 << 1 );	// pwm using T2
	OC3R	= initSpeedRight;
	OC3RS	= initSpeedRight;

	// Configure Timer 2 used for PWM
	TMR2	= 0; // clear T2 count
	PR2		= timer2MaxVal;

	// Configure Timer 3 used for real timing
	TMR3	= 0; // clear T3 count
	PR3		= timer3MaxVal;

	// Start timers and output compare units.
    
    // Bit 15 is the enable; setting TCKPS = [011] results in prescaler of 8
	T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 ) | ( 1 << TCKPS21 );	// timer 2 prescale = 8
    T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30 ); 	// timer 3 prescale = 8
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
    
    // Set IC3 and IC2 to rising edge only capture mode
    IC3CONSET = ( 1 << 1 ) | ( 1 << 0 );
    IC2CONSET = ( 1 << 1 ) | ( 1 << 0 );

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 22999; // period match every 23 ms
    
/* ------------------------------------------------------------ */
/*				Interrupt Priorities							*/
/* ------------------------------------------------------------ */
    
    // Level 7, sub 3
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // Timer 5
    
    // Level 6, sub 3
    IPC3SET	= ( 1 << 12 ) | ( 1 << 11 ) | ( 1 <<  9 ) | ( 1 <<  8 ); // IC3
    IPC2SET	= ( 1 << 12 ) | ( 1 << 11 ) | ( 1 <<  9 );// | ( 1 <<  8 ); // IC2
    IPC2SET = ( 1 << 20 ) | ( 1 << 19 ) | ( 1 << 17 ) | ( 1 << 16 ); // OC2
    
    // Level 5, sub 3
    IPC3SET = ( 1 << 4 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // Timer 3
    //IPC6SET = ( 1 << 28) | (1 << 26) | (1 << 25) | (1 << 24); // ADC
    
    
    // Clearing status flags
	IFS0CLR = ( 1 << 20 ); // Timer 5
    IFS0CLR = ( 1 << IC3IntFlag ); // IC3
    IFS0CLR = ( 1 << IC2IntFlag ); // IC2
    IFS0CLR = ( 1 << 10 ); // OC2
    IFS0CLR = ( 1 << T3IntFlag ); // T3
    //IFS1CLR = ( 1 << 1); // ADC
    
    // Enabling interrupts
    IEC0SET	= ( 1 << 20 ); // Timer 5
    IEC0SET	= ( 1 << IC3IntEnable ); // IC3
    IEC0SET	= ( 1 << IC2IntEnable ); // IC2
    IEC0SET = ( 1 << 10 ); // OC2
    IEC0SET = ( 1 << T3IntEnable ); // Timer 3
    //IEC1SET = ( 1 << 1 ); // ADC
	
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
    // Bit 5 enables 32-bit output compare; Bit 4 sets PWM fault
    
	//enable SPI
	SpiInit();

    // Turn on IC3 and IC2 ISRs
    IC3CONSET = (1 << 15);
    IC2CONSET = (1 << 15);
    
	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
    
    // Initialized speeds in code that is only executed one so it is not overwritten during debugging
    /*OC2R = 1000;
    OC2RS = 1000;
    OC3R = 10000;
    OC3RS = 10000;*/
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
