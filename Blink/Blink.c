//===========================================================================//
// Project: Blink - Led blinking firmware.                                   //
//                                                                           //
// Info: This example shows the simplest thing you can do with my EFM8 Board.//
//       It blinks the on-board LEDs connected to Port 2 Pins 4, 5.          //
//                                                                           //
// Note: Add "../inc" folder with the SI_EFM8UB2_Defs.h to the include path. //
// Date: July 2018                                                           //
//===========================================================================//
#include <SI_EFM8UB2_Defs.h>

sbit LED_RED    = P2^4;
sbit LED_GREEN  = P2^5;
sbit LED_BLUE   = P2^6;
sbit LED_YELLOW = P2^7;

//===========================================================================//
void main()
{
	int i;
	
	PCA0MD     = 0;                    // Disable watchdog timer
	P2SKIP     = 0x0F;                 // P2_0..P2_3 not used
	P2MDOUT    = 0xF0;                 // P2_4..P2_7 is output
	XBR1       = 0x40;                 // Enable ports

	LED_BLUE   = 0;                    // Turn off Blue
	LED_YELLOW = 0;                    // Turn off Yellow

	while(1)                           // Loop forever
	{
		for( i = 0; i < 10000; i++ ) ; // wait
		LED_RED   = 1;                 // Switch on Red
		LED_GREEN = 0;                 // Switch off Green
		for( i = 0; i < 10000; i++ ) ; // wait
		LED_RED   = 0;
		LED_GREEN = 1;
	}
}
