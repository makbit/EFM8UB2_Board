//---------------------------------------------------------------------------//
// This project demonstrates EXTernal interrupts (INT0, INT1) activated      //
// by user buttons (edge-mode).                                              //
// INT0 rising edge - button1 pressed.                                       //
// INT1 falling edge - button1 released.                                     //
//---------------------------------------------------------------------------//
#include <SI_EFM8UB2_Defs.h>

sbit BUTTON1    = P0^0;
sbit BUTTON2    = P0^1;

sbit LED_RED    = P2^4;
sbit LED_GREEN  = P2^5;
sbit LED_BLUE   = P2^6;
sbit LED_YELLOW = P2^7;

//===========================================================================//
void main(void)
{
	// disable WDT
	PCA0MD   &= ~0x40;

	// P2 Led pins as Push-Pull
	P2MDOUT   = 0xF0;
	P2SKIP    = 0x0F;
	// Enable crossbar (I/O)
	XBR1      = 0x40;

	// Irq (0, 1) are edge triggered
	TCON      = 0x05;
	// Clear initial ExtInt (0, 1) values
	TCON_IE0  = 0;
	TCON_IE1  = 0;
	// INT0 is active at P0_0 High, INT1 is active at P0_0 Low
	IT01CF    = 0x08;

	// Enable external events (/INT0, /INT1), enable All interrupts
	IE_EX0    = 1;
	IE_EX1    = 1;
	IE_EA     = 1;
	
	LED_RED = 1;

	while(1)
	{
		LED_GREEN = BUTTON2;
	}
}

//===========================================================================//
// The interrupt pending flag is automatically cleared by the MCU
//===========================================================================//
void Int0_ISR(void) interrupt INT0_IRQn
{
	// button pressed - light on
	LED_BLUE = 1;
}

void Int1_ISR(void) interrupt INT1_IRQn
{
	// button released - light off
	LED_BLUE = 0;
}