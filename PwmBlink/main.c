/*-----------------------------------------------------------------------------
	Info: This example demonstrates how to use PCA for PWM led blinking.
	Note: 1) High-Speed mode is not fast, it is good for beeper <100kHz.
	      2) Frequency mode is good for oscillator up to 48MHz/2!
		  3) PWM is easy in 8-bit mode, no IRQ. Use eComp to disable.
		     PWM frequency is: SYSCLK/255 (188kHz @48MHz, 11kHz @3MHz).
	Platform: Silicon Labs EFM8UB2
	Date: July 2018
-----------------------------------------------------------------------------*/
#include <SI_EFM8UB2_Defs.h>

#define PWM_LED_GREEN           PCA0CPH2
#define PWM_LED_BLUE            PCA0CPH3
#define PWM_LED_YELLOW          PCA0CPH4

#define PIN_BEEPER              P2_B3
#define LED_RED                 P2_B4
#define LED_GREEN               P2_B5
#define LED_BLUE                P2_B6
#define LED_YELLOW              P2_B7

#define PCA_SetHighSpeed(f)     (g_HighSpeedPCA = g_SYSCLK/((f)*2))
#define PCA_SetFrequency(f)     (PCA0CPH1 = g_SYSCLK/((f)*2));

// Default SYSCLK after Reset is 1.5MHz
uint32_t  xdata g_SYSCLK        = 1500000UL;
uint16_t  xdata g_HighSpeedPCA  = 1500 / (1 * 2); // 1.5MHz / 1kHz / 2
SI_UU16_t xdata g_NextPcaValue  = { 0 };

//===========================================================================//
void Delay(long n)
{
	while( n-- > 0 ) ;
}

//===========================================================================//
void Port_IO_Init()
{
	// P2_3: Beeper (HiSpeed), P2_4: Generator (Freq), P2_5..7: 8-bit PWM
	P2MDOUT    = 0xFF;
	P0SKIP     = 0xFF;
	P1SKIP     = 0xFF;
	P2SKIP     = 0x03;

	// Default value
	PIN_BEEPER = 1;
	XBR0       = 0x08; // SYSCLK @P2_2
	XBR1       = 0x45; // XBAREnable, 5xPCA: CEX0_CXE4 Enable
}

//===========================================================================//
void Oscillator_Init()
{
	int i;
	FLSCL     = 0x90;
	CLKSEL    = 8 | 0;             // 0-Div_4_cn, 3-48; // SYSCLK out, HFOSC (48MHz)
	HFO0CN    = 0x80;              // 0-div8, 1-div4, 2-div2, 3-div1 (HF on)
	for( i = 0; i < 5000; i++ )
	{
		if( HFO0CN & 0x40 ) break; // wait for oscillator READY flag
	}
	g_SYSCLK = 1500000UL;          // 48MHz/4/8
}

//===========================================================================//
void PCA_Init()
{
	PCA0CN0    = 0;          // Stop PCA Counter, clear everything.
	//------------------------------------------------------------//
	// PCA0MD :  Idle | WdTE |WdLock|r|     CPS     |  ECF  |     //
	//             0  |  0   |  0   | |    1 0 0    |  irq  |     //
	//------------------------------------------------------------//
	// CPS: 000=Sys/12, 001=Sys/4, 100=Sys, 101=Ext/8             //
	//------------------------------------------------------------//
	PCA0MD     = 8;          // Based on SYSCLK
	//------------------------------------------------------------//
	// PCA0CMP :  Pwm16|eComp|CapP|CapN||Match|Toggl| Pwm | ECF | //
	//             0   |  1  |  0 | 0  ||     |     |     | irq | //
	//------------------------------------------------------------//
	PCA0CPM0   = 0x4D;       // Ch#0: HiSpd, MAT|TOG|0|IRQ
	PCA0CPM1   = 0x46;       // Ch#1: Frequency output, no IRQ
	PCA0CPM2   = 0x42;       // Ch#2: 8-bit, PWM
	PCA0CPM3   = 0x42;       // Ch#3: 8-bit, PWM
	PCA0CPM4   = 0x42;       // Ch#4: 8-bit, PWM
	//------------------------------------------------------------//
	// Capture Module values
	g_NextPcaValue.u16 = g_HighSpeedPCA;
	PCA0CPL0           = g_NextPcaValue.u8[1];
	PCA0CPH0           = g_NextPcaValue.u8[0];
	PCA0CPH1           = 1;  // or g_SYSCLK / (750000UL * 2); // (max)
	//------------------------------------------------------------//
	// Page 143: For 16-bit PWM MAT & CCF required for sync!
	// Page 144: Clear EnComp bit to setup 0% duty cycle.
	PCA0CPH2   = 0x80;       // Ch#2 (LED_G): 8-bit PWM 50%
	PCA0CPH3   = 0xC0;       // Ch#3 (LED_B): 8-bit PWM 25%
	PCA0CPH4   = 0xE0;       // Ch#4 (LED_Y): 8-bit PWM 12%
	//------------------------------------------------------------//
	EIE1      |= 0x10;       // Enable extended interrupt EPCA0
	PCA0CN0_CR = 1;          // Start PCA Counter
}


//===========================================================================//
void main(void)
{
	uint16_t i;
	uint8_t pwm = 200;
	
	// Disable WD timer.
	PCA0MD = 0;
	
	Port_IO_Init();
	Oscillator_Init();
	PCA_Init();

	IE_EA  = 1;

	while(1)
	{
		for( i = 220; i < 880; i+=11 )
		{
			// Change Beeper frequency.
			PCA_SetHighSpeed(i);
			Delay(1000);
			// Change LED intensity.
			if( pwm >= 255 ) pwm = 220;
			PWM_LED_GREEN = pwm++;
		}
	}
}

//===========================================================================//
// This IRQ handler is not optimized, but it can process all config options. //
// Used only for High Speed Output [beeper] (to change frequency on the fly).//
//===========================================================================//
void PCA0_ISR(void) interrupt PCA0_IRQn
{
	// Main PCA Counter overflow action.
	if( PCA0CN0_CF )
	{
		PCA0CN0_CF = 0;
	}
	// Channel #0 action.
	if( PCA0CN0_CCF0 )
	{
		// Clear flag
		PCA0CN0_CCF0 = 0;
		// Update/Reset counters for "Hi-speed Output"
		PCA0CPL0           = g_NextPcaValue.u8[1];
		PCA0CPH0           = g_NextPcaValue.u8[0];
		g_NextPcaValue.u16 = PCA0CP0 + g_HighSpeedPCA;
	}
	if( PCA0CN0_CCF1 ) PCA0CN0_CCF1 = 0;
	if( PCA0CN0_CCF2 ) PCA0CN0_CCF2 = 0;
	if( PCA0CN0_CCF3 ) PCA0CN0_CCF3 = 0;
	if( PCA0CN0_CCF4 ) PCA0CN0_CCF4 = 0;
}
