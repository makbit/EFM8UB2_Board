//===========================================================================//
// 1) Driving 7-segment led diplay with Max7221 via SPI.                     //
// 2) Demonstrates Timer2 usage, ticks every 1ms.                            //
// 3) PCA is used for generating sounds with Beeper.                         //
//                                                                           //
// Note:                                                                     //
//     Add "../inc" folder with the SI_EFM8UB2_Defs.h to the include path.   //
//     The project requires "board.c/.h" files with board init functions.    //
// Version: July 2018                                                        //
//===========================================================================//
#include <SI_EFM8UB2_Defs.h>
#include "../board.c"

//===========================================================================//
void Delay(uint16_t n)
{
	while( n-- > 0 ) ;
}

//===========================================================================//
//                                                                           //
//===========================================================================//
void main(void)
{
	uint32_t i = 0;
	uint16_t sec = 0;

	PCA0MD = 0;                             // Disable WDT
	Port_IO_Init();                         // Ports Init
	Oscillator_Init( CPU_SPEED_DEFAULT );   // Osc 1.5MHz
	SPI_Init( SPI_SPEED_1M );               // SPI Master 1Mbit/s
	PCA_Init();                             // PCA: Beeper, Led, Freq
	Timer2_Init(1000);                      // Timer2, 1ms
	IE_EA = 1;                              // Enable IRQ
	
	MAX7221_init();
	MAX7221_cmd( MAX7221_DISPLAY_TEST, 1 ); // Test Mode: On
	MAX7221_cmd( MAX7221_DISPLAY_TEST, 0 ); // Test Mode: Off
	MAX7221_cmd( MAX7221_INTENSITY, 0x04 ); // Intensity: 0 - min, 0x0F - max

	while(1)
	{
		i++;
		LED_RED    = i & 0x10;
		LED_BLUE   = i & 0x20;
		LED_GREEN  = i & 0x40;
		PCA_SetLedPWM( 255 - (i%100)*2 );
		Delay(1500);
		
		if( PIN_BUTTON1 )
		{
			PCA_Beeper(1);
			PCA_SetBeeper( 110+(i % 100)*10 );
		}
		else
		{
			PCA_Beeper(0);
		}

		if( g_SystemTick >= 1000 )
		{
			sec++;
			g_SystemTick = 0;
			MAX7221_out( sec, sec & 127 );
			MAX7221_cmd( MAX7221_INTENSITY, (sec & 0x02) ? 1 : 9 );
		}
	}
}
