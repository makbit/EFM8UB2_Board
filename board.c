#include "board.h"

/*===========================================================================*
The 8051 memory model contains several different types of segments including:
	- DATA (128 bytes)
	- IDATA (128 bytes)
	- PDATA (first 256 bytes of XDATA)
	- XDATA (up to 64 KB)
	- CODE (up to 64 KB)
DATA, IDATA, PDATA, and XDATA all allow read and write access, whereas CODE is
read only.  The memory segments are listed in order of speed and code efficiency.

Timers usage:
	I2C   => Timer0 (clock), Timer3 (overflow)
	UART0 => Timer1 (baud rate generator)
	Timer0 and Timer1 share SYSCLK divider.
 *===========================================================================*/

//===========================================================================//
// Extern variables (flags), SYSCLK after Reset is 1.5MHz                    //
//===========================================================================//
uint32_t  xdata g_SYSCLK           = 1500000UL;
uint8_t   xdata g_PortInit         = 0;
uint16_t  pdata g_SystemTick       = 0;
uint16_t  pdata g_HighSpeedPCA     = 1500 / (1 * 2);
SI_UU16_t pdata g_NextPcaValue     = { 0 };

struct I2C_Data
{
	uint8_t Data[7];                   // I2C Address[0], Data In/Out [1..6]
	uint8_t Count : 4;                 // Data count (1..6)
	uint8_t Busy  : 1;                 // Busy flag
	uint8_t Error : 3;                 // Error flag/bits (TimeOut | Arbitr | i/o)
} xdata i2c = { 0 };

//===========================================================================//
// To configure a pin as a digital input:
//       1) the pin is configured as digital and open-drain and
//       2) the port latch should be set to a '1'.
// The weak-pullups are used to pull the pins high.
// Pressing the switch pulls the pins low.
//
// To configure a pin as a digital output, the pin is configured
// as digital and push-pull.  
//===========================================================================//
void Port_IO_Init()
{
	// 0 - Analog, 1 - Digital
	P0MDIN          = 0x3F;            // Analog: ExtOsc(+,-)
	P1MDIN          = 0xDF;            // Analog: VRef
	P2MDIN          = 0xFF;            // Digital (default)
	P3MDIN          = 0xFF;            // Digital (default)
	P4MDIN          = 0xFF;            // Digital (default)

	// 0 - OpenDrain, 1 - Push-Pull
	P0MDOUT         = 0x10;            // Push-Pull: Uart_TX, Open-Drain: *
	P1MDOUT         = 0xCD;            // OpenDrain: Miso, Cnvt, VRef
	P2MDOUT         = 0xFC;            // OpenDrain: SDA, SCL
	P3MDOUT         = 0x7F;            // OpenDrain: Card_CDN
	P4MDOUT         = 0x00;            // OpenDrain: Input All

	// Skip & XBR registers depends on perephirals selected.
	P0SKIP          = 0xCF;            // 4,5 - UART
	P1SKIP          = 0xF0;            // 0..3 - SPI
	P2SKIP          = 0x70;            // 0,1 - I2C, 2 - SysClk, 3..7-PCA
	P3SKIP          = 0xFE;            // 0 - PCA (CEX2)

	// Disable digital output drivers by writing 1 to latch register.
	// Set CS pins to inactive state, I2C pulled-up (off), other - default 0.
	PIN_BUTTON1     = 1;
	PIN_BUTTON2     = 1;
	PIN_VIDEO_VSYNC = 1;
	PIN_RADIO_IRQ   = 1;
	PIN_UART_RX     = 1;
	PIN_UART_TX     = 0;
	PIN_XTAL1       = 1;
	PIN_XTAL2       = 1;

	SPI_MISO        = 1;
	SPI_SCK         = 0;
	SPI_MOSI        = 0;
	SPI_NSS         = 1;
	PIN_CNVSTR      = 1;
	PIN_VREF        = 1;
	PIN_RADIO_CE    = 0;
	PIN_RADIO_CSN   = 1;

	I2C_SDA         = 1;
	I2C_SCL         = 1;
	PIN_BEEPER      = 1;
	LED_RED         = 0;
	LED_GREEN       = 0;
	LED_BLUE        = 0;
	LED_YELLOW      = 0;

	// OE, RRST are Active low;
	// WEN - active low, but (F_WEN, HREF) => Nand
	PIN_FIFO_RCLK   = 0;
	PIN_FIFO_RRST   = 1;
	PIN_FIFO_OE     = 1;
	PIN_FIFO_WEN    = 0;
	PIN_VIDEO_RST   = 1;
	PIN_MAX7221_CS  = 1;
	PIN_CARD_CS     = 1;
	PIN_CARD_CDN    = 1;

	// Port 4: input
	P4              = 0xFF;
	// Enable Crossbar (on/off peripherals).
	//-----------------------------------------------------------//
	// XBR0: |__7__|__6__|__5__|__4__|__3__|__2__|__1__|__0__|   //
	//       |CP1AE| CP1E|CP0AE| CP0E|SYSCE|SMB0E|SPI0E|URT0E|   //
	//-----------------------------------------------------------//
	// XBR1: |__7__|__6__|__5__|__4__|__3__|__2__|__1__|__0__|   //
	//       |WEAK |XBARE| T1E | T0E | ECIE|    PCA0ME       |   //
	//-----------------------------------------------------------//
	// PCA:   0-Disable, 1-CEX0, 2-CEX0_CEX1, 3,4,5-CEX0..4      //
	//-----------------------------------------------------------//
	XBR0            = 0x0F;            // SYSCLK out, I2C, SPI, UART
	XBR1            = 0x43;            // XBAR0, CEX0..CEX2 (Beeper, Yellow, FifoClk).
	g_PortInit      = 1;               // Global flag: initialization complete.
}

//===========================================================================//
//                                                                           //
//===========================================================================//
void Oscillator_Init(uint32_t nCpuSpeed)
{
	uint16_t xdata i;

	// Reset on:
	// 0x02: VDD Monitor reset.
	// 0x04: Missing clock detector - hangs with debugger.
	// 0x80: USB reset - fails to start with VCPXpress.
	RSTSRC  = 0x02;                    // Setup reset sources (VDD only)
	FLSCL   = 0x80;                    // Flash one-shot enabled (recommended, p.27)
	PFE0CN |= 0x20;                    // Prefetch Engine Control - read 2 instr per clock
	VDM0CN  = 0x80;                    // VDD Monitor enable

	switch( nCpuSpeed )
	{
		//-------------------------------------------//
		// CLKSEL: |-.-|-0.0.0.-||-1-|---.-.-.--|    //
		//         |Rsv    USB    OUT   C_SEL   |    //
		//-------------------------------------------//
		// HFO0CN: |-.-|-.-|-.--|.|.|.|---.-.---|    //
		//         | EN RDY SPND  |    IFCN(S/x)|    //
		//-------------------------------------------//
		// FLSCL: Flash one-shot | Flash read timing // 
		// for freq > 25 MHz set bit FLRT            //
		//-------------------------------------------//
		// USB clock derived from the Internal HFOsc //
		//-------------------------------------------//
		// SYSCLK synchronized with the Port I/O     //
		//-------------------------------------------//
		case CPU_SPEED_48M:            // 48/1/1
			FLSCL    |= 0x10;          // Below 48MHz
			CLKSEL    = 0x08 | 0x03;   // SysClk synch, HFOSC_DIV_1
			HFO0CN    = 0x80 | 0x03;   // Enable HFOsc, SYSCLK_Div_1
			break;
		case CPU_SPEED_24M:            // 48/2/1
			CLKSEL    = 0x08 | 0x02;   // SysClk synch, HFOSC_DIV_2
			HFO0CN    = 0x80 | 0x03;   // Enable HFOsc, SYSCLK_Div_1
			break;
		case CPU_SPEED_12M:            // 48/4/1
			CLKSEL    = 0x08 | 0x00;   // SysClk synch, HFOSC_DIV_4
			HFO0CN    = 0x80 | 0x03;   // Enable HFOsc, SYSCLK_Div_1
			break;
		case CPU_SPEED_6M:
			CLKSEL    = 0x08 | 0x00;   // SysClk synch, HFOSC_DIV_4
			HFO0CN    = 0x80 | 0x02;   // Enable HFOsc, SYSCLK_Div_2
			break;
		case CPU_SPEED_80K:
			LFO0CN   |= 0x83;          // LF on, DIVIDE_BY_1
			CLKSEL    = 0x0C;          // SYS_OUT_SYNCH, LFOSC
			HFO0CN    = 0x40;          // HF off
			break;
		case CPU_SPEED_10K:
			LFO0CN   |= 0x80;          // LF on, DIVIDE_BY_8
			CLKSEL    = 0x0C;          // SYS_OUT_SYNCH, LFOSC
			HFO0CN    = 0x40;          // HF off
			break;
		case CPU_SPEED_EXT:
		{
			// Ports must be configured as Analog I/O before this.
			while( !g_PortInit ) {}
			// See table on page 52. (0111b: 8MHz <= Freq <= 25MHz)
			// Start external oscillator (External: 0x60, Mode freq: 0x07).
			XOSC0CN = 0x60 | 0x07;
			// Wait for crystal osc. to start
			for( i = 0; i < 1000; i++ ) ;
			// Wait for crystal osc. to settle
			while( !(XOSC0CN & 0x80) )  ;
			// Select external oscillator as system clock source (SYSCLK).
			CLKSEL = 0x08 | 0x01; // SYSCLK_SYNC_IO, EXTOSC
			// Disable the internal oscillator.
			HFO0CN = 0x00;
		} break;
		case CPU_SPEED_DEFAULT:        // Page 49: 48 MHz/4/8 = 1.5MHz
		default:
			g_SYSCLK = 1500000UL;      // Default frequency 1.5MHz
			break;
	}
	if( nCpuSpeed != CPU_SPEED_EXT )
	{
		for( i = 0; i < 3000; i++ )
		{
			if( HFO0CN & 0x40 ) break; // Wait for oscillator READY flag
		}
	}
	g_SYSCLK = nCpuSpeed;
	return;
}

//===========================================================================//
// Programmable Counter Array (p.137)                                        //
// CEX0 - High Speed Output controls the Beeper (@10Hz - 100kHz)             //
// CEX1 - 8-bit PWM controls Yellow led (@48MHz/255=188kHz)                  //
// CEX2 - Frequency Output (fast generator up to 24MHz).                     //
// Note:  Clear EnComp bit to setup 0% duty cycle or disable (p. 144).       //
//===========================================================================//
void PCA_Init()
{
	PCA0CN0            = 0;                   // Stop PCA Counter.
	//------------------------------------------------------------//
	// PCA0MD :  Idle | WdTE |WdLock|r|     CPS     |  ECF  |     //
	//             0  |  0   |  0   | |    1 0 0    |  irq  |     //
	//------------------------------------------------------------//
	// CPS: 000=Sys/12, 001=Sys/4, 100=Sys, 101=Ext/8             //
	//------------------------------------------------------------//
	PCA0MD             = 8;                   // Based on SYSCLK
	//------------------------------------------------------------//
	// PCA0CMP :  Pwm16|eComp|CapP|CapN||Match|Toggl| Pwm | ECF | //
	//             0   |  1  |  0 | 0  ||     |     |     | irq | //
	//------------------------------------------------------------//
	PCA0CPM0           = 0x4D;                // Ch#0: HiSpeed, IRQ
	PCA0CPM1           = 0x42;                // Ch#1: 8-bit, PWM
	PCA0CPM2           = 0x46;                // Ch#2: Frequency output
	//------------------------------------------------------------//
	// Capture Module values
	g_NextPcaValue.u16 = g_HighSpeedPCA;      // precalculated
	PCA0CPL0           = g_NextPcaValue.u8[1];// Beeper frequency
	PCA0CPH0           = g_NextPcaValue.u8[0];// Beeper frequency
	PCA0CPH1           = 0x80;                // 8-bit PWM 50%
	PCA0CPH2           = 12;                  // SYSCLK/(2*Freq=2MHz)
	EIE1              |= 0x10;                // Enable extended IRQ EPCA0
	PCA0CN0_CR         = 1;                   // Start PCA Counter
}

//===========================================================================//
// This is a simplified IRQ handler to control the Beeper sounds only.       //
// Update/Reset counters for "Hi-speed Output"                               //
//===========================================================================//
void PCA0_ISR(void) interrupt PCA0_IRQn
{
	PCA0CN0_CCF0       = 0;
	PCA0CPL0           = g_NextPcaValue.u8[1];
	PCA0CPH0           = g_NextPcaValue.u8[0];
	g_NextPcaValue.u16 = PCA0CP0 + g_HighSpeedPCA;
}

//===========================================================================//
// EXTernal interrupt configuration (p. 84)                                  //
// Two direct-pin interrupt sources INT0 & INT1, level or edge sensitive.    //
// Note:                                                                     //
//    INT0 and INT1 will monitor their assigned port pins without disturbing //
//    the peripheral that was assigned the port pin via the crossbar.        //
//    To assign a port pin only to INT0 and/or INT1, configure the crossbar  //
//    to skip the selected pin(s).                                           //
//===========================================================================//
void INT01_Init(void)
{
	//------------------------------------------------------------//
	// IT01CF: || IN1PL |    IN1SL    || IN0PL |    IN0SL     ||  //
	//         ||   1   |  0   1   1  ||   1   |   0   1   0  ||  //
	//------------------------------------------------------------//
	IT01CF    |= 0x0A;                 // INT0: PolActHigh, P0_2 (VSync)
	IT01CF    |= 0xB0;                 // INT1: PolActHigh, P0_3 (Radio)
	TCON_IT0   = 1;                    // INT0 is edge triggered.
	TCON_IT1   = 1;                    // INT1 is edge triggered.
	TCON_IE0   = 0;                    // Clear initial ExtInt 0
	TCON_IE1   = 0;                    // Clear initial ExtInt 1
	IE_EX0     = 1;                    // Enable external events /INT0
	IE_EX1     = 1;                    // Enable external events /INT1
}
//===========================================================================//
// The IRQ pending flag is automatically cleared by the MCU (in edge mode).  //
//===========================================================================//
//void Int0_ISR(void) interrupt INT0_IRQn { }
//void Int1_ISR(void) interrupt INT1_IRQn { }

//===========================================================================//
// Timer 0: 8-bit timer with autoreload (p.252) (used by I2C)                //
// Based on SysClk/12; Ftimer = Fsys / (256 - TH0); [15.6kHz..4MHz]          //
// Timers #0 and #1 share frequency divider!                                 //
//===========================================================================//
void Timer0_Init(uint32_t f)
{
	f = g_SYSCLK / (f*12);             // SCL counter (30kHz, 100kHz, 400kHz)
	if( f < 1   ) f = 1;               // Minimum frequency divider
	if( f > 255 ) f = 255;             // Maximum frequency divider
	//------------------------------------------------------------//
	// CKCON0 : T3MH | T3ML | T2MH | T2ML || T1M | T0M |  SCA  |  //
	//           e/s | e/s  | e/s  | e/s  || p/s | p/s |  div  |  //
	// e(0) - external, s(1) - sysclk, p(0) - prescale.           //
	//------------------------------------------------------------//
	CKCON0  &= 0xF0;                   // Timer0,1 clock prescaler (SysClk/12)
	TMOD    |= 0x02;                   // Timer0 Mode2 Select (8-bit, autoreload)
	TH0      = 256 - (uint8_t)f;       // Set initial counter value (100kHz)
	TL0      = TH0;                    // Set counter
	TCON_TR0 = 1;                      // Start Timer0
	IE_ET0   = 0;                      // Timer0 interrupts disabled (not used)
}

//===========================================================================//
// Timer 1: 8-bit timer with autoreload (p.252) (used by UART0)              //
//===========================================================================//
void Timer1_Init(uint32_t f)
{
	f = g_SYSCLK/(f*12);               // UART Baudrate (9600, 31250, 115200)
	if( f < 1   ) f = 1;               // Minimum frequency divider
	if( f > 255 ) f = 255;             // Maximum frequency divider
	//------------------------------------------------------------//
	// TMOD : GATE1 | CT1 |   T1M   || GATE0 | CT0 |   T0M   |    //
	//         int1 | t/e |   mode  ||  int0 | t/e |  mode   |    //
	//------------------------------------------------------------//
	CKCON0  &= 0xF0;                   // Timer0,1 clock control (SYSCLK/12)
	TMOD    |= 0x20;                   // Timer1 Mode2 Select (8-bit, autoreload)
	TH1      = 256 - (uint8_t)f;       // Set initial counter value
	TL1      = TH1;                    // TLow := THigh
	TCON_TR1 = 1;                      // Timer1 enable
	IE_ET1   = 0;                      // Timer0 interrupts disabled (not used)
}

//===========================================================================//
// Timer 2: 16-bit timer with autoreload [62Hz..800kHz]                      //
// Note: Ftimer = Fsys / (65536 - TMR0RLH:TMR0RLL)                           //
//       MCU is too slow to process IRQ @1MHz+                               //
//===========================================================================//
void Timer2_Init(uint32_t f)
{
	f = g_SYSCLK / (12*f);             // Timer2 counter
	if( f < 1     ) f = 1;             // Minimum frequency: 62 Hz (4M/65536)
	if( f > 65535 ) f = 65535;         // Maximum frequency: 4 MHz (4M/1)
	CKCON0      &= 0xCF;               // Timer2 configured with TMR2CN reg
	TMR2CN0      = 0;                  // Timer2 SysClk/12, Auto, RunCtrl off
	TMR2RL       = 65536 - (uint16_t)f;// Timer2 initial counter value
	TMR2         = TMR2RL;             // Timer2 counter
	TMR2CN0_TR2  = 1;                  // Run Timer2
	IE_ET2       = 1;                  // Timer2 interrupts enabled
}

//===========================================================================//
void Timer2_ISR(void) interrupt TIMER2_IRQn
{
	g_SystemTick++;                    // Every millisecond (1/1000s)
	TMR2CN0_TF2H = 0;                  // Reser IRQ flag
}

//===========================================================================//
// Timer 3: 16-bit timer with autoreload, (25mS for I2C control)             //
// Range: [62Hz..800kHz],      Note: 25mS = 40Hz                             //
//===========================================================================//
void Timer3_Init(uint32_t f)
{
	f = g_SYSCLK / (12*f);             // Timer3 divider
	if( f < 1     ) f = 1;             // Minimum frequency: 62 Hz (4M/65536)
	if( f > 65535 ) f = 65535;         // Maximum frequency: 4 MHz (4M/1 xIRQ)
	CKCON0  &= 0x3F;                   // Timer3 configured with TMR3CN reg
	TMR3CN0  = 0;                      // Timer3: 16-bit autoreload, SYSCLK/12
	TMR3RL   = 65536 - (uint16_t)f;    // Timer3 configured to overflow after 15ms (~25ms)
	TMR3     = TMR3RL;                 // Timer2 counter
	TMR3CN0 |= 0x04;                   // Timer3 enable
	EIE1    |= 0x80;                   // Timer3 interrupts enable ("Reset SMB in IRQ")
}

//===========================================================================//
// Timer3 Interrupt Service Routine indicates an SMBus SCL low timeout.      //
// The I2C/SMBus is disabled and re-enabled here.                            //
//===========================================================================//
void Timer3_ISR(void) interrupt TIMER3_IRQn
{
	SMB0CF     &= 0x7F;                // Disable SMBus
	SMB0CF     |= 0x80;                // Re-enable SMBus
	SMB0CN0_STA = 0;                   // Clear any START on the bus
	TMR3CN0    &= 0x3F;                // Clear Timer3 interrupt-pending flags
	i2c.Error   = 0x04;                // Timeout Error
	i2c.Busy    = 0;                   // Set I2C/SMBus to free state
}

//===========================================================================//
// I2C (SMBus), p.224                                                        //
//===========================================================================//
void I2C_Reset(void)
{
	volatile uint8_t xdata i;
	uint8_t xdata xbr0   = XBR0;
	uint8_t xdata xbr1   = XBR1;
	uint8_t xdata p2out  = P2MDOUT;
	uint8_t xdata p2skip = P2SKIP;

	// Check if slave is holding SDA low (because of an improper reset or error).
	while( I2C_SDA==0 )
	{
		// Provide clock pulses to allow the slave to advance out
		// of its current state. This will allow it to release SDA.
		XBR0     = 0;                  // Disable ALL perephirals
		XBR1     = 0x40;               // Enable Crossbar
		P2MDOUT  = 0xFC;               // Set I2C pins to Oped-Drain
		P2SKIP   = 0;                  // Manual control over these pins
		I2C_SCL  = 0;                  // Drive the clock low
		for( i = 0; i < 255; i++ ) ;   // Hold the clock low
		I2C_SCL  = 1;                  // Release the clock
		while( I2C_SCL==0 ) {}         // Wait for open-drain pin to rise
		for( i = 0; i < 50; i++ ) ;    // Hold the clock high
		XBR1     = 0;                  // Disable Crossbar
   }

   P2MDOUT = p2out;                    // Restore config registers
   P2SKIP  = p2skip;
   XBR0    = xbr0;
   XBR1    = xbr1;
}

//===========================================================================//
// I2C_Init() includes Timer0 and Timer3 init                                //
// SMBus configured as follows:                                              //
//    - SMBus enabled                                                        //
//    - Slave mode inhibited                                                 //
//    - Timer0 used as clock source, overflow at 1/3 the SCL rate            //
//    - Setup and hold time extensions enabled                               //
//    - Bus Free and SCL Low timeout detection enabled (Timer3)              //
// Note:                                                                     //
//    Make sure the Timer0 can produce the required frequency in 8-bit mode. //
//    Min frequency: 48M / 256 / 3 = 62.5k                                   //
//===========================================================================//
void I2C_Init(uint32_t nSpeed)
{
	Timer0_Init(nSpeed);               // SCL counter (3/2)
	Timer3_Init(40);                   // 25ms (40Hz) I2C low timeout detector
	//----------------------------------------------------------------//
	// SMB0CF: || EnSMB | Inh | Busy|ExtHld||SmbTOE|SmbFTE| SMB CS || //
	//         ||   0   |  1  |  0  |   1  ||  1   |  1   |   00   || //
	//----------------------------------------------------------------//
	SMBTC   = 0x0A;                    // Add 4 SysClks to hold time window
	SMB0CF  = 0x00;                    // SMB Clock Source: Timer0 OverFlow
	SMB0CF |= 0x04;                    // Enable SMBus Free timeout detection
	SMB0CF |= 0x08;                    // Enable SCL low timeout detection (Timer 3)
	SMB0CF |= 0x10;                    // Enable setup & hold time extensions
	SMB0CF |= 0x40;                    // Slave inhibit (disable slave IRQs)
	SMB0CF |= 0x80;                    // Enable I2C/SMBus
	EIE1   |= 0x01;                    // Enable I2C/SMB interrupts
}

//===========================================================================//
typedef enum
{
	I2C_MASTER_START  = 0xE0,          // start transmitted (master)
	I2C_MASTER_TXDATA = 0xC0,          // data byte transmitted (master)
	I2C_MASTER_RXDATA = 0x80,          // data byte received (master)
	I2C_SLAVE_ADDRESS = 0x20,          // slave address received (slave)
	I2C_SLAVE_RX_STOP = 0x10,          // STOP detected during write (slave)
	I2C_SLAVE_RXDATA  = 0x00,          // data byte received (slave)
	I2C_SLAVE_TXDATA  = 0x40,          // data byte transmitted (slave)
	I2C_SLAVE_TX_STOP = 0x50,          // STOP detected during a write (slave)
} I2C_State_t;

//===========================================================================//
// I2C/SMBus Interrupt Service Routine (ISR)                                 //
//===========================================================================//
// I2C/SMBus ISR state machine                                               //
//    - Master only implementation                                           //
//    - All outgoing/incoming data is read/written to i2c.Data[1..6]         //
//===========================================================================//
void I2C_ISR(void) interrupt SMBUS0_IRQn
{
	if( 0==SMB0CN0_ARBLOST )                // No arbitration errors
	{
		switch( SMB0CN0 & 0xF0 )            // Get status bits for master (4-bits)
		{

			case I2C_MASTER_START:          // START condition transmitted
				SMB0CN0_STA = 0;            // Manually clear START bit
				SMB0CN0_STO = 0;            // Manually clear STOP bit
				SMB0DAT     = i2c.Data[0];  // Write address [0] of the target slave
				break;

			case I2C_MASTER_TXDATA:         // Address or Data byte was transmitted
				if( SMB0CN0_ACK )           // Slave has sent ACK (1) or NACK (0)
				{
					if( i2c.Data[0] & 1 )   // If it is READ cmd, then do nothing
					{
						// If this transfer is a READ, proceed with transfer
						// without writing to SMB0DAT (address was sent)
					}
					else                    // It is WRITE cmd
					{
						if( i2c.Count )     // If there are some bytes to send
						{                   // Send data byte (from end of array)
							SMB0DAT = i2c.Data[i2c.Count--]; 
						}
						else                // If there are no data bytes to send
						{
							SMB0CN0_STO = 1;// Set STOP to terminate transfer
							i2c.Busy    = 0;// And free SMBus interface
						}
					}
				}
				else                        // The slave has sent NACK (no reply)
				{
					SMB0CN0_STO = 1;        // Send STOP condition, 
					SMB0CN0_STA = 0;        // NOT followed by a START
					i2c.Error   = 1;        // Indicate error (no reply)
					i2c.Busy    = 0;        // And free SMBus interface
				}
				break;

			case I2C_MASTER_RXDATA:         // Data byte received
				if( i2c.Count )             // Check for free space
				{                           // Store received byte @1
					i2c.Data[i2c.Count--] = SMB0DAT;
				}
				else
				{
					i2c.Error = 1;          // No free space, error
				}
				if( i2c.Count==0 )
				{
					SMB0CN0_ACK = 0;        // Send NACK to indicate last byte tx
					SMB0CN0_STO = 1;        // Send STOP to terminate transfer
					i2c.Busy    = 0;        // Free SMBus interface
				}
				else
				{
					SMB0CN0_ACK = 1;        // Send ACK to continue RX
				}
				break;

			default:
				i2c.Error = 0x7;            // Indicate failed transfer, see end of ISR
				break;
		}
	}
	else
	{
		i2c.Error = 0x02;              // Set error flag, stop transmission
	}

	if( i2c.Error )                    // If the transfer failed...
	{                                  // RESET to default values
		SMB0CF     &= 0x7F;            // Disable I2C/SMBus
		SMB0CF     |= 0x80;            // Enable I2C/SMBus
		SMB0CN0_STA = 0;               // No START sequence
		SMB0CN0_STO = 0;               // No STOP condition
		SMB0CN0_ACK = 0;               // No active ACK
		i2c.Busy    = 0;               // Free SMBus
	}
	SMB0CN0_SI = 0;                    // Clear interrupt flag
}


//===========================================================================//
// I2C_Write() - writes a single byte to the slave with address specified    //
//               by the <TARGET> variable. See i2c struct;                   //
//===========================================================================//
void I2C_Write(uint8_t addr, uint8_t value)
{
	i2c.Data[0] = addr & 0xFE;         // Set slave address as a reciever
	i2c.Data[1] = value;               // Set value to transfer
	i2c.Count   = 1;                   // Number of bytes to transfer
	i2c.Busy    = 1;                   // Set I2C/SMBus to busy state
	i2c.Error   = 0;                   // Clear error flag
	SMB0CN0_STA = 1;                   // Start transfer
	while( i2c.Busy ) ;                // Wait for I2C/SMBus to be free.
}

//===========================================================================//
// I2C_Read() - reads a single byte from the slave with address specified    //
//              by the <TARGET> variable.                                    //
//===========================================================================//
uint8_t I2C_Read(uint8_t addr)
{
	i2c.Data[0] = addr | 0x01;         // Set slave address as a transmiter
	i2c.Data[1] = 0;                   // Set default value to 0
	i2c.Count   = 1;                   // Number of bytes to recieve
	i2c.Busy    = 1;                   // Set I2C/SMBus to busy state
	i2c.Error   = 0;                   // Clear error flag
	SMB0CN0_STA = 1;                   // Start transfer
	while( i2c.Busy ) ;                // Wait for transfer to complete
	return i2c.Data[1];
}

//===========================================================================//
void I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
	// Note: Transmits last byte 1st.
	i2c.Data[0] = addr & 0xFE;         // Set slave address as a reciever
	i2c.Data[1] = value;               // Set Data#2 to transfer (reversed order)
	i2c.Data[2] = reg;                 // Set Data#1 to transfer (see order in IRQ)
	i2c.Count   = 2;                   // Number of bytes to transfer
	i2c.Busy    = 1;                   // Set I2C/SMBus to busy state
	i2c.Error   = 0;                   // Clear error flag
	SMB0CN0_STA = 1;                   // Start transfer
	while( i2c.Busy ) ;                // Wait for I2C/SMBus to be free.
}

//===========================================================================//
uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg)
{
	I2C_Write( addr, reg );
	return i2c.Error ? 0 : I2C_Read(addr);
}

//===========================================================================//
// UART0 on P0 pins 4, 5                                                     //
//===========================================================================//
void UART0_Init(uint32_t baudRate)
{
	Timer1_Init(baudRate*2);           // Set Baudrate, example @115200
	SCON0  = 0x10;                     // 8-bit mode, TI/RI clear, Rcv Enable
	IP    |= 0x10;                     // UART0 high priority (PS0)
	IE_ES0 = 1;                        // Enable UART0 interrupts
}

//===========================================================================//
// SPI Config Page 223: fsck = SYSCLK / (2*(SPI0CKR+1))                      //
// Ports:               CLK=P1.0 MISO=P1.1 MOSI=P1.2 NSS=P1.3                //
//===========================================================================//
void SPI_Init(uint32_t nSpeed)
{
	SPI0CFG   = 0x40;                  // MasterMode, CKPHA Centered, CKPol low
	SPI0CN0   = 0x09;                  // 4-Wire sMaster, NSS output 0, Spi On
	if( nSpeed==SPI_SPEED_MAX )        // Min 100k, Max: 48/12=4M
	{
		SPI0CKR = 2;                   // Possible: 0 <= SPI0CKR <= 255, stable @4+
	}
	else
	{
		uint32_t div = g_SYSCLK / (2*nSpeed);
		//SPI0CKR   = 23;   // 1MHz = (48MHz/(2*(23+1))
		//SPI0CKR   =  9;   // 1MHz = (20MHz/(2*(9+1))
		if( div > 1 ) div = div - 1;
		SPI0CKR = (uint8_t)div;
	}
}


//===========================================================================//
uint8_t SPI_io(uint8_t b)             // Page: 216
{
	if( SPI0CN0_WCOL )                 // Clear: Write collision flag error
		SPI0CN0_WCOL = 0;
	if( SPI0CN0_MODF )                 // Clear: Mode fault flag
		SPI0CN0_MODF = 0;
	if( SPI0CN0_RXOVRN )               // Clear: Receive overrun flag
		SPI0CN0_RXOVRN = 0;
	
	SPI0CN0_NSSMD0 = 0;                // Manual NSS (not slave select) for SPI
	SPI0DAT = b;                       // Start transmit
	while( SPI0CN0_SPIF==0 ) ;         // Wait for i/o complete
	b = SPI0DAT;                       // Store rx data
	SPI0CN0_SPIF = 0;                  // Clear SPI done flag
	SPI0CN0_NSSMD0 = 1;                // Finish with SS

	return b;
}

//===========================================================================//
//                                                                           //
//===========================================================================//
void MAX7221_Init()
{
	//MAX7221_cmd( MAX7221_DISPLAY_TEST, 1 );  // Test Mode On (all on)
	MAX7221_cmd( MAX7221_SHUTDOWN,       0 );  // Shutdown
	MAX7221_cmd( MAX7221_SHUTDOWN,       1 );  // Turn On
	MAX7221_cmd( MAX7221_INTENSITY,      4 );  // Intensity: 0 - min, 0x0F - max
	MAX7221_cmd( MAX7221_SCAN_LIMIT,     4 );  // Scan-Limit (total digits: [0..4]) 
	MAX7221_cmd( MAX7221_DECODE_MODE, 0x0F );  // Decode Mode B for [0..3] digits
	MAX7221_out( 0,                      0 );  // Show zeros, no icons
}

//===========================================================================//
void MAX7221_cmd(uint8_t addr, uint8_t value)
{
	PIN_MAX7221_CS = 0;           // Chip-Select enable (NSS, not slave select)
	SPI_io( addr );               // Write register address
	SPI_io( value );              // Write register value
	PIN_MAX7221_CS = 1;           // Chip-select disable
}
//===========================================================================//
void MAX7221_out(uint32_t num, uint8_t icons)
{
// TODO: 1) Slow! 2) Change from use Code B, ADD TABLE!
	MAX7221_cmd( MAX7221_DIGIT_4, (num / 1) % 10 );    // decimal
	MAX7221_cmd( MAX7221_DIGIT_3, (num / 10) % 10 );   // tenths
	MAX7221_cmd( MAX7221_DIGIT_2, (num / 100) % 10 );  // hundredths
	MAX7221_cmd( MAX7221_DIGIT_1, (num / 1000) % 10 ); // thousandths
	MAX7221_cmd( MAX7221_ICONS, icons );
}
