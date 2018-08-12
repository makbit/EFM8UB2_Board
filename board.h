//-----------------------------------------------------------------------------
// EFM8UB20 Board Configuration
//-----------------------------------------------------------------------------
// File:        Board.h
// Description: This file contains PIN definitions and its connections on
//              the development board. There are common initialization routines
//              and some often used subroutines, macros and constants.
//
//
// Target:      EFM8UB20 Board or C8051F380
// Tool chain:  Keil C51 8.0+
// Release:     1.0, 28 March 2018
//
// Copyright 2017, 2018 Maximov
// https://makbit.com
//-----------------------------------------------------------------------------
#ifndef __EFM8_BOARD_H
#define __EFM8_BOARD_H

#include <stdint.h>
#include <SI_EFM8UB2_Defs.h>

extern uint8_t   xdata g_PortInit;
extern uint32_t  xdata g_SYSCLK;
extern SI_UU16_t xdata g_NextPcaValue;
extern uint16_t  xdata g_HighSpeedPCA;
extern uint16_t  xdata g_SystemTick;
extern void            (*g_TickHandler)();

//INTERRUPT_PROTO(Timer0_ISR, TIMER0_IRQn);

#define PIN_BUTTON1                  P0_B0
#define PIN_BUTTON2                  P0_B1
#define PIN_VIDEO_VSYNC              P0_B2
#define PIN_RADIO_IRQ                P0_B3
#define PIN_UART_TX                  P0_B4
#define PIN_UART_RX                  P0_B5
#define PIN_XTAL1                    P0_B6
#define PIN_XTAL2                    P0_B7

#define SPI_SCK                      P1_B0
#define SPI_MISO                     P1_B1
#define SPI_MOSI                     P1_B2
#define SPI_NSS                      P1_B3
#define PIN_CNVSTR                   P1_B4
#define PIN_VREF                     P1_B5
#define PIN_RADIO_CE                 P1_B6
#define PIN_RADIO_CSN                P1_B7

#define I2C_SDA                      P2_B0
#define I2C_SCL                      P2_B1
#define PIN_SYSCLK                   P2_B2
#define PIN_BEEPER                   P2_B3
#define LED_RED                      P2_B4
#define LED_GREEN                    P2_B5
#define LED_BLUE                     P2_B6
#define LED_YELLOW                   P2_B7

#define PIN_FIFO_RCLK                P3_B0
#define PIN_FIFO_RRST                P3_B1
#define PIN_FIFO_OE                  P3_B2
#define PIN_FIFO_WEN                 P3_B3
#define PIN_VIDEO_RST                P3_B4
#define PIN_MAX7221_CS               P3_B5
#define PIN_CARD_CS                  P3_B6
#define PIN_CARD_CDN                 P3_B7

#define CPU_SPEED_48M           48000000UL
#define CPU_SPEED_24M           24000000UL
#define CPU_SPEED_12M           12000000UL
#define CPU_SPEED_6M             6000000UL
#define CPU_SPEED_DEFAULT        1500000UL
#define CPU_SPEED_80K              80000UL
#define CPU_SPEED_10K              10000UL
#define CPU_SPEED_EXT           20000000UL

#define SPI_SPEED_MAX                  0UL
#define SPI_SPEED_4M             4000000UL
#define SPI_SPEED_2M             2000000UL
#define SPI_SPEED_1M             1000000UL
#define SPI_SPEED_500K            500000UL
#define SPI_SPEED_100K            100000UL

#define UART_SPEED_2400             2400UL
#define UART_SPEED_9600             9600UL
#define UART_SPEED_38400           38400UL
#define UART_SPEED_57600           57600UL
#define UART_SPEED_115200         115200UL
#define UART_SPEED_230400         230400UL
#define UART_SPEED_460800         460800UL
#define UART_SPEED_921600         921600UL
#define UART_SPEED_MIDI            31250UL

#define I2C_SPEED_100K            100000UL
#define I2C_SPEED_400K            400000UL

#define LOBYTE(w)                 ((unsigned char)(((w) >> 0) & 0xFF))
#define HIBYTE(w)                 ((unsigned char)(((w) >> 8) & 0xFF))

#define PCA_SetBeeper(f)          (g_HighSpeedPCA = g_SYSCLK/((f)*2))
#define PCA_Beeper(on)            do{ if(on) PCA0CPM0 |= 0x40; else PCA0CPM0 &= ~0x40; } while(0)
#define PCA_SetLedPWM(f)          (PCA0CPH1 = (f))
#define PCA_LedPWM(on)            do{ if(on) PCA0CPM1 |= 0x40; else PCA0CPM1 &= ~0x40; } while(0)
#define PCA_SetFrequency(f)       (PCA0CPH2 = g_SYSCLK/((f)*2))


/*
#define WATCHDOG_ENABLE
#define WATCHDOG_DISABLE
#define WATCHDOG_RESET
*/

// Max7221 command codes
#define MAX7221_DIGIT_1                0x1
#define MAX7221_DIGIT_2                0x2
#define MAX7221_DIGIT_3                0x3
#define MAX7221_DIGIT_4                0x4
#define MAX7221_ICONS                  0x5
#define MAX7221_DECODE_MODE            0x9
#define MAX7221_INTENSITY              0xA
#define MAX7221_SCAN_LIMIT             0xB
#define MAX7221_SHUTDOWN               0xC
#define MAX7221_DISPLAY_TEST           0xF

// Code B Fonts (p.8), Digits:  0-9 
#define MAX7221_CODE_MINUS             0xA
#define MAX7221_CODE_E                 0xB
#define MAX7221_CODE_H                 0xC
#define MAX7221_CODE_L                 0xD
#define MAX7221_CODE_P                 0xE
#define MAX7221_CODE_BLANK             0xF

// Max7221 ICONS (Segment #5: MP3 | FM | USB)
#define MAX7221_NONE                     0
#define MAX7221_MP3                      1
#define MAX7221_FM                       2
#define MAX7221_DOT                      4
#define MAX7221_SD                       8
#define MAX7221_USB                     16
#define MAX7221_PAUSE                   32
#define MAX7221_PLAY                    64


extern void    I2C_Reset(void);
extern void    I2C_Master_Init(uint32_t nSpeed);
extern void    I2C_Write(uint8_t addr, uint8_t value);
extern void    I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t value);
extern uint8_t I2C_Read(uint8_t addrRd);
extern uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg);
// I2C_Slave_Init(addr?);
// UART_Init(baud, 8-n-1??? irq???);

extern void    Port_IO_Init(void);
extern void    Oscillator_Init(uint32_t nCpuSpeed);
extern void    PCA_Init(void);
extern void    Timer0_Init(void);
extern void    Timer2_Init(uint16_t hz);
extern void    INT01_Init(void);
extern void    SPI_Init(uint32_t nSpeed);
extern uint8_t SPI_io(uint8_t d);

extern void MAX7221_init(void);
extern void MAX7221_cmd(uint8_t addr, uint8_t value);
extern void MAX7221_out(uint32_t number, uint8_t icons);


#endif // end of file
