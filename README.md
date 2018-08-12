## Example firmware for **EFM8UB2 development board** based on [8-bit SiLbas MCU](https://www.silabs.com/products/mcu/8-bit).
> These simple projects can be used for tutorial purposes.

[The Configuration Wizard 2](https://www.silabs.com/products/development-tools/software/8-bit-8051-microcontroller-software) was used to create [`board.cwg`](board.cwg) file and to manage on-board ports and perepherials (SPI, UART, I2C, etc).
Required header files and libraries are stored in `inc` and `lib` folders. Project files were created with the [Keil uVision C51](http://www.keil.com/c51/).
Functions for initialization of the on-board perephirials are located in [`board.c`](board.c) file. Pins definitions and function prototypes can be found in [`board.h`](board.h) file.

### EFM8UB2_Board aka SilDuino :smile:
![EFM8UB2_Board](EFM8UB2_Board.jpg)
# This is the MCU :cool:
![EFM8UB2_Board_MCU](EFM8UB2_Board_MCU.jpg)
## TODO :pen:
- [x] Initial commit :+1:
- [ ] PCA/Ports change - remove LED_YELLOW PWM :bell:
- [ ] UART_init and tests
- [ ] Upload schematis
- [ ] Write an article for [My site and forum](https://makbit.com/web)

[GitHub readme tips&tricks](https://help.github.com/articles/basic-writing-and-formatting-syntax/)

