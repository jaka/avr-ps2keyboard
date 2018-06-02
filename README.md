# PS/2 Keyboard

This repository contains a sample code for Atmel Atmega32U4 acting as PS2 controller. Key scan codes are sent in hex format to UART.

PS2 clock must be connected to interrupt pin, in this case it is PD0 (INT0). PS2 data could be any pin, in this case it is PE6.

This example was developed on Pro Micro development board and programmed by USB ASP programmer via SPI.