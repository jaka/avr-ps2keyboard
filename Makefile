#!/usr/bin/make -f

OPTS	+= -mmcu=$(MCU_TARGET)
CFLAGS	= -Os -Wall -Werror -Wextra -pedantic -std=c99 -ffunction-sections -fdata-sections -finline-functions -DF_CPU=$(MCU_FREQ)
LDFLAGS	= -Wl,--gc-sections
LIBS	=

CC := avr-gcc
OBJCOPY := avr-objcopy

MCU_FREQ := 16000000UL
MCU_TARGET := atmega32u4
PROGRAMMER_MCU := m32u4
PROGRAMMER := usbasp

SRC	:= avr-ps2.c
OBJ	:= $(patsubst %.c,%.o,$(SRC))
NAME = $(MCU_TARGET)-ps2

all: build

build: $(NAME).hex

clean:
	rm -f $(OBJ)
	rm -f $(NAME).elf
	rm -f $(NAME).hex

flash: $(NAME).hex
	avrdude -c $(PROGRAMMER) -p $(PROGRAMMER_MCU) -B 3 -U flash:w:$<

$(NAME).elf: $(OBJ)
	$(CC) $(CFLAGS) $(OPTS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.o: %.c
	$(CC) $(CFLAGS) $(OPTS) -c -o $@ $<