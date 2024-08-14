# AVR-GCC Makefile for ATmega328P

# Project name
TARGET = main

# Define the microcontroller
MCU = atmega1284p

# Define the clock frequency
F_CPU = 16000000UL

# Source files
SRC = src/*.c

# Included directories
INCLUDE_DIR = include

# Compiler and linker flags
# Considerar: -Werror -Wfatal-errors -Wall -Wextra
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -I${INCLUDE_DIR}
LDFLAGS = -mmcu=$(MCU)

# Output directory
BUILD_DIR = build

# Tools
CC = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
OBJCOPY = avr-objcopy
HID = hiduploader.exe

# HID options
HID_MCU = atmega1284p

# Output file names
ELF = $(BUILD_DIR)/$(TARGET).elf
HEX = $(BUILD_DIR)/$(TARGET).hex

# Default target
all: $(BUILD_DIR) $(HEX)

$(BUILD_DIR):
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

# Build elf from source
$(ELF): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@

# Build hex from elf
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@


# Clean up build directory
clean:
	if exist $(BUILD_DIR) del /Q $(BUILD_DIR)\*

# Flash the microcontroller
flash: $(HEX)
	$(HID) -mmcu=$(HID_MCU) -v -usb=0x2842,0x0001 "$(HEX)"

# Phony targets
.PHONY: all clean flash