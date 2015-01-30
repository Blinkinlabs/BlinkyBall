#
# Makefile template for ATtiny85
# Derived from AVR Crosspack template
#

DEVICE     = attiny85           # See avr-help for all possible devices
CLOCK      = 4000000            # Clock speed
PROGRAMMER = -c usbtiny -P usb # For using Adafruit USBtiny
LFUSE      = 0xD2
HFUSE      = 0xDF
EFUSE      = 0xFF
FUSES      = -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m  # settings as taken from http://www.engbedded.com/fusecalc/

TARGET = blinkyball

CC = avr-gcc
CXX = avr-g++


CPPFLAGS = -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
CXXFLAGS = 
CFLAGS = -std=c99

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)

C_FILES = \
	main.c \
	IRremote.c \
        crc.c \
        blinkyball.c


CPP_FILES = 

OBJECTS := $(CPP_FILES:.cpp=.o) $(C_FILES:.c=.o)

# symbolic targets:
all:	$(TARGET).hex

verify-program:
	$(AVRDUDE) -b 1 -U flash:v:$(TARGET).hex:i -U lfuse:v:$(LFUSE):m -U hfuse:v:$(HFUSE):m -U efuse:v:$(EFUSE):m

flash:	all
	$(AVRDUDE) -b 1 -U flash:w:$(TARGET).hex:i

fuse:
	$(AVRDUDE) -b 200 -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

install: fuse flash


# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID $(TARGET).hex

clean:
	rm -f $(TARGET).hex $(TARGET).elf $(OBJECTS)

# file targets:
$(TARGET).elf: $(OBJECTS)
	$(CXX) $(CPPFLAGS) -o $@ $(OBJECTS)

$(TARGET).hex: $(TARGET).elf
	rm -f $(TARGET).hex
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
	avr-size --format=avr --mcu=$(DEVICE) $(TARGET).elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	$(TARGET).elf
	avr-objdump -d $(TARGET).elf
