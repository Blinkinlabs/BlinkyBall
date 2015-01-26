#
# Makefile template for ATtiny85
# Derived from AVR Crosspack template
#

DEVICE     = attiny85           # See avr-help for all possible devices
CLOCK      = 8000000            # 8Mhz
PROGRAMMER = -c usbtiny -P usb -B 3 # For using Adafruit USBtiny
FUSES      = -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m  # settings as taken from http://www.engbedded.com/fusecalc/

TARGET = blinkyball

CC = avr-gcc
CXX = avr-g++


CPPFLAGS = -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
CXXFLAGS = 
CFLAGS = 

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)

CPP_FILES = \
	main.cpp \
	irremote.cpp

OBJECTS := $(CPP_FILES:.cpp=.o)

# symbolic targets:
all:	$(TARGET).hex

flash:	all
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i

fuse:
	$(AVRDUDE) $(FUSES)

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
