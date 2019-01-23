# Zheng Hao Tan
# Makefile for the blink application.
# Type 'make' and then whatever that follows to compile the code.

all: blink.o
	#avrdude -c usbasp -p m328p -U flash:w:blink.hex
		avrdude -c usbtiny -p t85 -U flash:w:blink.hex

blink.o: blink.c 
	avr-g++ -mmcu=attiny85 -Wall -Os -DF_CPU=8000000UL -o blink.elf blink.c ./ssd1306xled/ssd1306xled.c ../usitwix/usitwix/usitwim.c
	avr-objcopy -j .text -j .data -O ihex blink.elf blink.hex
	avr-size blink.hex

# make clean - remove .o files and the executable file.
clean:
	rm -f *.o blink.hex blink.elf 

# That's all folks!
