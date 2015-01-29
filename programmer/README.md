# Test Programmer for RasPi

This is a test system for Raspberry Pi. To use it, you'll first need to install avrdude:

	sudo apt-get install avrdude

Make sure that this repository is downloaded to /home/pi/BlinkyBall.

Then add it to init.d:

	cd ~pi/BlinkyBall
	cp programmer/blinkyball-programmer /etc/init.d
	update-rc.d blinkyball-programmer defaults	


# Pinout

This is the pinout for the board. Note that the pin numbers listed are Pi pins, not GPIO

Button should be connected from pin 3 to GND

LED anode to +3.3V
Red LED cathode to pin 22
Green LED cathode to pin 24
Blue LED cathode to pin 26

# Usage

Once the script is installed, it should start automatically at boot. Press the button to start the upload procedure. The light should turn blue while testing, then red if the upload failed or green if it passed.
