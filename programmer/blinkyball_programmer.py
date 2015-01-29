import IcspUtils
import RPi.GPIO as GPIO

def programFuses():
    eFuses    = 0xFF
    hFuses    = 0xDF
    lFuses    = 0x62
    
    result = IcspUtils.writeFuses(eFuses, hFuses, lFuses)
    return result[0] == 0

def loadFlash():
    result = IcspUtils.loadFlash("/home/pi/BlinkyBall/bin/blinkyball-0.9.hex")
    return result[0] == 0


BUTTON_PIN = 3 # Start button
RED_LED_PIN = 22
GREEN_LED_PIN = 24 
BLUE_LED_PIN = 26 

GPIO.setmode(GPIO.BOARD)  # Use board numbering
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(BLUE_LED_PIN, GPIO.OUT)


def setLEDs(red, green, blue):
    GPIO.output(RED_LED_PIN, red==0)
    GPIO.output(GREEN_LED_PIN, green==0)
    GPIO.output(BLUE_LED_PIN, blue==0)

setLEDs(1,0,0)

#for i in range(0,1):
while True:

    # wait for button press
    while GPIO.input(BUTTON_PIN) == 1:
        pass

    setLEDs(0,0,1)

    success = True
    if success:
        if not programFuses():
            success = False

    if success:
        if not loadFlash():
            success = False

    if success:
        setLEDs(0,1,0)
    else:
        setLEDs(1,0,0)
