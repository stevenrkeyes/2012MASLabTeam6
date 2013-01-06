import sys
sys.path.append("../..")
import time

import arduino

ard = arduino.Arduino()
led = arduino.DigitalOutput(ard, 13)
ard.run() # Start the thread which communicates with the Arduino

# Make the LED blink once a second
while True:
    led.setValue(0)
    time.sleep(1)
    led.setValue(1)
    time.sleep(1)
