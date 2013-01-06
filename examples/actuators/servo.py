import sys
sys.path.append("../..")
import time

import arduino

# A simple example of using the arduino library to
# control a servo.

ard = arduino.Arduino()
servo = arduino.Servo(ard, 7)  # Create a Servo object
ard.run()  # Run the Arduino communication thread

while True:
    # Sweep the servo back and forth
    for i in range(-45, 45):
        servo.setAngle(i)
        time.sleep(0.05)
    # Reset
    servo.setAngle(-45)
    time.sleep(1)
    
