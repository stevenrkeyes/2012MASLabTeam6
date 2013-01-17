import arduino
import time
import math


l=0.15
W=sqrt(0.084914-0.466*l+l*l)
sina=0.175/W


class Omni:
        # Initializes motor given arduino object input
        #  Solution: initialize motors outside of class, use as parameters..?
        def __init__(self, ard):
                self.mA = arduino.Motor(ard, 2, 3, 4)
                self.mB = arduino.Motor(ard, 5, 6, 7)
                self.mC = arduino.Motor(ard, 8, 9, 10)
                ard.run()

		# Forward unlimited at speed "power"
        def forward(self, power):
                self.mA.setSpeed(power)
                self.mB.setSpeed(power)
                self.mC.setSpeed(0)
                
        # Backward unlimited at speed "power"
        def backward(self, power):
                self.forward(-power)

        # Drive right at speed "power"
        def right(self, power):
                self.mA.setSpeed(l/(2*W*sina))
                self.mB.setSpeed(-l/(2*W*sina))
                self.mC.setSpeed(power)
                
        # Drive left at speed "power"
        def left(self, power):
                self.right(-power)
                
        # Turn left method with "power" and angle "a".
        def turnLeft(self, power, a):
                self.mA.setSpeed(-power*0.7511)
                self.mB.setSpeed(power*0.7511)
                self.mC.setSpeed(power)
                time.sleep(a/100)
                self.stopMotors()
                
        # Turn right method with "power" and angle "a".
        ## Experimentally tune
        def turnRight(self, power, a):
                self.turnLeft(-power, a)
                self.stopMotors()

        # Stop motor method
        def stopMotors(self):
                self.m0.setSpeed(0)
                self.m1.setSpeed(0)
                time.sleep(.1)

