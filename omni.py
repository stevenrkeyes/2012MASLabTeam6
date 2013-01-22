import arduino
import time
import math

l=0.159
W=math.sqrt(0.084914-0.466*l+l*l)
sina=0.175/W
mASpeed = 0
mBSpeed = 0
mCSpeed = 0

class Omni:
        # Initializes motor given arduino object input
        #  Solution: initialize motors outside of class, use as parameters..?
        def __init__(self, ard):
		global mASpeed, mBSpeed, mCSpeed
                self.mA = arduino.Motor(ard, 2, 3, 4)
                self.mB = arduino.Motor(ard, 5, 6, 7)
                self.mC = arduino.Motor(ard, 8, 9, 10)
                #ard.run() <-- This is fucking shit up...

		# Forward unlimited at speed "power"
        def forward(self, power):
                mASpeed = mBSpeed = power
		mCSpeed = 0
		self.mA.setSpeed(power)
                self.mB.setSpeed(power)
                self.mC.setSpeed(0)

	def forwardArc(self, power, radius):
		if radius < 0:
			mASpeed = int((-power*radius)/(radius+35))
			mBSpeed = power
			mCSpeed = 16
			self.mA.setSpeed(mASpeed)
                	self.mB.setSpeed(mBSpeed)
                	self.mC.setSpeed(mCSpeed)
		if radius > 0:
			mASpeed = power
			mBSpeed = int((power*radius)/(radius+35))
			mCSpeed = -16
			self.mA.setSpeed(mASpeed)
                	self.mB.setSpeed(mBSpeed)
                	self.mC.setSpeed(mCSpeed)

	def forwardSlide(self, power, angle):
		mASpeed = int((math.cos(angle/57))*power+(math.sin(angle))*(l/(2*W*sina)))
		mBSpeed = int((math.cos(angle/57))*power+(math.sin(angle))*(-l/(2*W*sina)))
		mCSpeed = int((math.sin(angle))*power)
		self.mA.setSpeed(mASpeed)
		self.mB.setSpeed(mBSpeed)
		self.mC.setSpeed(mCSpeed)
		
        
	def backDrift(self, power):
		self.mC.setSpeed(power)


        # Backward unlimited at speed "power"
        def backward(self, power):
                self.forward(-power)

        # Drive right at speed "power"
        def right(self, power): 
		mASpeed = int(l/(2*W*sina))
		mBSpeed = int(-l/(2*W*sina))
		mCSpeed = power
                self.mA.setSpeed(int(l/(2*W*sina)))
                self.mB.setSpeed(int(-l/(2*W*sina)))
                self.mC.setSpeed(power)
                
        # Drive left at speed "power"
        def left(self, power):
                self.right(-power)
                
        # Turn left method with "power" and angle "a".
        def turnLeft(self, power, a):
                self.mA.setSpeed(int(-power*0.7511))
                self.mB.setSpeed(int(power*0.7511))
                self.mC.setSpeed(int(power))
                time.sleep(a/100)
                self.stopMotors()
		mASpeed, mBSpeed, mCSpeed = 0,0,0
                
        # Turn right method with "power" and angle "a".
        ## Experimentally tune
        def turnRight(self, power, a):
                self.turnLeft(-power, a)
                self.stopMotors()

	# Add power to all motors (turning) for a certain time
	def turn(self, power):
		self.mA.setSpeed(mASpeed + int(power * 0.7511))
		self.mB.setSpeed(mBSpeed - int(power * 0.7511))
		self.mC.setSpeed(mCSpeed - power)
		mASpeed += power
		mBSpeed -= power
		mCSpeed -= power

        # Stop motor method
        def stopMotors(self):
                self.mA.setSpeed(0)
                self.mB.setSpeed(0)
                self.mC.setSpeed(0)
                time.sleep(.1)

