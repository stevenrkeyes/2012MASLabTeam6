import arduino
import time

class Drive:
	# Initializes motor given arduino object input
	# **This whole ard.run() shit is kind of fucked up. 
	#  Solution: initialize motors outside of class, use as parameters..?
	def __init__(self, ard):
		self.m0 = arduino.Motor(ard, 2, 3, 4)
		self.m1 = arduino.Motor(ard, 5, 6, 7)
		ard.run()
	
	# Drive forward at speed "power" and time "t" (ms)
	def forwardTime(self, power, t):
		self.m0.setSpeed(power)
		self.m1.setSpeed(power)
		time.sleep(t)
		self.stopMotors()
	
	# Drive forward at speed "power" and time "t" (ms)
	def backwardTime(self, power, t):
		self.forward(-power, t)

	# Turn method with "power" and time "t".
	# Power positive for right turn, negative for left turn
	def turn(self, power, t):
		self.m0.setSpeed(-1 * power)
		self.m1.setSpeed(power)
		time.sleep(t)
		self.stopMotors()

	# Forward unlimited at speed "power"
	def forward(self, power):	
		self.m0.setSpeed(power)
		self.m1.setSpeed(power)

	# Backward unlimited at speed "power"
	def backward(self, power)
		self.forward(-power)
	
	# Stop motor method
	def stopMotors():
		self.m0.setSpeed(0)
		self.m1.setSpeed(0)

if __name__ == "__main__":
	ard = arduino.Arduino()
	motors = self.Drive(ard)
	motors.forward(50, 100)
	motors.backward(50, 100)
	motors.turn(50, 100)
	motors.turn(-50, 100)
