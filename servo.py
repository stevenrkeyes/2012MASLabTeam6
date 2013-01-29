import time, arduino

class Servo:
	def __init__(self, ard):
		self.servo = arduino.Servo(ard, 48)  # Create a Servo object

	def openGate(self):
		self.servo.setAngle(65)
		print "Gate opened"
		time.sleep(0.5)

	def closeGate(self):
		self.servo.setAngle(0)
		print "Gate opened"
		time.sleep(0.5)
