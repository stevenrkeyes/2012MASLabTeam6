import arduino, threading

#Flashes the random lights on the top of the robot

class Bumper:
	def __init__(self, ard):
		self.leftBumper = arduino.DigitalInput(ard, 52)
		self.rightBumper = arduino.DigitalInput(ard, 50)
	
	def leftBumped(self):	
		return self.leftBumper.getValue()>0
		
	def rightBumped(self):	
		return self.rightBumper.getValue()>0

	# Threading crap
	#def powerOn(self):	
	#	t = threading.Thread(target = self.lightFlash, args = [])
	#	t.start()
