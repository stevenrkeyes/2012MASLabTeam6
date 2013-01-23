import arduino, time, threading
from random import randint, random

#Flashes the random lights on the top of the robot

class masterLight:
	def __init__(self, ard):
		self.l1 = arduino.DigitalOutput(ard, 24)
		self.l2 = arduino.DigitalOutput(ard, 26)
		self.l3 = arduino.DigitalOutput(ard, 28)
		self.l4 = arduino.DigitalOutput(ard, 30)
		self.l5 = arduino.DigitalOutput(ard, 32)
		self.l6 = arduino.DigitalOutput(ard, 34)
		self.lightList = [self.l1, self.l2, self.l3, self.l4, self.l5, self.l6]
	
	def lightFlash(self):	
		while True:
			for i in self.lightList:
				i.setValue(randint(0,1))
				time.sleep(random()/2)
		
	def powerOn(self):	
		t = threading.Thread(target = self.lightFlash, args = [])
		t.start()


if __name__ == "__main__":
	ard = arduino.Arduino()  # Create the Arduino object
	masterLight=masterLight(ard)
	
	ard.run()  # Start the thread which communicates with the Arduino
	masterLight.powerOn() # Start the light effects

