import arduino, math, time, threading

#This function checks if the Arduino 
#has been fed 5v, and if it has, checks 
#the global 'checker' variable to see if 
#appropriate action (delay and power on) 
#has already been taken

# class to store the configuration of the power monitoring pins and power controller pins
# this class could be expanded to have lots of monitors and controllers
class powerMonitorController:
	def __init__(self, ard):
		self.ard = ard
		# pin that reads if the regulated 5V is ok
		self.mon5V = arduino.AnalogInput(ard, 1)
		# pin that turns on the switched 12V for the lights and MC
		self.con12V = arduino.AnalogOutput(ard, 3)
	
	def checkPowerOK(self):
		if self.mon5V.getValue>1000:
			return True
		else:
			print "Battery monitor test not passed"
			return False
	
	def powerOnController(self):
		self.con12V.setValue(1023)
	
	def powerOffController(self):
		self.con12V.setValue(0)

	def powerMonitorLoop(self):
		while True: # should be replaced with some flag that turns off when the robot turns off
			# if the power is still ok, keep the robot on
			if self.checkPowerOK():
				self.powerOnController()
			else:
				self.powerOffController()
			# repeat every 1 second
			time.sleep(1)
	
	# run this to run power monitors and power up the robot
	def powerOn(self):	
		# first, check if we can power up the robot the first time
		while not self.checkPowerOK():
			time.sleep(0.25)
			
		# wait a second after power comes online
		time.sleep(1)
		print "Battery active, sending impulse to control circuit"
			
		# start a thread running
		# that loops and checks if the power is normal
		# and, if so, powers the logic
		# and, if not, gives some warning
		t = threading.Thread(target = self.powerMonitorLoop, args = [])
		t.start()


if __name__ = "__main__":
	ard = arduino.Arduino()  # Create the Arduino object
	monCon = powerMonitorController(ard) # initialize a controller for checking the power
	
	ard.run()  # Start the thread which communicates with the Arduino
	monCon.powerOn() # Start the monitor/controller for power

