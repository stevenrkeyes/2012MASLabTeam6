import arduino

class wallDetector():

	def __init__(self, ard):
		self.leftIR = arduino.AnalogInput(ard, 12)
		self.rightIR = arduino.AnalogInput(ard, 13)

	def getDistances(self):
		return [self.leftIR.getValue(), self.rightIR.getValue()]
		#print  [self.leftIR.getValue(), self.rightIR.getValue()], "IR shit"

	def getTurn(self):
		d = self.getDistances()
		if d[0] < d[1]:
			return (1)
		else:
			return (-1)
		#print  [self.leftIR.getValue(), self.rightIR.getValue()], "IR shit"

	
	def detectWall(self):
		d = self.getDistances()
		#print "Wall Distances: ", d[0], d[1]
		return d[0] > 600 or d[1] > 600

	def detectCloseWall(self):
		d = self.getDistances()
		#print "Wall Distances: ", d[0], d[1]
		return d[0] > 850 or d[1] > 850


