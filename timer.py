import time
import math

class Timer:
	def __init__(self):
		self.startTime=time.time()
	
	def timerOver(self):
		return time.time() - self.startTime > 180
		
	def requestStage(self):
		return math.floor((time.time()-self.startTime)/60)+1
