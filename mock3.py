import arduino, cv, time, math, threading
import rectangulate, timer, roller
import omni, walls, light, balls


def pidShit(xpos, xsize, errors):
	previousError = errors[len(errors) - 1]
	currentError = xsize/2 - xpos
	derivative = previousError - currentError
	integral = 0
	for error in errors:
		integral += error
	Kp = 3.0/40 # Set
	Ki = (3.0/140) * 1.5 # These
	Kd = (3.0/140) / 5 # Values
	PID = int((Kp * currentError) + (Kd * derivative) + (Ki * integral))
	print PID, "PID"	
	return PID

def findWall(feed, wall_HSV_values):
	temp = walls.findYellowWall(feed, wall_HSV_values)
	return temp	

def findBall(feed, ball_HSV_values):
	temp = balls.followBall(feed, ball_HSV_values[0], ball_HSV_values[1])
	return temp

def chaseStuff(temp, listOfErrors):
	camwidth=temp[4]
	camheight=temp[5]
	motors.forward(-80)
	motors.backDrift((temp[0]-(camwidth/2))/6)
	#getPID = 3*pidShit(temp[0], camwidth, listOfErrors)
	#if getPID < 100:
		#motors.backDrift(-getPID)
	#time.sleep(.1)	
	#if (temp[0] > (camwidth/2)):
	#	motors.backDrift(getPID)
	#	#TODO: Implement omni drive and Omni PID
	#	time.sleep(.1)
	#	print "turning right" , getPID 
	#else:
	#	motors.backDrift(-getPID)
	#	#TODO: Implement omni drive and Omni PID
	#	print "turning left", getPID
	#	time.sleep(.1)
	#listOfErrors.append(getPID)
	#for error in listOfErrors:
	#	print error
	return listOfErrors

if __name__ == "__main__":
	#Create class instances
	ard = arduino.Arduino()
	timer = timer.Timer()
	motors = omni.Omni(ard)
	light=light.masterLight(ard)
	ard.run()
	light.powerOn()
	
	hasBalls = False
	counter = 0 
	cam = cv.CaptureFromCAM(0)		# Initialize camera
	wall_values = walls.readWallsData()
	HSV_values = balls.readBallData() 	# Calibration
	listOfErrors = [0]
	oldSearch = 0			# Variables to reset listOfErrors
	newSearch = 1			# when chasing a different ball or chasing a wall
	motors.forward(-40)
	time.sleep(0.1)
	while not timer.timerOver():
		img = cv.QueryFrame(cam)
		wallList = findWall(img, wall_values)
		print list(wallList), "--> walls"
		ballList = findBall(img, HSV_values)
		if (len(wallList) > 0 and hasBalls):
			newSearch = 0
			if oldSearch != newSearch:
				listOfErrors = [0]
			listOfErrors = chaseStuff(wallList, listOfErrors)
			counter = 0
			oldSearch = 0
		if (len(ballList) > 2):
			newSearch = 1
			if oldSearch != newSearch:
				listOfErrors = [0]
			listOfErrors = chaseStuff(ballList, listOfErrors)
			counter = 0
			hasBalls = True #impliment touch sensor later
			oldSearch = 1
		else:
			motors.stopMotors()
			print "Nope"
			counter+= 1
			if (counter >= 6):
				motors.turnRight(80, 60)
				time.sleep(0.1)
				print "searching..."
				counter = 0
				listOfErrors = [0]
				oldSearch = 2	
				newSearch = 2
	motors.stopMotors()
	ard.stop()
