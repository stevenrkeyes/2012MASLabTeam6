import arduino, balls, rectangulate, time, cv, threading, omni, math, walls


def pidShit(xpos, xsize, errors):
	previousError = errors[len(errors) - 1]
	currentError = xsize/2 - xpos
	derivative = previousError - currentError
	integral = 0
	for error in errors:
		integral += error
	Kp = 3.0/xsize # Set
	Ki = (3.0/xsize) * 1.5 # These
	Kd = (3.0/xsize) / 5 # Values
	PID = int((Kp * currentError) + (Kd * derivative) + (Ki * integral))
	print PID, "PID"	
	return abs(PID)
def findWall(feed, wall_HSV_values):
	walls = walls.findYellowWall(feed, wall_HSV_values)
	return walls	

def findBall(feed, ball_HSV_values):
	temp = balls.followBall(feed, ball_HSV_values[0], ball_HSV_values[1])
	return temp

def chaseStuff(temp, listOfErrors):
	camwidth=temp[4]
	camheight=temp[5]
	if (temp[0] > (camwidth/2)):
		#motors.forward(80 - int(pidShit((temp[0]-camwidth), camwidth, listOfErrors)), 80)
		#TODO: Implement omni drive and Omni PID
		time.sleep(.1)
		print "turning right"
	else:
		#motors.forward(80, 80 - int(pidShit(temp[0], camwidth, listOfErrors)))
		#TODO: Implement omni drive and Omni PID
		print "turning left"
		time.sleep(.1)
	listOfErrors.append((camwidth/2) - temp[0])
	return listOfErrors

if __name__ == "__main__":
	counterBallNotSeen = 0
	begintime = time.time()
	ard = arduino.Arduino()
	motors = omni.Omni(ard)
	ard.run()
	hasBalls = False
	counter = 0
	cam = cv.CaptureFromCAM(1)		# Initialize camera
	wall_values = walls.readWallsData()
	HSV_values = balls.readBallData() 	# Calibration
	listOfErrors = [0]
	oldSearch = 0			# Variables to reset listOfErrors
	newSearch = 1			# when chasing a different ball or chasing a wall
	#motors.forward(80)
	while time.time() < (timer + 180):
		img = cv.QueryFrame(cam)
		walls = findWalls()
		balls = findWall()
		if (len(walls) > 2 and hasBalls):
			newSearch = 0
			if oldSearch != newSearch:
				listOfErrors = [0]
			listOfErrors = chaseStuff(walls, listOfErrors)
			counter = 0
			oldSearch = 0
		elif (len(balls) > 2):
			newSearch = 1
			if oldSearch != newSearch:
				listOfErrors = [0]
			listOfErrors = chaseStuff(walls, listOfErrors)
			counter = 0
			hasBalls = True #impliment touch sensor later
			oldSearch = 1
		else:
			motors.stopMotors()
			print "Nope"
			counter+= 1
			if (counter >= 6):
				motors.turn(80, 0.2)
				print "searching..."
				counter = 0
				listOfErrors = [0]
				oldSearch = 2	
				newSearch = 2
	motors.stopMotors()
	ard.stop()








