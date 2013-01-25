import arduino, cv, time, math, threading
import rectangulate, timer, roller, bumper, ballDetector
import omni, walls, light, balls, servo, ir

timerOver = False
ard = 0
motors = 0
roller = 0
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

def lineUp(backBumper, gate):
	motors.backward(80)
	time.sleep(0.2)
	motors.turnRight(80, 180)
	atWall = False
	#motors.backwardsUnlimited() <--How go backwards???? ;)
	while not atWall:
		if bumper.leftBumped() and not bumper.rightBumped():
			omni.stopA()
		elif not bumper.leftBumped() and bumper.rightBumped():
			omni.stopB()
		elif bumper.leftBumped() and bumper.rightBumped():
			time.sleep(1)
			omni.stopMotors()
			atWall = True
	gate.openGate()
	time.sleep(2.5)
	gate.closeGate()

def halt():
	_lock = threading.Lock()
	_lock.acquire()
	motors.stopMotors()
	roller.stopRoller()
	ard.stop()
	_lock.release()
def chaseStuff(temp, listOfErrors):
	camwidth=temp[4]
	camheight=temp[5]
	motors.forward(-80)
	motors.backDrift(-(temp[0]-(camwidth/2))/6)
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
	timer = threading.Timer(180.0, halt)
	timer.start()
	ard = arduino.Arduino()
	motors = omni.Omni(ard)
	light=light.masterLight(ard)
	onSwitch = ballDetector.switch(ard, 11)
	gate = servo.Servo(ard)
	irSensors = ir.wallDetector(ard)
	bumper=bumper.Bumper(ard)
	roller = roller.Roller(ard)
	ard.run()
	roller.startRoller()
	light.powerOn()
	while onSwitch.getValue() != True:
		time.sleep(0.1)
	hasBalls = False
	counter = 0 
	cam = cv.CaptureFromCAM(1)		# Initialize camera
	wall_values = walls.readWallsData()
	HSV_values = balls.readBallData() 	# Calibration
	listOfErrors = [0]
	oldSearch = 0			# Variables to reset listOfErrors
	newSearch = 1			# when chasing a different ball or chasing a wall
	motors.forward(-40)
	dirTurn = 1
	time.sleep(0.1)
	isYellowWall = False
	counterTurn = 0
	while not timerOver:
		img = cv.QueryFrame(cam)
		wallList = findWall(img, wall_values)
		print list(wallList), "--> walls"
		ballList = findBall(img, HSV_values)
		if irSensors.detectWall():
			print "THERE IS A WALL LOL"
			if (isYellowWall):				
				lineUp(irSensors, gate)
			else:
				print "turning away from wall LOL"
				motors.turnLeft(irSensors.getTurn()*80, 30) #Just Position based correction
				time.sleep(0.1)
		elif (len(wallList) > 0 and hasBalls):
			print "CHASING WALL LOL"
			newSearch = 0
			if oldSearch != newSearch:
				listOfErrors = [0]
				isYellowWall = True
			listOfErrors = chaseStuff(wallList, listOfErrors)
			counter = 0
			oldSearch = 0
		elif (len(ballList) > 2):
			print "CHASING BALL LOL"
			newSearch = 1
			if oldSearch != newSearch:
				listOfErrors = [0]
				isYellowWall = False
			listOfErrors = chaseStuff(ballList, listOfErrors)
			counter = 0
			hasBalls = True #impliment touch sensor later
			oldSearch = 1
		else:
			motors.stopMotors()
			print "Nope"
			counter+= 1
			if (counter >= 6):
				isYellowWall = False
				motors.turnRight(dirTurn * -80, 40)
				time.sleep(0.1)
				print "searching..."
				counter = 0
				listOfErrors = [0]
				oldSearch = 2	
				newSearch = 2
				counterTurn += 1
			if counterTurn > 5:
				motors.forward(-80)
				dirTurn = -dirTurn
				time.sleep(1)
				counterTurn = 0
				motors.stopMotors()
