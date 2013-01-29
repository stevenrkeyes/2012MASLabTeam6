import arduino, cv, time, math, threading
import rectangulate, timer, roller, bumper, ballDetector
import omni, walls, light, balls, servo, ir

# note: not necessary to initialize these in python
timerOver = False
ard = 0
motors = 0
_roller = 0
wallList = []
ballList = []

# calculate the PID correction from the camera based on the
# x position of a ball and the width (xsize) of the camera
def pidShit(xpos, xsize, errors):
	previousError = errors[-1]
	currentError = xsize/2 - xpos
	derivative = previousError - currentError
	integral = sum(errors)
	Kp = 3.0/40 # Set
	Ki = (3.0/140) * 1.5 # These
	Kd = (3.0/140) / 5 # Values
	PID = int(round((Kp * currentError) + (Kd * derivative) + (Ki * integral)))
	print PID, "PID"	
	return PID

def findWall(feed, wall_HSV_values):
	temp = walls.findYellowWall(feed, wall_HSV_values)
	return temp

def findBall(feed, ball_HSV_values):
	temp = balls.followBall(feed, ball_HSV_values[0], ball_HSV_values[1])
	return temp

def lineUp(backBumper, gate, motors):
	# back away from the wall a little
	motors.backward(-80)
	time.sleep(0.5)
	
	# rotate around to face the wall
	motors.turnRight(80, 180)
	
	# drive to the wall, throttling the appropriate motor until aligned
	atWall = False
	motors.forward(40)
	while not atWall:
		if backBumper.leftBumped() and not backBumper.rightBumped():
			motors.right(40)		
		elif not backBumper.leftBumped() and backBumper.rightBumped():
			motors.left(40)
		elif backBumper.leftBumped() and backBumper.rightBumped():
			# you're lined up!
			atWall = True
		else:
			motors.forward(40)

	# go in a bit
	motors.backward(60)
	time.sleep(0.5)
	gate.openGate()
	motors.forward(80)
	time.sleep(1)
	motors.stopMotors()
	time.sleep(2.5)
	gate.closeGate()
def cameraShit(cam):
        while not timerOver:
                img = cv.QueryFrame(cam)
        	wallList = findWall(img, wall_values)
                ballList = findBall(img, HSV_values)
                time.sleep(0)
def halt():
	_lock = threading.Lock()
	_lock.acquire()
	motors.stopMotors()
	_roller.stopRoller()
	ard.stop()
	# note: ard.killReceived becomes True when the arduino is stopped
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
	# Create class instances
	ard = arduino.Arduino()
	motors = omni.Omni(ard)
	light=light.masterLight(ard)
	onSwitch = arduino.DigitalInput(ard, 11)
	gate = servo.Servo(ard)
	irSensors = ir.wallDetector(ard)
	backBumper=bumper.Bumper(ard)
	ballDetect = ballDetector.BallDetector(ard)
	
	_roller = roller.Roller(ard)

	# Run the arduino, power up systems
	ard.run()
	light.powerOn()
	# wait until the start button is pressed
	isGreen = False
	isBumped = False
	while not isBumped:
		if backBumper.leftBumped():
			isBumped = True
			print "RED"
			isGreen = False
		elif backBumper.rightBumped():
			isBumped = True
			print "GREEN"
			isGreen = True
		time.sleep(0.05)

	# create and start a timer for the match
	timer = threading.Timer(180.0, halt)
	timer.start()
	ballDetect.start()
	_roller.startRoller()
	
	hasBalls = False
	counter = 0
	
	cam = cv.CaptureFromCAM(1)		# Initialize camera
	wall_values = walls.readWallsData()
	HSV_values = balls.readBallData(isGreen) 	# Calibration
	listOfErrors = [0]
	oldSearch = 0			# Variables to reset listOfErrors
	newSearch = 1			# when chasing a different ball or chasing a wall

	motors.forward(-40)
	dirTurn = 1
	time.sleep(0.1)
	isYellowWall = False
	counterTurn = 0
	
	#getCam = threading.Thread(target = cameraShit, args = [cam])
	#getCam.start()
	
	while not timerOver:
		# Find walls, find balls, get wrecked
		img = cv.QueryFrame(cam)
        	wallList = findWall(img, wall_values)
		#cv.SaveImage("lol.png", img)
                ballList = findBall(img, HSV_values)
		
		# State Machine lulz
		hasBalls = ballDetect.getBallCount() > 1
		print ballDetect.getBallCount(), "<--Ballz"
		if irSensors.detectWall():
			print "THERE IS A WALL LOL"
			if (isYellowWall):		
				# Rotate, Line up, and Score		
				lineUp(backBumper, gate)
				ballDetect.resetBallCount()
			else: # not a yellow wall
				print "turning away from wall LOL"
				motors.turnLeft(irSensors.getTurn()*80, 30) # Just Position based correction
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
			oldSearch = 1
		else:
			motors.stopMotors()
			print "Nope"
			counter+= 1
			if (counter >= 3):
				if counterTurn < 5:
					motors.forward(-80)
				time.sleep(2)
				print "searching..."
				counter = 0
				listOfErrors = [0]
				oldSearch = 2	
				newSearch = 2
				counterTurn += 1
			if counterTurn > 3:
				motors.turnRight(dirTurn*80, 90)
				dirTurn *= -1
				counterTurn = 0
