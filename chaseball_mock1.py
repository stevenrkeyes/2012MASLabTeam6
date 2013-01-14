import arduino, balls, rectangulate, drive2, time, cv
 
begintime = time.time()
ard = arduino.Arduino()
motors = drive2.DrivePeg(ard)

cam = cv.CaptureFromCAM(1)

def pidShit(ypos, ysize, errors):
	previousError = errors[len(errors) - 1]
	currentError = ypos - ysize
	derivative = previousError - currentError
	integral = 0
	for error in errors:
		integral += error
	Kp = 60/ysize # Set
	Ki = (60/ysize) * 1.5 # These
	Kd = (60/ysize) / 10 # Values
	PID = (Kp * currentError) + (Kd * derivative) + (Ki * integral)
	return PID

counterBallNotSeen = 0
listOfErrors = (0)
while time.time() < begintime + 180:
	detectWall = False		# Implement this code	
	temp=balls.followBall(cam)
	if (detectWall):		# later, when I figure out IR
		pass 		#Avoid Wall
	elif len(temp)>2:
		counter = 0
		camwidth=temp[4]
		camheight=temp[5]
		if (temp[0] > camwidth):
			motors.forward(50, 50 - pidShit(temp[0]-camwidth, camwidth, listOfErrors))
			print "turning right"
			listofErrors += temp[0]-camwidth
		else:
			motors.forward(50 - pidShit(temp[0], camwidth, listOfErrors, 50))
			print "turning left"
			listOfErrors += -temp[0]
		print str(camheight) + "Camheight"
		chaseBall(temp[0])
	else:
		motors.stopMotors()
		print "Nope"
		counter+= 1
		if (counter >= 6):
			motors.turn(50, .3)
			print "searching..."

motors.stopMotors()
ard.stop()








