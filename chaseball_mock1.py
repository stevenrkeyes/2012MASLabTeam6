import arduino, balls, rectangulate, drive2, time, cv
 
begintime = time.time()
ard = arduino.Arduino()
motors = drive2.DrivePeg(ard)
ard.run()
counter = 0
cam = cv.CaptureFromCAM(1)
HSV_values = balls.readBallData()
listOfErrors = [0]

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

counterBallNotSeen = 0
while time.time() < (begintime + 180):
	detectWall = False		# Implement this code	
	temp=balls.followBall(cam, HSV_values[0], HSV_values[1])
	if (detectWall):		# later, when I figure out IR
		pass 		#Avoid Wall
	elif len(temp)>2:
		counter = 0
		camwidth=temp[4]
		camheight=temp[5]
		if (temp[0] > (camwidth/2)):
			motors.forward(80 - int(pidShit((temp[0]-camwidth), camwidth, listOfErrors)), 80)
			time.sleep(.1)
			print "turning right"
		else:
			motors.forward(80, 80 - int(pidShit(temp[0], camwidth, listOfErrors)))
			print "turning left"
			time.sleep(.1)
		listOfErrors.append((camwidth/2) - temp[0])
		#print str(camheight) + "Camheight"

	else:
		motors.stopMotors()
		print "Nope"
		counter+= 1
		if (counter >= 6):
			motors.turn(80, 0.2)
			print "searching..."
			counter = 0
			listOfErrors = [0]
	#print "time", str(time.time() - begintime)
motors.stopMotors()
ard.stop()








