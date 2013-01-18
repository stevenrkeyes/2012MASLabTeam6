import arduino, balls, rectangulate, time, cv, threading, omni, math


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

def chaseBalls(timer, counter, listOfErrors, ball_HSV_values):
	detectWall = False		# TODO: Implement this code (IR sensors, bumpers, etc.)
	temp=balls.followBall(cam, ball_HSV_values[0], ball_HSV_values[1])
	if (detectWall):		# later, when I figure out IR
		pass 			# TODO: Avoid Wall
	elif len(temp)>2:
		counter = 0
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

if __name__ == "__main__":
	counterBallNotSeen = 0
	begintime = time.time()
	ard = arduino.Arduino()
	motors = omni.Omni(ard)
	ard.run()
	
	counter = 0
	cam = cv.CaptureFromCAM(1)		# Initialize camera
	HSV_values = balls.readBallData() 	# Calibration
	listOfErrors = [0]
	motors.forward(80)
	#while time.time() < (timer + 180):
			#chaseballs(begintime, counter, listOfErrors, HSV_values)	
	motors.stopMotors()
	ard.stop()








