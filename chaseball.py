import arduino, drivepeg, time, balls, cv

begintime = time.time()
ard = arduino.Arduino()
motors = drivepeg.DrivePeg(ard)

cam = cv.CaptureFromCAM(1)

def chaseBall(xpos):
	if (camwidth/2)-(camwidth/6) < xpos < (camwidth/2)+(camwidth/6):
		motors.stopMotors()
	else:
		if xpos<camwidth/2:
			#motors.turnForever(-50)
			motors.turn(-50, abs(1-(xpos/(camwidth/2)))*27.6/90)
			print "turning left"
			time.sleep(0.1)
		else:
			#motors.turnForever(50)
			motors.turn(50, abs(1-(xpos/(camwidth/2)))*27.6/90)
			print "turning right"
			time.sleep(0.1)

while True:
	motors.stopMotors()
	temp=balls.followBall(cam)
	if len(temp)>2:
		camwidth=temp[4]
		camheight=temp[5]
		print str(camheight) + "Camheight"
		chaseBall(temp[0])
	else:
		print "Nope"
