import arduino, drivepeg, time, balls, cv, math

begintime = time.time()
ard = arduino.Arduino()
motors = drivepeg.DrivePeg(ard)

cam = cv.CaptureFromCAM(1)

def chaseBall(xpos, ypos):
	if (camwidth/2)-(camwidth/6) <= xpos <= (camwidth/2)+(camwidth/6):
		motors.forwardTime(30, 1)
	else:
		if xpos<camwidth/2:
			#motors.turnForever(-50)
			motors.turn(-30, math.fabs(1-(xpos/(camwidth/2.0)))*50/90)
			print "turning left"
			time.sleep(0.1)
			motors.stopMotors()
			
		else:
			#motors.turnForever(50)
			motors.turn(30, math.fabs(1-((camwidth-xpos)/(camwidth/2.0)))*50/90)
			print "turning right"
			time.sleep(0.1)
			motors.stopMotors()

while True:
	motors.stopMotors()
	temp=balls.followBall(cam)
	if len(temp)>2:
		camwidth=temp[4]
		camheight=temp[5]
		chaseBall(temp[0], temp[1])
	else:
		print "Nope"
