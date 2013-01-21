import arduino, time, math, omni

l=0.159
W=math.sqrt(0.084914-0.466*l+l*l)
sina=0.175/W

if __name__ == "__main__":
	ard = arduino.Arduino()
	omni=omni.Omni(ard)
	ard.run()
	omni.forwardArc(-80,-20)

	time.sleep(5)
	omni.stopMotors()
	print "lol"
	ard.stop()
