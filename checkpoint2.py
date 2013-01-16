import arduino
import drivepeg

if __name__ == "__main__":
	ard = arduino.Arduino()
	bumper = arduino.Analog(ard, 0)
	motors = drivepeg.DrivePeg(ard)
	while True:
		motors.forward(50)
		if bumper.getValue() > 1000:
			motors.stopMotors()
			motors.backwardTime(50, 100)
			motors.turn(50, 100)
			
