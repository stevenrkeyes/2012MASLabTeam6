# program for testing wall-following behavior

# import useful modules
import time, math, threading
# import vision things
#import cv, walls, balls, rectangulate
# import our modules for controlling the robot devices
import arduino, roller, bumper, ballDetector, omni, light, servo, ir



if __name__ == "__main__":
    # Create class instances
    ard = arduino.Arduino()
    driveSystem = omni.Omni(ard)
    blinkies = light.masterLight(ard)
    startSwitch = ballDetector.switch(ard, 11)
    gateServo = servo.Servo(ard)
    irSensors = ir.wallDetector(ard)
    bumperSensors = bumper.Bumper(ard)
    intakeRoller = roller.Roller(ard)
        
    # Run the arduino, power up systems
    ard.run()
    time.sleep(0.25) # wait for the pins to initialize
    blinkies.powerOn() # power on the debug lights
    
    # wait until the start button is pressed
    while startSwitch.getValue() != True:
            time.sleep(0.1)
    
    # create and start a timer for the match
    timer = threading.Timer(60.0, lambda: None)
    timer.start()
    
    intakeRoller.startRoller()
    
    # do this while the timer is still running
    while not timer.finished.isSet():
        # if the robot's backed into a wall, drive away
        if bumperSensors.leftBumped and bumperSensors.rightBumped:
            driveSystem.forward(100)
        # or if just a corner, strafe away
        elif bumperSensors.leftBumped:
            driveSystem.forwardSlide(100, 60)
        elif bumperSensors.rightBumped:
            driveSystem.forwardSlide(100, -60)

        # this is where you probably want to look for balls, also
        # same with yellow walls
        
        # otherwise, do some wall following
        # see a wall on the right?
        elif irSensors.rightIR.getValue() > 20:
            driveSystem.forward(100)
        elif irSensors.rightIR.getValue() < 20:
            driveSystem.forwardSlide(-100, 45)

        time.sleep(0.1)

    driveSystem.stopMotors()
    ard.stop()
            
        
    
