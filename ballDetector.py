import arduino, time, threading

# this is a class that represents a limit switch with
# a cooldown function to act as a low pass filter for
# noisy signals from bouncy switches
class switch(arduino.DigitalInput):

    # amount of time to ignore a switch after it is first pressed
    cooldownTime = 1.0 # seconds
    
    def __init__(self, ard, pin):
        
        # variable to tell if the switch has been pressed recently
        # and, thus, should be ignored for a brief amount of time
        self.cooldown = False
        
        arduino.DigitalInput.__init__(self, ard, pin)

    def activateCooldown(self):
        # set the switch to cooldown, indicating it has been pressed recently
        # and should not be read for a brief amount of time
        self.cooldown = True
        t = threading.Thread(target=self.reactivateSwitch, args=[self.cooldownTime])
        t.start()

    # reactiveate the switch after a cooldown delay
    def reactivateSwitch(self, delay):
        time.sleep(delay)
        self.cooldown = False
        
        

# This is a class for a tool that checks the array of limit switches
# at the ball intake to detect if balls have been collected
class BallDetector:

    # time between checking the sensors
    refreshTime = 0.050 # seconds
        
    def __init__(self, ard):
        self.ard = ard
        self.switches = [switch(ard, pin)
                         for pin in range(35, 55, 2)] # odd pins 35 to 53
        self.ballCount = 0
        self.ballCountLock = threading.Lock()
    
    # check a switch
    # if it's turned on, wait til its turned off
    # increment the counter, wait a delay before checking again
    # to account for slipping

    def readSwitches(self):
        switchReadings = []

        # a ball can activate as many as 2 switches at a time, so the
        # neighboring 1 switch on either side should also be ignored
        # (see below)

        for n in range(len(self.switches)):
            # if the switch is cooling down, ignore it
            if self.switches[n].cooldown:
                switchReadings.append(False)
            # otherwise, read the value from the switch
            else:
                switchReadings.append(self.switches[n].getValue())
                # if the switch was on, activate cooldown for it
                if switchReadings[n]:
                    self.switches[n].activateCooldown()
                    # if there are neighbors, cool them down, too
                    for m in [n-1, n+1]:
                        try: self.switches[m].activateCooldown()
                        except IndexError: pass
        
        return switchReadings

    def resetBallCount(self):
        with self.ballCountLock:
            self.ballCount = 0

    # do i need to add the lock here?
    def getBallCount(self):
        return int(self.ballCount)

    def hasBalls(self):
        return self.getBallCount() > 0

    def detectionLoop(self):
        while True:
            state = self.readSwitches()
            numBallsDetected = sum(state)
            with self.ballCountLock:
                self.ballCount += numBallsDetected
            
            time.sleep(self.refreshTime)

    def start(self):
        t = threading.Thread(target=self.detectionLoop)
        t.start()

if __name__ == "__main__":
    import roller
    ard = arduino.Arduino()
    bD = BallDetector(ard)
    r = roller.Roller(ard)

    ard.run()
    time.sleep(0.5) # wait for the pins to stabilize
    
    r.startRoller()
    bD.start()
    
    start = time.time()
    while time.time()-start < 20:
        print bD.getBallCount()#, bD.readSwitches()
        time.sleep(0.25)
    
    r.stopRoller()

    #bD.run()
