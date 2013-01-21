import arduino, time, threading

# This is a class for a checker of the ball intake limit switch array


class ballDetector:
    def __init__(self, ard):
        self.ard = ard
        
        self.switches = [arduino.DigitalInput(ard, pin)
                         for pin in range(35, 55, 2)] # odd pins 35 to 53

    # check a switch
    # if it's turned on, wait til its turned off
    # increment the counter, wait a delay before checking again
    # to account for slipping

    def readSwitches(self):
        return [switch.getValue() for switch in self.switches]


if __name__ == "__main__":
    import roller
    ard = arduino.Arduino()
    bD = ballDetector(ard)
    r = roller.Roller(ard)

    ard.run()

    start = time.time()
    while time.time()-start < 10:
        print bD.readSwitches()
        time.sleep(0.025)

    #bD.start()
