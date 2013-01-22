import arduino, time

# class that is a controller for the roller motor
class Roller:
    def __init__(self, ard):
        self.ard = ard
        self.mRoller = arduino.Motor(ard, 11, 12, 13)

    def startRoller(self):
        self.mRoller.setSpeed(120)

    def stopRoller(self):
        self.mRoller.setSpeed(0)

if __name__ == "__main__":
    ard = arduino.Arduino()
    r = Roller(ard)

    ard.run()
    r.startRoller()
    time.sleep(5)
    r.stopRoller()
    arduino.stop()
