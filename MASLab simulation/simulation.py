# This file calls the relevant classes and makes a simulation
#

from world import *
from robot import *

# here are some walls
w1 = wall((10,10),(490,10))
w2 = wall((490,10),(490,450))
w3 = yellowWall((490,450),(450,490))
w4 = wall((450,490),(10,490))
w5 = wall((10,490),(10,10))

# here is a ball
b = ball((100,100), "red")

# here is a robot
r = robot((250,200))

# simulate a world with them in it
w = world((500,500),[w1,w2,w3,w4,w5],[b],r)
win = Window("Team 666 MASLab Simulation")

# start the display, start the world running, and run the display
w.initializeDisplay(win)
w.run()
w.win.mainloop()
