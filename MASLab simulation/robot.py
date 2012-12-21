# Author: Steven Keyes
# MASLab team 6
# 18 Dec 2012
# this file contains the robot class
# which is a simulated robot that lives in a simulated world.
# the robot takes a python brain that is a function of what to do based on inputs
# and which could hypothetically be used in a real robot

#to draw the simulation
from graphics import *
import time
import math

# sums a list term by term
def listSum(l1, l2):
    return [x+y for x,y in zip(l1,l2)]


# class for simulation of robot body
class robot:
    def __init__(self, pos):
        # position (x,y)
        self.pos = (float(pos[0]),float(pos[1]));
        # angular heading from vertical
        self.heading = 0
        
        # the speed at which the robot is currently driving fwd
        self.speed = 50.0
        # the speed at which the robot is currently rotating ccw
        self.angvel = 0.0002*math.pi/2.0;

        self.radius = 20.0

        self.body = Circle(Point(pos[0],pos[1]),self.radius)
        self.indicator = Line(Point(pos[0],pos[1]),Point(pos[0],pos[1]+self.radius))
        self.indicator.setArrow("last")

        # a function that defines how the robot should behave
        # by taking the robot's possible sensor inputs
        # and suggesting outputs
        #self.brain = brain

    def draw(self,canvas):
        self.body.draw(canvas)
        #self.leftWheel.draw(canvas)
        #self.rightWheel.draw(canvas)
        self.indicator.draw(canvas)

    def requestUpdate(self, timePassed):
        
        dpos = (self.speed*timePassed*math.sin(self.heading)/1000.0,
                self.speed*timePassed*math.cos(self.heading)/1000.0)
        dheading = self.angvel*timePassed

        return (self.pos, dpos, self.heading, dheading)

    def updatePos(self, dpos, dheading, canvas):
        self.pos = (self.pos[0] + dpos[0],
                    self.pos[1] + dpos[1])
        self.heading += dheading

        self.body.move(dpos[0], dpos[1])
        #self.leftWheel.move(dpos[0], dpos[1])
        #self.rightWheel.move(dpos[0], dpos[1])
        #self.indicator.move(dpos[0], dpos[1])
        self.indicator.undraw()
        self.indicator.p1 = Point(self.pos[0],
                                  self.pos[1])
        self.indicator.p2 = Point(self.pos[0] + self.radius*math.sin(self.heading),
                                  self.pos[1] + self.radius*math.cos(self.heading))
        self.indicator.draw(canvas)

        # have a property of the robot that is a function
        # the function takes in the current state of the field
        # and it returns angvel and speed settings
        # this is the brain of the robot
        # and, in the actual robot, it would be the main loop of the robot's code
        '''
        (self.speed, self.angvel) = self.plan(self.pos,
                                              self.heading,
                                              self.speed,
                                              self.angvel,
                                              walls, #get this somehow
                                              balls #same
                                              )
        '''
        # and maybe this function should also have access
        # to move balls around

    
