# Author: Steven Keyes
# MASLab team 6
# 18 Dec 2012
# this file contains the world class
# which is a simulated world featuring walls, balls, and a robot on a field

#to draw the simulation
from graphics import *
import time

import math

# sums a list term by term
def listSum(l1, l2):
    return [x+y for x,y in zip(l1,l2)]

class world:
    def __init__(self, dims, walls, balls, robot):
        # a list of the walls of the field
        self.walls = walls

        # a listing of the balls currently on the field
        self.balls = balls

        self.robot = robot

        (self.width, self.height) = dims

    def initializeDisplay(self, win):
        # make a canvas
        self.win = win
        self.canvas = CanvasFrame(self.win,
                                  self.width,
                                  self.height)
        self.canvas.setBackground("white")

        # draw all the walls
        for wall in self.walls:
            wall.draw(self.canvas)

        # draw all the balls
        for ball in self.balls:
            ball.draw(self.canvas)

        # draw the robot
        self.robot.draw(self.canvas)

        print "display initialized"

    # update the world based on the amount of time passed
    def update(self, timePassed):
        # request is (pos, dposRequested, heading, dheadingRequested); it's what the robot tries to do
        request = self.robot.requestUpdate(timePassed)

        # update the request to take the physics of the world into account
        request = self.updateForPhysics(request)

        # you should reorganize this data so it's not dumb
        self.robot.updatePos(request[1], request[3], self.canvas)
        
        self.canvas.update()

    # right now this is just checking for wall collisions
    def updateForPhysics(self, request):
        # scalar for friction against wall
        fricScalar = .5
        
        pos = request[0]
        dpos = request[1]
        
        collidedWall = self.checkCollisions(pos)
        # if there's a collision, push the robot back against the wall
        # also slow its movement along the wall to account for friction
        
        if collidedWall:
            #print "collision!"
            
            # break down dpos into two components:
            # the motion vector prior to collision
            # and the motion vector after collision
            # let the robot travel in the motion vector prior to collision
            # but project the vector after collision onto the wall
            # as if the robot is sliding against it
            (vec1, vec2) = world.splitProjectedPath(collidedWall.start[0],
                                                    collidedWall.end[0],
                                                    collidedWall.start[1],
                                                    collidedWall.end[1],
                                                    pos,
                                                    dpos,
                                                    self.robot.radius)
            # Then, scale this vector down by a scalar to simulate friction
            vec2 = (fricScalar*vec2[0], fricScalar*vec2[1])
            # (This isn't strictly well modeled since friction acts on acceleration
            # not on velocity, but this physics model doesn't take acceleration into account)


            dpos = listSum(vec1, vec2)

        request = (pos, dpos, request[2], request[3])

        return request
            

    def run(self):
        self.update(15)
        self.win.after(15, self.run)
        

    # check if the robot has collided with a wall, and returns that wall if so
    def checkCollisions(self, pos):
        for wall in self.walls:
            if world.checkWallInCircle(wall.start[0],
                                       wall.start[1],
                                       wall.end[0],
                                       wall.end[1],
                                       pos,
                                       self.robot.radius):
                return wall
        return False

    # method to split a motion vector into a pre-collision vector
    # and a post-collision projection
    @classmethod
    def splitProjectedPath(cls, x_1, x_2, y_1, y_2, pos, dpos, radius):

        # let b_vec be a vector along the line of the wall
        b_x = x_2 - x_1
        b_y = y_2 - y_1
        b = (b_x, b_y)
        b_mag = (b_x**2.0 + b_y**2.0)**0.5
        
        # first, we find the radius of the robot that is perpendicular to the wall
        # and thus is the point at which the robot hits the wall.
        # I calculated this using a dot product
        r = (radius*b_y/b_mag, -radius*b_x/b_mag)

        # then, the motion of the edge of the robot can be parameterized for 0<=t<=1 as
        # x = pos[0] + dpos[0]*t + r[0]
        # y = pos[1] + dpos[1]*t + r[1]
        # and the wall line as
        # x = b_x*s + x_1
        # x = b_y*s + y_1
        # and these can be combined and solved, resulting in the equation Ax=b, giving x = A^-1 b
        # where x is (s, t), A is the matrix [[b_x, dpos_x], [b_y, dpos_y]], and b is pos+r-(x_1,y_1)
        Aa = b_x
        Ab = dpos[0]
        Ac = b_y
        Ad = dpos[1]
        
        # then, the point of collision is at the t found here,
        detA = Aa*Ad - Ab*Ac
        t = (-1.0/detA)*((-Ac)*(pos[0]+r[0]-x_1) + (Aa)*(pos[1]+r[1]-y_1))

        # and the first vector is t*dpos
        firstVec = (t*dpos[0], t*dpos[1])
        # the second vector is ((1-t)*dpos) projected onto b
        secondVec = world.proj(((1-t)*dpos[0],(1-t)*dpos[1]), b)
        
        return (firstVec, secondVec)


    @classmethod
    def proj(cls,a,b):
        scale = (a[0]*b[0] + a[1]*b[1])/(b[0]**2.0 + b[1]**2.0)
        return (scale*b[0], scale*b[1])

    # method to see if a wall with start and endpoints intersects
    # the circular body of the robot with position and radiu
    @classmethod
    def checkWallInCircle(cls,x_1, y_1, x_2, y_2, pos, radius):
        # I parametriezed the line segment for 0<t<1
        # and combined the x and y terms with the equation for a circle of r=radius centered at pos
        # now we solve and check if the intersection of these equations exists
        # and it if occurs with t between 0 and 1 (circle intersects segment)
        # or with one t greater than 1 and one less than 0 (circle encompasses segment)
        # t^2*((x_1 - x_2)^2 + (y_1 - y_2)^2) + t*2*((x_1 - x_2)*(x_2 - pos[0]) + (y_1 - y_2)*(y_2 - pos[1]))
        #  + (x_2 - pos[0])^2 + (y_2 - pos[1])^2 - radius^2 = 0
        # a*t^2 + b*t + c = 0
        a = ((x_1 - x_2)**2.0 + (y_1 - y_2)**2.0)
        b = 2.0*((x_1 - x_2)*(x_2 - pos[0]) + (y_1 - y_2)*(y_2 - pos[1]))
        c = (x_2 - pos[0])**2.0 + (y_2 - pos[1])**2.0 - radius**2.0
        # determinant
        det = b**2.0 - 4.0*a*c
        
        # no intersection case (most likely)
        if (det<0):
            return False

        t = [(-b + det**0.5) / (2*a), (-b - det**0.5) / (2*a)]
        t.sort()
        # now t[0] is the smaller one

        # check the cases
        # if t between 0 and 1 (circle intersects segment)
        # or one t greater than 1 and one less than 0 (circle encompasses segment)
        if (0 <= t[0] <= 1):
            return True
        if ((0 <= t[1] <= 1)):
            return True
        if (t[0] < 0 and t[1] > 1):
            return True

        return False
        

# class for walls on the field
class wall(Line):
    def __init__(self, startPoint, endPoint):
        # start and endpoints are tuples (x,y)
        self.start = startPoint
        self.end = endPoint
        Line.__init__(self,
                      Point(startPoint[0],startPoint[1]),
                      Point(endPoint[0],endPoint[1]))
        
                      

    #draw the wall on the field

class yellowWall(wall):
    def __init__(self, startPoint, endPoint):
        wall.__init__(self, startPoint, endPoint)
        self.setOutline("yellow")

# class for balls on the field
class ball:
    # ball with a color and initial position
    def __init__(self, pos, color):
        self.pos = pos
        self.color = color
        
        self.radius = 5
        self.vel = (0,0)

        self.body = Circle(Point(self.pos[0],self.pos[1]), self.radius)
        self.body.setFill(color)
        
    def draw(self, canvas):
        self.body.draw(canvas)
        



