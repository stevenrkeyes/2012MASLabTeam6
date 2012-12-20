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
        # request is (pos, dposRequest, heading, dheadingRequest)
        request = self.robot.requestUpdate(timePassed)

        # update the request to take the physics of the world into account
        request = self.updateForPhysics(request)

        # you should reorganize this data so it's not dumb
        self.robot.updatePos(request[1], request[3], self.canvas)
        
        self.canvas.update()

    # right now this is just checking for wall collisions
    def updateForPhysics(self, request):
        coeffFric = 1.0
        
        pos = request[0]
        dpos = request[1]
        
        collision = self.checkCollisions(pos)
        # if there's a collision, push the robot back against the wall
        # also slow its movement along the wall to account for friction
        if collision:
            pass

        return request
            
            

    def run(self):
        self.update(15)
        self.win.after(15, self.run)
        

    # check if the robot has collided with a wall, and returns that wall if so
    def checkCollisions(self, pos):
        #for wall in self.walls:
        #    if world.checkWallInCircle(wall.start[0],
        #                               wall.start[1],
        #                               wall.end[0],
        #                               wall.end[1],
        #                               pos,
        #                               self.robot.radius):
        #        return wall
        return False


    def checkWallInCircle(x_1, y_1, x_2, y_2, pos, radius):
        # I parametriezed the line segment for 0<t<1
        # and combined the x and y terms with the equation for a circle of r=radius centered at pos
        # now we solve and check if the intersection of these equations exists
        # and it if occurs with t between 0 and 1 (circle intersects segment)
        # or with one t greater than 1 and one less than 0 (circle encompasses segment)
        # t^2*((x_1 - x_2)^2 + (y_1 - y_2)^2) + t*2*((x_1 - x_2)*(x_2 - pos[0]) + (y_1 - y_2)*(y_2 - pos[1])) + (x_2 - pos[0])^2 + (y_2 - pos[1])^2 - radius^2 = 0
        # a*t^2 + b*t + c = 0
        a = ((x_1 - x_2)^2 + (y_1 - y_2)^2)
        b = 2*((x_1 - x_2)*(x_2 - pos[0]) + (y_1 - y_2)*(y_2 - pos[1]))
        c = (x_2 - pos[0])^2 + (y_2 - pos[1])^2 - radius^2
        # determinant
        det = b^2 - 4*ac

        # no intersection case (most likely)
        if (det<0):
            return False

        t = [(-b + det^0.5) / 2*a, (-b - det^0.5) / 2*a]
        t.sort()
        # now t[0] is the smaller one

        # check the cases
        # if t between 0 and 1 (circle intersects segment)
        # or one t greater than 1 and one less than 0 (circle encompasses segment)
        if (0 <= t[0] <= 1) or ((0 <= t[1] <= 1)) or (t[0] < 0 and t[1] > 1):
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
        
        


# class for simulation of robot body
class robot:
    def __init__(self, pos):
        # position (x,y)
        self.pos = (float(pos[0]),float(pos[1]));
        # angular heading from vertical
        self.heading = 0
        
        # the speed at which the robot is currently driving fwd
        self.speed = 2.0
        # the speed at which the robot is currently rotating ccw
        self.angvel = 0.001*math.pi/2.0;

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
        
        dpos = (self.speed*math.sin(self.heading),
                self.speed*math.cos(self.heading))
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

    

# class for balls on the field
class ball:
    def __init__(self):
        pass

    def draw(self, canvas):
        pass



w1 = wall((10,10),(490,10))
w2 = wall((490,10),(490,450))
w3 = wall((490,450),(450,490))
w4 = wall((450,490),(10,490))
w5 = wall((10,490),(10,10))

b = ball()
r = robot((200,200))
w = world((500,500),[w1,w2,w3,w4,w5],[b],r)
win = Window("Team 666 MASLab Simulation")
w.initializeDisplay(win)
w.run()
w.win.mainloop()
