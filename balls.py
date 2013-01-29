import cv, rectangulate
def readBallData(isGreen):
	data = []
	f = 0
	if isGreen:
		f = open("HSV_GREEN_BALL.data", 'r')
	else:
		f = open("HSV_RED_BALL.data", 'r')
	for line in f:
		data.append(int(float(line.strip())))
	f.close()
	output=[(data[0], data[1], data[2]), (data[3], data[4], data[5])]
	return output
def followBall(cam, lowerHSV, upperHSV):
	# Capture camera frame and find balls
	feed = cam
	balls = rectangulate.findObjects(feed, lowerHSV, upperHSV)
	line25Percent = feed.width/4
	midlineHorizontal = feed.width/2
	line75Percent = line25Percent * 3

	print list(balls)
	# Find the biggest recangle --> represents closest ball or biggest set of balls
	biggestBall = ()
	maxSize = 0
	currentSize = 0
	for ball in balls:
		currentSize = ball[2]*ball[3]
		if currentSize > maxSize:
			biggestBall = ball
			maxSize = currentSize
	# Debug/Test code
	#if biggestBall == ():
	#	print "no Ball, LOL"
	#elif 0 <= biggestBall[0] < line25Percent:
	#	print "x coord: ", biggestBall[0]
	#	print "move far right"
	#elif line25Percent <= biggestBall[0] < midlineHorizontal:
	#	print "x coord: ", biggestBall[0]	
	#	print "move somewhat right"
	#elif midlineHorizontal <= biggestBall[0] < line75Percent:
	#	print "x coord: ", biggestBall[0]
	#	print "move somewhat left"
	#elif line75Percent <= biggestBall[0] < feed.width:
	#	print "x coord: ", biggestBall[0]
	#	print "move far left"
	output = biggestBall + (feed.width, feed.height)
	return output

if __name__ == "__main__":
	cam = cv.CaptureFromCAM(0)
	values = readBallData()
	while True:
		followBall(cam, values[0], values[1])
