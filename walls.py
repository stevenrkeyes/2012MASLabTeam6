import cv, rectangulate
def readWallsData():
	data = []
	yellow = open("HSV_WALL_YELLOW.data", 'r')
	for line in yellow:
		data.append(int(float(line.strip())))
	yellow.close()
	blue = open("HSV_WALL_BLUE.data", 'r')
	for line in blue:
		data.append(int(float(line.strip())))
	blue.close()
	#for d in data: # debug
	#	print d
	output=[(data[0], data[1], data[2]), (data[3], data[4], data[5]), (data[6], data[7], data[8]), (data[9], data[10], data[11])]
	return output

def findYellowWall(img, wallData):
	'''blueMin = wallData[2]
	blueMax = wallData[3]
	yellowMin = wallData[0]
	yellowMax = wallData[1]
	blueRectangles = rectangulate.findObjects(img, blueMin,blueMax)
	yellowRectangles = rectangulate.findObjects(img, yellowMin, yellowMax)
	yellowWall = []
	foundYellowWall = False
	for blueRectangle in blueRectangles:
		for yellowRectangle in yellowRectangles:
			blueBottomRight = (blueRectangle[0] + (blueRectangle[3]/2), blueRectangle[1] + (blueRectangle[2]/2))
			blueBottomLeft = (blueRectangle[0] - (blueRectangle[3]/2), blueRectangle[1] + (blueRectangle[2]/2))
			averagePoint = ((blueBottomRight[0]+blueBottomLeft[0])/2, (blueBottomRight[1]+blueBottomLeft[1])/2)
			if (yellowRectangle[3] > (averagePoint[1] - yellowRectangle[1]) > 0): #Threshold might need to be adjusted :)
				foundYellowWall = True
				yellowWall.append(blueRectangle)
				yellowWall.append(yellowRectangle)
				#cv.Rectangle(img, (blueRectangle[0] - blueRectangle[2], blueRectangle[1]-blueRectangle[3]), (blueRectangle[0] + blueRectangle[2], blueRectangle[1]+blueRectangle[3]), (200,200,200), 4)
				#cv.Rectangle(img, (yellowRectangle[0] - yellowRectangle[2], yellowRectangle[1]-yellowRectangle[3]), (yellowRectangle[0] + yellowRectangle[2], yellowRectangle[1]+yellowRectangle[3]), (200,200,200), 4)
				print "HELLO YELLOW"
				break
		if foundYellowWall:
			break
		#print list(blueRectangle)
	output = []
	if foundYellowWall:
		bRect = yellowWall[0]
		yRect = yellowWall[1]
		output += [(bRect[0] + yRect[0]) / 2 , (bRect[1] + yRect[1]) / 2, bRect[2] + yRect[2], bRect[3] + yRect[3], img.width, img.height]
	#cv.SaveImage("walls.png", img)
	return output'''
	yellowMin = wallData[0]
	yellowMax = wallData[1]
	wallList = rectangulate.findObjects(img, yellowMin, yellowMax)
	print list(wallList)
	biggestWall = ()
	maxSize = 0
	line25Percent = img.width/4
	line75Percent = line25Percent * 3
	for wall in wallList:
		currentSize = wall[2]*wall[3]
		if line25Percent < wall[0] < line75Percent:
			if currentSize > maxSize:
				biggestWall = wall
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
	output = list(biggestWall)+ [img.width,img.height]
	return output
