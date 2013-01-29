import cv, time

if __name__ == "__main__":
	cam = cv.CaptureFromCAM(1)
	objects = ["green ball", "red ball", "yellow wall", "blue wall"] 
	filenames = ["HSV_GREEN_BALL.data", "HSV_RED_BALL.data", "HSV_WALL_YELLOW.data", "HSV_WALL_BLUE.data"]
	i = 0
	while i < 4:
		print "Calibrating the ", objects[i]
		while True:
			feed = cv.QueryFrame(cam)
			proportion = feed.width/640
			feed_resize = cv.CreateImage((feed.width/proportion, feed.height/proportion), cv.IPL_DEPTH_8U, 3)
			cv.Resize(feed, feed_resize)
			feed_hsv = cv.CreateImage((640, feed_resize.height), cv.IPL_DEPTH_8U, 3)
			cv.CvtColor(feed_resize, feed_hsv, cv.CV_RGB2HSV);
			cv.Circle(feed_resize, (feed_resize.width/2, feed_resize.height/2), 40, cv.Scalar(100,200,200),  3)
			#point = cv.Get2D(feed_hsv,  feed_resize.height/2, feed_resize.width/2)
			#print "H: ", point[0], "\tS:", point[1], "\tV:", point[2]
			cv.NamedWindow("image")
			cv.ShowImage("image",feed_resize)
			keyPressed = cv.WaitKey(100)
			if keyPressed != -1:
				print "Saving Values..."
				counter = 0
				points = []
				while (counter < 80):
					points.append(cv.Get2D(feed_hsv, (feed_resize.height/2) - 40 + counter, feed_resize.width/2))
					counter += 1
				MaxH = 0
				MinH = 1000
				MaxS = 0
				MinS = 1000
				MaxV = 0
				MinV = 1000
				for point in points:
					if (point[0] > MaxH):
						MaxH = point[0]
					elif (point[0] < MinH):
						MinH = point[0]
					else:
						pass
					if (point[1] > MaxS):
						MaxS = point[1]
					elif (point[1] < MinS):
						MinS = point[1]
					else:
						pass
					if (point[2] > MaxV):
						MaxV = point[2]
					elif (point[2] < MinV):
						MinV = point[2]
					else:
						pass
				f = open(filenames[i], 'w')
				f.write(str(MinH)+"\n")
				f.write(str(MinS)+"\n")
				f.write("50.0\n") #V wider threshold
				f.write(str(MaxH)+"\n")
				f.write(str(MaxS)+"\n")
				f.write("200.0\n") #V wider threshold
				f.close()
				print "Done!"
				break
		i+=1
	print "Calibration Complete ;)"
