import cv, time

if __name__ == "__main__":
	cam = cv.CaptureFromCAM(-1)
	while True:
		feed = cv.QueryFrame(cam)
		feed_hsv = cv.CreateImage((feed.width, feed.height), cv.IPL_DEPTH_8U, 3)
		cv.CvtColor(feed, feed_hsv, cv.CV_RGB2HSV);
		cv.Circle(feed, (feed.width/2, feed.height/2), 40, cv.Scalar(100,200,200),  3)
		point = cv.Get2D(feed_hsv,  feed.height/2, feed.width/2)
		print "H: ", point[0], "\tS:", point[1], "\tV:", point[2]
		time.sleep(0.5)
		cv.SaveImage("feed.jpg", feed)
