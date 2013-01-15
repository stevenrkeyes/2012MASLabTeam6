import cv
import time

def findObjects(img=cv.LoadImageM("foo.jpg"), lowerHSV=cv.Scalar(45,160,120), upperHSV=cv.Scalar(60,215,220) , blurNum=21):
	stt=time.time()
	# Load example image
	# print type(img)
	#initialize images (same size and type)
	img_hsv = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 3)
	img_threshold = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1) #Just black and white pixels
	img_blur = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 3)
	# Gaussian blur and conversion of RGB to HSV
	cv.Smooth(img, img_blur, cv.CV_GAUSSIAN, blurNum, blurNum)
	cv.CvtColor(img_blur, img_hsv, cv.CV_RGB2HSV)
	#cv.SaveImage("lol3.png",img_blur)2
	# Find Red Balls
	cv.InRangeS(img_hsv, lowerHSV, upperHSV, img_threshold)
	# Detect Contors, create list
	first_contour = cv.FindContours(img_threshold, cv.CreateMemStorage())
	# cv.DrawContours(img_threshold, first_contour, cv.Scalar(180,255,255), cv.Scalar(100,200,200), 1, 5)
	if len(first_contour) == 0:
		return ()
	contours = [first_contour]
	next = first_contour.h_next()
	while next:
		contours.append(next)
		next = next.h_next()

	# print len(contours)
	# Sort out "important" contours that are big enough
	important_contours = []
	for contour in contours:
		if len(contour) > cv.GetSize(img)[0]/7: # size of contour depends on size of image
			important_contours.append(contour)
	if important_contours == []:
		return ()
	# Test Code
	# print sorted(list(important_contours))
	# print sorted(list(important_contours[0]))
	# testimg = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 3)
	# cv.DrawContours(testimg, important_contours[0], cv.Scalar(180,255,255), cv.Scalar(100,200,200), 1, 5)

	# Inscribing contour in rectangle, 
	rectangles = []	
	for important_contour in important_contours:
		rectangles.append(cv.BoundingRect(important_contour))
	output = []
	for rectangle in rectangles:
		output.append((rectangle[0]+ (rectangle[2]/2), rectangle[1] + (rectangle[3]/2), rectangle[2], rectangle[3]))
	return output

	# for debugging
	# cv.SaveImage("lol2.png", img_threshold)
	# cv.SaveImage("lol5.png", testimg)
	# print "Running time: "+str(time.time()-stt)


if __name__ == "__main__":
	print list(findObjects())
