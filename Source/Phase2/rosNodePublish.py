import cv2
import numpy as np
import re
import math
import operator
import rospy
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

user = "pierril"
lastBoxPosition = -1
MaxCoords = 8 

def ValidWaypointCheck((x,y),image):
	leftXLimit = (y - 50)  # Box vertice based on given waypoint
	rightXLimit = (y + 50)  # Box vertice based on given waypoint
	topYLimit = (x - 50)  # Box vertice based on given waypoint
	bottomYLimit = (x + 50)  # Box vertice based on given waypoint
	totalPixelValue = 0

	for j in range((leftXLimit), (rightXLimit), 1):
		for i in range((bottomYLimit), (topYLimit), -1):  # loop over all columns
			totalPixelValue += image[j, i]  # grayscale image all pixel values are the same
			#image[j, i] = 0 #Debug tool to see what pixels are being checked

	averagePixelValue = (totalPixelValue / 10000)
	if (averagePixelValue < 254): #needs to be tested
		#print ("Invalid Waypoint")
		return (1)
	#print  ("Valid Waypoint Found")
	return (0)

def TopRightWaypoint(image):

	distanceFromCenter = []
	validTopRightWaypoints = []
	imHeight, imWidth = image.shape
	index = -1

	#Loop all pixels in top right quadrant of image, save locations of valid waypoints
	for i in range(0,(imHeight/2 - 50),100):
		for j in range(imWidth/2,(imWidth - 50),100):
			#cv2.rectangle(image, (j - 50, i - 50), (j + 50, i + 50), 128, 1)
			if (ValidWaypointCheck((j,i), image) == 0):
				validTopRightWaypoints.append((j, i))

	#calculate distances from valid points to center
	for x in range(0, len(validTopRightWaypoints), 1):
		distanceFromCenter.append(math.sqrt(((imWidth/2 - validTopRightWaypoints[x][0]) ** 2) + ((imHeight/2 - validTopRightWaypoints[x][1]) ** 2)))  # calculate distance between waypoints

		# Find greatest distance in list
		if (distanceFromCenter):
			index, value = max(enumerate(distanceFromCenter), key=operator.itemgetter(1))

	if (index == -1):
		return(-1)
	return (validTopRightWaypoints[index])

def BottomRightWaypoint(image):

	distanceFromCenter = []
	validBottomRightWaypoints = []
	imHeight, imWidth = image.shape
	index = -1

	# Loop all pixels in top right quadrant of image, save locations of valid waypoints
	for i in range(imHeight/2, (imHeight- 50), 100):
		for j in range(imWidth / 2, (imWidth - 50), 100):
			#cv2.rectangle(image, (j - 50, i - 50), (j + 50, i + 50), 128, 1)
			if (ValidWaypointCheck((j, i), image) == 0):
				validBottomRightWaypoints.append((j, i))

				# calculate distances from valid points to center
	for x in range(0, len(validBottomRightWaypoints), 1):
		distanceFromCenter.append(math.sqrt(((imWidth / 2 - validBottomRightWaypoints[x][0]) ** 2) + ((imHeight / 2 - validBottomRightWaypoints[x][1]) ** 2)))  # calculate distance between waypoints

		# Find greatest distance in list
		if (distanceFromCenter):
			index, value = max(enumerate(distanceFromCenter), key=operator.itemgetter(1))

	if (index == -1):
		return (-1)
	return (validBottomRightWaypoints[index])

def BottomLeftWaypoint(image):

	distanceFromCenter = []
	validBottomLeftWaypoints = []
	imHeight, imWidth = image.shape
	index = -1

	# Loop all pixels in top right quadrant of image, save locations of valid waypoints
	for i in range(imHeight / 2, (imHeight - 50), 100):
		for j in range(0, (imWidth/2 - 50), 80):
			# cv2.rectangle(image, (j - 40, i - 40), (j + 40, i + 40), 128, 1)
			if (ValidWaypointCheck((j, i), image) == 0):
				validBottomLeftWaypoints.append((j, i))

			# calculate distances from valid points to center
	for x in range(0, len(validBottomLeftWaypoints), 1):
		distanceFromCenter.append(math.sqrt(((imWidth / 2 - validBottomLeftWaypoints[x][0]) ** 2) + ((imHeight / 2 - validBottomLeftWaypoints[x][1]) ** 2)))  # calculate distance between waypoints

		# Find greatest distance in list
		if (distanceFromCenter):
			index, value = max(enumerate(distanceFromCenter), key=operator.itemgetter(1))

	if (index == -1):
		return (-1)
	return (validBottomLeftWaypoints[index])

def TopLeftWaypoint(image):

	distanceFromCenter = []
	validTopLeftWaypoints = []
	imHeight, imWidth = image.shape
	index = -1

	# Loop all pixels in top right quadrant of image, save locations of valid waypoints
	for i in range(0, (imHeight/2 - 50), 100):
		for j in range(0, (imWidth/2 - 50), 100):
			#cv2.rectangle(image, (j - 40, i - 40), (j + 40, i + 40), 128, 1)
			if (ValidWaypointCheck((j, i), image) == 0):
				validTopLeftWaypoints.append((j, i))

			# calculate distances from valid points to center
	for x in range(0, len(validTopLeftWaypoints), 1):
		distanceFromCenter.append(math.sqrt(((imWidth / 2 - validTopLeftWaypoints[x][0]) ** 2) + ((imHeight / 2 - validTopLeftWaypoints[x][1]) ** 2)))  # calculate distance between waypoints

	# Find greatest distance in list
	if (distanceFromCenter):
		index, value = max(enumerate(distanceFromCenter), key=operator.itemgetter(1))

	if (index == -1):
		return (-1)
	return (validTopLeftWaypoints[index])

def FindTopWaypoints((topLeftCoord),(topRightCoord),image):

	imHeight, imWidth = image.shape
	validTopWaypoints = []
	topMapWidth = ((topRightCoord[0] + topLeftCoord[0]) / 2)
	greatestYValue = 0
	xCoord = topMapWidth
	#yOffset = ((topRightCoord[1]) / (topRightCoord[0] * slopeOfLine))

	for y in range(0, imHeight/2 - 50, 100):
		#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
		if (ValidWaypointCheck((xCoord, y),image) == 0):
			validTopWaypoints.append((xCoord, y))
			#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
	#cv2.rectangle(image, (validTopWaypoints[0][0] - 40, validTopWaypoints[0][1] - 40), (validTopWaypoints[0][0] + 40, validTopWaypoints[0][1] + 40), 128, 1)
	if (validTopWaypoints):
		return(validTopWaypoints[0])
	return(-1)

def FindBottomWaypoints((bottomLeftCoord),(bottomRightCoord),image):

	imHeight, imWidth = image.shape
	validBottomWaypoints = []
	bottomMapWidth = ((bottomRightCoord[0] + bottomLeftCoord[0]) / 2)
	xCoord = bottomMapWidth

	for y in range(imHeight/2, imHeight - 50, 100):
		#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
		if (ValidWaypointCheck((xCoord, y),image) == 0):
			validBottomWaypoints.append((xCoord, y))
			#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
	#cv2.rectangle(image, (validBottomWaypoints[-1][0] - 40, validBottomWaypoints[-1][1] - 40), (validBottomWaypoints[-1][0] + 40, validBottomWaypoints[-1][1] + 40), 128, 1)
	if (validBottomWaypoints):
		return(validBottomWaypoints[-1])
	return(-1)

def FindRightWaypoints((bottomRightCoord),(topRightCoord),image):

	imHeight, imWidth = image.shape
	validRightWaypoints = []
	bottomMapWidth = ((bottomRightCoord[1] + topRightCoord[1]) / 2)
	yCoord = bottomMapWidth

	for x in range(imWidth/2, imWidth - 50, 100):
		#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
		if (ValidWaypointCheck((x, yCoord),image) == 0):
			validRightWaypoints.append((x, yCoord))
			#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
	#cv2.rectangle(image, (validRightWaypoints[-1][0] - 40, validRightWaypoints[-1][1] - 40), (validRightWaypoints[-1][0] + 40, validRightWaypoints[-1][1] + 40), 128, 1)
	if (validRightWaypoints):
		return(validRightWaypoints[-1])
	return (-1)

def FindLeftWaypoints((bottomLeftCoord),(topLeftCoord),image):

	imHeight, imWidth = image.shape
	validLeftWaypoints = []
	bottomMapWidth = ((bottomLeftCoord[1] + topLeftCoord[1]) / 2)
	yCoord = bottomMapWidth

	for x in range(0, imWidth/2 - 50, 100):
		#cv2.rectangle(image, (x - 40, yCoord - 40), (x + 40, yCoord + 40),128, 1)
		if (ValidWaypointCheck((x, yCoord),image) == 0):
			validLeftWaypoints.append((x, yCoord))
			#cv2.rectangle(image, (xCoord - 40, y - 40), (xCoord + 40, y + 40),128, 1)
	#cv2.rectangle(image, (validLeftWaypoints[0][0] - 40, validLeftWaypoints[0][1] - 40), (validLeftWaypoints[0][0] + 40, validLeftWaypoints[0][1] + 40), 128, 1)
	if (validLeftWaypoints):
		return(validLeftWaypoints[0])
	return(-1)

def DisplayPatrolPath(image, preferredWaypoints):

	for i in range(0,len(preferredWaypoints),1):
		leftXLimit = (preferredWaypoints[i][0]- 50)  # Box vertice based on given waypoint
		rightXLimit = (preferredWaypoints[i][0] + 50)  # Box vertice based on given waypoint
		topYLimit = (preferredWaypoints[i][1] - 50)  # Box vertice based on given waypoint
		bottomYLimit = (preferredWaypoints[i][1] + 50)  # Box vertice based on given waypoint
		cv2.rectangle(image,(leftXLimit,topYLimit),(rightXLimit,bottomYLimit),0,1)

def ConvertToWorldCoords(listOfWaypoints, image):
	imHeight, imWidth = image.shape
	convertedWaypoints = []

	#currently hard coded but works well for test maps as they are all similar resolution
	for i in range(0, len(listOfWaypoints), 1):
		convertedWaypoints.append(((listOfWaypoints[i][0] * 0.0195) - 10, (listOfWaypoints[i][1] * 0.0195) - 20))
		#convertedWaypoints.append(((listOfWaypoints[i][0] * (imWidth/30)) - 10, (listOfWaypoints[i][1] * (imWidth/30)) - 20))

	return (convertedWaypoints)

def PatrolStateMachine(listOfWaypoints):
	tempWaypoint = ()
	waypoints = []
	#convert waypoints to ros topic format
	#waypoints.append([str(0), (listOfWaypoints[-1][1],listOfWaypoints[-1][0]), (0.0, 0.0, 0.0, 1.0)])
	for i in range(0, len(listOfWaypoints), 1):
		tempWaypoint = (listOfWaypoints[i][1],listOfWaypoints[i][0])
		waypoints.append([str(i), tempWaypoint, (0.0, 0.0, 0.0, 1.0)])

	#start state machine
	patrol = StateMachine(['succeeded', 'aborted', 'preempted'])
	with patrol:
		for i, w in enumerate(waypoints):
			goal_pose = MoveBaseGoal()
			goal_pose.target_pose.header.frame_id = 'map'

			goal_pose.target_pose.pose.position.x = w[1][0]
			goal_pose.target_pose.pose.position.y = w[1][1]
			goal_pose.target_pose.pose.position.z = 0.0

			goal_pose.target_pose.pose.orientation.x = w[2][0]
			goal_pose.target_pose.pose.orientation.y = w[2][1]
			goal_pose.target_pose.pose.orientation.z = w[2][2]
			goal_pose.target_pose.pose.orientation.w = w[2][3]

			StateMachine.add(w[0],
							 SimpleActionState('move_base',
											   MoveBaseAction,
											   goal=goal_pose),
							 transitions={'succeeded': waypoints[(i + 1) % \
																 len(waypoints)][0]})
	patrol.execute()

def CropImage(img,tol=0):

	#any pixel with a value greater than 253 become pure white
	_, thresh = cv2.threshold(img, 253, 255, cv2.THRESH_BINARY)

	#find pixels that have a non zero value
	mask = thresh>tol

	#return cropped image
	return img[np.ix_(mask.any(1),mask.any(0))], thresh

def ScaleWaypoints(dst,crop, thresh, patrolPath):
	minXDistance = 10000  # arbitrary
	maxYDistance = 10000
	dstHeight, dstWidth = dst.shape
	imgHeight, imgWidth = crop.shape
	scaledWaypoints = []

	#detect edges in image
	edges = cv2.Canny(thresh, 0, 255)
	ans = []
	for y in range(0, edges.shape[0]):
		for x in range(0, edges.shape[1]):
			if edges[y, x] != 0:
				ans = ans + [[x, y]]
	ans = np.array(ans)

	for i in range(0,len(ans),1):
		# find x coord closest to right edge of the image
		if (dstWidth - ans[i][0]) < minXDistance:
			minXDistance = dstWidth - ans[i][0]
			#print "xContour: %d",ans[i][0]

		# find y coord closest to bottom edge of the image
		if (dstHeight - ans[i][1]) < maxYDistance:
			maxYDistance = dstHeight - ans[i][1]
			#print "yContour: %d",ans[i][1]

	#use calculated distances to determine distance to scale waypoints
	for i in range(0, len(patrolPath), 1):
		scaledWaypoints.append((patrolPath[i][0] + (dstWidth - (minXDistance + imgWidth)), patrolPath[i][1] + (dstHeight - (maxYDistance + imgHeight)) )) #+20 is a current test

	return scaledWaypoints

if __name__ == '__main__':

	patrolPath = []
	convertedWaypoints = []
	scaledWaypoints = []
	TotalCoords = 0

	rospy.init_node('patrol')

	cv2.namedWindow('Display_Window', cv2.WND_PROP_FULLSCREEN)
    	cv2.setWindowProperty('Display_Window', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_NORMAL)

	myMap = cv2.imread("/home/"+ user +"/GeneratedMaps/map5.pgm", cv2.IMREAD_GRAYSCALE)  # open map image

	#Rotate map
	print "[rotating map]"
	rows, cols = myMap.shape
	M = cv2.getRotationMatrix2D((cols / 2, rows / 2), 270, 1)
	dst = cv2.warpAffine(myMap, M, (cols, rows))

	#crop map to isolate portion that is valid for pathing
	print "[croping map]"
	crop, thresh = CropImage(dst)

	#find corner coords
	topRightCoord = TopRightWaypoint(crop)
	if (topRightCoord != -1):
		patrolPath.append(topRightCoord)
		TotalCoords += 1
	else:
		print "[E] - Top Right Coord Not Added"

	bottomRightCoord = BottomRightWaypoint(crop)
	if (bottomRightCoord != -1):
	 	patrolPath.append(bottomRightCoord)
		TotalCoords += 1
	else:
		print "[E] - Bottom Right Coord Not Added"

	bottomLeftCoord = BottomLeftWaypoint(crop)
	if (bottomLeftCoord != -1):
		patrolPath.append(bottomLeftCoord)
		TotalCoords += 1
	else:
		print "[E] - Bottom Left Coord Not Added"

	topLeftCoord = TopLeftWaypoint(crop)
	if (topLeftCoord != -1):
		patrolPath.append(topLeftCoord)
		TotalCoords += 1
	else:
		print "[E] - Top Left Coord Not Added"

	#find midpoint coords
	if (topRightCoord != -1 and bottomRightCoord != -1):
		rightCoord = FindRightWaypoints((bottomRightCoord), (topRightCoord), crop)
		if (rightCoord != -1):
			patrolPath[1:1] = [rightCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Right Coord Midpoint Not Added"
	else:
		print "[E] - Right Midpoints Coord Not Calculated, Dependency Missing"

	if (bottomLeftCoord != -1 and bottomRightCoord != -1):
		bottomCoord = FindBottomWaypoints((bottomLeftCoord), (bottomRightCoord), crop)
		if (bottomCoord != -1):
			patrolPath[3:3] = [bottomCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Bottom Coord Midpoint Not Added"
	else:
		print "[E] - Bottom Midpoints Coord Not Calculated, Dependency Missing"

	if (bottomLeftCoord != -1 and topLeftCoord != -1):
		leftCoord = FindLeftWaypoints((bottomLeftCoord), (topLeftCoord), crop)
		if (leftCoord != -1):
			patrolPath[5:5] = [leftCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Left Coord Midpoint Not Added"
	else:
		print "[E] - left Midpoints Coord Not Calculated, Dependency Missing"

	if (topRightCoord != -1 and topLeftCoord != -1):
		topCoord = FindTopWaypoints((topLeftCoord), (topRightCoord), crop)
		if (topCoord != -1):
			patrolPath[7:7] = [topCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Top Coord Midpoint Not Added"
	else:
		print "[E] - Top Midpoints Coord Not Calculated, Dependency Missing"
	
	if TotalCoords < MaxCoords:
		print "[I] - patrol will only include " + str(TotalCoords) + " coords, out of " + str(MaxCoords) + " coords"
	else:
		print "[I] - patrol will include all" + str(TotalCoords) + "Coords"

	#scale waypoints from cropped coords to image coords
	scaledWaypoints = ScaleWaypoints(dst, crop, thresh, patrolPath)

	#DisplayPatrolPath(crop, patrolPath)
	DisplayPatrolPa<th(dst, scaledWaypoints)
	#DisplayPatrolPath(crop, patrolPath)


	#convert waypoints from map coords to world coords
	convertedWaypoints = ConvertToWorldCoords(scaledWaypoints, dst)

	cv2.imshow('Display_Window', dst)
	cv2.waitKey(0)

	#start state machine publishing waypoints
	PatrolStateMachine(convertedWaypoints)
