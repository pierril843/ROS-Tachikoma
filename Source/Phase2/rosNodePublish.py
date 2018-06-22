import rospy
import cv2
import waypoints

user = "pierril"
MaxCoords = 8

if __name__ == '__main__':

	patrolPath = []
	convertedWaypoints = []
	scaledWaypoints = []
	TotalCoords = 0
	patrol = waypoints.WaypointsClass()

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
	crop, thresh = patrol.CropImage(dst)

	#find corner coords
	topRightCoord = patrol.TopRightWaypoint(crop)
	if (topRightCoord != -1):
		patrolPath.append(topRightCoord)
		TotalCoords += 1
	else:
		print "[E] - Top Right Coord Not Added"

	bottomRightCoord = patrol.BottomRightWaypoint(crop)
	if (bottomRightCoord != -1):
	 	patrolPath.append(bottomRightCoord)
		TotalCoords += 1
	else:
		print "[E] - Bottom Right Coord Not Added"

	bottomLeftCoord = patrol.BottomLeftWaypoint(crop)
	if (bottomLeftCoord != -1):
		patrolPath.append(bottomLeftCoord)
		TotalCoords += 1
	else:
		print "[E] - Bottom Left Coord Not Added"

	topLeftCoord = patrol.TopLeftWaypoint(crop)
	if (topLeftCoord != -1):
		patrolPath.append(topLeftCoord)
		TotalCoords += 1
	else:
		print "[E] - Top Left Coord Not Added"

	#find midpoint coords
	if (topRightCoord != -1 and bottomRightCoord != -1):
		rightCoord = patrol.FindRightWaypoints((bottomRightCoord), (topRightCoord), crop)
		if (rightCoord != -1):
			patrolPath[1:1] = [rightCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Right Coord Midpoint Not Added"
	else:
		print "[E] - Right Midpoints Coord Not Calculated, Dependency Missing"

	if (bottomLeftCoord != -1 and bottomRightCoord != -1):
		bottomCoord = patrol.FindBottomWaypoints((bottomLeftCoord), (bottomRightCoord), crop)
		if (bottomCoord != -1):
			patrolPath[3:3] = [bottomCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Bottom Coord Midpoint Not Added"
	else:
		print "[E] - Bottom Midpoints Coord Not Calculated, Dependency Missing"

	if (bottomLeftCoord != -1 and topLeftCoord != -1):
		leftCoord = patrol.FindLeftWaypoints((bottomLeftCoord), (topLeftCoord), crop)
		if (leftCoord != -1):
			patrolPath[5:5] = [leftCoord]
			TotalCoords += 1
		else:
			print "[E] - Cal Left Coord Midpoint Not Added"
	else:
		print "[E] - left Midpoints Coord Not Calculated, Dependency Missing"

	if (topRightCoord != -1 and topLeftCoord != -1):
		topCoord = patrol.FindTopWaypoints((topLeftCoord), (topRightCoord), crop)
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
	scaledWaypoints = patrol.ScaleWaypoints(dst, crop, thresh, patrolPath)

	#DisplayPatrolPath(crop, patrolPath)
	patrol.DisplayPatrolPath(dst, scaledWaypoints)
	#DisplayPatrolPath(crop, patrolPath)


	#convert waypoints from map coords to world coords
	convertedWaypoints = patrol.ConvertToWorldCoords(scaledWaypoints, dst)

	cv2.imshow('Display_Window', dst)
	cv2.waitKey(0)

	#start state machine publishing waypoints
	patrol.PatrolStateMachine(convertedWaypoints)
