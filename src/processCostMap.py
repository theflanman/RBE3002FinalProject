

#subs, pubs, and service functionsdef mapCallBack(msg):
	#while (msg == OccupancyGrid()):
		#rospy.sleep(0.01)
	global worldMap
	worldMap = msg
	return msg

#turn a map into an array
def mapToArray(mapMsg):

	data = list(mapMsg.data)
	width = mapMsg.info.width
	height = mapMsg.info.height


	array = [0]*width
	array = [array]*height

	i = 0
	while (i < height):

		#print i

		array[i] = data[i*width:(i+1)*width]

		i += 1

	#print (len(array)), len(array[0]

	return array

#adapt a costMap to C-Space
def adaptMapt(costArray, threshold):

	adaptedMap = costArray

	for i in range(len(adaptedMap)):
		for j in range(len(adaptedMap[0])):
			if adaptedMap[i][j] != -1:
				if adaptedMap[i][j] > threshold:
					adaptedMap[i][j] = 100
				else:
					adaptedMap[i][j] = 0

	return adaptedMap

#create a list of coordinates which represent the nodes on the frontier
def calcFrontier(costMap):
	frontCoords = [[]]

	for i in range(len(costMap)):
		for j in range(len(costMap[0])):
			if costMap[i][j] == 0:
				if checkAdjUn(costMap, [i,j]):
					frontCoords.append[i,j]

	return frontCoords

#return true if an adjacent cell is unexplored
def checkAdjUn(costMap, coord):
	for i in range(3):
		for j in range(3):
			if (coord[0] + i - 1 >= 0) and (coord[0] + i - 1 < len(costMap)) and (coord[1] + j - 1 >= 0) and (coord[1] + j - 1 < len(costMap[0])):
				if costMap[coord[0]+i-1][coord[1]+j-1] == -1:
					return true
	return false

#eliminate coordinates which cannot be navigated to
def eliminateUndesireables(frontierCoords, currentPosition, costMap):
	navMap = AMap(costMap)

	#TODO: WRITE THIS FUNCTION

	return frontierCoords



if __name__ == '__main__':

	costMapArray = [[]]
	costMapAdapt = [[]]
	frontierCoords = [[]]
	threshold = 127
	
	#create subscribers and publishers and services
	mapSub = rospy.Subscriber(rospy.get_param('/map_input_topic','/map'),OccupancyGrid, mapCallBack, queue_size=1)
	odomSub = rospy.Subscriber('/odom', Odometry, odomCallBack, queue_size=1)

	#pull data out of map subscription and place into 2d array
	costMapArray = mapToArray(costMap)

	#convert all values to -1, 0, or 100.  100 is any value over the threshold, 0 is any value under, and -1 remains -1
	costMapAdapt = adaptMapt(costMapArray, threshold)

	#create a list of coordinates of frontier nodes
	frontierCoords = calcFrontier(costMapAdapt)

	#for each frontier node, run A* from the current position of the robot to that node, remove it if no path can be found
	frontierCoords = eliminateUndesireables(frontierCoords, currentPosition, costMap)

	#create a list of frontiers, the list of frontiers is the list of connected components of the frontier nodes

	#calculate the center of mass of each frontier

	#publish a message to stop the current navigation and another to navigate to the center of mass of the nearest frontier