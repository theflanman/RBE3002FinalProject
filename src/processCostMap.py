import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import MarkerArray, Marker
from AMap import AMap
import math

global costMap
global currentPosition

costMap = 0
currentPosition = 0

#subs, pubs, and service functions
def mapCallBack(msg):
	global costMap
	costMap = msg
	return msg

def odomCallBack(msg):
	global currentPosition
	currentPosition = [msg.pose.pose.position.x,msg.pose.pose.position.y]
	return msg

def publishCells(array, mapMapMap, pub):
	frontierCells = GridCells()
	cells = []

	frontierCells.cell_width = mapMapMap.info.resolution
	frontierCells.cell_height = mapMapMap.info.resolution
	frontierCells.header.frame_id = 'map'

	for a in array:
		point = Point()
		point.x = (a[1] + 0.5)*mapMapMap.info.resolution + mapMapMap.info.origin.position.x
		point.y = (a[0] + 0.5)*mapMapMap.info.resolution + mapMapMap.info.origin.position.y
		cells.append(point)

	frontierCells.cells = cells

	pub.publish(frontierCells)

	return

def publishMarkers(array, mapMapMap, pub):

	markers = MarkerArray()

	print markers

	i = 0

	for a in array:

		m = Marker()

		m.id = i
		m.type = 3
		m.header.frame_id = 'map'
		m.pose.position.x = (a[1] + 0.5)*mapMapMap.info.resolution + mapMapMap.info.origin.position.x
		m.pose.position.y = (a[0] + 0.5)*mapMapMap.info.resolution + mapMapMap.info.origin.position.y
		m.pose.position.z = math.sqrt(a[2])*mapMapMap.info.resolution/2
		m.pose.orientation.w = 1
		m.scale.x = math.sqrt(a[2])*mapMapMap.info.resolution
		m.scale.y = m.scale.x
		m.scale.z = m.scale.x
		m.color.r = 1
		m.color.a = 1

		print markers

		markers.markers.append(m)

		i += 1

	pub.publish(markers)

	return

def publishGoal(coordCoM, coordArr, mapMapMap, pub):

	goal = PoseStamped()
	res = mapMapMap.info.resolution
	origin = [mapMapMap.info.origin.position.y, mapMapMap.info.origin.position.x]
	goalPos = [coordCoM[0]*res + origin[0], coordCoM[1]*res + origin[1]]

	goal.header.frame_id = 'map'

	goal.pose.position.y = goalPos[0]
	goal.pose.position.x = goalPos[1]

	pub.publish(goal)

	return

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
					frontCoords.append([int(i),int(j)])

	frontCoords.pop(0)

	return frontCoords

#return true if an adjacent cell is unexplored
def checkAdjUn(costMap, coord):
	for i in range(3):
		for j in range(3):
			if (coord[0] + i - 1 >= 0) and (coord[0] + i - 1 < len(costMap)) and (coord[1] + j - 1 >= 0) and (coord[1] + j - 1 < len(costMap[0])):
				if costMap[coord[0]+i-1][coord[1]+j-1] == -1:
					return True
	return False

#eliminate coordinates which cannot be navigated to
def eliminateUndesireables(frontierCoords, currentPosition, costMap):
	navMap = AMap(costMap)
	navigableNodes = []

	for n in frontierCoords:
		#print "current position is: ", type(currentPosition[0]), currentPosition[0], type(currentPosition[1]), currentPosition[1]
		#print "checking naviation to: ", n
		if navMap.navigate(currentPosition, n, costMap) is not False:
			if navigableNodes is []:
				navigableNodes = [n]
			else:
				navigableNodes.append(n)

	return navigableNodes

#takes a graph in the form of a list of coordinates and returns the connected component subgraphs as a list of list of coordinates
def connectedComponents(coordList):
	comps = [[[]]]

	#pull the first entry out of the coordinates, and insert it into a new list
	while len(coordList) > 0:
		subGraph = [[]]

		#for each other node in coordList, if that node is adjacent to any entry,
		#remove it and put it in subGraph, repeat for all entries in subGraph until no nodes in SubGraph are adjacent to nodes in coordList
		subGraph.append(coordList.pop(0))

		subGraph.pop(0)

		growth = 1
		while growth > 0:

			growth = 0

			for n in subGraph:

				for m in coordList:

					#print m, n, n[0] - m[0], n[1] - m[1], (n[0] - m[0])**2, (n[1] - m[1])**2, (n[0] - m[0])**2 + (n[1] - m[1])**2, math.sqrt((n[0] - m[0])**2 + (n[1] - m[1])**2)

					if math.sqrt((n[0] - m[0])**2 + (n[1] - m[1])**2) < 1.5:

						subGraph.append(m)
						coordList.remove(m)
						growth += 1

		#print subGraph

		comps.append(subGraph)

	return comps

#takes a list of lists of coordinates and returns the centers of mass and masses of the lists
def centersOfMass(subGraphs):
	comCoords = [[]]

	#print subGraphs

	subGraphs.pop(0)

	#print subGraphs

	for g in subGraphs:

		xCoord = 0
		yCoord = 0

		for n in g:
			xCoord += n[0]
			yCoord += n[1]

		xCoord /= len(g)
		yCoord /= len(g)

		comCoords.append([xCoord, yCoord, len(g)])

	comCoords.pop(0)
	return comCoords

#converts a global coordinate into the coordiante of a cell on the map
def cellAt(costMap, currentPosition):

	currentPosition[0] = int((currentPosition[0] + costMap.info.origin.position.y)/costMap.info.resolution)
	currentPosition[1] = int((currentPosition[1] + costMap.info.origin.position.x)/costMap.info.resolution)
	
	return currentPosition



if __name__ == '__main__':
	rospy.init_node('Korra_the_Explora')

	global costMap
	global currentPosition

	costMapArray = [[]]
	costMapAdapt = [[]]
	frontierCoords = [[]]
	frontiers = [[[]]]
	frontierCoMs = [[]]
	closeFrontier = []
	threshold = 50
	bestDist = -1
	
	#create subscribers and publishers and services
	mapSub = rospy.Subscriber(rospy.get_param('/map_input_topic','/map'),OccupancyGrid, mapCallBack, queue_size=1)
	odomSub = rospy.Subscriber('/odom', Odometry, odomCallBack, queue_size=1)
	frontierPub = rospy.Publisher('/frontier', GridCells, queue_size=1)
	markerPub = rospy.Publisher('/frontier_CoMs', MarkerArray, queue_size=1)
	goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

	while costMap is 0 or currentPosition is 0:
		if costMap is 0 and currentPosition is 0:
			print "No Odometry or Map found, sleeping"
		elif costMap is 0:
			print "No map found, sleeping"
		else:
			print "No Odometry found, sleeping"
		rospy.sleep(0.1)

	print "Program Start"

	while(True):

		currentPosition = cellAt(costMap, currentPosition)

		#pull data out of map subscription and place into 2d array
		costMapArray = mapToArray(costMap)

		#convert all values to -1, 0, or 100.  100 is any value over the threshold, 0 is any value under, and -1 remains -1
		costMapAdapt = adaptMapt(costMapArray, threshold)

		#create a list of coordinates of frontier nodes
		frontierCoords = calcFrontier(costMapAdapt)

		#for each frontier node, run A* from the current position of the robot to that node, remove it if no path can be found
		#frontierCoords = eliminateUndesireables(frontierCoords, currentPosition, costMapAdapt)

		#publish a GridCells of the frontier nodes
		publishCells(frontierCoords, costMap, frontierPub)

		#create a list of frontiers, the list of frontiers is the list of connected components of the frontier nodes
		frontiers = connectedComponents(frontierCoords)

		#calculate the center of mass of each frontier
		frontierCoMs = centersOfMass(frontiers)

		#find the nearest frontier
		for n in frontierCoMs:
			if bestDist is -1 or bestDist > math.sqrt((n[0] - currentPosition[0])**2 + (n[1] - currentPosition[1])**2):
				closeFrontier = n
				bestDist = math.sqrt((n[0] - currentPosition[0])**2 + (n[1] - currentPosition[1])**2)

		closeFrontierCoords = frontiers[frontierCoMs.index(closeFrontier)]

		#publish whatever it is that displays blocks in RViz for each CoM, with the size of the frontier driving the size of the block,
		#and the distance of the block from the current position driving the color
		publishMarkers(frontierCoMs, costMap, markerPub)

		#publish a message to stop the current navigation and another to navigate to the center of mass of the nearest frontier
		publishGoal(closeFrontier, closeFrontierCoords, costMap, goalPub)