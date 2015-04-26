

#subs, pubs, and service functions

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
	navigableNodes = []

	for n in frontCoords:
		if navMap.navigate(currentPosition, n, costMap) is not False:
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
		growth = 1
		while growth > 0:

			growth = 0

			for n in subGraph:

				for m in coordList:

					if math.sqrt((n[0] - m[0])**2 + (n[1] - m[0])**2) < 1.5:

						subGraph.append(m)
						coordList.remove(m)
						growth += 1

		comps.append(subGraph)

	return comps

def centersOfMass(subGraphs):
	comCoords = [[]]

	for g in subGraphs:

		xCoord = 0
		yCoord = 0

		for n in g:
			xCoord += n[0]
			yCoord += n[1]

		xCoord /= len(g)
		yCoord /= len(g)

		comCoords.append[xCoord, yCoord, len(g)]

	return comCoords



if __name__ == '__main__':

	costMapArray = [[]]
	costMapAdapt = [[]]
	frontierCoords = [[]]
	frontiers = [[[]]]
	frontierCoMs = [[]]
	closeFrontier = []
	threshold = 127
	bestDist = -1
	
	#create subscribers and publishers and services

	#pull data out of map subscription and place into 2d array
	costMapArray = mapToArray(costMap)

	#convert all values to -1, 0, or 100.  100 is any value over the threshold, 0 is any value under, and -1 remains -1
	costMapAdapt = adaptMapt(costMapArray, threshold)

	#create a list of coordinates of frontier nodes
	frontierCoords = calcFrontier(costMapAdapt)

	#for each frontier node, run A* from the current position of the robot to that node, remove it if no path can be found
	frontierCoords = eliminateUndesireables(frontierCoords, currentPosition, costMapAdapt)

	#publish a GridCells of the frontier nodes

	#create a list of frontiers, the list of frontiers is the list of connected components of the frontier nodes
	frontiers = connectedComponents(frontCoords)

	#calculate the center of mass of each frontier
	frontierCoMs = centersOfMass(frontiers)

	#find the nearest frontier
	for n in frontierCoMs:
		if bestDist is -1 or bestDist > math.sqrt((n[0] - currentPosition[0])**2 + (n[1] - currentPosition[1])**2):
			closeFrontier = n
			bestDist = math.sqrt((n[0] - currentPosition[0])**2 + (n[1] - currentPosition[1])**2)

	#publish whatever it is that displays blocks in RViz for each CoM, with the size of the frontier driving the size of the block,
	#and the distance of the block from the current position driving the color

	#publish a message to stop the current navigation and another to navigate to the center of mass of the nearest frontier