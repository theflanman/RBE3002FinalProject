"""making an A* class in Python, takes an array as an argument.
    Constructs a list of nodes for each entry in the array.
    Each node contains a list of edges to adjacent nodes and the cost to traverse that edge.
    Methods return lists of 2D coordinates."""

import math

debug = True

class AMap:
	size = [0,0]
	mapArray = []
	nodesArray = []
	openSet = []
	closedSet = []
	thing = []

	"""constructor"""
	def __init__(self, mapArray):

		self.mapArray = mapArray
		self.nodesArray = mapArray
		self.size = [len(mapArray),len(mapArray[0])]

		y = 0

		#print self.mapArray

		for f in mapArray:
			#print f
			
			x = 0
			
			for g in f:
				#print g
				#print mapArray[x][y]
				
				self.nodesArray[x][y] = ANode(mapArray[x][y], [x,y])
				
				x += 1

			y += 1

		self.generateEdges()

		self.mapArray = mapArray

		return

	"""for each node, add all nodes which are diagonally or orthagonally adjacent, and are not occupied, to the list of edges"""
	def generateEdges(self):

		count = 0

		#traverse the list of lists of nodes on each row
		y = 0
		while y < self.size[1]:
			#traverse the list of nodes for each row
			x = 0
			while x < self.size[0]:
				#check each adjacent node
				edges = []
				j = y - 1
				while j <= y + 1:
					i = x - 1
					while i <= x + 1:
						#if it's within range
						if((i > 0) and (j > 0) and (i < self.size[0]) and (j < self.size[1])):
							#print "valid Range"
							if (self.nodesArray[i][j] != 100):
								#if it's value is not 100, add it to the list of edges
								if (i + 3*j != 0):
									#print " not self"
									if (self.nodesArray[i][j].value != 100):
										#print "  adjacent node is valid"
										if (self.nodesArray[x][y].value != 100):
											edges.append(self.nodesArray[i][j])
											#print x,y,i,j
											#print edges
											self.nodesArray[x][y].edges = edges
											#print self.nodesArray[x][y].edges
						i += 1
					j += 1
				x += 1
			y += 1

		return

	"""Run the A* algorithm from a given start to a given goal.
		This code is based off of the pseudocode on Wikipedia.
		Returns true if a valid path is found, false otherwise."""
	def navigate(self, start, goal, mapArray):

		if (False):
			print self.getNodeAt(start), self.getNodeAt(goal)
			print start, goal
			"""for f in self.nodesArray:
				for g in f:
					print g"""

		if (self.getNodeAt(start).value != 100 and self.getNodeAt(goal).value != 100):

			self.openSet.append(self.getNodeAt(start))
			self.openSet[0].gScore = 0
			self.openSet[0].hScore = self.heuristicFun(start, goal)
			self.openSet[0].fscore = self.openSet[0].hScore

			openCoords = [self.openSet[0].coord]

			while (len(self.openSet) > 0):

				current = self.openSet.pop(0)


				if (True):
					mapArray[current.coord[0]][current.coord[1]] = int(current.fScore)
					self.thing.append(current.coord)

				if (current.coord == goal):
					for m in mapArray:
						print m
					self.thing = openCoords
					return current.reconstruct(start)

				self.closedSet.append(current)

				for e in current.edges:

					if ([e.coord[0] - current.coord[0], e.coord[1] - current.coord[1]] != [0,0]) and math.fabs(e.coord[0] - current.coord[0]) <= 1 and math.fabs(e.coord[1] - current.coord[1]) <= 1:

						print e.coord[0] - current.coord[0], e.coord[1] - current.coord[1]	
						tentG = current.gScore + self.pythagoras(current.coord, e.coord)
						print self.pythagoras(current.coord, goal)

						if (tentG < e.gScore):

							e.fromNode = current
							e.gScore = tentG
							e.hScore = self.heuristicFun(e.coord, goal)
							e.fScore = e.gScore + e.hScore

							if e not in self.openSet:
								openCoords.append(e.coord)
								self.sortedInsert(e)

		return False

	"""return the node at a coordinate"""
	def getNodeAt(self, coord):

		node = self.mapArray[coord[0]][coord[1]]

		return node

	def pythagoras(self, pointA, pointB):
		print pointA, pointB
		print (pointA[0] - pointB[0])**2, (pointA[1] - pointB[1])**2
		return math.sqrt(((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2))

	"""this is the heuristic, currently it's the euclidean distance,
		this is its own function to make it easy to modify"""
	def heuristicFun(self, start, goal):

		return self.pythagoras(start, goal)

	"""run a sorted insert of a node into the open set, using F score to determine order"""
	def sortedInsert(self, node):

		self.openSet.insert(0, node)

		#let's hope my first adventure with lambda functions goes well
		cmpF = lambda nodeA: nodeA.fScore

		self.openSet = sorted(self.openSet, key=cmpF)

		return


class ANode:
	coord = [0,0]
	edges = []
	fromNode = None
	value = 100

	#scores
	fScore = 0
	gScore = float('inf')
	hScore = 0

	"""constructor"""
	def __init__(self, value, coord):
		#print value

		self.coord = coord
		self.value = value

		#print self.value

	def reconstruct(self, start):
		path = [self.coord]
		nextNode = self.fromNode

		while nextNode.coord != start:
			print nextNode
			path.append(nextNode.coord)
			nextNode = nextNode.fromNode

		path.append(nextNode.coord)

		return path

	def __str__(self):

		return "coordinate: " + str(self.coord) + ", value: " + str(self.value)