import math
import copy
import numpy as np
import random as r
import statistics
import matplotlib.pyplot as plt
from bresenham import bresenham
from Dijkstra import dijkstra


class prm:
	def __init__(self, map_in, start_pos=[0, 0], end_pos=[1, 1]):
		self.V = []
		self.V.append(start_pos)
		self.V.append(end_pos)
		self.E = []
		self.W = []
		self.map = map_in

	def find_valid_positions(self, num_nodes, wall_thresh):
		# iterate through the nodes
		for i in range(0, num_nodes-2):
			
			while(True):
				# Generate a random position
				x = r.random() * (self.map.shape[1] - 1)
				y = r.random() * (self.map.shape[0] - 1)
				# Check it is not near a wall
				cond1 = self.map[round(y + wall_thresh), round(x + wall_thresh)] == 0
				cond2 = self.map[round(y + wall_thresh), round(x - wall_thresh)] == 0
				cond3 = self.map[round(y - wall_thresh), round(x + wall_thresh)] == 0
				cond4 = self.map[round(y - wall_thresh), round(x - wall_thresh)] == 0
				# If it is not near a wall exit
				if (cond1 and cond2 and cond3 and cond4):
					break	
			# Stack the list into an array
			self.V.append([x, y])
		
		# Save the vertices
		self.V = np.vstack(self.V).astype(float)

		# Return that it worked
		return True

	def plan(self, dist_thresh):
		# Get all the edges
		temp_edge = []
		for each_node in range(0, len(self.V)):
			for each_other_node in range(0, len(self.V)):
				# Dont compare the same node
				if(each_node != each_other_node):
					dx = (self.V[each_node, 0] - self.V[each_other_node, 0])**2
					dy = (self.V[each_node, 1] - self.V[each_other_node, 1])**2
					dist = math.sqrt(dx + dy)
					# Check to see if the nodes can be linked
					if(dist < dist_thresh):
						add = True
						# Check to see if this line intersects any map points
						grid_cells = list(bresenham(x0=int(round(self.V[each_node, 1])),
													y0=int(round(self.V[each_node, 0])),
													x1=int(round(self.V[each_other_node, 1])),
													y1=int(round(self.V[each_other_node, 0]))))
						for each_cell in range(0, len(grid_cells)):
							x = grid_cells[each_cell][0]
							y = grid_cells[each_cell][1]
							if self.map[x,y] == 1:
								add = False
								break
						# Check the opposite is not in the list
						if (add):
							for each_edge in range(0,len(temp_edge)):
								if (temp_edge[each_edge] == [each_other_node, each_node]):
									add = False
									break
							# Add the edge to the list
							if (add):
								temp_edge.append([each_node, each_other_node])
								self.W.append(dist)


			# Save the edges
			self.E = np.vstack(temp_edge)

		# Return that it worked
		return True

	def getVerticies(self):
		return self.V

	def getEdges(self):
		return self.E

	def getWeights(self):
		return self.W

	def adjacencyMatrix(self):
		# Create the adjacency matrix
		m = np.zeros((len(self.V), len(self.V)))
		# Go through the list of edges and add each to the matrix
		for edge_itt in range(0,len(self.E)):
			e = self.E[edge_itt]
			# Add the weight in both directions
			m[e[0],e[1]] = self.W[edge_itt]
			m[e[1],e[0]] = self.W[edge_itt]
		return m

	def findShortestPath(self):
		try:
			# Get the adjacency matrix
			matrix = self.adjacencyMatrix()
			d = dijkstra() 
			pathdetails = d.shortest_path(matrix,0)
		except:
			print("Graph not connected")
			pathdetails = {}
			pathdetails["Path"] = []
	
		return pathdetails

	def findAllPaths(self):
		edges_copy = copy.deepcopy(self.E)
		weights_copy = copy.deepcopy(self.W)
		all_paths = []
		while(True):
			try:
				pathdetails = self.findShortestPath()
				# if no path was found
				if len(pathdetails["Path"]) <= 0:
					break
				# Get the path
				path = pathdetails["Path"]
				all_paths.append(path)
				pathweights = []

				# For each edge in the path
				path_indices = self.getEdgeIndicesFromPath(path)

				# Get the weight of the path
				for index in path_indices:
					pathweights.append(self.W[index])
				
				# Remove the edge with the lowest cost
				lowest_cost = np.argmin(pathweights)
				lowest_edge_index = path_indices[lowest_cost]
				self.E = np.delete(self.E, lowest_edge_index, axis=0)
				self.W = np.delete(self.W, lowest_edge_index)
			except:
				print("All paths found")
				break

		# Reset edges and weights
		self.E = edges_copy
		self.W = weights_copy

		return all_paths

	def generateLineHeadings(self, path):
		# Check if the path is non empty
		if len(path) <= 0:
			return []

		# Keep track of the source so we can move in a constant direction
		src = 0
		edge_headings = []

		# First get all the edge indices
		edge_indices = self.getEdgeIndicesFromPath(path)
		for edge in edge_indices:
			# Get the correct vertex numbers
			if src == self.E[edge, 0]:
				edge_num = [self.E[edge, 0], self.E[edge, 1]]
				src = self.E[edge, 1]
			else:
				edge_num = [self.E[edge, 1], self.E[edge, 0]]
				src = self.E[edge, 0]

			# Get the x and y co-ordinates of each vertex
			[start_coord_x, start_coord_y] = self.V[edge_num[0]]
			[end_coord_x, end_coord_y] = self.V[edge_num[1]]

			# Use these-cordinates to calculate a heading
			dx = float(end_coord_x) - start_coord_x
			dy = float(end_coord_y) - start_coord_y

			# Calculate the heading between -180 and 180 (0 == East)
			heading = math.degrees(math.atan2(dy, dx))
			edge_headings.append(heading)

		return edge_headings

	def scoreAllPaths(self, paths):
		scores = []
		for path in paths:
			headings = self.generateLineHeadings(path)
			standard_deviation = statistics.stdev(headings)
			scores.append(standard_deviation)

		return scores

	def getEdgeIndicesFromPath(self, path):

		edge_number = []
		# For each edge in the path
		for i in range(1, len(path)):
			cur_edge = [path[i-1], path[i]]
			# For each edge in all edges
			for edge_itt in range(0, len(self.E)):
				crit1 = (self.E[edge_itt, 0] == cur_edge[0] and self.E[edge_itt, 1] == cur_edge[1])
				crit2 = (self.E[edge_itt, 0] == cur_edge[1] and self.E[edge_itt, 1] == cur_edge[0])
				# If we have found matching edges
				if (crit1 or crit2):
					# Add that edge to the edge list
					edge_number.append(edge_itt)

		return edge_number

	def plot(self, highlighted_paths=[], tsuffix=""):
		# Print Map
		if len(tsuffix) >= 0:
			plt.title('Occupancy Map - ' + str(tsuffix))
		else:
			plt.title('Occupancy Map')
		plt.imshow(self.map, cmap='binary')
		plt.xlim([0,self.map.shape[1]-1])
		plt.ylim([0,self.map.shape[1]-1])

		# Print the random points and give them labels
		plt.scatter(self.V[:,0], self.V[:,1], s=2)
		labels = np.arange(len(self.V))
		labels = [str(i) for i in labels]
		for l in range(0,len(labels)):
			plt.text(self.V[l,0]+.03, self.V[l,1]+.03, labels[l], fontsize=9)

		# Display the edges in the map
		for edge in range(0,len(self.E)):
			linex = [self.V[self.E[edge,0],0],self.V[self.E[edge,1],0]]
			liney = [self.V[self.E[edge,0],1],self.V[self.E[edge,1],1]]
			plt.plot(linex,liney,color='b')

		# Highlight the paths in the map
		for path in highlighted_paths:
			edge_indices = self.getEdgeIndicesFromPath(path)
			for edge in edge_indices:
				linex = [self.V[self.E[edge,0],0],self.V[self.E[edge,1],0]]
				liney = [self.V[self.E[edge,0],1],self.V[self.E[edge,1],1]]
				plt.plot(linex,liney,color='r')

		plt.show()

	def getWaypointsFromPath(self, path):
		# Keep track of the source so we can move in a constant direction
		src = 0
		verticies = []

		# First get all the edge indices
		edge_indices = self.getEdgeIndicesFromPath(path)
		for edge in edge_indices:
			# Get the correct vertex numbers
			if src == self.E[edge, 0]:
				edge_num = [self.E[edge, 0], self.E[edge, 1]]
				src = self.E[edge, 1]
			else:
				edge_num = [self.E[edge, 1], self.E[edge, 0]]
				src = self.E[edge, 0]

			# Get thoe x and y c-ordinates of the source vertex
			single_vertex = self.V[edge_num[0]]
			verticies.append(single_vertex)

		# Add the end vertex
		single_vertex = self.V[edge_num[1]]
		verticies.append(single_vertex)

		return verticies

	def windowMapFromWaypoints(self, waypoints, window_gap=1, min_wall_distance=1):
		new_map = copy.deepcopy(self.map)
		# For each of the waypoints
		for point in waypoints:
			north_wall_start = [point[0], point[1] + window_gap]
			north_wall_end = [point[0], self.map.shape[0]-1]
			south_wall_start = [point[0], point[1] - window_gap]
			south_wall_end = [point[0], 0]

			north_wall = list(bresenham(x0=int(round(north_wall_start[0])),
										y0=int(round(north_wall_start[1])),
										x1=int(round(north_wall_end[0])),
										y1=int(round(north_wall_end[1]))))

			south_wall = list(bresenham(x0=int(round(south_wall_start[0])),
										y0=int(round(south_wall_start[1])),
										x1=int(round(south_wall_end[0])),
										y1=int(round(south_wall_end[1]))))

			# add the walls to a list
			walls = [north_wall, south_wall]

			# Check if we already have a wall in this line
			if not (1 in new_map[:, walls[0][0][0]]):
				insert_wall = True
				# Check there is no walls within the minimum distance constraint
				for dc in range(1, min_wall_distance + 1):
					# Check there is no wall to the left
					if walls[0][0][0] - dc >= 0:
						if 1 in new_map[:, walls[0][0][0] - dc]:
							insert_wall = False

					# Check there is no wall to the right
					if walls[0][0][0] + dc < new_map.shape[1]:
						if 1 in new_map[:, walls[0][0][0] + dc]:
							insert_wall = False

				if insert_wall:
					# For each of the walls
					for wall in walls:
						# For each cell in a wall
						for cell in wall:
							# If we can place the wall:
							if 0 <= cell[1] < new_map.shape[0] and 0 <= cell[0] < new_map.shape[1]:
								new_map[cell[1], cell[0]] = 1
			# There was already a wall so add an opening
			else:
				# Calculate the new opening:
				opening = list(bresenham(x0=north_wall[0][0],
										 y0=north_wall[0][1],
										 x1=south_wall[0][0],
										 y1=south_wall[0][1]))

				# Add the opening
				for cell in opening:
					if 0 <= cell[1] < new_map.shape[0] and 0 <= cell[0] < new_map.shape[1]:
						new_map[cell[1], cell[0]] = 0

		# Return the map
		return new_map

	def corridorMapFromWaypoints(self, waypoints, corridor_gap=1):
		# Create a map of obstacles
		new_map = np.ones(self.map.shape)

		# Start and end of the segment
		start_p = []
		end_p = []

		# For each point in the waypoints
		for point in waypoints:
			end_p = start_p
			start_p = point
			# If we have a segment
			if len(end_p) >= 1:
				corridor = list(bresenham(x0=int(round(start_p[0])),
										  y0=int(round(start_p[1])),
										  x1=int(round(end_p[0])),
										  y1=int(round(end_p[1]))))

				# Add the corridor to the map
				for cell in corridor:
					# Add buffing on each end for the gap
					for gap in range(0, int(round(corridor_gap/2.0))+1):
						space_above = cell[1] + gap < self.map.shape[0]
						space_below = cell[1] - gap >= 0
						space_right = cell[0] + gap < self.map.shape[1]
						space_left = cell[0] - gap >= 0

						# If we can add gaps above
						if space_above:
							new_map[cell[1] + gap][cell[0]] = 0
						# If we can add gaps below
						if space_below:
							new_map[cell[1] - gap][cell[0]] = 0
						# If we can add gaps to the right
						if space_right:
							new_map[cell[1]][cell[0] + gap] = 0
						# If we can add gaps to the left
						if space_left:
							new_map[cell[1]][cell[0] - gap] = 0
						# If we can add gaps above right
						if space_above and space_right:
							new_map[cell[1] + gap][cell[0] + gap] = 0
						# If we can add gaps above left
						if space_above and space_left:
							new_map[cell[1] + gap][cell[0] - gap] = 0
						# If we can add gaps below right
						if space_below and space_right:
							new_map[cell[1] - gap][cell[0] + gap] = 0
						# If we can add gaps below left
						if space_below and space_left:
							new_map[cell[1] - gap][cell[0] - gap] = 0

		# Return the map
		return new_map