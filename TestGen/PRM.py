import math
import numpy as np
import random as r
import matplotlib.pyplot as plt
from bresenham import bresenham
from functools import lru_cache
from Trajectory import Trajectory

class prm:
	def __init__(self, map_in, start_pos=[0, 0], end_pos=[1, 1]):
		# Create the list of vertices
		self.V = []
		self.V.append(start_pos)
		self.source_index = 0
		self.V.append(end_pos)
		self.end_index = 1

		self.E = []
		self.W = []
		self.E_headings = []
		self.map = map_in
		self.Ajmatrix = []

	def findValidPositions(self, num_vertices, wall_thresh):
		# We know the number of verticies so we can initilize the line heading array
		self.E_headings = np.empty((num_vertices, num_vertices))
		self.E_headings[:] = np.nan

		# iterate through the vertices
		for i in range(0, num_vertices-2):
			
			while True:
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

	def plan(self, max_distance=2, min_distance=1):
		# Get all the edges
		temp_edge = []
		for each_vertex in range(0, len(self.V)):
			for each_other_vertex in range(0, len(self.V)):
				# Dont compare the same vertex
				if each_vertex != each_other_vertex:
					# Get the distance between the vertices
					dx = (self.V[each_vertex, 0] - self.V[each_other_vertex, 0])**2
					dy = (self.V[each_vertex, 1] - self.V[each_other_vertex, 1])**2
					dist = math.sqrt(dx + dy)
					# Check to see if the nodes can be linked
					if min_distance < dist < max_distance:
						add = True
						# Check to see if the line intersects
						grid_cells = list(bresenham(x0=int(round(self.V[each_vertex, 1])),
													y0=int(round(self.V[each_vertex, 0])),
													x1=int(round(self.V[each_other_vertex, 1])),
													y1=int(round(self.V[each_other_vertex, 0]))))
						for each_cell in range(0, len(grid_cells)):
							x = grid_cells[each_cell][0]
							y = grid_cells[each_cell][1]
							if self.map[x, y] == 1:
								add = False
								break
						# Check a edge in the opposite direction is not in the list
						if add:
							for each_edge in range(0, len(temp_edge)):
								if temp_edge[each_edge] == [each_other_vertex, each_vertex]:
									add = False
									break
							# Add the edge to the list
							if add:
								# Get the line headings and save them
								edge_angle = self.calculateLineHeading([each_vertex, each_other_vertex])
								self.E_headings[each_vertex, each_other_vertex] = edge_angle
								edge_angle = self.calculateLineHeading([each_other_vertex, each_vertex])
								self.E_headings[each_other_vertex, each_vertex] = edge_angle

								temp_edge.append([each_vertex, each_other_vertex])
								self.W.append(dist)

			# Check if any edges were found
			if len(temp_edge) <= 0:
				print("No edges found")
				exit()

			# Save the edges
			self.E = np.vstack(temp_edge)

			# Turn our graph into an adjancency Matrix to speed up finding edges connected to it
			self.Ajmatrix = self.adjacencyMatrix()

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
		for edge_itt in range(0, len(self.E)):
			e = self.E[edge_itt]
			# Add the weight in both directions
			m[e[0], e[1]] = self.W[edge_itt]
			m[e[1], e[0]] = self.W[edge_itt]
		return m

	def getEdgesConnectedToVertex(self, vertex):
		edges_found = []
		# For each edge
		for edge in self.E:
			# If the edge is connected to the vertex
			if np.all(self.V[edge[0]] == vertex) or np.all(self.V[edge[1]] == vertex):
				edges_found.append(edge)

		return edges_found

	@lru_cache(maxsize=None)
	def getEdgesConnectedToVertexFromAjMatrix(self, vertex):
		edges_found = []
		for col in range(0, self.Ajmatrix.shape[1]):
			if self.Ajmatrix[vertex, col] != 0:
				# Convert it to an array so it functions like the other get Edges function
				#edges_found.append(np.array([vertex, col]))
				edges_found.append([vertex, col])

		return edges_found

	def findAllPaths(self, heading, min_turn_deg=-90, max_turn_deg=90, depth=100, max_traj=1000):

		# Check that the min and max turn are correct
		assert(min_turn_deg < max_turn_deg)
		assert(max_turn_deg <= 180)
		assert(min_turn_deg >= -180)

		# Create a stack containing the source index, the robot heading, the current path and the path headings.
		stack = [(self.source_index, heading, [self.source_index], [])]

		# Keeps track of the edges and path headings
		goal_paths = []
		goal_headings = []

		# Use DFS to explore all paths
		while stack:
			# Get the vertex and the path on top of the stack
			(vertex, robot_heading, path, path_heading) = stack.pop()

			# Dont consider paths greater than certain depth
			if len(path) > depth:
				continue

			# Get all the edges leading out of that vertex
			edges = self.getEdgesConnectedToVertexFromAjMatrix(vertex)
			# Removed for being super slow
			# edges1 = self.getEdgesConnectedToVertex(self.V[vertex])

			# Make sure each of the edges is ordered correctly
			for edge in edges:
				if edge[0] != vertex:
					next_vertex = edge[0]
				else:
					next_vertex = edge[1]

				# Check that the next vertex has not been visited on this path
				if next_vertex in path:
					# Skip everything after this
					continue

				# Get the line heading
				edge_angle = self.readLineHeadings(tuple([vertex, next_vertex]))
				# Read it from the saved array rather than recalculating
				# edge_angle = self.calculateLineHeading([vertex, next_vertex])

				# If the next vertex is within our reachability set
				delta_angle = edge_angle - robot_heading
				if delta_angle > 180:
					delta_angle -= 360
				if delta_angle < -180:
					delta_angle += 360

				if min_turn_deg <= delta_angle <= max_turn_deg:
					# If the next vertex is not the goal
					if next_vertex != self.end_index:
						stack.append((next_vertex, edge_angle, path + [next_vertex], path_heading + [edge_angle]))
					else:
						goal_paths.append(path + [next_vertex])
						goal_headings.append(path_heading + [edge_angle])
						if len(goal_paths) >= max_traj:
							# Stop Looking
							stack = False
							break

		# Add each of the trajectories to a trajectory class for easier processing
		final_trajectories = []
		for i in range(0, len(goal_paths)):
			waypoints = self.getWaypointsFromPath(goal_paths[i])
			traj = Trajectory()
			traj.set_waypoints(waypoints)
			traj.set_headings(goal_headings[i])
			final_trajectories.append(traj)

		return final_trajectories

	def calculateLineHeading(self, edge):
		# Get the x and y co-ordinates of each vertex
		[start_coord_x, start_coord_y] = self.V[edge[0]]
		[end_coord_x, end_coord_y] = self.V[edge[1]]

		# Use these-coordinates to calculate a heading
		dx = float(end_coord_x) - start_coord_x
		dy = float(end_coord_y) - start_coord_y

		# Calculate the heading between -180 and 180 (0 == East)
		heading = math.degrees(math.atan2(dy, dx))

		return heading

	@lru_cache(maxsize=None)
	def readLineHeadings(self, edge):
		return self.E_headings[edge[0], edge[1]]

	def generateLineHeadingsFromPath(self, path):
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

			# Calculate the heading between -180 and 180 (0 == East)
			#heading = self.calculateLineHeading(edge_num)
			heading = self.readLineHeadings(tuple(edge_num))
			edge_headings.append(heading)

		return edge_headings

	@lru_cache(maxsize=None)
	def getEdgeIndex(self, edge):
		# For each edge in all edges
		for edge_itt in range(0, len(self.E)):
			crit1 = (self.E[edge_itt, 0] == edge[0] and self.E[edge_itt, 1] == edge[1])
			crit2 = (self.E[edge_itt, 0] == edge[1] and self.E[edge_itt, 1] == edge[0])
			# If we have found matching edge
			if (crit1 or crit2):
				# Add that edge to the edge list
				return edge_itt

	def getEdgeIndicesFromPath(self, path):

		edge_number = []
		# For each edge in the path
		for i in range(1, len(path)):
			cur_edge = [path[i-1], path[i]]
			# Get the index from the current edge
			cur_index = self.getEdgeIndex(tuple(cur_edge))
			edge_number.append(cur_index)

		return edge_number

	def getPlot(self, highlighted_paths=[], path_class=[], tsuffix="", color_map="jet_r", figure_size=(15, 13)):

		plt.figure(figsize=figure_size)

		# Print Map
		if len(tsuffix) >= 0:
			plt.title('Occupancy Map - ' + str(tsuffix))
		else:
			plt.title('Occupancy Map')
		plt.imshow(self.map, cmap='binary')
		plt.xlim([0, self.map.shape[1]-1])
		plt.ylim([0, self.map.shape[0]-1])

		# Check if we have added more than the start and end vertex
		# Check if we have highlighted paths
		if (len(self.V) > 2) and highlighted_paths == []:
			# Print the random points and give them labels
			plt.scatter(self.V[:, 0], self.V[:, 1], s=2)
			labels = np.arange(len(self.V))
			labels = [str(i) for i in labels]
			for l in range(0, len(labels)):
				plt.text(self.V[l, 0]+ .03, self.V[l, 1]+.03, labels[l], fontsize=9)


			# Display the edges in the map
			for edge in range(0, len(self.E)):
				linex = [self.V[self.E[edge, 0], 0], self.V[self.E[edge, 1], 0]]
				liney = [self.V[self.E[edge, 0], 1], self.V[self.E[edge, 1], 1]]
				plt.plot(linex, liney, color='b', linestyle=":", linewidth=0.3)

		# Highlight the paths in the map
		i = 0
		# total_paths = len(highlighted_paths)
		# cmap = plt.get_cmap(color_map)
		for path in highlighted_paths:
			if path_class[i] == True:
				selected_color = 'green'
			else:
				selected_color = 'red'
			# selected_color = cmap(float(i) / total_paths)
			i += 1

			linex = []
			liney = []
			for v1, v2 in zip(path, path[1:]):
				if len(linex) == 0:
					linex = [self.V[v1, 0], self.V[v2, 0]]
					liney = [self.V[v1, 1], self.V[v2, 1]]
				else:
					linex.append(self.V[v2, 0])
					liney.append(self.V[v2, 1])
			plt.plot(linex, liney, color=selected_color)

		return plt

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

	def getEdgeListFromPath(self, path):
		edgeList = []
		# Get the edge list
		for v1, v2 in zip(path, path[1:]):
			edgeList.append([v1, v2])

		return edgeList
