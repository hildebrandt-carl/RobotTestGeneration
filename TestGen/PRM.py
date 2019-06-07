import math
import copy
import numpy as np
import random as r
import statistics
import matplotlib.pyplot as plt
from bresenham import bresenham
from Dijkstra import dijkstra
from functools import lru_cache


class prm:
	def __init__(self, map_in, start_pos=[0, 0], end_pos=[1, 1]):
		self.V = []
		self.V.append(start_pos)
		self.V.append(end_pos)
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

	def findShortestPath(self):
		try:
			# Get the adjacency matrix
			matrix = self.adjacencyMatrix()
			d = dijkstra() 
			pathdetails = d.shortest_path(matrix, 0)
		except:
			print("Graph not connected")
			pathdetails = {}
			pathdetails["Path"] = []
	
		return pathdetails

	def findAllPathsUsingShortest(self):
		edges_copy = copy.deepcopy(self.E)
		weights_copy = copy.deepcopy(self.W)
		all_paths = []
		while True:
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

	def classifyPathsReachability(self, paths, source_index=0, heading=0, min_turn_deg=90, max_turn_deg=90, delta=5):
		# List to save the reachability variables
		classification = []

		# For each path in the paths
		for path in paths:
			# Get the edges
			edges_indices = self.getEdgeIndicesFromPath(path)
			edges = self.E[edges_indices]

			# Assume the path is inside the reachability set
			path_pass = True

			# Traverse the path
			vertex = source_index
			robot_heading = heading
			for edge in edges:
				# Make sure you are traversing it correctly
				if edge[0] != vertex:
					next_vertex = edge[0]
				else:
					next_vertex = edge[1]
				# Get the edge angle
				# edge_angle = self.calculateLineHeading([vertex, next_vertex])
				edge_angle = self.readLineHeadings(tuple([vertex, next_vertex]))
				# Get how much the robot had to turn
				delta_angle = edge_angle - robot_heading
				if delta_angle > 180:
					delta_angle -= 360
				if delta_angle < -180:
					delta_angle += 360

				# Save the end of this edge as the start of the next
				vertex = next_vertex
				robot_heading = edge_angle

				# If the robot is outside of the reachable set
				if not (min_turn_deg < delta_angle < max_turn_deg):
					path_pass = False

			# Append whether the path failed or passed
			classification.append(path_pass)

		return classification

	def findAllPaths(self, source_index, goal_index, heading, min_turn_deg=-90, max_turn_deg=90, depth=100, max_traj=1000):
		# Create a stack containing the source index and current path
		stack = [(source_index, heading, [source_index])]
		goal_paths = []

		# Used to count the number of early stops
		number_early_stop = 0

		# Use DFS to explore all paths
		while stack:
			# Get the vertex and the path on top of the stack
			(vertex, robot_heading, path) = stack.pop()

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

				# Count the number of paths we stopped early



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
					if next_vertex != goal_index:
						stack.append((next_vertex, edge_angle, path + [next_vertex]))
					else:
						goal_paths.append(path + [next_vertex])
						if len(goal_paths) >= max_traj:
							return goal_paths
				else:
					# Increment the count for how many possible trees we were able to stop searching
					number_early_stop += 1

		return goal_paths, number_early_stop

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

	def scoreAllPaths(self, paths, source_index=0, heading=0, min_turn=-90, max_turn=90, coverage_segments=1):

		# Count how many unique elements are between min and max
		coverage_range = max_turn - min_turn
		segmentsize = coverage_range / float(coverage_segments)

		# List for the scores
		scores = np.zeros((len(paths), 6))
		score_counter = 0
		# For each path in the scores
		for path in paths:
			edges_indices = self.getEdgeIndicesFromPath(path)
			edges = self.E[edges_indices]

			# Used to calculate reachability
			direction_travelled = []

			# Traverse the path
			vertex = source_index
			robot_heading = heading
			for edge in edges:
				# Make sure you are traversing it correctly
				if edge[0] != vertex:
					next_vertex = edge[0]
				else:
					next_vertex = edge[1]
				# Get the edge angle
				#edge_angle = self.calculateLineHeading([vertex, next_vertex])
				edge_angle = self.readLineHeadings(tuple([vertex, next_vertex]))
				# Get how much the robot had to turn
				delta_angle = edge_angle - robot_heading
				if delta_angle > 180:
					delta_angle -= 360
				if delta_angle < -180:
					delta_angle += 360

				# Save the end of this edge as the start of the next
				vertex = next_vertex
				robot_heading = edge_angle

				# Reachability Set Coverage
				direction_travelled.append(round(delta_angle))

			# Reachability Set Coverage
			coverage_array = np.zeros(coverage_segments)
			direction_travelled = list(set(direction_travelled))
			for i in range(0, len(direction_travelled)):
				for cov in range(0, coverage_segments):
					left_segment = (cov * segmentsize) + min_turn
					right_segment = ((cov + 1) * segmentsize) + min_turn
					if left_segment <= direction_travelled[i] <= right_segment:
						coverage_array[cov] = 1

			coverage = float(np.sum(coverage_array))/len(coverage_array)

			# Average Segment Length
			distances = []
			total_segments = len(edges)
			for edge in edges:
				# Get the points from the edges
				point1 = self.V[edge[0]]
				point2 = self.V[edge[1]]

				# Get the distance of those two points
				dx = (point1[0] - point2[0]) ** 2
				dy = (point1[1] - point2[1]) ** 2
				distances.append(math.sqrt(dx + dy))

			average_length = np.sum(distances) / total_segments

			# Segment Variance
			segment_length_variance = statistics.variance(distances)

			# Variance Line Headings
			headings = self.generateLineHeadingsFromPath(path)
			path_heading_variance = statistics.variance(headings)

			# Total path length
			path_length = np.sum(distances)

			# Number of turns
			total_turns = total_segments

			# Save the scores
			scores[score_counter, 0] = coverage
			scores[score_counter, 1] = average_length
			scores[score_counter, 2] = segment_length_variance
			scores[score_counter, 3] = path_heading_variance
			scores[score_counter, 4] = path_length
			scores[score_counter, 5] = total_turns
			score_counter += 1

		return scores

	def normalizeScores(self, scores):

		if scores.shape[0] > 1:
			# Get the minimum and maximum score for each of the metrics
			min_scores = np.min(scores, axis=0)
			max_scores = np.max(scores, axis=0)

			# Reachability set coverage actually has a min of 0 and max of 1
			min_scores[0] = 0
			max_scores[0] = 1

			# Calculate the range
			range = max_scores - min_scores

			# If the range is 0 divide by 1 (this means that normalized score will be 0)
			range[range == 0] = 1

			# Normalize the scores
			normalized_scores = (scores - min_scores) / range
		else:
			print("No scores to normalize")
			return None

		return normalized_scores

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

	def plotTrajectories(self, selected_tests, path_class, path_scores, total_plots=100, figure_size=(70, 70), tsuffix=""):
		# For use in plotting
		tests = copy.deepcopy(list(selected_tests))

		# Calculate the axis lengths
		axis_length = int(round(math.sqrt(total_plots)))
		total = axis_length**2

		# Create the figure
		fig = plt.figure(figsize=figure_size)
		axes = fig.subplots(nrows=axis_length+1, ncols=axis_length)
		# Print Map
		if len(tsuffix) >= 0:
			fig.suptitle(t='Individual Trajectories (Sorted by Score) - ' + str(tsuffix),
						 fontsize=54)
		else:
			fig.suptitle(t='Individual Trajectories (Sorted by Score)',
						 fontsize=54)

		# Get the individual scores for the tests
		path_scores = np.sum(path_scores, axis=1)
		# Sort decending
		sorted_indicies = (np.argsort(path_scores))[::-1]


		# Select tests with the highest score
		path_scores = list(np.array(path_scores)[sorted_indicies])
		plotted_tests = list(np.array(tests)[sorted_indicies])
		plotted_class = list(np.array(path_class)[sorted_indicies])

		# Check if the total is less than the number of tests
		if len(plotted_tests) < total:
			print("Not enough tests given appending blank tests")
			while len(plotted_tests) < total:
				path_scores.append(6)
				plotted_tests.append([0, 0])
				plotted_class.append("False")

		i = 0
		for row in range(1, axis_length+1):
			for col in range(0, axis_length):
				# Make sure the first row is blank
				if row == 1:
					axes[row-1, col].set_xlim([0, self.map.shape[1] - 1])
					axes[row-1, col].set_ylim([0, self.map.shape[0] - 1])
					axes[row-1, col].axis('off')
				# Draw the path in the
				path = plotted_tests[i]
				# Create the lines we are going to plot
				linex = []
				liney = []
				# Build the line
				for v1, v2 in zip(path, path[1:]):
					if len(linex) == 0:
						linex = [self.V[v1, 0], self.V[v2, 0]]
						liney = [self.V[v1, 1], self.V[v2, 1]]
					else:
						linex.append(self.V[v2, 0])
						liney.append(self.V[v2, 1])

				# Determine what class the path is
				if plotted_class[i] == 1:
					c = 'green'
				else:
					c = 'red'

				# Plot the trajectory's
				axes[row, col].plot(linex, liney, color=c, linewidth=5)
				axes[row, col].set_xlim([0, self.map.shape[1] - 1])
				axes[row, col].set_ylim([0, self.map.shape[0] - 1])
				axes[row, col].axis('off')

				# Increment i to get the next path
				i += 1


		return fig

	def getEdgeListFromPath(self, path):
		edgeList = []
		# Get the edge list
		for v1, v2 in zip(path, path[1:]):
			edgeList.append([v1, v2])

		return edgeList

	def selectTestsBasedOnCoverage(self, selected_tests):
		# Keep track of which edges where visited
		indices = []
		edgesVisited = []
		selected_paths = []
		new_edges_count = []

		# For each path in selected_tests
		i = 0
		for path in selected_tests:
			edgeList = self.getEdgeListFromPath(path)
			# For each edge
			firstTimeVists = 0
			for edge in edgeList:
				# Check if the edge has been visited
				if edge not in edgesVisited:
					# We have now visited it
					edgesVisited.append(edge)
					firstTimeVists += 1

			if firstTimeVists > 0:
				indices.append(i)
				selected_paths.append(path)
				new_edges_count.append(firstTimeVists)

			# Increment the indices
			i += 1
			#print("Path: " + str(path))
			#print("Unvisited Edges In Path: " + str(firstTimeVists))

		return selected_paths, indices
