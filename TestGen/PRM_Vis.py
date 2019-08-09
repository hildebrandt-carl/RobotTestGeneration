import time
import random
import numpy as np
from Trajectory import Trajectory
from Node import Node
from Edge import Edge
from DroneKinematic import DroneKinematic
from DroneReachabilitySet import DroneReachabilitySet
from RankingSystem import RankingSystem
import matplotlib.pyplot as plt
from Trajectory import Trajectory
from mpl_toolkits.mplot3d import Axes3D
import math
from queue import *
import copy
from scipy.spatial import ConvexHull


class PRM:
    def __init__(self, start_pos, end_pos, map_x_bounds=[0, 1], map_y_bounds=[0, 1], map_z_bounds=[0, 1]):

        # Create the list of vertices and edges
        self.V = []
        self.E = []

        # Create an adjacency matrix for quick lookup of connected nodes
        self.AdjacencyMatrix = None

        # Check the map bounds are correct
        if map_x_bounds[1] > map_x_bounds[0] and map_y_bounds[1] > map_y_bounds[0] and map_z_bounds[1] > map_z_bounds[0]:
            # Set the map bounds
            self.x_bounds = map_x_bounds
            self.y_bounds = map_y_bounds
            self.z_bounds = map_z_bounds
        else:
            # The map bounds are incorrect
            print("Map bounds do not make sense")
            exit()

        # Set the nodes range
        Node.x_range = {"lower": self.x_bounds[0], "upper": self.x_bounds[1]}
        Node.y_range = {"lower": self.y_bounds[0], "upper": self.y_bounds[1]}
        Node.z_range = {"lower": self.z_bounds[0], "upper": self.z_bounds[1]}

        # Create the start and the end node
        start_node = Node(source=True)
        end_node = Node(sink=True)

        # Set the start and the end nodes position
        start_worked = start_node.set_position(x=start_pos[0], y=start_pos[1], z=start_pos[2])
        end_worked = end_node.set_position(x=end_pos[0], y=end_pos[1], z=end_pos[2])

        # If the setting the position worked
        if start_worked == 1 and end_worked == 1:
            # Add the node to the list of nodes
            self.V.append(start_node)
            self.V.append(end_node)
        else:
            # There was an error
            print("There was an error setting the start or end nodes position")
            exit()

    # Return the vertices
    def get_vertices(self):
        return self.V

    # Return the edges
    def get_edges(self):
        return self.E

    # Get the sink vertex
    def get_sink_vertex(self):
        # For each node
        for node in self.V:
            # If this is the sink node
            if node.get_sink():
                return node

    # Get the sink vertex
    def remove_node(self, x, y, z):
        # Keep a list of nodes which need to be removed
        remove_nodes = []

        # For each node
        for i in range(0, len(self.V)):
            # Get the node
            node = self.V[i]

            # If this is the sink node or source node you cant remove it
            if node.get_sink() or node.get_source():
                continue

            # Get the nodes position
            node_x, node_y, node_z = node.get_position()

            # If this node is the one to be removed, remove it
            if node_x == x and node_y == y and node_z == z:
                remove_nodes.append(i)

        # Remove the nodes
        assert len(remove_nodes) <= 1

        for r in remove_nodes:
            self.V.pop(r)

    # Used to populate the graph with a set of nodes in random positions
    def populate_with_nodes(self, num_vertices, input_seed=0):
        
        if input_seed == 0:
            # Set the seed based on the time
            random.seed(time.time())
        else:
            random.seed(input_seed)

        # Iterate through the vertices
        for i in range(0, num_vertices-2):

            # Generate a random position (0-1) * (range) + minimum
            x = random.random() * (self.x_bounds[1] - self.x_bounds[0]) + self.x_bounds[0]
            y = random.random() * (self.y_bounds[1] - self.y_bounds[0]) + self.y_bounds[0]
            z = random.random() * (self.z_bounds[1] - self.z_bounds[0]) + self.z_bounds[0]

            # Create the new node
            new_node = Node()
            set_worked = new_node.set_position(x=x, y=y, z=z)

            # If the setting the position worked
            if set_worked == 1:
                # Add the node to the list of nodes
                self.V.append(new_node)
            else:
                # There was an error
                print("There was an error setting the nodes position")
                exit()

        # Create the adjacency matrix
        self.AdjacencyMatrix = np.zeros((num_vertices, num_vertices))

        # Return that it worked
        return True

    # Used to populate the graph with a set of edges connecting nodes
    def populate_with_edges(self, min_distance=1, max_distance=2):
        # Check that the min and max distances max sense
        if min_distance > max_distance:
            print("Edges cant have a minimum distance greater than the maximum distance")
            exit()

        # Go through each of the nodes
        for vertex in range(0, len(self.V)):
            for other_vertex in range(0, len(self.V)):
                # Dont compare the same vertex
                if vertex != other_vertex:
                    # Get the vertices
                    source_vertex = self.V[vertex]
                    sink_vertex = self.V[other_vertex]

                    # Check if the node is not connected
                    if self.AdjacencyMatrix[vertex, other_vertex] == 0:
                        # Calculate the distance
                        dist = source_vertex.distance_to_node(sink_vertex)

                        # Can these nodes be linked
                        if min_distance < dist < max_distance:
                            # These nodes can be linked, add them to the adjacency matrix
                            self.AdjacencyMatrix[vertex, other_vertex] = 1
                            self.AdjacencyMatrix[other_vertex, vertex] = 1

                            # Create the edge
                            new_edge = Edge(node1=source_vertex, node2=sink_vertex)
                            new_edge.set_distance(dist)

                            # Append the edge to each of the nodes
                            source_vertex.add_edge(new_edge)
                            sink_vertex.add_edge(new_edge)

                            # Add the edge to the list of edges
                            self.E.append(new_edge)

            # Check to see that some edges were added
            if len(self.E) <= 0:
                print("No edges found")
                exit()

        # Return that it worked
        return True

    # Finds paths from start vertex to end vertex which satisfy the kinematic model
    def find_all_paths(self, drone_kinematic_values, kinematic_sample_resolution=5, total_waypoints=5, beam_width=1, score_baseline=False):

        # Debug Counter
        debug_counter = 0

        # Make copies of vertices so they can be re-assigned after this function
        temp_v = copy.deepcopy(self.V)

        # Keeps tracks of the finished paths
        finished_paths = []

        # Create a path scorer to score each path
        ranking_obj = RankingSystem(paths=[], baseline=score_baseline)

        # Create a drone kinematic model at the starting position
        # Create an initial state
        starting_state = DroneKinematic(mass=drone_kinematic_values['m'],
                                        arm_length=drone_kinematic_values['d'],
                                        thrust_constant=drone_kinematic_values['kf'],
                                        moment_constant=drone_kinematic_values['km'],
                                        max_rotor_speed=drone_kinematic_values['max_rotor_speed'],
                                        inertial_properties=drone_kinematic_values['inertial_properties'],
                                        position=drone_kinematic_values['position'],
                                        attitude=drone_kinematic_values['attitude'],
                                        velocity=drone_kinematic_values['velocity'],
                                        angular_vel=drone_kinematic_values['angular_velocity'])

        # Add this starting position to the possible paths array
        frontier = []
        frontier.append([starting_state])

        # Get the sink node
        sink_node = self.get_sink_vertex()
        sink_x, sink_y, sink_z = sink_node.get_position()

        counter = 0
        # While there are unfinished paths
        while len(frontier) > 0:

            # TODO: FOR PLOTTING
            considered = []
            kinematic_draw = []

            # Sort the current paths to be ordered based on score
            # You should have a temporary paths which is added after all nodes have been processed

            # Used to keep track of the new frontier and list of finished nodes
            temp_finished_paths = []
            temp_frontier = []

            # Used to track how many nodes on the frontier have been processed
            beam_counter = 0

            # Process beam width
            while (beam_counter < beam_width) and (len(frontier) > 0):
                print("Processing " + str(beam_counter) + " frontier")

                if debug_counter == 12:
                    print("")

                # Increment the beam counter
                beam_counter += 1

                # Get the current path and current kinematic
                current_path = frontier.pop()
                # The current kinematic is the last kinematic in the path
                current_kinematic = current_path[-1]
                previous_kinematic = None
                if len(current_path) <= 1:
                    previous_kinematic = current_kinematic
                else:
                    previous_kinematic = current_path[-2]

                # Get the current kinematics position
                source_pos = current_kinematic.get_position()
                previous_pos = previous_kinematic.get_position()

                # If the current node is the sink node.
                if source_pos[0] == sink_x and source_pos[1] == sink_y and source_pos[2] == sink_z:
                    # Add this path to the finished paths array
                    temp_finished_paths.append(current_path)
                    print("Found a complete path")
                    # Restart the beam search
                    continue

                # Create the reachability set generator object
                reachability_space_generator = DroneReachabilitySet(drone_kinematic=current_kinematic)

                # Calculate the reachable space for that drone kinematic in one time step
                positions = reachability_space_generator.calculate_reachable_area(sample_resolution=kinematic_sample_resolution)

                #TODO: For plotting
                considered.append(source_pos)
                kinematic_draw.append(positions)

                # Calculate the largest distance between the source node and all sample points in the reachable area.
                current_kinematic.calculate_maximum_velocity(positions)

                # Create a list of standard nodes
                x_vals = []
                y_vals = []
                z_vals = []
                sink_position = [0, 0, 0]

                # For each node
                for node in self.V:
                    # Get the x,y and z values
                    x, y, z = node.get_position()

                    # Create a list of the x,y and z values
                    x_vals.append(x)
                    y_vals.append(y)
                    z_vals.append(z)

                    # If this is the sink node
                    if node.get_sink():
                        # Save it
                        sink_position = [x, y, z]

                # For all waypoints find which are inside the reachability set
                waypoints = np.column_stack((x_vals, y_vals, z_vals))
                inside = reachability_space_generator.is_in_hull(waypoints=waypoints)

                print(str(sum(inside)) + " nodes found inside the reachable set")

                # Used to save the x,y and z position of the waypoints inside the hull
                in_x = []
                in_y = []
                in_z = []

                # Create a list of x, y and z, points inside the hull
                for i in range(0, len(inside)):
                    # If the waypoint is inside
                    if inside[i]:
                        # And are not the same position or previous (stop oscillation)
                        not_same_pos = (waypoints[i][0] != source_pos[0] and waypoints[i][1] != source_pos[1] and waypoints[i][2] != source_pos[2])
                        not_prev_pos = (waypoints[i][0] != previous_pos[0] and waypoints[i][1] != previous_pos[1] and waypoints[i][2] != previous_pos[2])
                        if not_same_pos and not_prev_pos:
                            in_x.append(waypoints[i][0])
                            in_y.append(waypoints[i][1])
                            in_z.append(waypoints[i][2])

                # Create a list of edges which would be traversed if each of the nodes was visited
                # For each of the waypoints inside the hull
                for j in range(0, len(in_x)):

                    # Find the vector between them (This is the velocity)
                    vector_x = in_x[j] - source_pos[0]
                    vector_y = in_y[j] - source_pos[1]
                    vector_z = in_z[j] - source_pos[2]

                    # Calculate the magnitude of the vector
                    magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)

                    # Calculate the angle of that vector
                    # r = math.acos(vector_x / magnitude)
                    # p = math.acos(vector_y / magnitude)
                    # y = math.acos(vector_z / magnitude)

                    # We are going to assume the drone enters the waypoint level
                    r = 0
                    p = 0
                    y = 0

                    # Create a new drone kinematic for this waypoint
                    new_kinematic = DroneKinematic(mass=drone_kinematic_values['m'],
                                                   arm_length=drone_kinematic_values['d'],
                                                   thrust_constant=drone_kinematic_values['kf'],
                                                   moment_constant=drone_kinematic_values['km'],
                                                   max_rotor_speed=drone_kinematic_values['max_rotor_speed'],
                                                   inertial_properties=drone_kinematic_values['inertial_properties'],
                                                   position=[in_x[j], in_y[j], in_z[j]],
                                                   attitude=[r, p, y],
                                                   velocity=[vector_x, vector_y, vector_z],
                                                   angular_vel=drone_kinematic_values['angular_velocity'])

                    # create a new path
                    new_path = current_path.copy()

                    # Add this new path to the unfinished paths queue
                    new_path.append(new_kinematic)

                    # If the path is shorter than the requested length
                    if len(new_path) <= total_waypoints:
                        temp_frontier.append(new_path)
                    else:
                        print("Path not added as it is too long")








                # TO BE DELETED
                # Create the figure
                fig = plt.figure()
                ax = Axes3D(fig)

                # Create a list of standard nodes
                x_vals = []
                y_vals = []
                z_vals = []

                # Create a list of source and sink nodes
                source_x = []
                source_y = []
                source_z = []
                sink_x = []
                sink_y = []
                sink_z = []

                # For each node
                for node in self.V:
                    # Get the x,y and z values
                    x, y, z = node.get_position()

                    # Check if this is a source
                    if node.get_source():
                        # Save the position of the source node
                        source_x.append(x)
                        source_y.append(y)
                        source_z.append(z)
                    # Check if this is a source
                    elif node.get_sink():
                        # Save the position of the source node
                        sink_x.append(x)
                        sink_y.append(y)
                        sink_z.append(z)
                    else:
                        # Save the positions or random nodes
                        x_vals.append(x)
                        y_vals.append(y)
                        z_vals.append(z)

                cons_x = []
                cons_y = []
                cons_z = []
                for c in considered:
                    cons_x.append(c[0])
                    cons_y.append(c[1])
                    cons_z.append(c[2])

                    item_to_remove = []
                    for j in range(0, len(x_vals)):
                        if x_vals[j] == c[0] and y_vals[j] == c[1] and z_vals[j] == c[2]:
                            item_to_remove.append(j)

                    for r in item_to_remove:
                        x_vals.pop(r)
                        y_vals.pop(r)
                        z_vals.pop(r)

                kx = []
                ky = []
                kz = []
                hull = None
                for points in kinematic_draw:
                    try:
                        hull = ConvexHull(points)
                    except:
                        hull = None
                    for point in points:
                        kx.append(point[0])
                        ky.append(point[1])
                        kz.append(point[2])

                for s in hull.simplices:
                    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
                    ax.plot(points[s, 0], points[s, 1], points[s, 2], "r-")

                # Plot the values
                ax.scatter(source_x, source_y, source_z, c='g', label='Starting Position')
                ax.scatter(sink_x, sink_y, sink_z, c='r', label='Ending Position')
                ax.scatter(x_vals, y_vals, z_vals, c='b', label='Possible Waypoints')
                ax.scatter(cons_x, cons_y, cons_z, c='m', label='Processed Waypoints')

                for kinematic_path in temp_frontier:
                    x_arr = []
                    y_arr = []
                    z_arr = []
                    for p in kinematic_path:
                        x, y, z = p.get_position()
                        x_arr.append(x)
                        y_arr.append(y)
                        z_arr.append(z)

                    ax.plot3D(x_arr, y_arr, z_arr, color='green', linewidth=1.2)

                x_arr = []
                y_arr = []
                z_arr = []
                for kin in current_path:
                    x, y, z = kin.get_position()
                    x_arr.append(x)
                    y_arr.append(y)
                    z_arr.append(z)

                ax.plot3D(x_arr, y_arr, z_arr, color='blue', linewidth=1.2)

                for kinematic_path in frontier:
                    x_arr = []
                    y_arr = []
                    z_arr = []
                    for p in kinematic_path:
                        x, y, z = p.get_position()
                        x_arr.append(x)
                        y_arr.append(y)
                        z_arr.append(z)

                    ax.plot3D(x_arr, y_arr, z_arr, color='green', linewidth=1.2)

                ax.set_xlabel('X-axis')
                ax.set_ylabel('Y-axis')
                ax.set_zlabel('Z-axis')
                ax.set_xlim([-20, 40])
                ax.set_ylim([-20, 40])
                ax.set_zlim([-20, 30])
                plt.legend()
                plt.show()

            print(str(len(temp_finished_paths)) + " complete paths found")
            print("-----------------------" + str(debug_counter) + "--------------------------")
            debug_counter += 1

            # When we are done processing the frontier
            # Check if we a have found any finished paths
            if len(temp_finished_paths) > 0:
                # Add the finished paths to the finilized paths
                finished_paths += temp_finished_paths

                # Remove all nodes along the paths from the graph
                for path in finished_paths:
                    for point in path:
                        x, y, z = point.get_position()
                        self.remove_node(x=x, y=y, z=z)

                # Reset the search
                beam_counter = np.inf
                frontier = []
                frontier.append([starting_state])
                continue

            # Only add the frontier if there is one
            if len(temp_frontier) > 0:

                # Add the new frontier to the frontier
                frontier += temp_frontier

                # Get the scores for each of the paths
                ranking_obj.update_paths(frontier)
                scores = ranking_obj.calculate_scores()

                # Sort the frontier based on path score
                # TODO: THIS IS THE WRONG WAY I WANT THE HIGHEST SCORE FIRST
                sorted_zipped_list = sorted(zip(scores, frontier))

                # Unzip the sorted list
                [scores, frontier] = list(zip(*sorted_zipped_list))
                scores = list(scores)
                frontier = list(frontier)

            # TO BE DELETED
            # Create the figure
            fig = plt.figure()
            ax = Axes3D(fig)

            # Create a list of standard nodes
            x_vals = []
            y_vals = []
            z_vals = []

            # Create a list of source and sink nodes
            source_x = []
            source_y = []
            source_z = []
            sink_x = []
            sink_y = []
            sink_z = []

            # For each node
            for node in self.V:
                # Get the x,y and z values
                x, y, z = node.get_position()

                # Check if this is a source
                if node.get_source():
                    # Save the position of the source node
                    source_x.append(x)
                    source_y.append(y)
                    source_z.append(z)
                # Check if this is a source
                elif node.get_sink():
                    # Save the position of the source node
                    sink_x.append(x)
                    sink_y.append(y)
                    sink_z.append(z)
                else:
                    # Save the positions or random nodes
                    x_vals.append(x)
                    y_vals.append(y)
                    z_vals.append(z)

            cons_x = []
            cons_y = []
            cons_z = []
            for c in considered:
                cons_x.append(c[0])
                cons_y.append(c[1])
                cons_z.append(c[2])

                item_to_remove = []
                for j in range(0, len(x_vals)):
                    if x_vals[j] == c[0] and y_vals[j] == c[1] and z_vals[j] == c[2]:
                        item_to_remove.append(j)

                for r in item_to_remove:
                    x_vals.pop(r)
                    y_vals.pop(r)
                    z_vals.pop(r)

            kx = []
            ky = []
            kz = []
            hull = None
            for points in kinematic_draw:
                try:
                    hull = ConvexHull(points)
                except:
                    hull = None
                for point in points:
                    kx.append(point[0])
                    ky.append(point[1])
                    kz.append(point[2])

            # Plot the values
            ax.scatter(source_x, source_y, source_z, c='g', label='Starting Position')
            ax.scatter(sink_x, sink_y, sink_z, c='r', label='Ending Position')
            ax.scatter(x_vals, y_vals, z_vals, c='b', label='Possible Waypoints')
            ax.scatter(cons_x, cons_y, cons_z, c='m', label='Processed Waypoints')

            for kinematic_path in frontier:
                x_arr = []
                y_arr = []
                z_arr = []
                for p in kinematic_path:
                    x, y, z = p.get_position()
                    x_arr.append(x)
                    y_arr.append(y)
                    z_arr.append(z)

                ax.plot3D(x_arr, y_arr, z_arr, color='green', linewidth=1.2)



            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')
            ax.set_xlim([-20, 40])
            ax.set_ylim([-20, 40])
            ax.set_zlim([-20, 30])
            plt.legend()
            plt.show()

        # put back all deleted verticies
        self.V = temp_v

        # Return the finished paths
        return finished_paths















# def getEdgesConnectedToVertex(self, vertex):
# 	edges_found = []
# 	# For each edge
# 	for edge in self.E:
# 		# If the edge is connected to the vertex
# 		if np.all(self.V[edge[0]] == vertex) or np.all(self.V[edge[1]] == vertex):
# 			edges_found.append(edge)
#
# 	return edges_found
#
# @lru_cache(maxsize=None)
# def getEdgesConnectedToVertexFromAjMatrix(self, vertex):
# 	edges_found = []
# 	for col in range(0, self.Ajmatrix.shape[1]):
# 		if self.Ajmatrix[vertex, col] != 0:
# 			# Convert it to an array so it functions like the other get Edges function
# 			#edges_found.append(np.array([vertex, col]))
# 			edges_found.append([vertex, col])
#
# 	return edges_found
#
# def findAllPaths(self, heading, min_turn_deg=-90, max_turn_deg=90, depth=100, max_traj=1000):
#
# 	# Check that the min and max turn are correct
# 	assert(min_turn_deg < max_turn_deg)
# 	assert(max_turn_deg <= 180)
# 	assert(min_turn_deg >= -180)
#
# 	# Create a stack containing the source index, the robot heading, the current path and the path headings.
# 	stack = [(self.source_index, heading, [self.source_index], [])]
#
# 	# Keeps track of the edges and path headings
# 	goal_paths = []
# 	goal_headings = []
#
# 	# Use DFS to explore all paths
# 	while stack:
# 		# Get the vertex and the path on top of the stack
# 		(vertex, robot_heading, path, path_heading) = stack.pop()
#
# 		# Dont consider paths greater than certain depth
# 		if len(path) > depth:
# 			continue
#
# 		# Get all the edges leading out of that vertex
# 		edges = self.getEdgesConnectedToVertexFromAjMatrix(vertex)
# 		# Removed for being super slow
# 		# edges1 = self.getEdgesConnectedToVertex(self.V[vertex])
#
# 		# Make sure each of the edges is ordered correctly
# 		for edge in edges:
# 			if edge[0] != vertex:
# 				next_vertex = edge[0]
# 			else:
# 				next_vertex = edge[1]
#
# 			# Check that the next vertex has not been visited on this path
# 			if next_vertex in path:
# 				# Skip everything after this
# 				continue
#
# 			# Get the line heading
# 			edge_angle = self.readLineHeadings(tuple([vertex, next_vertex]))
# 			# Read it from the saved array rather than recalculating
# 			# edge_angle = self.calculateLineHeading([vertex, next_vertex])
#
# 			# If the next vertex is within our reachability set
# 			delta_angle = edge_angle - robot_heading
# 			if delta_angle > 180:
# 				delta_angle -= 360
# 			if delta_angle < -180:
# 				delta_angle += 360
#
# 			if min_turn_deg <= delta_angle <= max_turn_deg:
# 				# If the next vertex is not the goal
# 				if next_vertex != self.end_index:
# 					stack.append((next_vertex, edge_angle, path + [next_vertex], path_heading + [edge_angle]))
# 				else:
# 					goal_paths.append(path + [next_vertex])
# 					goal_headings.append(path_heading + [edge_angle])
# 					if len(goal_paths) >= max_traj:
# 						# Stop Looking
# 						stack = False
# 						break
#
# 	# Add each of the trajectories to a trajectory class for easier processing
# 	final_trajectories = []
# 	for i in range(0, len(goal_paths)):
# 		waypoints = self.getWaypointsFromPath(goal_paths[i])
# 		traj = Trajectory()
# 		traj.set_waypoints(waypoints)
# 		traj.set_headings(goal_headings[i])
# 		final_trajectories.append(traj)
#
# 	return final_trajectories
#
# def calculateLineHeading(self, edge):
# 	# Get the x and y co-ordinates of each vertex
# 	[start_coord_x, start_coord_y] = self.V[edge[0]]
# 	[end_coord_x, end_coord_y] = self.V[edge[1]]
#
# 	# Use these-coordinates to calculate a heading
# 	dx = float(end_coord_x) - start_coord_x
# 	dy = float(end_coord_y) - start_coord_y
#
# 	# Calculate the heading between -180 and 180 (0 == East)
# 	heading = math.degrees(math.atan2(dy, dx))
#
# 	return heading
#
# @lru_cache(maxsize=None)
# def readLineHeadings(self, edge):
# 	return self.E_headings[edge[0], edge[1]]
#
# def generateLineHeadingsFromPath(self, path):
# 	# Check if the path is non empty
# 	if len(path) <= 0:
# 		return []
#
# 	# Keep track of the source so we can move in a constant direction
# 	src = 0
# 	edge_headings = []
#
# 	# First get all the edge indices
# 	edge_indices = self.getEdgeIndicesFromPath(path)
# 	for edge in edge_indices:
# 		# Get the correct vertex numbers
# 		if src == self.E[edge, 0]:
# 			edge_num = [self.E[edge, 0], self.E[edge, 1]]
# 			src = self.E[edge, 1]
# 		else:
# 			edge_num = [self.E[edge, 1], self.E[edge, 0]]
# 			src = self.E[edge, 0]
#
# 		# Calculate the heading between -180 and 180 (0 == East)
# 		#heading = self.calculateLineHeading(edge_num)
# 		heading = self.readLineHeadings(tuple(edge_num))
# 		edge_headings.append(heading)
#
# 	return edge_headings
#
# @lru_cache(maxsize=None)
# def getEdgeIndex(self, edge):
# 	# For each edge in all edges
# 	for edge_itt in range(0, len(self.E)):
# 		crit1 = (self.E[edge_itt, 0] == edge[0] and self.E[edge_itt, 1] == edge[1])
# 		crit2 = (self.E[edge_itt, 0] == edge[1] and self.E[edge_itt, 1] == edge[0])
# 		# If we have found matching edge
# 		if (crit1 or crit2):
# 			# Add that edge to the edge list
# 			return edge_itt
#
# def getEdgeIndicesFromPath(self, path):
#
# 	edge_number = []
# 	# For each edge in the path
# 	for i in range(1, len(path)):
# 		cur_edge = [path[i-1], path[i]]
# 		# Get the index from the current edge
# 		cur_index = self.getEdgeIndex(tuple(cur_edge))
# 		edge_number.append(cur_index)
#
# 	return edge_number
#
# def getPlot(self, highlighted_paths=[], path_class=[], tsuffix="", color_map="jet_r", figure_size=(15, 13)):
#
# 	plt.figure(figsize=figure_size)
#
# 	# Print Map
# 	if len(tsuffix) >= 0:
# 		plt.title('Occupancy Map - ' + str(tsuffix))
# 	else:
# 		plt.title('Occupancy Map')
# 	plt.imshow(self.map, cmap='binary')
# 	plt.xlim([0, self.map.shape[1]-1])
# 	plt.ylim([0, self.map.shape[0]-1])
#
# 	# Check if we have added more than the start and end vertex
# 	# Check if we have highlighted paths
# 	if (len(self.V) > 2) and highlighted_paths == []:
# 		# Print the random points and give them labels
# 		plt.scatter(self.V[:, 0], self.V[:, 1], s=2)
# 		labels = np.arange(len(self.V))
# 		labels = [str(i) for i in labels]
# 		for l in range(0, len(labels)):
# 			plt.text(self.V[l, 0]+ .03, self.V[l, 1]+.03, labels[l], fontsize=9)
#
#
# 		# Display the edges in the map
# 		for edge in range(0, len(self.E)):
# 			linex = [self.V[self.E[edge, 0], 0], self.V[self.E[edge, 1], 0]]
# 			liney = [self.V[self.E[edge, 0], 1], self.V[self.E[edge, 1], 1]]
# 			plt.plot(linex, liney, color='b', linestyle=":", linewidth=0.3)
#
# 	# Highlight the paths in the map
# 	i = 0
# 	# total_paths = len(highlighted_paths)
# 	# cmap = plt.get_cmap(color_map)
# 	for path in highlighted_paths:
# 		if path_class[i] == True:
# 			selected_color = 'green'
# 		else:
# 			selected_color = 'red'
# 		# selected_color = cmap(float(i) / total_paths)
# 		i += 1
#
# 		linex = []
# 		liney = []
# 		for v1, v2 in zip(path, path[1:]):
# 			if len(linex) == 0:
# 				linex = [self.V[v1, 0], self.V[v2, 0]]
# 				liney = [self.V[v1, 1], self.V[v2, 1]]
# 			else:
# 				linex.append(self.V[v2, 0])
# 				liney.append(self.V[v2, 1])
# 		plt.plot(linex, liney, color=selected_color)
#
# 	return plt
#
# def getWaypointsFromPath(self, path):
# 	# Keep track of the source so we can move in a constant direction
# 	src = 0
# 	verticies = []
#
# 	# First get all the edge indices
# 	edge_indices = self.getEdgeIndicesFromPath(path)
# 	for edge in edge_indices:
# 		# Get the correct vertex numbers
# 		if src == self.E[edge, 0]:
# 			edge_num = [self.E[edge, 0], self.E[edge, 1]]
# 			src = self.E[edge, 1]
# 		else:
# 			edge_num = [self.E[edge, 1], self.E[edge, 0]]
# 			src = self.E[edge, 0]
#
# 		# Get thoe x and y c-ordinates of the source vertex
# 		single_vertex = self.V[edge_num[0]]
# 		verticies.append(single_vertex)
#
# 	# Add the end vertex
# 	single_vertex = self.V[edge_num[1]]
# 	verticies.append(single_vertex)
#
# 	return verticies
#
# def getEdgeListFromPath(self, path):
# 	edgeList = []
# 	# Get the edge list
# 	for v1, v2 in zip(path, path[1:]):
# 		edgeList.append([v1, v2])
#
# 	return edgeList
