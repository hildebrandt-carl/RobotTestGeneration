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
    def populate_with_nodes(self, num_vertices):
        # Set the seed based on the time
        #random.seed(time.time())
        random.seed(9)

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
    def find_all_paths(self, drone_kinematic_values, kinematic_sample_resolution=5, total_waypoints=5, beam_width=1):
        # Make copies of vertices so they can be re-assigned after this function
        temp_v = copy.deepcopy(self.V)

        # Keeps tracks of the finished paths
        finished_paths = []

        # Create a path scorer to score each path
        ranking_obj = RankingSystem(paths=[])

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

            # Sort the current paths to be ordered based on score
            # You should have a temporary paths which is added after all nodes have been processed

            # Used to keep track of the new frontier and list of finished nodes
            temp_finished_paths = []
            temp_frontier = []

            # Used to track how many nodes on the frontier have been processed
            beam_counter = 0

            # Process beam width
            while (beam_counter < beam_width) and (len(frontier) > 0):

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
                    # Restart the beam search
                    continue

                # Create the reachability set generator object
                reachability_space_generator = DroneReachabilitySet(drone_kinematic=current_kinematic)

                # Calculate the reachable space for that drone kinematic in one time step
                positions = reachability_space_generator.calculate_reachable_area(sample_resolution=kinematic_sample_resolution)

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

            # When we are done processing the frontier
            # Check if we a have found any finished paths
            if len(temp_finished_paths) > 0:
                # Add the finished paths to the finilized paths
                finished_paths += temp_finished_paths
                print("Finished Paths Found: " + str(len(finished_paths)))

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
                scores, linear_scores, angular_scores = ranking_obj.calculate_scores()

                # Sort the frontier based on path score
                # Save them from smallest to largest as we take the item from the back of the queue
                sorted_zipped_list = sorted(zip(scores, frontier))

                # Unzip the sorted list
                [scores, frontier] = list(zip(*sorted_zipped_list))
                scores = list(scores)
                frontier = list(frontier)

        # put back all deleted verticies
        self.V = temp_v

        # Return the finished paths
        return finished_paths
