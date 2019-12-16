import time
import random
import time
import math
import copy
import numpy as np

from Node import Node
from DroneKinematic import DroneKinematic
from DroneReachabilitySet import DroneReachabilitySet


class PRM:
    def __init__(self, start_pos, end_pos, start_time, map_x_bounds=[0, 1], map_y_bounds=[0, 1], map_z_bounds=[0, 1]):

        # Create the list of waypoints
        self.V = []

        # Create the time it was started
        self.start_time = start_time

        # Check the map bounds are correct
        if map_x_bounds[1] > map_x_bounds[0] and map_y_bounds[1] > map_y_bounds[0] and map_z_bounds[1] > map_z_bounds[
            0]:
            # Set the map bounds
            self.x_bounds = map_x_bounds
            self.y_bounds = map_y_bounds
            self.z_bounds = map_z_bounds
        else:
            # Exit as we do not know the type of drone
            sys.exit("Error 2: Map bounds passed to PRM do not make sense")

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
            sys.exit("Error 3: Unable to place the starting and ending waypoint in PRM")

    # Get the waypoints
    def get_waypoints(self):
        return self.V 

    # Get the sink vertex
    def get_sink_waypoint(self):
        # For each node
        for node in self.V:
            # If this is the sink node
            if node.get_sink():
                return node

    # Get the sink vertex
    def get_source_waypoint(self):
        # For each node
        for node in self.V:
            # If this is the sink node
            if node.get_source():
                return node

    # Used to populate the graph with a set of nodes in random positions
    def populate_with_nodes(self, num_vertices, input_seed=0):

        # If there is no seed set the seed to the current time
        if input_seed == 0:
            random.seed(time.time())
        else:
            random.seed(input_seed)

        # Iterate through the vertices
        for i in range(0, num_vertices - 2):

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
                sys.exit("Error 4: Unable to insert a node during PRM population phase")

        # Return that it worked
        return True

    # Finds paths from start vertex to end vertex using random search
    def find_all_paths_random(self, total_waypoints=5, beam_width=1, search_time=60):

        # Keeps track of the number of trajectories we explored
        total_explored_traj = 0

        # Keeps tracks of the complete trajectories
        complete_traj = []

        # Get the source waypoint and its position
        source_waypoint = self.get_source_waypoint()
        source_x, source_y, source_z = source_waypoint.get_position()

        # Create an initial state
        starting_state = [source_x, source_y, source_z]

        # Add this starting position to the frontier
        frontier = []
        frontier.append([starting_state])

        # Get the sink waypoints and position
        sink_waypoint = self.get_sink_waypoint()
        sink_x, sink_y, sink_z = sink_waypoint.get_position()

        counter = 0
        # While there are unfinished trajectories
        while len(frontier) > 0:

            # Used to keep track of the new frontier and list of finished waypoints
            temp_complete_traj = []
            temp_frontier = []

            # Used to track how many waypoints on the frontier have been processed
            beam_counter = 0

            # Process beam width (explore the frontier)
            while (beam_counter < beam_width) and (len(frontier) > 0):

                # Increment the beam counter
                beam_counter += 1

                # Get the current path
                current_traj = frontier.pop()

                # The robots current position is the last position in the trajectory
                current_pos = current_traj[-1]

                # Get the previous position
                previous_pos = None
                if len(current_traj) <= 1:
                    previous_pos = current_pos
                else:
                    previous_pos = current_traj[-2]

                # If the current waypoint is the sink waypoint and the length requirement is met
                found_end = (current_pos[0] == sink_x and current_pos[1] == sink_y and current_pos[2] == sink_z)
                correct_length = (len(current_traj) == total_waypoints)

                if found_end and correct_length:
                    # Add this path to the finished paths array
                    temp_complete_traj.append(current_traj)
                    total_explored_traj += 1

                    # Continue with beam search the beam search
                    continue

                # Used to save the x,y and z position of all next possible waypoints
                in_x = []
                in_y = []
                in_z = []

                # For each waypoint
                for waypoint in self.V:

                    # Get the x,y and z values
                    x, y, z = waypoint.get_position()

                    # Make sure the current waypoint is not in the graph
                    in_traj = False

                    # For each waypoint in the current path
                    for w in current_traj:

                        # Get that waypoints position
                        px, py, pz = w

                        # If this considered waypoint is in the trajectory
                        if x == px and y == py and z == pz:
                            in_traj = True

                    # If it is not in the trajectory, add them to the list of next possible waypoints
                    if not in_traj:
                        in_x.append(x)
                        in_y.append(y)
                        in_z.append(z)

                # Create a list of edges which would be traversed if each of the waypoints were visited
                # For each waypoint we could get to
                for j in range(0, len(in_x)):

                    # Create a new drone position for this waypoint
                    new_pos = [in_x[j], in_y[j], in_z[j]]

                    # create a new trajectory
                    new_traj = current_traj.copy()

                    # Add this new trajectory to the unfinished trajectory queue
                    new_traj.append(new_pos)

                    # If the trajectory is shorter than the requested length
                    if len(new_traj) <= total_waypoints:
                        temp_frontier.append(new_traj)
                    else:
                        total_explored_traj += 1
            
            #----------------------------------------------------------------------
            # At this point we are done with a single iteration of process frontier
            # We have also checked that any complete trajectories found in the exploration are added to the list "temp_complete_traj"
            #----------------------------------------------------------------------

            # Check if we have any trajectories to add
            if len(temp_complete_traj) > 0:

                # Add the newly found complete trajectories to the list of complete trajectories
                complete_traj += temp_complete_traj

                # Print out the time these paths were added
                print("Found " + str(len(temp_complete_traj)) + " new complete trajectories")
                print("Time passed since start of PRM search: " + str(time.time() - self.start_time ))

                # Print out details about the search
                print("Current number of complete trajectories found: " + str(len(complete_traj)))

                # At this point we want to reset the PRM graph
                beam_counter = np.inf
                frontier = []
                break

            # Only add the frontier if there is one to add
            if len(temp_frontier) > 0:

                # Add the new frontier to the frontier
                frontier += temp_frontier

                # Assign a random score to each of the trajectories in the frontiers as this is just random
                scores = []
                for trajectory in frontier:
                    # Trajectory
                    path_score = 0

                    # For each waypoint in the trajectory
                    for wpoint in trajectory:
                        # Add a random value between 0 and 1 to the score
                        path_score += random.uniform(0, 1)

                    # Save the score
                    scores.append(path_score)

                # Sort the frontier based on path score
                # Save them from smallest to largest as we take the item from the back of the queue
                sorted_zipped_list = sorted(zip(scores, frontier))

                # Unzip the sorted list
                [scores, frontier] = list(zip(*sorted_zipped_list))
                scores = list(scores)
                frontier = list(frontier)

            # Check if you have run out of time
            if (time.time() - self.start_time ) > search_time:
                print("Exiting PRM as we are out of time")
                beam_counter = np.inf
                frontier = []
                break

        # We need to return a list of kinematics so turn the trajectories into kinematic models as the rest of the code expects it in that format
        final_trajectories = []

        # For each trajectory in the list of complete trajectories
        for t in complete_traj:

            # Create a kinematic trajectory represetnation
            kinematic_traj = []

            # Get the starting state
            start_state = DroneKinematic(mass=math.inf,
                                         arm_length=math.inf,
                                         thrust_constant=math.inf,
                                         moment_constant=math.inf,
                                         max_rotor_speed=math.inf,
                                         inertial_properties=[math.inf, math.inf, math.inf],
                                         position=[t[0][0], t[0][1], t[0][2]],
                                         attitude=[0, 0, 0],
                                         velocity=[0, 0, 0],
                                         angular_vel=[0, 0, 0])
        
            # Save the kinematic starting state
            kinematic_traj.append(start_state)

            # For each segment of the trajectory assume the velocity along the axis is the same as the distance
            # This assumption holds for t=1
            for p1, p2 in zip(t, t[1:]):
                velx = p2[0] - p1[0]
                vely = p2[1] - p1[1]
                velz = p2[2] - p1[2]

                # Assume the robot enters each waypoint level
                next_state = DroneKinematic(mass=math.inf,
                                            arm_length=math.inf,
                                            thrust_constant=math.inf,
                                            moment_constant=math.inf,
                                            max_rotor_speed=math.inf,
                                            inertial_properties=[math.inf, math.inf, math.inf],
                                            position=[p2[0], p2[1], p2[2]],
                                            attitude=[0, 0, 0],
                                            velocity=[velx, vely, velz],
                                            angular_vel=[0, 0, 0])

                # Save the kinematic model
                kinematic_traj.append(next_state)
        
            # Save this trajectory to the kinematic trajectory representation
            final_trajectories.append(kinematic_traj)

        # Print the search time
        print("Time taken to find " + str(len(final_trajectories)) + " trajectories: " + str(time.time() - self.start_time ))
        print("Total complete trajectories returned this run: " + str(len(final_trajectories)))
        print("Total explored trajectories during this run: " + str(total_explored_traj))

        # Return the finished trajectories
        return final_trajectories, total_explored_traj

    # Finds paths from start vertex to end vertex which satisfy the maximum velocity of the drone
    def find_all_paths_maxvel(self, max_velocity=1, total_waypoints=5, beam_width=1, search_time=60):

        # Keeps track of the number of trajectories we explored
        total_explored_traj = 0

        # Keeps tracks of the complete trajectories
        complete_traj = []

        # Get the source waypoint and its position
        source_waypoint = self.get_source_waypoint()
        source_x, source_y, source_z = source_waypoint.get_position()

        # Create an initial state
        starting_state = [source_x, source_y, source_z]

        # Add this starting position to the frontier
        frontier = []
        frontier.append([starting_state])

        # Get the sink waypoints and position
        sink_waypoint = self.get_sink_waypoint()
        sink_x, sink_y, sink_z = sink_waypoint.get_position()

        counter = 0
        # While there are unfinished trajectories
        while len(frontier) > 0:

            # Used to keep track of the new frontier and list of finished waypoints
            temp_complete_traj = []
            temp_frontier = []

            # Used to track how many waypoints on the frontier have been processed
            beam_counter = 0

            # Process beam width (explore the frontier)
            while (beam_counter < beam_width) and (len(frontier) > 0):

                # Increment the beam counter
                beam_counter += 1

                # Get the current path
                current_traj = frontier.pop()

                # The robots current position is the last position in the trajectory
                current_pos = current_traj[-1]

                # Get the previous position
                previous_pos = None
                if len(current_traj) <= 1:
                    previous_pos = current_pos
                else:
                    previous_pos = current_traj[-2]

                # If the current waypoint is the sink waypoint and the length requirement is met
                found_end = (current_pos[0] == sink_x and current_pos[1] == sink_y and current_pos[2] == sink_z)
                correct_length = (len(current_traj) == total_waypoints)

                if found_end and correct_length:
                    # Add this path to the finished paths array
                    temp_complete_traj.append(current_traj)
                    total_explored_traj += 1

                    # Continue with beam search the beam search
                    continue

                # Used to save the x,y and z position of all next possible waypoints, i.e inside the maximum velocity
                in_x = []
                in_y = []
                in_z = []

                # For each waypoint
                for waypoint in self.V:

                    # Get the x,y and z values
                    x, y, z = waypoint.get_position()

                    # Compute the difference between the current waypoint and the waypoint we are considering
                    dx = x - current_pos[0]
                    dy = y - current_pos[1]
                    dz = z - current_pos[2]

                    # Calculate the distance to the current waypoint
                    distance_to_waypoint = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                    # If the distance to the new waypoint is less than maximum_velocity
                    if distance_to_waypoint <= max_velocity:

                        # Make sure the current waypoint is not in the graph
                        in_traj = False

                        # For each waypoint in the current path
                        for w in current_traj:

                            # Get that waypoints position
                            px, py, pz = w

                            # If this considered waypoint is in the trajectory
                            if x == px and y == py and z == pz:
                                in_traj = True

                        # If it is not in the trajectory, add them to the list of next possible waypoints
                        if not in_traj:
                            in_x.append(x)
                            in_y.append(y)
                            in_z.append(z)

                # Create a list of edges which would be traversed if each of the waypoints were visited
                # For each waypoint we could get to
                for j in range(0, len(in_x)):

                    # Create a new drone position for this waypoint
                    new_pos = [in_x[j], in_y[j], in_z[j]]

                    # create a new trajectory
                    new_traj = current_traj.copy()

                    # Add this new trajectory to the unfinished trajectory queue
                    new_traj.append(new_pos)

                    # If the trajectory is shorter than the requested length
                    if len(new_traj) <= total_waypoints:
                        temp_frontier.append(new_traj)
                    else:
                        total_explored_traj += 1

            #----------------------------------------------------------------------
            # At this point we are done with a single iteration of process frontier
            # We have also checked that any complete trajectories found in the exploration are added to the list "temp_complete_traj"
            #----------------------------------------------------------------------

            # Check if we have any trajectories to add
            if len(temp_complete_traj) > 0:

                # Add the newly found complete trajectories to the list of complete trajectories
                complete_traj += temp_complete_traj

                # Print out the time these paths were added
                print("Found " + str(len(temp_complete_traj)) + " new complete trajectories")
                print("Time passed since start of PRM search: " + str(time.time() - self.start_time ))

                # Print out details about the search
                print("Current number of complete trajectories found: " + str(len(complete_traj)))

                # At this point we want to reset the PRM graph
                beam_counter = np.inf
                frontier = []
                break

            # Only add the frontier if there is one to add
            if len(temp_frontier) > 0:

                # Add the new frontier to the frontier
                frontier += temp_frontier

                # Assign a random score to each of the trajectories in the frontiers as this is just random
                scores = []
                for trajectory in frontier:
                    # Trajectory
                    path_score = 0

                    # For each waypoint in the trajectory
                    for wpoint in trajectory:
                        # Add a random value between 0 and 1 to the score
                        path_score += random.uniform(0, 1)

                    # Save the score
                    scores.append(path_score)

                # Sort the frontier based on path score
                # Save them from smallest to largest as we take the item from the back of the queue
                sorted_zipped_list = sorted(zip(scores, frontier))

                # Unzip the sorted list
                [scores, frontier] = list(zip(*sorted_zipped_list))
                scores = list(scores)
                frontier = list(frontier)

            # Check if you have run out of time
            if (time.time() - self.start_time ) > search_time:
                print("Exiting PRM as we are out of time")
                beam_counter = np.inf
                frontier = []
                break

        # We need to return a list of kinematics so turn the trajectories into kinematic models as the rest of the code expects it in that format
        final_trajectories = []

        # For each trajectory in the list of complete trajectories
        for t in complete_traj:

            # Create a kinematic trajectory represetnation
            kinematic_traj = []

            # Get the starting state
            start_state = DroneKinematic(mass=math.inf,
                                         arm_length=math.inf,
                                         thrust_constant=math.inf,
                                         moment_constant=math.inf,
                                         max_rotor_speed=math.inf,
                                         inertial_properties=[math.inf, math.inf, math.inf],
                                         position=[t[0][0], t[0][1], t[0][2]],
                                         attitude=[0, 0, 0],
                                         velocity=[0, 0, 0],
                                         angular_vel=[0, 0, 0])
        
            # Save the kinematic starting state
            kinematic_traj.append(start_state)

            # For each segment of the trajectory assume the velocity along the axis is the same as the distance
            # This assumption holds for t=1
            for p1, p2 in zip(t, t[1:]):
                velx = p2[0] - p1[0]
                vely = p2[1] - p1[1]
                velz = p2[2] - p1[2]

                # Assume the robot enters each waypoint level
                next_state = DroneKinematic(mass=math.inf,
                                            arm_length=math.inf,
                                            thrust_constant=math.inf,
                                            moment_constant=math.inf,
                                            max_rotor_speed=math.inf,
                                            inertial_properties=[math.inf, math.inf, math.inf],
                                            position=[p2[0], p2[1], p2[2]],
                                            attitude=[0, 0, 0],
                                            velocity=[velx, vely, velz],
                                            angular_vel=[0, 0, 0])

                # Save the kinematic model
                kinematic_traj.append(next_state)
        
            # Save this trajectory to the kinematic trajectory representation
            final_trajectories.append(kinematic_traj)

        # Print the search time
        print("Time taken to find " + str(len(final_trajectories)) + " trajectories: " + str(time.time() - self.start_time ))
        print("Total complete trajectories returned this run: " + str(len(final_trajectories)))
        print("Total explored trajectories during this run: " + str(total_explored_traj))

        # Return the finished trajectories
        return final_trajectories, total_explored_traj

    # Finds paths from start vertex to end vertex which satisfy the kinematic model and scoring criteria
    def find_all_paths_kinematic(self, robot_kinematic_model, kinematic_sample_resolution=5, total_waypoints=5, beam_width=1, search_time=60, score_func=None):
        # If no gen_type is added exit
        if score_func == None:
            sys.exit("Error 6: No scoring function given to path generation with scoring")

        # Keeps track of the number of files processed
        total_process_paths = 0

        # Keeps track of the number of trajectories we explored
        total_explored_traj = 0

        # Keeps tracks of the complete trajectories
        complete_traj = []

        # Create a trajectory scorer object
        ranking_obj = copy.deepcopy(score_func)

        # Create a drone kinematic model at the starting position
        # Create an initial state
        starting_state = DroneKinematic(mass=robot_kinematic_model['m'],
                                        arm_length=robot_kinematic_model['d'],
                                        thrust_constant=robot_kinematic_model['kf'],
                                        moment_constant=robot_kinematic_model['km'],
                                        max_rotor_speed=robot_kinematic_model['max_rotor_speed'],
                                        inertial_properties=robot_kinematic_model['inertial_properties'],
                                        position=robot_kinematic_model['position'],
                                        attitude=robot_kinematic_model['attitude'],
                                        velocity=robot_kinematic_model['velocity'],
                                        angular_vel=robot_kinematic_model['angular_velocity'])

        # Add this starting position to the frontier
        frontier = []
        frontier.append([starting_state])

        # Get the sink waypoints and position
        sink_waypoint = self.get_sink_waypoint()
        sink_x, sink_y, sink_z = sink_waypoint.get_position()

        counter = 0
        # While there are unfinished trajectories
        while len(frontier) > 0:

            # Used to keep track of the new frontier and list of finished waypoints
            temp_complete_traj = []
            temp_frontier = []

            # Used to track how many waypoints on the frontier have been processed
            beam_counter = 0

            # Process beam width (explore the frontier)
            while (beam_counter < beam_width) and (len(frontier) > 0):

                # Increment the beam counter
                beam_counter += 1

                # Get the current path
                current_traj = frontier.pop()

                # The current kinematic is the last kinematic in the path
                current_kinematic = current_traj[-1]

                # Get the previous kinematic model
                previous_kinematic = None
                if len(current_traj) <= 1:
                    previous_kinematic = current_kinematic
                else:
                    previous_kinematic = current_traj[-2]

                # Get the current kinematics position
                current_pos = current_kinematic.get_position()
                previous_pos = previous_kinematic.get_position()

                # If the current waypoint is the sink waypoint and the length requirement is met
                found_end = (current_pos[0] == sink_x and current_pos[1] == sink_y and current_pos[2] == sink_z)
                correct_length = (len(current_traj) == total_waypoints)

                if found_end and correct_length:
                    # Add this path to the finished paths array
                    temp_complete_traj.append(current_traj)
                    total_explored_traj += 1

                    # Continue with beam search the beam search
                    continue

                # Create the reachability set generator object
                reachability_space_generator = DroneReachabilitySet(robot_kinematic=current_kinematic)

                # Calculate the reachable space for that drone kinematic in one time step
                positions = reachability_space_generator.calculate_reachable_area(
                    sample_resolution=kinematic_sample_resolution)

                # Compute the maximum distance the drone can travel from this point
                current_kinematic.calculate_maximum_velocity(positions)

                # Used to save the x,y and z position of all next possible waypoints, i.e all waypoints not in the path
                considered_x = []
                considered_y = []
                considered_z = []

                # For each waypoint
                for waypoint in self.V:
                    
                    # Get the x,y and z values
                    x, y, z = waypoint.get_position()

                    # Make sure the current waypoint is not in the graph
                    in_traj = False

                    # For each waypoint in the current path
                    for w in current_traj:

                        # Get that waypoints position
                        px, py, pz = w.get_position()

                        # If this considered waypoint is in the trajectory
                        if x == px and y == py and z == pz:
                            in_traj = True

                    # If it is not in the trajectory, add them to the list of next possible waypoints
                    if not in_traj:
                        considered_x.append(x)
                        considered_y.append(y)
                        considered_z.append(z)

                # For all waypoints find which are inside the reachability set
                waypoints = np.column_stack((considered_x, considered_y, considered_z))
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
                        not_same_pos = (waypoints[i][0] != current_pos[0] and waypoints[i][1] != current_pos[1] and
                                        waypoints[i][2] != current_pos[2])
                        not_prev_pos = (waypoints[i][0] != previous_pos[0] and waypoints[i][1] != previous_pos[1] and
                                        waypoints[i][2] != previous_pos[2])

                        # Save the waypoint
                        if not_same_pos and not_prev_pos:
                            in_x.append(waypoints[i][0])
                            in_y.append(waypoints[i][1])
                            in_z.append(waypoints[i][2])

                # Create a list of edges which would be traversed if each of the waypoints were visited
                # For each of the waypoints inside the hull
                for j in range(0, len(in_x)):

                    # Find the vector between them (This is the represents velocity)
                    vector_x = in_x[j] - current_pos[0]
                    vector_y = in_y[j] - current_pos[1]
                    vector_z = in_z[j] - current_pos[2]

                    # Calculate the magnitude of the vector
                    # magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)

                    # Calculate the angle of that vector
                    # r = math.acos(vector_x / magnitude)
                    # p = math.acos(vector_y / magnitude)
                    # y = math.acos(vector_z / magnitude)

                    # We are going to assume the drone enters the waypoint level
                    r = 0
                    p = 0
                    y = 0

                    # Create a new drone kinematic for this waypoint
                    new_kinematic = DroneKinematic(mass=robot_kinematic_model['m'],
                                                   arm_length=robot_kinematic_model['d'],
                                                   thrust_constant=robot_kinematic_model['kf'],
                                                   moment_constant=robot_kinematic_model['km'],
                                                   max_rotor_speed=robot_kinematic_model['max_rotor_speed'],
                                                   inertial_properties=robot_kinematic_model['inertial_properties'],
                                                   position=[in_x[j], in_y[j], in_z[j]],
                                                   attitude=[r, p, y],
                                                   velocity=[vector_x, vector_y, vector_z],
                                                   angular_vel=robot_kinematic_model['angular_velocity'])

                    # create a new path
                    new_traj = current_traj.copy()

                    # Add this new path to the unfinished paths queue
                    new_traj.append(new_kinematic)

                    # If the trajectory is shorter than the requested length
                    if len(new_traj) <= total_waypoints:
                        temp_frontier.append(new_traj)
                    else:
                        total_explored_traj += 1

            #----------------------------------------------------------------------
            # At this point we are done with a single iteration of process frontier
            # We have also checked that any complete trajectories found in the exploration are added to the list "temp_complete_traj"
            #----------------------------------------------------------------------

            # Check if we have any trajectories to add
            if len(temp_complete_traj) > 0:

                # Add the newly found complete trajectories to the list of complete trajectories
                complete_traj += temp_complete_traj

                # Print out the time these paths were added
                print("Found " + str(len(temp_complete_traj)) + " new complete trajectories")
                print("Time passed since start of PRM search: " + str(time.time() - self.start_time ))

                # Print out details about the search
                print("Current number of complete trajectories found: " + str(len(complete_traj)))

                # At this point we want to reset the PRM graph
                beam_counter = np.inf
                frontier = []
                break

            # Only add the frontier if there is one
            if len(temp_frontier) > 0:
                
                # Add the new frontier to the frontier
                frontier += temp_frontier

                # Get the scores for each of the trajectories
                scores = ranking_obj.calculate_scores(trajectories=frontier)

                # Create a list of unique values. This list will be used to sort scores which have the same value
                unique_list = np.arange(0, len(scores), 1)

                # Sort the frontier based on trajectory score
                # Save them from smallest to largest as we take the item from the back of the queue
                sorted_zipped_list = sorted(zip(scores, unique_list, frontier))

                # Unzip the sorted list
                [scores, _, frontier] = list(zip(*sorted_zipped_list))
                scores = list(scores)
                frontier = list(frontier)

            # Check if you have run out of time
            if (time.time() - self.start_time ) > search_time:
                print("Exiting PRM as we are out of time")
                beam_counter = np.inf
                frontier = []
                break

        # We dont need to convert anything to kinematic form as it is already in that form therefore
        final_trajectories = complete_traj

        # Print the search time
        print("Time taken to find " + str(len(final_trajectories)) + " trajectories: " + str(time.time() - self.start_time ))
        print("Total complete trajectories returned this run: " + str(len(final_trajectories)))
        print("Total explored trajectories during this run: " + str(total_explored_traj))

        # Return the finished trajectories
        return final_trajectories, total_explored_traj
