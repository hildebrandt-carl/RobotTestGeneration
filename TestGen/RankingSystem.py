from FigureManager import FigureManager
from UnityConverter import UnityConverter
from MavlinkConverter import MavlinkConverter
from DroneReachabilitySet import DroneReachabilitySet
from DroneKinematic import DroneKinematic
from math import sqrt, radians
import numpy as np
import os
import random

class RankingSystem:

    def __init__(self):
        pass

    # Calculate the scores
    def calculate_scores(self, paths):
        # Used to save the scores of each path
        scores = []
        linear_scores = []
        angular_scores = []

        # For each path
        for path in paths:
            maximum_vel_magnitude = []
            velocities = []
            positions = []

            # Used to save the path score
            linear_path_score = []
            angular_path_score = []
            point_path_score = []

            # For each kinematic
            for point in path:
                # Get the current position
                positions.append(point.get_position())

                # Get the final distance flown in 1 time step
                current_velocity = point.get_velocity()
                velocities.append(current_velocity)

                # Get the maximum velocity you could reach leaving that point
                maximum_vel_magnitude.append(point.get_maximum_velocity())

            point_score = []

            # Calculate the score at each waypoint
            for score_counter in range(0, len(velocities) - 1):
                # Get the vectors going into and out of a waypoint
                in_vec = velocities[score_counter]
                out_vec = velocities[score_counter + 1]

                # If we are on the first point there so set the vectors such that the score is 0.5
                prev_largest_mag = None
                if score_counter == 0:
                    prev_largest_mag = 1
                    # If its the first point assume the vector is straight up (as it is hovering fighting gravity)
                    # This is done so we can calculate the angle relative to it
                    in_vec = [0, 0, 0.5]
                else:
                    # Get the largest magnitude at that waypoint
                    prev_largest_mag = maximum_vel_magnitude[score_counter - 1]

                # Calculate linear score
                in_vec_magnitude = self.get_magnitude_vector(in_vec)
                linear_score = in_vec_magnitude / float(prev_largest_mag)
                linear_path_score.append(linear_score)

                # Calculate the angle between vectors
                angle = self.angle_between_vectors(in_vec, out_vec)
                angular_vel_score = angle / radians(180)
                angular_path_score.append(angular_vel_score)

                # Calculate the point score
                point_path_score.append(linear_score * angular_vel_score)

            # Save the scores
            linear_scores.append(sum(linear_path_score))
            angular_scores.append(sum(angular_path_score))
            scores.append(sum(point_path_score))

        # Confirm that each path a linear and angular score
        assert (len(linear_scores) == len(paths))
        assert (len(angular_scores) == len(paths))
        assert (len(scores) == len(paths))

        return scores, linear_scores, angular_scores

    def validate_paths(self, paths, robot_kinematic_model, kinematic_sample_resolution):

        valid_paths = []

        for path in paths:
            # Assume the path is valid
            valid_path = True

            # Get the previous position and velocity
            previous_pos = path[0].get_position()
            previous_vel = path[0].get_velocity()
            for kinematic in path[1:]:
                # Get the current position and velocity
                current_pos = kinematic.get_position()
                current_vel = kinematic.get_velocity()

                # Create the previous kinematic model
                previous_kinematic = DroneKinematic(mass=robot_kinematic_model['m'],
                                                    arm_length=robot_kinematic_model['d'],
                                                    thrust_constant=robot_kinematic_model['kf'],
                                                    moment_constant=robot_kinematic_model['km'],
                                                    max_rotor_speed=robot_kinematic_model['max_rotor_speed'],
                                                    inertial_properties=robot_kinematic_model['inertial_properties'],
                                                    position=previous_pos,
                                                    attitude=robot_kinematic_model['attitude'],
                                                    velocity=previous_vel,
                                                    angular_vel=robot_kinematic_model['angular_velocity'])

                # Create the reachability set generator object
                reachability_space_generator = DroneReachabilitySet(robot_kinematic=previous_kinematic)

                # Calculate the reachable space for that drone kinematic in one time step
                positions = reachability_space_generator.calculate_reachable_area(sample_resolution=kinematic_sample_resolution)

                # Check the current waypoint is inside
                waypoints = np.column_stack((current_pos[0], current_pos[1], current_pos[2]))
                inside = reachability_space_generator.is_in_hull(waypoints=waypoints)

                # There should only be a single true value so assert the list sums to at most 1
                assert(sum(inside) <= 1)

                # The next position is not inside the node break out of this loop
                if sum(inside) == 0:
                    valid_path = False

                # Save the new previous state for next iteration
                previous_pos = current_pos
                previous_vel = current_vel

            if valid_path:
                valid_paths.append(path)

        return valid_paths

    # Save the paths to a directory
    def save_trajectories_according_to_score(self, paths, folder):
        # Create the converter class's
        unity_converter = UnityConverter(save_directory=folder)
        mavlink_converter = MavlinkConverter(save_directory=folder)

        # Get the scores
        scores, _, _ = self.calculate_scores(paths=paths)

        # Sort the scores and get the new indices
        score_indicies = list(np.argsort(scores))

        # Reverse the scores so that its from largest to smallest
        score_indicies.reverse()

        # Create the Figure manager
        fig_manager = FigureManager(folder)

        path_counter = 0
        for index in score_indicies:
            path_counter += 1
            waypoints = []
            velocity = []
            angles = []
            # Get the waypoints and velocities for that path
            for each_point in paths[index]:
                waypoints.append(list(each_point.get_position()))
                velocity.append(list(each_point.get_velocity()))
                angles.append(list(each_point.get_attitude()))

            # Create the file names
            save_directory = "maps/map" + str(path_counter) + "/"
            unity_file_name = save_directory + "test.txt"
            details_file_name = save_directory + "details.txt"
            mavros_file_name = save_directory + "flightplan.mavlink"

            # Check the save location is created
            if not os.path.isdir(folder + save_directory):
                # If not create it
                os.makedirs(folder + save_directory)

            # Write this information to file
            file = open(folder + details_file_name, "w")
            file.write("Path: " + str(index) + '\n')
            file.write("Path Total Score: " + str(scores[index]) + '\n')
            file.write("Waypoints: " + str(waypoints) + '\n')
            file.write("Velocity: " + str(velocity) + '\n')
            file.write("Attitude: " + str(angles) + '\n')
            file.write("Save Location: " + unity_file_name + '\n')
            file.close()

            # Save the test to a unity file
            unity_converter.unity_text_file(waypoints=waypoints, expected_velocity=velocity, corridor=[], save_name=unity_file_name)

            # Save the test to a mavlink waypoint file
            mavlink_converter.mavlink_waypoint_file(waypoints=waypoints, save_name=mavros_file_name)

            # Create the plot and save it
            plt = fig_manager.plot_single_trajectory(waypoints)
            fig_manager.display_and_save(fig=plt,
                                         save_name="trajectory" + str(path_counter),
                                         save_directory=save_directory,
                                         only_save=True,
                                         figure_number=False)

    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    # Returns the unit vector of the vector
    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    # Returns the angle in radians between vectors 'v1' and 'v2'
    def angle_between_vectors(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def get_magnitude_vector(self, v1):
        mag = sqrt(pow(v1[0], 2) + pow(v1[1], 2) + pow(v1[2], 2))
        return mag