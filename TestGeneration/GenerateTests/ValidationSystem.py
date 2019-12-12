import os
import numpy as np
import warnings

from FigureManager import FigureManager
from DroneReachabilitySet import DroneReachabilitySet
from DroneKinematic import DroneKinematic
from UnityConverter import UnityConverter
from math import sqrt, pow, acos, degrees

class ValidationSystem:

    def __init__(self):
        pass

    # Given a set of trajectories and a kinematic model, this function validates that a trajectory is physically valid
    def validate_trajectories(self, trajectories, robot_kinematic_model, kinematic_sample_resolution):

        # Create a list of valid trajectories
        valid_trajs = []

        for traj in trajectories:
            # Assume the trajectory is valid
            valid = True

            # Get the first position and velocity
            previous_pos = traj[0].get_position()
            previous_vel = traj[0].get_velocity()

            # For each of the subsequent kinematic states in the trajectory
            for kinematic in traj[1:]:

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
                waypoint = np.column_stack((current_pos[0], current_pos[1], current_pos[2]))
                inside = reachability_space_generator.is_in_hull(waypoints=waypoint)

                # There should only be a single true value as we only consider the next waypoint so assert the list sums to at most 1
                assert(sum(inside) <= 1)

                # The next position is not inside the node break out of this loop
                if sum(inside) == 0:
                    valid = False

                # Save the new previous state for next iteration
                previous_pos = current_pos
                previous_vel = current_vel

            # If the trajectory is valid save it
            if valid:
                valid_trajs.append(traj)

        # Return valid trajectories
        return valid_trajs

    # Save the trajectories based on the score of the trajectories
    def save_trajectories_according_to_score(self, trajectories, scoring_function, folder):

        # Used to save to our unity format
        unity_converter = UnityConverter(save_directory=folder)

        # Get the score for each trajectory
        scores = scoring_function.calculate_scores(trajectories=trajectories)

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
            velocities = []
            in_vec = []
            out_vec = []
            in_vec_mag = []
            out_vec_mag = []
            euler_angles = []
            vec_angles = []

            cur_vel = None
            prev_vel = None

            # Get the waypoints and velocities for that path
            for w in trajectories[index]:
                # Get the current velocity
                cur_vel = w.get_velocity()

                waypoints.append(list(w.get_position()))
                velocity.append(list(cur_vel))
                angles.append(list(w.get_attitude()))

                # Get the previous vector
                if prev_vel is None:
                    cur_vel = [0, 0, 0.5]
                    prev_vel = [0, 0, 0]

                # Save the in and out vectors
                in_vec.append(prev_vel)
                out_vec.append(cur_vel)

                # Compute the magnitude of the vectors
                in_vec_mag.append(self.get_magnitude_vector(prev_vel))
                out_vec_mag.append(self.get_magnitude_vector(cur_vel))

                # Get angles between vectors
                euler_angles.append(self.euler_angles_between_vectors(prev_vel, cur_vel))
                vec_angles.append(self.angle_between_vectors(prev_vel, cur_vel))

                # Reset the input vector
                prev_vel = cur_vel

            # Create the file names
            save_directory = "maps/map" + str(path_counter) + "/"
            unity_file_name = save_directory + "test.txt"
            details_file_name = save_directory + "details.txt"

            # Check the save location is created
            if not os.path.isdir(folder + save_directory):
                # If not create it
                os.makedirs(folder + save_directory)

            # Write this information to file
            file = open(folder + details_file_name, "w")
            file.write("Path: " + str(index) + '\n')
            file.write("Save Location: " + unity_file_name + '\n')
            file.write("----------------------------\n")
            file.write("Path Total Score: " + str(scores[index]) + '\n')
            file.write("----------------------------\n")
            file.write("Waypoints: " + str(waypoints) + '\n')
            file.write("Velocity: " + str(velocity) + '\n')
            file.write("Attitude: " + str(angles) + '\n')
            file.write("----------------------------\n")
            file.write("Out Vector Magnitude: " + str(out_vec_mag) + '\n')
            file.write("In Vector Magnitude: " + str(in_vec_mag) + '\n')
            file.write("Out Vector: " + str(out_vec) + '\n')
            file.write("In Vector: " + str(in_vec) + '\n')
            file.write("Angles: " + str(vec_angles) + '\n')
            file.write("Euler Angles: " + str(euler_angles) + '\n')
            file.close()

            # Save the test to a unity file
            unity_converter.unity_text_file(waypoints=waypoints, expected_velocity=velocity, corridor=[], save_name=unity_file_name)

            # Create the plot and save it
            plt = fig_manager.plot_single_trajectory(waypoints)
            fig_manager.display_and_save(fig=plt,
                                         save_name="trajectory" + str(path_counter),
                                         save_directory=save_directory,
                                         only_save=True,
                                         figure_number=False)
            plt = fig_manager.plot_single_trajectory_top(waypoints)
            fig_manager.display_and_save(fig=plt,
                                         save_name="trajectory_top" + str(path_counter),
                                         save_directory=save_directory,
                                         only_save=True,
                                         figure_number=False)
            plt = fig_manager.plot_single_trajectory_XZ(waypoints)
            fig_manager.display_and_save(fig=plt,
                                         save_name="trajectory_xz" + str(path_counter),
                                         save_directory=save_directory,
                                         only_save=True,
                                         figure_number=False)
            plt = fig_manager.plot_single_trajectory_YZ(waypoints)
            fig_manager.display_and_save(fig=plt,
                                         save_name="trajectory_yz" + str(path_counter),
                                         save_directory=save_directory,
                                         only_save=True,
                                         figure_number=False)

    # Calculate the x,y and z angle between two vectors
    def euler_angles_between_vectors(self, a, b):
        axy = np.array([a[0], a[1]])
        bxy = np.array([b[0], b[1]])
        ayz = np.array([a[1], a[2]])
        byz = np.array([b[1], b[2]])
        axz = np.array([a[0], a[2]])
        bxz = np.array([b[0], b[2]])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            xangle = acos(np.dot(ayz, byz) / (np.linalg.norm(ayz) * np.linalg.norm(byz)))
            yangle = acos(np.dot(axz, bxz) / (np.linalg.norm(axz) * np.linalg.norm(bxz)))
            zangle = acos(np.dot(axy, bxy) / (np.linalg.norm(axy) * np.linalg.norm(bxy)))

        # Check if any of the angles is NaN. This happens when you have a 0 vector
        if np.isnan((xangle)):
            xangle = 0
        if np.isnan((yangle)):
            yangle = 0
        if np.isnan((zangle)):
            zangle = 0

        return [degrees(xangle), degrees(yangle), degrees(zangle)]

    # Calculate the magnitude ofa  vector
    def get_magnitude_vector(self, v1):
        mag = sqrt(pow(v1[0], 2) + pow(v1[1], 2) + pow(v1[2], 2))
        return mag

    # Returns a single angle in radians between vectors 'v1' and 'v2'
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    def angle_between_vectors(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # Returns the unit vector of the vector
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    def unit_vector(self, v):
        return v / np.linalg.norm(v)
