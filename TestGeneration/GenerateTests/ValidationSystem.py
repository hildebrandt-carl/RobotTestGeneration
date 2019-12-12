import os
import numpy as np

from FigureManager import FigureManager
from DroneReachabilitySet import DroneReachabilitySet
from DroneKinematic import DroneKinematic
from UnityConverter import UnityConverter

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
            # Get the waypoints and velocities for that path
            for w in trajectories[index]:
                waypoints.append(list(w.get_position()))
                velocity.append(list(w.get_velocity()))
                angles.append(list(w.get_attitude()))

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
            # file.write("Out Vector Magnitude: " + str(out_mag[index]) + '\n')
            # file.write("largest Out Vector Magnitude: " + str(largest_out_mag[index]) + '\n')
            # file.write("Angles: " + str(waypoint_angles[index]) + '\n')
            # file.write("Euler Angles: " + str(euler_angles[index]) + '\n')
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