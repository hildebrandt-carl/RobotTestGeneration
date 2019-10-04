from FigureManager import FigureManager
from UnityConverter import UnityConverter
from MavlinkConverter import MavlinkConverter
from DroneReachabilitySet import DroneReachabilitySet
from DroneKinematic import DroneKinematic
from math import sqrt, radians, degrees, acos
from sklearn.preprocessing import PolynomialFeatures
import numpy as np
import os
import random
import warnings

class RankingSystem:

    def __init__(self, random_scoring=False):
        self.random_scoring = random_scoring
        pass

    # Calculate the scores
    def calculate_scores(self, paths, gen_type="waypoint", all_points=False):

        poly_features = []
        poly_reg = []

        # Used to calculate the scores
        if gen_type == "waypoint":
            poly_features = np.load("Models/waypoint_poly_features.npy", allow_pickle=True).item()
            poly_reg = np.load("Models/waypoint_regression_model.npy", allow_pickle=True).item()
        elif gen_type == "constant":
            poly_features = np.load("Models/constant_poly_features.npy", allow_pickle=True).item()
            poly_reg = np.load("Models/constant_regression_model.npy", allow_pickle=True).item()
        else:
            exit()

        # Used to save the scores of each path
        scores = []
        linear_scores = []
        angular_scores = []

        # Used to save the score for each point in the path
        linear_score_points = []
        angular_score_points = []
        final_score_points = []

        # Generate a random score for each trajectory
        if self.random_scoring:
            for path in paths:

                # Generate a series of path scores which are random
                linear_path_score = []
                angular_path_score = []
                point_path_score = []
                for point in path:
                    # Generate a random score for both the linear and angular component
                    linear_score = random.uniform(0, 1)
                    angular_vel_score = random.uniform(0, 1)
                    # Save those scores
                    linear_path_score.append(linear_score)
                    angular_path_score.append(angular_vel_score)
                    # Save the final score
                    point_path_score.append(linear_score * angular_vel_score)

                # Save the scores for each point in the path
                linear_score_points.append(linear_path_score)
                angular_score_points.append(angular_path_score)
                final_score_points.append(point_path_score)

                # Save the scores
                linear_scores.append(sum(linear_path_score))
                angular_scores.append(sum(angular_path_score))
                scores.append(sum(point_path_score))

        # Generate a score according to our metric
        else:
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

                    # Calculate the angle
                    euler_angles = self.euler_angles_between_vectors(in_vec, out_vec)

                    # Used to create our sample input
                    vel_x_out = out_vec[0]
                    vel_y_out = out_vec[1]
                    vel_z_out = out_vec[2]
                    vel_x_in = in_vec[0]
                    vel_y_in = in_vec[1]
                    vel_z_in = in_vec[2]
                    ang_x = euler_angles[0]
                    ang_y = euler_angles[1]
                    ang_z = euler_angles[2]

                    # Create the sample data
                    sample_data = np.array([ang_x, ang_y, ang_z, vel_x_out, vel_y_out, vel_z_out, vel_x_in, vel_y_in, vel_z_in]).reshape(1, -1)
                    sample_data_poly = poly_features.fit_transform(sample_data)

                    poly_predict = poly_reg.predict(sample_data_poly)


                    # Angular and linear score not used anymore
                    linear_path_score.append(-1)
                    angular_path_score.append(-1)

                    # Save the final score
                    point_path_score.append(poly_predict.item())

                # Save the scores for each point in the path
                linear_score_points.append(linear_path_score)
                angular_score_points.append(angular_path_score)
                final_score_points.append(point_path_score)

                # Save the scores
                linear_scores.append(sum(linear_path_score))
                angular_scores.append(sum(angular_path_score))
                scores.append(sum(point_path_score))

        # Confirm that each path a linear and angular score
        assert (len(linear_scores) == len(paths))
        assert (len(angular_scores) == len(paths))
        assert (len(scores) == len(paths))


        # If we want to return all the points or just the final points
        if all_points:
            return final_score_points, linear_score_points, angular_score_points
        else:
            return scores, linear_scores, angular_scores

    def calculate_path_details(self, paths):

        # Used to save the scores of each path
        final_maximum_out_mag_array = []
        final_out_mag_array = []
        final_angle_array = []
        final_euler_angle_array = []

        # For each path
        for path in paths:
            maximum_vel_magnitude = []
            velocities = []
            positions = []

            # Used to save the scores of each path
            maximum_out_mag_array = []
            out_mag_array = []
            angle_array = []
            euler_angle_array = []

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

                # Save components used in the linear score
                out_vec_magnitude = self.get_magnitude_vector(out_vec)
                largest_mag = maximum_vel_magnitude[score_counter]
                maximum_out_mag_array.append(largest_mag)
                out_mag_array.append(out_vec_magnitude)

                # Score is calculated around a certain point
                angle = self.angle_between_vectors(in_vec, out_vec)
                euler_angle = self.euler_angles_between_vectors(in_vec, out_vec)

                # Save components used in the angular score
                angle_array.append(degrees(angle))
                euler_angle_array.append(euler_angle)

            # Save the path details
            final_maximum_out_mag_array.append(maximum_out_mag_array)
            final_out_mag_array.append(out_mag_array)
            final_angle_array.append(angle_array)
            final_euler_angle_array.append(euler_angle_array)

        # Return details about the path
        return final_out_mag_array, final_maximum_out_mag_array, final_angle_array, final_euler_angle_array

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
    def save_trajectories_according_to_score(self, paths, folder, gen_type="waypoint"):
        # Create the converter class's
        unity_converter = UnityConverter(save_directory=folder)
        mavlink_converter = MavlinkConverter(save_directory=folder)

        # Get the scores
        scores, lin_score, ang_score = self.calculate_scores(paths=paths,
                                                             gen_type=gen_type)
        scores_all, lin_score_all, ang_score_all = self.calculate_scores(paths=paths,
                                                                         gen_type=gen_type,
                                                                         all_points=True)

        out_mag, largest_out_mag, waypoint_angles, euler_angles = self.calculate_path_details(paths=paths)

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
            mavros_file_name = save_directory + "flightplan_raw.mavlink"

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
            file.write("Path Total Linear Score: " + str(lin_score[index]) + '\n')
            file.write("Path Total Angular Score: " + str(ang_score[index]) + '\n')
            file.write("----------------------------\n")
            file.write("Path Scores: " + str(scores_all[index]) + '\n')
            file.write("Path Linear Scores: " + str(lin_score_all[index]) + '\n')
            file.write("Path Angular Scores: " + str(ang_score_all[index]) + '\n')
            file.write("----------------------------\n")
            file.write("Waypoints: " + str(waypoints) + '\n')
            file.write("Velocity: " + str(velocity) + '\n')
            file.write("Attitude: " + str(angles) + '\n')
            file.write("----------------------------\n")
            file.write("Out Vector Magnitude: " + str(out_mag[index]) + '\n')
            file.write("largest Out Vector Magnitude: " + str(largest_out_mag[index]) + '\n')
            file.write("Angles: " + str(waypoint_angles[index]) + '\n')
            file.write("Euler Angles: " + str(euler_angles[index]) + '\n')
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

    # Expects angles in degrees
    def calculate_angular_score(self, current_angle, best_angle, larger):
        if larger and current_angle > best_angle:
            return 1.0
        else:
            return 1.0 - min(abs(current_angle - best_angle) / 90.0, 1.0)

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