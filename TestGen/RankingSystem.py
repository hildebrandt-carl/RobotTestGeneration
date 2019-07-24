from FigureManager import FigureManager
from UnityConverter import UnityConverter
from math import sqrt, radians
import numpy as np
import os


class RankingSystem:

	# Class variable shared among all instances
	# Keeps track of the bounds the x,y and z bounds should be set
	x_range = {"lower": 0, "upper": 1}
	y_range = {"lower": 0, "upper": 1}
	z_range = {"lower": 0, "upper": 1}

	def __init__(self, paths):
		# Used to save the paths
		self.paths = paths
		# Used to save the scores of each path
		self.scores = []

	# Return the scores
	def get_scores(self):
		return self.scores

	# Calculate the scores
	def calculate_scores(self):
		# For each path
		for path in self.paths:
			selected_velocities = []
			maximum_velocities = []
			angles = []
			# For each kinematic
			for point in path:
				# Get the final distance flown in 1 time step
				velocities = point.get_velocity()

				# Calculate the velocity entering that waypoint
				magnitude = sqrt(velocities[0] ** 2 + velocities[1] ** 2 + velocities[2] ** 2)
				selected_velocities.append(magnitude)

				# Get the maximum velocity you could reach leaving that point
				maximum_velocities.append(point.get_maximum_velocity())

				# Get the angle to the entering that waypoint
				angles.append(list(point.get_attitude()))

			# Remove the first velocity from selected velocities as this is simply the initial condition of the robot
			selected_velocities = selected_velocities[1:]

			# Remove the first angle as that is the initial condition of the drone
			angles = angles[1:]

			# Remove the last velocity from maximum velocities as no reachability set was calculated from the last point
			maximum_velocities = maximum_velocities[:-1]

			# Calculate the scores
			score = 0
			for s, m, a in zip(selected_velocities, maximum_velocities, angles):
				vel_score = float(s) / float(m)
				angle_x_score = float(a[0]) / radians(180)
				angle_y_score = float(a[1]) / radians(180)
				angle_z_score = float(a[2]) / radians(180)
				score += (vel_score + angle_x_score + angle_y_score + angle_z_score)

			# Save the score
			self.scores.append(score)

	# Save the paths to a directory
	def save_trajectories_according_to_score(self, folder):
		# Create the converter class
		converter = UnityConverter(save_directory=folder)

		# Sort the scores and get the new indices
		score_indicies = list(np.argsort(self.scores))

		# Reverse the scores so that its from largest to smallest
		score_indicies.reverse()

		print("\nINFO: Printing out the path information")

		# Create the Figure manager
		fig_manager = FigureManager(folder)

		path_counter = 0
		for index in score_indicies:
			path_counter += 1
			waypoints = []
			velocity = []
			angles = []
			# Get the waypoints and velocities for that path
			for each_point in self.paths[index]:
				waypoints.append(list(each_point.get_position()))
				velocity.append(list(each_point.get_velocity()))
				angles.append(list(each_point.get_attitude()))

			# Create the file names
			save_directory = "maps/map" + str(path_counter) + "/"
			unity_file_name = save_directory + "test.txt"
			details_file_name = save_directory + "details.txt"

			# Check the save location is created
			if not os.path.isdir(folder + save_directory):
				# If not create it
				os.makedirs(folder + save_directory)

			print("Path: " + str(index))
			print("Path Score: " + str(self.scores[index]))
			print("Waypoints: " + str(waypoints))
			print("Velocity: " + str(velocity))
			print("Attitude: " + str(angles))
			print("Save Location: " + unity_file_name)
			print("------------")

			# Write this information to file
			file = open(folder + details_file_name, "w")
			file.write("Path: " + str(index) + '\n')
			file.write("Path Score: " + str(self.scores[index]) + '\n')
			file.write("Waypoints: " + str(waypoints) + '\n')
			file.write("Velocity: " + str(velocity) + '\n')
			file.write("Attitude: " + str(angles) + '\n')
			file.write("Save Location: " + unity_file_name + '\n')
			file.close()

			# Save the test to a unity file
			converter.unity_text_file(waypoints=waypoints,
									  expected_velocity=velocity,
									  corridor=[],
									  save_name=unity_file_name)

			# Create the plot and save it
			plt = fig_manager.plot_single_trajectory(waypoints)
			fig_manager.display_and_save(fig=plt,
										 save_name="trajectory" + str(path_counter),
										 save_directory=save_directory,
										 only_save=True,
										 figure_number=False)
