from FigureManager import FigureManager
from UnityConverter import UnityConverter
from math import sqrt, radians
import numpy as np
import os
import random

class RankingSystem:

	def __init__(self, paths=[], baseline=False):
		# Used to save the paths
		self.paths = paths
		# Used to save the scores of each path
		self.scores = []
		self.linear_scores = []
		self.angular_scores = []
		# Determines whether the ranking system is generating a baseline or not
		self.baseline = baseline

	# Used to update the paths
	def update_paths(self, new_paths):
		self.paths = new_paths
		# Used to save the scores of each path
		self.scores = []
		self.linear_scores = []
		self.angular_scores = []

	# Return the scores
	def get_scores(self):
		return self.scores, self.linear_scores, self.angular_scores

	# Calculate the scores
	def calculate_scores(self):
		# Used to save the scores of each path
		self.scores = []
		self.linear_scores = []
		self.angular_scores = []

		# For each path
		for path in self.paths:
			maximum_vel_magnitude = []
			velocities = []
			positions = []
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

				# Get the largest magnitude at that waypoint
				largest_mag = maximum_vel_magnitude[score_counter]

				# Calculate the difference of the vectors
				diff_vec = [out_vec[0] - in_vec[0], out_vec[1] - in_vec[1], out_vec[2] - in_vec[2]]

				# Calculate the magnitude of the vector
				diff_vec_mag = sqrt(pow(diff_vec[0], 2) + pow(diff_vec[1], 2) + pow(diff_vec[2], 2))

				# Normalize the vector
				normalized_magnitude = diff_vec_mag / (largest_mag * 2)
				point_score.append(normalized_magnitude)

			# The final score is the summation of point_scores
			self.scores.append(sum(point_score))

		return self.scores

	# Save the paths to a directory
	def save_trajectories_according_to_score(self, folder):
		# Create the converter class
		converter = UnityConverter(save_directory=folder)

		# Sort the scores and get the new indices
		score_indicies = list(np.argsort(self.scores))

		# Reverse the scores so that its from largest to smallest
		score_indicies.reverse()

		# print("\nINFO: Printing out the path information")

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

			# print("Path: " + str(index))
			# print("Path Score: " + str(self.scores[index]))
			# print("Waypoints: " + str(waypoints))
			# print("Velocity: " + str(velocity))
			# print("Attitude: " + str(angles))
			# print("Save Location: " + unity_file_name)
			# print("------------")

			# Write this information to file
			file = open(folder + details_file_name, "w")
			file.write("Path: " + str(index) + '\n')
			file.write("Path Total Score: " + str(self.scores[index]) + '\n')
			file.write("Path Linear Score: " + str(self.linear_scores[index]) + '\n')
			file.write("Path Angular Score: " + str(self.angular_scores[index]) + '\n')
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