from FigureManager import FigureManager
from UnityConverter import UnityConverter
from math import sqrt, radians
import numpy as np
import os

class RankingSystem:

	def __init__(self, paths=[]):
		# Used to save the paths
		self.paths = paths
		# Used to save the scores of each path
		self.scores = []
		self.linear_scores = []
		self.angular_scores = []

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
			selected_velocities = []
			maximum_velocities = []
			velocities = []
			positions = []
			angles = []
			# For each kinematic
			for point in path:
				# Get the current position
				positions.append(point.get_position())

				# Get the final distance flown in 1 time step
				current_velocity = point.get_velocity()
				velocities.append(current_velocity)

				# Calculate the magnitude of those velocities
				magnitude = sqrt(current_velocity[0] ** 2 + current_velocity[1] ** 2 + current_velocity[2] ** 2)
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

			# Scores
			linear_vel_scores = []
			angular_vel_scores = []

			# Calculate the linear velocity score
			for s, m in zip(selected_velocities, maximum_velocities):
				vel_score = float(s) / float(m)
				linear_vel_scores.append(vel_score)

			# Calculate the angular velocity score
			for i in range(0, len(positions) - 1):

				# Get the current position and velocity
				current_pos = positions[i]
				current_vel = velocities[i]

				# For the initial state
				if i == 0:
					# assume the velocity is moving directly upwards
					current_vel = [0, 0, 1]

				# Calculate the next position if it continued in the current direction
				no_angular_vel_pos = current_pos + current_vel

				# Get the true next position
				real_next_pos = positions[i + 1]

				# Calculate the angle between them
				angle = self.angle_between_vectors(no_angular_vel_pos, real_next_pos)

				# Calculate the score
				angular_vel_score = angle / radians(180)
				angular_vel_scores.append(angular_vel_score)

			# The final score is the summation of both linear and angular scores
			test_score = [x + y for x, y in zip(linear_vel_scores, angular_vel_scores)]
			self.scores.append(sum(test_score))
			self.linear_scores.append(sum(linear_vel_scores))
			self.angular_scores.append(sum(angular_vel_scores))

		return self.scores, self.linear_scores, self.angular_scores

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