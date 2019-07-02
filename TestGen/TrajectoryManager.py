import numpy as np


class TrajectoryManager:

	def __init__(self, kinematics):
		# Initialize the kinematics
		self.max_turn = kinematics["max_turn"]
		self.min_turn = kinematics["min_turn"]
		self.max_vel = kinematics["max_velocity"]
		self.min_vel = kinematics["min_velocity"]

	# Score each of the trajectories
	def calculate_score(self, trajectories):
		# Go through all the trajectories
		for traj in trajectories:
			# Get the trajectories list of distances
			distances = traj.get_distances()
			headings = traj.get_headings()

			# Get the score for each distance
			vel_score = 0
			for distance in distances:
				vel_score += self.velocity_score(distance)

			# Get the score for each heading
			head_score = 0
			for heading in headings:
				head_score += self.angle_score(heading)

			# Take the average of the score
			vel_score = vel_score / float(len(distances))
			head_score = head_score / float(len(headings))

			final_score = (vel_score + head_score) / 2

			# Make sure the scores make sense
			assert(final_score >= 0)
			assert(final_score <= 1)

			# Save the score
			traj.set_score(final_score)

		# Return success
		return 1

	def velocity_score(self, velocity):

		# Calculate how close to the edge of velocity the robot is
		vel_dif = abs(self.max_vel - velocity) / self.max_vel

		# We want the best score to be exactly on the edge but cant be less than 0
		score = max(1 - vel_dif, 0)

		# Return the score
		return score

	def angle_score(self, angle):

		# Calculate how close to the edge of kinematic the robot is
		if angle < 0:
			ang_dif = abs(self.min_turn - angle) / abs(self.min_turn)
		else:
			ang_dif = abs(self.max_turn - angle) / abs(self.max_turn)

		# We want the best score to be exactly on the edge but cant be less than 0
		score = max(1 - ang_dif, 0)

		# Return the score
		return score

	def get_all_scores(self, trajectories):
		# Get all the trajectories scores
		scores = []
		for traj in trajectories:
			scores.append(traj.get_score())

		# Return the scores
		return scores

	def get_all_classes(self, trajectories):
		# Get all the trajectories scores
		classes = []
		for traj in trajectories:
			classes.append(traj.get_classification())

		# Return the classes
		return classes

	def sort_by_score(self, trajectories):
		# Get all the scores
		scores = self.get_all_scores(trajectories=trajectories)

		# Get the order they should be in
		indices = np.argsort(a=scores)

		# Create the final trajectories
		final_trajectories = []

		# Go backwards so we get the scores from highest to lowest
		for index in indices[::-1]:
			final_trajectories.append(trajectories[index])

		# Return the final trajectories
		return final_trajectories

	def sort_by_class_and_score(self, trajectories):
		# Get the trajectories classes
		classes = self.get_all_classes(trajectories=trajectories)

		# Get the total number of classes
		total_classes = max(classes) + 1

		# Create a dictionary to hold the trajectories
		traj_by_class = {}

		# For each class number
		for i in range(0, total_classes):
			# Create a list to save trajectories of that class in
			traj_by_class["class" + str(i)] = []

		# For each trajectory
		for traj in trajectories:
			# Get the trajectories class
			classification = traj.get_classification()
			# Put the trajectory in the correct list
			traj_by_class["class" + str(classification)].append(traj)

		# For each class, sort the trajectories by score
		for key in traj_by_class:
			traj_by_class[key] = self.sort_by_score(traj_by_class[key])

		# Return the trajectories
		return traj_by_class

