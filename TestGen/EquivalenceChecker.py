import math
from Utils import distance_to_line

class EquivalenceChecker:

	def __init__(self, epsilon):
		# Used to check which trajectories are equivalent
		self.epsilon = epsilon

	# Return the waypoints
	def find_equivalence(self, trajectories):
		# Class assignment
		class_counter = 0

		# For each trajectory
		for i in range(0, len(trajectories)):

			# get trajectory
			main_traj = trajectories[i]

			# Get the waypoints of that trajectory
			main_waypoints = main_traj.get_waypoints()

			# If the main trajectory does not have a class, assign one
			if main_traj.get_classification() == -1:
				main_traj.set_classification(classification=class_counter)
			else:
				continue

			# For all other trajectories
			for j in range(0, len(trajectories)):
				# Dont compare two of the same trajectories
				if i == j:
					continue

				# Get the trajectory to compare against
				compare_traj = trajectories[j]

				# Dont assign a class to a trajectory which already has a class
				if compare_traj.get_classification() != -1:
					continue

				# Get the waypoints from this trajectory
				compare_waypoints = compare_traj.get_waypoints()

				# Assume the compare trajectory is the same class as the main trajectory
				same_class = True

				# For each of the waypoints in the comparison trajectory
				for p3 in compare_waypoints:
					# Used to keep track of the minimum distance between p3 and the main line segments
					min_distance = math.inf

					# For each line in the main trajectory
					for p1, p2 in zip(main_waypoints, main_waypoints[1:]):
						# Get the distance from the point to the line
						distance = distance_to_line(line_p1=p1,
													line_p2=p2,
													point=p3)

						# save the minimum distance.
						min_distance = min(distance, min_distance)

					# If the minimum distance is greater than epsilon
					if min_distance > self.epsilon:
						same_class = False

				# If this is the same class, assign this trajectory the same class as the main trajectory
				if same_class:
					compare_traj.set_classification(classification=class_counter)

			# Increment the lass counter
			class_counter += 1

		# Return success
		return class_counter

