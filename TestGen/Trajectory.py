from Utils import euclidean_distance
from Utils import correct_degrees


class Trajectory:

	# Class variable shared among all instances
	# Keeps track of the maximum bounds of the trajectory
	x_range = {"lower": 0, "upper": 1}
	y_range = {"lower": 0, "upper": 1}

	def __init__(self):
		# Used to save the trajectories waypoints
		self.waypoints = []
		# Calculated the distance between the waypoints
		self.distances = []
		# Used to save the heading of the trajectory
		# 0 is east, -90 is south, 90 is north, -180/180 is west
		self.headings = []
		# Used to keep the score of the trajectory
		self.score = -1
		# Used to keep track of the test is pass or fail
		self.passing = True
		# Used to keep track of what class a trajectory is in
		self.classification = -1
		# Used to keep track of the difference between headings
		self.heading_difference = []

	# Return the waypoints
	def get_waypoints(self):
		return self.waypoints

	# Return the distances
	def get_distances(self):
		return self.distances

	# Return the headings
	def get_headings(self):
		return self.headings

	# Return the heading
	def get_score(self):
		return self.heading

	# Return the score
	def get_score(self):
		return self.score

	# Return if the test should pass or fail
	def get_passing(self):
		return self.passing

	# Return the classification
	def get_classification(self):
		return self.classification

	# Set the waypoints and check that they are in bounds
	def set_waypoints(self, points):
		# Go through the waypoints and check that it is in bounds
		for point in points:
			# if the point is inside the bounds
			cond1 = (Trajectory.x_range["lower"] <= point[0] <= Trajectory.x_range["upper"])
			cond2 = (Trajectory.y_range["lower"] <= point[1] <= Trajectory.y_range["upper"])

			if cond1 and cond2:
				self.waypoints.append(point)
			else:
				# Dont save any waypoints
				self.reset_trajectory()
				# Display and error
				print("Error: Setting waypoints out of bounds")
				# Return failure
				return -1

		# Calculate the distances between points
		self.calculate_distances()

		# Return success
		return 1

	# Set the score of the trajectory
	def set_headings(self, headings):
		# Go through the headings and check they make sense
		for heading in headings:
			# If the angle is in degrees
			if -180 <= heading <= 180:
				# Add it to the headings
				self.headings.append(heading)
			else:
				# Return failure
				return -1

		# Return success
		return 1

	# Calculate the distances between the waypoints
	def calculate_distances(self):
		# Go through the waypoints in order
		for p1, p2 in zip(self.waypoints, self.waypoints[1:]):
			distance = euclidean_distance(p1, p2)
			self.distances.append(distance)

	# Check if this test is passing or failing
	def check_if_passing(self, max_turn, min_turn, robot_heading=0):
		# Keeps track of the robots current heading
		current_heading = robot_heading

		# Reset the old difference tracker
		self.heading_difference = []

		# For each of the headings
		for heading in self.headings:
			# Calculate the difference between the segement heading and the current heading
			delta_heading = current_heading - heading

			# Make sure heading is in the correct bounds:
			delta_heading = correct_degrees(delta_heading)

			# Append the difference
			self.heading_difference.append(delta_heading)

			# Check if this segement would fail
			if (delta_heading < min_turn) or (delta_heading > max_turn):
				self.passing = False

			# Set the robots current heading to be the same as that segment
			current_heading = heading

		# return success
		return 1

	# Set the score of the trajectory
	def set_score(self, score):
		self.score = score

	# Set the score of the trajectory
	def set_classification(self, classification):
		self.classification = classification

	# Reset the waypoints, distance and score
	def reset_trajectory(self):
		self.waypoints = []
		self.distances = []
		self.score = -1
