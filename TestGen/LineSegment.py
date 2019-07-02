import math


class LineSegment:

	def __init__(self, start_point, end_point):
		# Check the start and end point make sense
		assert(len(start_point) > 0)
		assert (len(end_point) > 0)

		# Used to save the start of the line
		self.start_point = start_point
		# used to save the end of the line
		self.end_point = end_point
		# Used to hold the gradient
		self.gradient = 0
		# Used to hold the y-intercept
		self.b = 0

		# Update the gradient
		self.update_gradient()

		# Update the y-intercept
		self.update_y_intercept()

	# Return the start_point
	def get_start(self):
		return self.start_point

	# Return the end_point
	def get_end(self):
		return self.end_point

	# Return the gradient
	def get_gradient(self):
		return self.gradient

	# Return the y-intercept
	def get_y_intercept(self):
		return self.b

	# Set the starting point
	def set_start(self, start_point):
		# Set the starting point
		self.start_point = start_point
		# Update the gradient
		self.update_gradient()
		# Update the y-intercept
		self.update_y_intercept()

	# Set the ending point
	def set_end(self, end_point):
		# Set the ending point
		self.end_point = end_point
		# Update the gradient
		self.update_gradient()
		# Update the y-intercept
		self.update_y_intercept()

	# Return the gradient
	def update_gradient(self):
		# Calculate the gradient
		dx = self.end_point[0] - self.start_point[0]
		dy = self.end_point[1] - self.start_point[1]
		# If the dx == 0, the line goes straight up
		if dx == 0:
			self.gradient = math.inf
		else:
			self.gradient = float(dy) / float(dx)

	# Return the gradient
	def update_y_intercept(self):
		# If the gradient is inf, then there is no y-intercept
		if self.gradient == math.inf:
			self.b = None
		else:
			# Calculate the y intercept
			self.b = self.start_point[1] - self.gradient * self.start_point[0]

	# Find where these two lines intersect
	def find_intersection(self, other_segment):
		# Get the other segments y-intercept and gradient
		other_b = other_segment.get_y_intercept()
		other_m = other_segment.get_gradient()

		# If the gradients are the same, the lines are parallel
		assert(other_m != self.gradient)

		# If the other segment has an infinite gradient
		if other_m == math.inf:
			# The X intercept is the other segments x value
			x_intercept = other_segment.get_start()[0]
		# If this segment has an infinite gradient
		elif self.gradient == math.inf:
			# The X intercept is the other segments x value
			x_intercept = self.get_start()[0]
		# Otherwise calculate the x_intercept using mx + b = mx + b
		else:
			# Calculate the X value where they intersect
			delta_b = other_b - self.b
			delta_m = self.gradient - other_m
			x_intercept = delta_b / float(delta_m)

		# Use the equation for the line which does not have an infinite gradient
		# If neither do it does not matter which one is used
		if self.gradient == math.inf:
			# Calculate the Y value where they intersect (y = mx +b)
			y_intercept = other_m * x_intercept + other_b
		else:
			# Calculate the Y value where they intersect (y = mx +b)
			y_intercept = self.gradient * x_intercept + self.b

		# Return the point
		return [x_intercept, y_intercept]
