import numpy as np
from scipy.spatial import ConvexHull


class DroneReachabilitySet:

	def __init__(self, x_positions, y_positions, z_positions):
		# keeps a list of all drone positions
		self.x_vals = np.copy(x_positions)
		self.y_vals = np.copy(y_positions)
		self.z_vals = np.copy(z_positions)

		# Calculate the convex hull of the points
		self.points = np.column_stack((self.x_vals, self.y_vals, self.z_vals))
		self.hull = ConvexHull(self.points)

	def is_in_hull(self, waypoints):
		'''
		Datermine if the list of points P lies inside the hull
		:return: list
		List of boolean where true means that the point is inside the convex hull
		'''
		# https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
		A = self.hull.equations[:, 0:-1]
		b = np.transpose(np.array([self.hull.equations[:, -1]]))
		isInHull = np.all((A @ np.transpose(waypoints)) <= np.tile(-b, (1, len(waypoints))), axis=0)
		return isInHull

	# Get the hull simplices for plotting
	def get_hull_simplices(self):

		# Create lists to save the data
		x_positions = []
		y_positions = []
		z_positions = []

		# For each of the hull simplices
		for s in self.hull.simplices:
			s = np.append(s, s[0])

			# Create the points
			x_positions.append(self.points[s, 0])
			y_positions.append(self.points[s, 1])
			z_positions.append(self.points[s, 2])

		# Return the position
		return x_positions, y_positions, z_positions
