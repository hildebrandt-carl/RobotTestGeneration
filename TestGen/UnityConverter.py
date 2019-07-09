from Utils import euclidean_distance
from LineSegment import LineSegment


class UnityConverter:
	def __init__(self, test_dimensions=[25, 25], unity_dimensions=[100, 100]):
		# Initialize unity dimensions
		self.unity_x_dimension = unity_dimensions[0]
		self.unity_y_dimension = unity_dimensions[1]

		# Get the test dimensions
		self.test_x_dimension = test_dimensions[0]
		self.test_y_dimension = test_dimensions[1]

	def scale_to_unity(self, trajectory):
		# Calculate the scale factor x and y
		x_scale = self.unity_x_dimension / float(self.test_x_dimension)
		y_scale = self.unity_y_dimension / float(self.test_y_dimension)

		# Shift the Y-Axis due to unity having middle Y = 0
		y_shift = self.unity_y_dimension / 2.0

		# Get the waypoints
		waypoints = trajectory.get_waypoints()

		# Used to save the unity waypoints
		unity_goal_waypoints = []

		# Go through the waypoints and scale them up
		for point in waypoints:
			new_point = [0, 0]
			# scale the points
			new_point[0] = point[0] * x_scale
			new_point[1] = (point[1] * y_scale) - y_shift

			# Append the new point to the unity waypoints
			unity_goal_waypoints.append(new_point)

		# Return the new trajectory
		return unity_goal_waypoints

	def corridor_generator(self, waypoints, corridor_gap=0.5):
		# We want a corridor above and below the waypoints
		gaps = [corridor_gap, -1 * corridor_gap]

		# Final wall_segments
		final_segments = []

		# For above and below the line
		for offset in gaps:
			# Create a set of line segments (walls)
			wall_segments = []

			# Populate the walls
			for p1, p2 in zip(waypoints, waypoints[1:]):
				# Calculate the length of the line segment
				length = euclidean_distance(p1, p2)

				# Calculate a wall parallel to the line segment but offset.
				x1 = p1[0] + offset * (p2[1] - p1[1]) / length
				x2 = p2[0] + offset * (p2[1] - p1[1]) / length
				y1 = p1[1] + offset * (p1[0] - p2[0]) / length
				y2 = p2[1] + offset * (p1[0] - p2[0]) / length

				# Save the new wall segment
				wall_segment = LineSegment(start_point=[x1, y1],
										   end_point=[x2, y2])
				wall_segments.append(wall_segment)

			# Go through each line segment and find their intersections
			for seg1, seg2 in zip(wall_segments, wall_segments[1:]):
				# Calculate the intersection
				intersection = seg1.find_intersection(seg2)

				# Update the lines so that their start and end point is the intersection
				seg1.set_end(intersection)
				# Set line 2's start to the intersection point
				seg2.set_start(intersection)

			# Append these segements to the final segment
			final_segments.append(wall_segments)

		# Return the final segments
		return final_segments

	def unity_text_file(self, waypoints, corridor, save_location, raining=False, day=True):
		file = open(save_location + "corridor_test.txt", "w")

		file.write('# Note +ve X is north\n')
		file.write('# Note +ve Y is West\n')
		file.write('# x, y, height\n')
		file.write("# waypoints\n")

		# Go through each waypoint
		for waypoint in waypoints:
			file.write("G: (" + str(waypoint[0]) + ',' + str(waypoint[1]) + ",10)\n")

		# Write a final waypoint above the the final waypoing
		file.write("G: (" + str(waypoints[-1][0]) + ',' + str(waypoints[-1][1]) + ",30)\n")

		file.write("\n\n# wall segments\n")
		file.write('# x1, y1, x2, y2, z\n')

		# For walls below and above the line
		for wall_segements in corridor:
			# For a segment in the wall segments
			for segment in wall_segements:
				x_start = str(segment.get_start()[0])
				y_start = str(segment.get_start()[1])
				x_end = str(segment.get_end()[0])
				y_end = str(segment.get_end()[1])
				file.write("W: (" + x_start + "," + y_start + "," + x_end + "," + y_end + ",30)\n")

		file.write("\n\n# environment settings\n")
		file.write("R: (" + str(int(raining)) + ")\n")
		file.write("D: (" + str(int(day)) + ")\n")

		file.close()