from Utils import euclidean_distance
from LineSegment import LineSegment


class UnityConverter:
	def __init__(self, save_directory):
		# Save directory
		self.save_directory = save_directory

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

	def unity_text_file(self, waypoints, expected_velocity, corridor, save_name, raining=False, day=True, cage=False):
		file = open(self.save_directory + save_name, "w")

		file.write('# Note +ve X is north\n')
		file.write('# Note +ve Y is West\n')
		file.write('# x, y, height\n')
		file.write("# waypoints\n")

		# Go through each waypoint
		for waypoint in waypoints:
			file.write("G: (" + str(waypoint[0]) + ',' + str(waypoint[1]) + "," + str(waypoint[2]) + ")\n")

		file.write('\n\n# Vx, Vy, Vz\n')
		file.write("# velocity\n")

		# Go through each waypoint
		for vel in expected_velocity:
			file.write("V: (" + str(vel[0]) + ',' + str(vel[1]) + "," + str(vel[2]) + ")\n")

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
		file.write("C: (" + str(int(cage)) + ")\n")

		file.close()