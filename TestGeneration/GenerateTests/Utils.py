import math

# Calculates the distance between two points given as a list
def euclidean_distance(p1, p2):
	# Calculate the difference between the points
	dx = p1[0] - p2[0]
	dy = p1[1] - p2[1]

	# Calculate the euclidean distance
	distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

	# Return the distance
	return distance


# Make sure degrees are between -180 and 180
def correct_degrees(angle):
	# If the angle is too small
	while angle < -180:
		angle = angle + 360

	# If the angle is too large
	while angle > 180:
		angle = angle - 360

	# Return the angle
	return angle


def distance_to_line(line_p1, line_p2, point):
	# Return from a point to the line distance to the line
	# Reference: (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
	dx = line_p2[0] - line_p1[0]
	dy = line_p2[1] - line_p1[1]

	numerator = (dy * point[0]) - (dx * point[1]) + (line_p2[0] * line_p1[1]) - (line_p2[1] * line_p1[0])
	denominator = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

	distance = abs(numerator) / denominator

	return distance
