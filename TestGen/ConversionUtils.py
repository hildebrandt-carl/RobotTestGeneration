import numpy as np
import matplotlib.pyplot as plt
from skimage import measure

def pythonMaptoUnityFile(map, distance_threshold):

	# Find contours at a constant value of 0.8
	contours = measure.find_contours(map, 0.8)

	# Display the image and plot all contours found
	fig, ax = plt.subplots()
	ax.imshow(map, interpolation='nearest', cmap='binary')

	# Convert each contour into line segements
	lines = []
	for n, contour in enumerate(contours):

		# Start by assuming the line is the first and the last point
		start_point = 0
		end_point = len(contour) - 1

		# Check that the line is not a circular list
		while np.all(contour[start_point] == contour[end_point]):
			end_point = end_point - 1

		# We need to find a line for this contour
		line_complete = False

		# Saves our final line
		final_lines = []

		# For the entire contour
		while not line_complete:

			retry = False
			# Calculate the distance between each point and the line:
			for point in range(start_point + 1, end_point):
				# line between p1 and p2
				p1 = contour[start_point]
				p2 = contour[end_point]
				# random point
				p3 = contour[point]
				# Calculate distance between line P1->P2 and point P3
				d = np.linalg.norm(np.cross(p2 - p1, p1 - p3)) / np.linalg.norm(p2 - p1)
				# If distance is greater than threshold
				if d > distance_threshold:
					# Move end point back by one
					end_point = end_point - 1
					retry = True

			# If none of the points where too far away
			if not retry:
				final_lines.append([contour[start_point], contour[end_point]])

				# Check to see if we have completed the line
				if end_point == len(contour) - 1:
					line_complete = True
				else:
					# Find the next line
					start_point = end_point
					end_point = len(contour) - 1

		# Plot each straight line segment
		for line in final_lines:
			stacked = np.stack(line)
			plt.plot(stacked[:, 1], stacked[:, 0], color='red')

	# Plot each contour
	for n, contour in enumerate(contours):
		ax.plot(contour[:, 1], contour[:, 0], linewidth=2, color='blue')

	plt.xlim([0, map.shape[1] - 1])
	plt.ylim([0, map.shape[1] - 1])
	plt.show()