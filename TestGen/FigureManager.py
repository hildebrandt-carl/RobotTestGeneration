import os
import matplotlib.pyplot as plt
from Trajectory import Trajectory

class FigureManager:

	def __init__(self, save_path):
		# Used to save where the file is saved
		self.save_location = save_path

		# Used to keep track of the number of figures plotted
		self.total_figures = 0

	# Create the save directory
	def create_directory(self):
		# Check the save location is created
		if not os.path.isdir(self.save_location):
			# If not create it
			os.makedirs(self.save_location)

		# Create the directory where the maps will be saved
		maps_save_path = self.save_location + "maps/"
		if not os.path.isdir(maps_save_path):
			os.makedirs(maps_save_path)

	def display_and_save(self, fig, save_name, save_directory="", only_save=False):
		# Create the filename
		file_name = save_directory + str(self.total_figures).zfill(2) + "-" + str(save_name) + ".png"
		# Save the figure
		fig.savefig(self.save_location + file_name)
		if not only_save:
			# Display the figure
			fig.show()

		# Close the figures
		plt.close()

	def plot_trajectories_by_class(self, trajectories_with_class, number_per_class=5, tsuffix="", figure_size=(15, 13)):

		# Get the total number of classes
		total_classes = len(trajectories_with_class.keys())

		# Create the figure
		fig = plt.figure(figsize=figure_size)
		axes = fig.subplots(nrows=number_per_class+1, ncols=total_classes)

		# Add a title
		if len(tsuffix) >= 0:
			fig.suptitle(t='Individual Trajectories (Sorted by Score and Class) - ' + str(tsuffix),
						 fontsize=25)
		else:
			fig.suptitle(t='Individual Trajectories (Sorted by Score and Class)',
						 fontsize=25)

		# Used to keep track of where I am plotting
		row = 0
		col = 0

		# Turn all sub figures axis off
		while row <= number_per_class:
			axes[row, col].axis('off')
			# Increment the row and col
			col += 1
			if col >= total_classes:
				col = 0
				row += 1

		# Reset the row and column
		row = 1
		col = 0

		# For each class
		for key in trajectories_with_class:
			# Reset the row number
			row = 1
			# For the number of trajectories we want to plot
			for i in range(0, number_per_class):
				# Check there is still a trajectory of that class to display
				if len(trajectories_with_class[key]) <= i:
					continue

				# Get the x and y lines for that trajectory
				linex = []
				liney = []

				# Get the trajectory waypoints
				waypoints = trajectories_with_class[key][i].get_waypoints()

				# Get the color for the line
				if trajectories_with_class[key][i].get_passing():
					# If this is a passing test show it in green
					color = 'green'
				else:
					# Otherwise show it in red
					color = 'red'

				# For each waypoint
				for waypoint in waypoints:
					# Add it to the line
					linex.append(waypoint[0])
					liney.append(waypoint[1])

				# Plot that trajectory
				axes[row, col].set_xlim([Trajectory.x_range["lower"], Trajectory.x_range["upper"]])
				axes[row, col].set_ylim([Trajectory.y_range["lower"], Trajectory.y_range["upper"]])
				axes[row, col].plot(linex, liney, color=color)

				# Increment the row counter
				row += 1

			# Increment the col counter
			col += 1

		# Return the figure
		return fig

	def plot_allpaths(self, trajectories=[], tsuffix="", figure_size=(15, 13)):
		# Create a plot
		plt.figure(figsize=figure_size)

		# Set the maps dimensions to the correct size
		plt.xlim([Trajectory.x_range["lower"], Trajectory.x_range["upper"]])
		plt.ylim([Trajectory.y_range["lower"], Trajectory.y_range["upper"]])

		# Check if we want a title for the figure
		if len(tsuffix) >= 0:
			plt.title('Occupancy Map - ' + str(tsuffix))
		else:
			plt.title('Occupancy Map')

		# For each trajectory
		for traj in trajectories:
			# Get the trajectory waypoints
			waypoints = traj.get_waypoints()

			# Create a list for the x and y co-ordinates
			linex = []
			liney = []
			color = ''

			# For each waypoint
			for waypoint in waypoints:
				# Add it to the line
				linex.append(waypoint[0])
				liney.append(waypoint[1])

			# Get the color for the line
			if traj.get_passing():
				# If this is a passing test show it in green
				color = 'green'
			else:
				# Otherwise show it in red
				color = 'red'

			# Create the plot
			plt.plot(linex, liney, color=color)

		# return the plot
		return plt

	def plot_class_histogram(self, class_array):
		# Create a new figure
		plt.figure()

		# Get the total number of classes
		total_classes = max(class_array) + 1

		# Create the plot
		plt.hist(x=class_array,
				 bins=total_classes,
				 range=(0, total_classes))
		plt.title("Total of Each Equivalence Class")
		plt.xlabel("Class Number")
		plt.ylabel("Total")

		return plt

	def plot_scores(self, score_array):
		# Create a new figure
		plt.figure()

		# Create the plot
		plt.plot(score_array)
		plt.title("Trajectory Scores")
		plt.xlabel("Trajectory Number")
		plt.ylabel("Score")

		return plt

	def plot_corridor(self, waypoints, wall_segments, unity_dimensions=[100, 100]):
		# Create a new figure
		plt.figure()

		# Get the waypoints
		waypoints_x = []
		waypoints_y = []

		# Extract the waypoints
		for waypoint in waypoints:
			waypoints_x.append(waypoint[0])
			waypoints_y.append(waypoint[1])

		# Display the waypoints
		plt.scatter(waypoints_x, waypoints_y)
		plt.plot(waypoints_x, waypoints_y, "--")

		# For all the wall segments above the line
		for above_below in wall_segments:
			wall_x = []
			wall_y = []
			# For all the wall segments below the line
			for segment in above_below:
				# Get the points for the wall:
				wall_x.append(segment.get_start()[0])
				wall_x.append(segment.get_end()[0])
				wall_y.append(segment.get_start()[1])
				wall_y.append(segment.get_end()[1])

			# Show the walls in red
			plt.plot(wall_x, wall_y, "r")

		# Scale the plot to the correct size
		plt.xlim([0, unity_dimensions[0]])
		plt.ylim([(-1 * unity_dimensions[1] / 2.0), (unity_dimensions[1] / 2.0)])

		# Return the plot
		return plt