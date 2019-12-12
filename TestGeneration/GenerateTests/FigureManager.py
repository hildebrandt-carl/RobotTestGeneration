import os

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from Node import Node


class FigureManager:

	# Class variable shared among all instances
	# Keeps track of the bounds the x,y and z bounds should be set
	x_range = {"lower": 0, "upper": 1}
	y_range = {"lower": 0, "upper": 1}
	z_range = {"lower": 0, "upper": 1}

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

	def display_and_save(self, fig, save_name, save_directory="", only_save=False, figure_number=True):
		# Create the filename
		if figure_number:
			file_name = save_directory + str(self.total_figures).zfill(2) + "-" + str(save_name) + ".png"
			# Increment the figure number
			self.total_figures += 1
		else:
			file_name = save_directory + str(save_name) + ".png"
		# Save the figure
		fig.savefig(self.save_location + file_name, bbox_inches='tight')
		if not only_save:
			# Display the figure
			fig.show()


		# Close the figures
		plt.close()

	def plot_prm_graph(self, nodes, edges, figure_size=(10, 10)):
		# Create the figure
		fig = plt.figure(figsize=figure_size)
		ax = Axes3D(fig)

		# Create a list of standard nodes
		x_vals = []
		y_vals = []
		z_vals = []

		# Create a list of source and sink nodes
		source_x = []
		source_y = []
		source_z = []
		sink_x = []
		sink_y = []
		sink_z = []

		# For each node
		for node in nodes:
			# Get the x,y and z values
			x, y, z = node.get_position()

			# Check if this is a source
			if node.get_source():
				# Save the position of the source node
				source_x.append(x)
				source_y.append(y)
				source_z.append(z)
			# Check if this is a source
			elif node.get_sink():
				# Save the position of the source node
				sink_x.append(x)
				sink_y.append(y)
				sink_z.append(z)
			else:
				# Save the positions or random nodes
				x_vals.append(x)
				y_vals.append(y)
				z_vals.append(z)

		# Plot the values
		ax.scatter(source_x, source_y, source_z, c='g', label='Starting Position')
		ax.scatter(sink_x, sink_y, sink_z, c='r', label='Ending Position')
		ax.scatter(x_vals, y_vals, z_vals, c='b', label='Possible Waypoints')

		# For each of the edges
		for edge in edges:
			# Get the nodes for that edge
			nodes = edge.get_nodes()

			# Turn the positions into numpy arrays
			xline = [nodes[0].get_position()[0], nodes[1].get_position()[0]]
			yline = [nodes[0].get_position()[1], nodes[1].get_position()[1]]
			zline = [nodes[0].get_position()[2], nodes[1].get_position()[2]]

			# Plot the positions
			ax.plot3D(xline, yline, zline, color='gray', linestyle=":", linewidth=0.5)

		# Set the labels
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		ax.set_xlim(FigureManager.x_range["lower"], FigureManager.x_range["upper"])
		ax.set_ylim(FigureManager.y_range["lower"], FigureManager.y_range["upper"])
		ax.set_zlim(FigureManager.z_range["lower"], FigureManager.z_range["upper"])

		# Add the legend
		ax.legend()

		# Return the plot
		return plt

	def plot_selected_trajectories(self, nodes, selected_paths, figure_size=(10, 10)):
		# Create a plot
		fig = plt.figure(figsize=figure_size)
		ax = Axes3D(fig)

		# Create a list of standard nodes
		x_vals = []
		y_vals = []
		z_vals = []

		# Create a list of source and sink nodes
		source_x = []
		source_y = []
		source_z = []
		sink_x = []
		sink_y = []
		sink_z = []

		# For each node
		for node in nodes:
			# Get the x,y and z values
			x, y, z = node.get_position()

			# Check if this is a source
			if node.get_source():
				# Save the position of the source node
				source_x.append(x)
				source_y.append(y)
				source_z.append(z)
			# Check if this is a source
			elif node.get_sink():
				# Save the position of the source node
				sink_x.append(x)
				sink_y.append(y)
				sink_z.append(z)
			else:
				# Save the positions or random nodes
				x_vals.append(x)
				y_vals.append(y)
				z_vals.append(z)

		ax.scatter(x_vals, y_vals, z_vals, c='b', label='Waypoints')
		ax.scatter(source_x, source_y, source_z, c='g', label='Starting Position')
		ax.scatter(sink_x, sink_y, sink_z, c='r', label='Ending Position')

		xline = []
		yline = []
		zline = []
		# Create a list of paths
		for path in selected_paths:
			xl = []
			yl = []
			zl = []
			for point in path:
				position = point.get_position()

				# Turn the positions into numpy arrays
				xl.append(position[0])
				yl.append(position[1])
				zl.append(position[2])

			# Plot the positions
			ax.plot3D(xl, yl, zl, color='green', linestyle=":", linewidth=0.75)
			xline.append(xl)
			yline.append(yl)
			zline.append(zl)

		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		ax.set_xlim(FigureManager.x_range["lower"], FigureManager.x_range["upper"])
		ax.set_ylim(FigureManager.y_range["lower"], FigureManager.y_range["upper"])
		ax.set_zlim(FigureManager.z_range["lower"], FigureManager.z_range["upper"])

		ax.legend()

		return plt

	def plot_single_trajectory(self, path, figure_size=(10, 10)):
		# Create a plot
		fig = plt.figure(figsize=figure_size)
		ax = Axes3D(fig)

		# Create a list of standard nodes
		x_vals = []
		y_vals = []
		z_vals = []

		# Create a list of source and sink nodes
		source_x = []
		source_y = []
		source_z = []
		sink_x = []
		sink_y = []
		sink_z = []

		# Create the lines
		xline = []
		yline = []
		zline = []

		# For each node
		for i in range(0, len(path)):

			# Check if this is a source
			if i == 0:
				# Save the position of the source node
				source_x.append(path[i][0])
				source_y.append(path[i][1])
				source_z.append(path[i][2])
			# Check if this is a source
			elif i == len(path) - 1:
				# Save the position of the source node
				sink_x.append(path[i][0])
				sink_y.append(path[i][1])
				sink_z.append(path[i][2])
			else:
				# Save the positions or random nodes
				x_vals.append(path[i][0])
				y_vals.append(path[i][1])
				z_vals.append(path[i][2])

			xline.append(path[i][0])
			yline.append(path[i][1])
			zline.append(path[i][2])

		ax.scatter(x_vals, y_vals, z_vals, c='b', label='Waypoints')
		ax.scatter(source_x, source_y, source_z, c='g', label='Starting Position')
		ax.scatter(sink_x, sink_y, sink_z, c='r', label='Ending Position')
		ax.plot3D(xline, yline, zline, color='green', linestyle=":", linewidth=0.75)

		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		ax.set_xlim(FigureManager.x_range["lower"], FigureManager.x_range["upper"])
		ax.set_ylim(FigureManager.y_range["lower"], FigureManager.y_range["upper"])
		ax.set_zlim(FigureManager.z_range["lower"], FigureManager.z_range["upper"])

		ax.legend()

		return plt

	def plot_single_trajectory_top(self, path, figure_size=(10, 10)):
		# Create a plot
		fig = plt.figure(figsize=figure_size)

		# Create a list of standard nodes
		x_vals = []
		y_vals = []

		# Create a list of source and sink nodes
		source_x = []
		source_y = []
		sink_x = []
		sink_y = []

		# Create the lines
		xline = []
		yline = []

		# For each node
		for i in range(0, len(path)):

			# Check if this is a source
			if i == 0:
				# Save the position of the source node
				source_x.append(path[i][0])
				source_y.append(path[i][1])
			# Check if this is a source
			elif i == len(path) - 1:
				# Save the position of the source node
				sink_x.append(path[i][0])
				sink_y.append(path[i][1])
			else:
				# Save the positions or random nodes
				x_vals.append(path[i][0])
				y_vals.append(path[i][1])

			xline.append(path[i][0])
			yline.append(path[i][1])

		plt.scatter(x_vals, y_vals, c='b', label='Waypoints')
		plt.scatter(source_x, source_y, c='g', label='Starting Position')
		plt.scatter(sink_x, sink_y, c='r', label='Ending Position')
		plt.plot(xline, yline, color='green', linestyle=":", linewidth=0.75)

		plt.xlabel('X-axis')
		plt.ylabel('Y-axis')

		plt.xlim(FigureManager.x_range["lower"], FigureManager.x_range["upper"])
		plt.ylim(FigureManager.y_range["lower"], FigureManager.y_range["upper"])

		plt.legend()

		return plt

	def plot_single_trajectory_XZ(self, path, figure_size=(10, 10)):
		# Create a plot
		fig = plt.figure(figsize=figure_size)

		# Create a list of standard nodes
		x_vals = []
		z_vals = []

		# Create a list of source and sink nodes
		source_x = []
		source_z = []
		sink_x = []
		sink_z = []

		# Create the lines
		xline = []
		zline = []

		# For each node
		for i in range(0, len(path)):

			# Check if this is a source
			if i == 0:
				# Save the position of the source node
				source_x.append(path[i][0])
				source_z.append(path[i][2])
			# Check if this is a source
			elif i == len(path) - 1:
				# Save the position of the source node
				sink_x.append(path[i][0])
				sink_z.append(path[i][2])
			else:
				# Save the positions or random nodes
				x_vals.append(path[i][0])
				z_vals.append(path[i][2])

			xline.append(path[i][0])
			zline.append(path[i][2])

		plt.scatter(x_vals, z_vals, c='b', label='Waypoints')
		plt.scatter(source_x, source_z, c='g', label='Starting Position')
		plt.scatter(sink_x, sink_z, c='r', label='Ending Position')
		plt.plot(xline, zline, color='green', linestyle=":", linewidth=0.75)

		plt.xlabel('X-axis')
		plt.ylabel('Z-axis')

		plt.xlim(FigureManager.x_range["lower"], FigureManager.x_range["upper"])
		plt.ylim(FigureManager.z_range["lower"], FigureManager.z_range["upper"])

		plt.legend()

		return plt

	def plot_single_trajectory_YZ(self, path, figure_size=(10, 10)):
		# Create a plot
		fig = plt.figure(figsize=figure_size)

		# Create a list of standard nodes
		y_vals = []
		z_vals = []

		# Create a list of source and sink nodes
		source_y = []
		source_z = []
		sink_y = []
		sink_z = []

		# Create the lines
		yline = []
		zline = []

		# For each node
		for i in range(0, len(path)):

			# Check if this is a source
			if i == 0:
				# Save the position of the source node
				source_y.append(path[i][1])
				source_z.append(path[i][2])
			# Check if this is a source
			elif i == len(path) - 1:
				# Save the position of the source node
				sink_y.append(path[i][1])
				sink_z.append(path[i][2])
			else:
				# Save the positions or random nodes
				y_vals.append(path[i][1])
				z_vals.append(path[i][2])

			yline.append(path[i][1])
			zline.append(path[i][2])

		plt.scatter(y_vals, z_vals, c='b', label='Waypoints')
		plt.scatter(source_y, source_z, c='g', label='Starting Position')
		plt.scatter(sink_y, sink_z, c='r', label='Ending Position')
		plt.plot(yline, zline, color='green', linestyle=":", linewidth=0.75)

		plt.xlabel('Y-axis')
		plt.ylabel('Z-axis')

		plt.xlim(FigureManager.y_range["lower"], FigureManager.y_range["upper"])
		plt.ylim(FigureManager.z_range["lower"], FigureManager.z_range["upper"])

		plt.legend()

		return plt