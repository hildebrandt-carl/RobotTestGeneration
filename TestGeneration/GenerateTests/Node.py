import math


class Node:
	# Class variable shared among all instances
	# Keeps track of the bounds the x,y and z bounds should be set
	x_range = {"lower": 0, "upper": 1}
	y_range = {"lower": 0, "upper": 1}
	z_range = {"lower": 0, "upper": 1}

	def __init__(self, source=False, sink=False):
		# Used to save the x position
		self.x = math.inf
		# Used to save the y position
		self.y = math.inf
		# Used to save the z position
		self.z = math.inf
		# Used to keep track if the node is a source or a sink
		self.source = source
		self.sink = sink
		# Used to keep track of other nodes this node connects to
		self.edges_out = []

	# Over ride the default Equals behavior
	def __eq__(self, other):
		# List all the conditions for equality
		cond1 = self.x == other.x
		cond2 = self.y == other.y
		cond3 = self.z == other.z
		cond4 = self.source == other.source
		cond5 = self.sink == other.sink
		cond6 = len(self.edges_out) == len(other.edges_out)

		# Return if all the conditions are met
		return cond1 and cond2 and cond3 and cond4 and cond5 and cond6

	# Return the position
	def get_position(self):
		return self.x, self.y, self.z

	# Return the distances
	def set_position(self, x, y, z):
		# Assume the position is correct
		correct = True

		# Set the x
		if Node.x_range["lower"] <= x <= Node.x_range["upper"]:
			self.x = x
		else:
			correct = False

		# Set the y
		if Node.y_range["lower"] <= y <= Node.y_range["upper"]:
			self.y = y
		else:
			correct = False

		# Set the z
		if Node.z_range["lower"] <= z <= Node.z_range["upper"]:
			self.z = z
		else:
			correct = False

		if not correct:
			# Clear the values
			self.reset_node()
			return -1
		else:
			return 1

	# Reset the nodes position
	def reset_node(self):
		# Clear the values
		self.x = math.inf
		self.y = math.inf
		self.z = math.inf
		return 1

	# Calculates the distance to another node
	def distance_to_node(self, node_in):
		# Get the other nodes position
		x, y, z = node_in.get_position()

		# Calculate the distance between the nodes
		d = math.sqrt(math.pow(self.x-x, 2) + math.pow(self.y-y, 2) + math.pow(self.z-z, 2))

		# Return the distance
		return d

	# Return if the node is a source node
	def get_source(self):
		return self.source

	# Return if the node is a sink node
	def get_sink(self):
		return self.sink

	# Return the edges connected to this node
	def get_edges_out(self):
		return self.edges_out

	# Return the edges connected to this node
	def add_edge(self, edge):
		# Get the nodes for this edge
		nodes = edge.get_nodes()
		# Check that one of the edges nodes is this node
		if nodes[0] == self or nodes[1] == self:
			self.edges_out.append(edge)
		else:
			print("Cant add edge to node when the edge does not contain this node")
			exit()
