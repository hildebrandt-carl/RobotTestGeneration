import math


class Edge:
	# Initialize the edge
	def __init__(self, node1, node2):
		# Used to save what nodes it connects
		self.nodes = []
		self.nodes.append(node1)
		self.nodes.append(node2)

		# Used to save the distance
		self.distance = math.inf
		# Used to calculate the headings
		self.heading = math.inf

	# Return the nodes
	def get_nodes(self):
		return self.nodes

	# Return the distance
	def get_distance(self):
		return self.distance

	# Return the heading
	def get_heading(self):
		return self.heading

	# Set the distance if its already known
	def set_distance(self, d):
		self.distance = d

	# Calculate the distance if its not known
	def calculate_distance(self):
		self.distance = self.nodes[0].distance_to_node(self.nodes[1])