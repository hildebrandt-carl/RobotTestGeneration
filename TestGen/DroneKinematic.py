import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

drone_position = [0,0,0]

# Calculate all reachable positions based on this

drone_state = {}
drone_state['position'] = [0, 0, 0]
drone_state['attitude'] = [0, 0, 0]
drone_state['velocity'] = [0, 0, 0]
drone_state['angularv'] = [0, 0, 0]

kf = 1
d = 1
km = 1

number_steps = 5
max_rotor_speed = 1000

search_space = np.linspace(0, 1, number_steps)

# For all combinations of motor speeds
for w1 in search_space:
	for w2 in search_space:
		for w3 in search_space:
			for w4 in search_space:
				# Calculate u values
				a = np.array([[1, 2], [3, 4]])
				b = np.array([[1, 2], [3, 4]])
				c = np.multiply(a, b)








# Create the figure
fig = plt.figure()
ax = Axes3D(fig)

# Create a list of standard nodes
x_vals = []
y_vals = []
z_vals = []

ax.scatter(drone_position[0], drone_position[1], drone_position[2], c='g', label='Drone Position')

# Set the labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Add the legend
ax.legend()






plt.show()