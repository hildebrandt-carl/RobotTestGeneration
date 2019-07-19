from copy import copy
from queue import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from DroneKinematic import DroneKinematic
from DroneReachabilitySet import DroneReachabilitySet

# Waypoint we are considering
waypoint = np.array([[3, 4, 5],
										 [3, 4, 6],
										 [0, 0, 1],
										 [0, 0, 2]])

# Motor Characteristics
kf = 6.11e-8
km = 1.5e-9

# Inertial properties
# Hummingbird values (Unmanned Aerial Vehicles pg:311)
Ixx = 2.32e-3
Iyy = 2.32e-3
Izz = 4.00e-3

# Vehicle Properties
d = 0.175
m = 0.5

# Physics properties
g = 9.8
max_rotor_speed = 6000

# Simulation steps
total_time_steps = 1
motor_speed_sampling_steps = 5

# Keeps track of all possible drone states
possible_drone_states = Queue()
plotting_drone_state = Queue()

# Create an initial state
drone_object = DroneKinematic(mass=m,
															arm_length=d,
															thrust_constant=kf,
															moment_constant=km,
															max_rotor_speed=max_rotor_speed,
															inertial_properties=[Ixx, Iyy, Izz],
															position=[0, 0, 0],
															attitude=[0, 0, 0],
															velocity=[0, 0, 0],
															angular_vel=[0, 0, 0])

# Add the starting state to possible drone states
possible_drone_states.put(drone_object)

# Calculate the space we will search
search_space = np.linspace(0, max_rotor_speed, motor_speed_sampling_steps)

# While we have no completed the number of time steps
while not possible_drone_states.empty():

	# Get a new state
	current_processing_state = possible_drone_states.get()

	# If the current drone state has a time step greater than request ignore it
	if current_processing_state.get_time_step() < total_time_steps:

		# For all combinations of motor speeds (create new states)
		for w1 in search_space:
			for w2 in search_space:
				for w3 in search_space:
					for w4 in search_space:

						# Create a new drone state
						new_drone_state = copy(current_processing_state)

						# Update the new drones position
						new_position = new_drone_state.next_state(w1=w1, w2=w2, w3=w3, w4=w4)

						# Check to make sure the drone has not hit the floor
						if new_position[2] > 0:
							# Append the new drone state to the possible positions
							possible_drone_states.put(new_drone_state)

	# We are done with this state, so we can now plot it
	plotting_drone_state.put(current_processing_state)

# Create the figure
fig = plt.figure()
ax = Axes3D(fig)

# Used to save the positions and angle
x_pos = []
y_pos = []
z_pos = []

print("Plotting " + str(plotting_drone_state.qsize()) + " states")

# For each item in the queue
while not plotting_drone_state.empty():
	drone_state = plotting_drone_state.get()
	drone_pos = drone_state.get_position()
	x_pos.append(drone_pos[0])
	y_pos.append(drone_pos[1])
	z_pos.append(drone_pos[2])

# Create the reachability set
reachability_obj = DroneReachabilitySet(x_positions=x_pos,
																				y_positions=y_pos,
																				z_positions=z_pos)

# Get the points that make the hull
x_hull, y_hull, z_hull = reachability_obj.get_hull_simplices()

# Plot the hull
for x, y, z in zip(x_hull, y_hull, z_hull):
	ax.plot(x, y, z, color='r', linestyle='-', linewidth=0.75)


inside = []
outside = []
# For each waypoint check if it is inside or outside the reachable area:
for w in waypoint:
	# If it is inside
	if reachability_obj.is_in_hull([w]):
		inside.append(list(w))
	# if it is outside
	else:
		outside.append(list(w))

outside = np.array(outside)
inside = np.array(inside)

ax.scatter(inside[:, 0], inside[:, 1], inside[:, 2], c='b', marker='o', label="Inside", s=6)
ax.scatter(outside[:, 0], outside[:, 1], outside[:, 2], c='g', marker='o', label="Outside", s=6)

# Set the labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Add the legend
ax.legend()
plt.show()