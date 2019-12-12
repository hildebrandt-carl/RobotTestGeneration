import math
import copy
import mpmath
from queue import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

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
motor_speed_sampling_steps = 18

# Keeps track of all possible drone states
possible_drone_states = Queue()
plotting_drone_state = Queue()

# Calculate all reachable positions based on this
# Drone state x, y, z, r, p, y, vx, vy, vz, wx, wy, wz, time_step
start_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Add the starting state to possible drone states
possible_drone_states.put(start_state)

# Calculate the space we will search
search_space = np.linspace(0, max_rotor_speed, motor_speed_sampling_steps)

# Count the number of states processed
states_processed = 0

# Calculate all possible states
possible_states = 0
for i in range(1, total_time_steps+1):
	possible_states += math.pow(math.pow(motor_speed_sampling_steps, 4), i)

# While we have no completed the number of time steps
while not possible_drone_states.empty():
	# Increment the state processor
	states_processed += 1
	percentage = (states_processed / possible_states) * 100
	print("Processing: " + str(round(percentage, 2)) + " %")

	# Get a new state
	current_processing_state = possible_drone_states.get()

	# If the current drone state has a time step greater than request ignore it
	if current_processing_state[12] < total_time_steps:

		# For all combinations of motor speeds (create new states)
		for w1 in search_space:
			for w2 in search_space:
				for w3 in search_space:
					for w4 in search_space:

						# Create a new drone state
						new_drone_state = np.array(current_processing_state, dtype=float)

						# Calculate u values
						constant_matrix = np.array([[kf, kf, kf, kf],
																				[0, d * kf, 0, -d * kf],
																				[-d * kf, 0, d * kf, 0],
																				[km, -km, km, -km]])
						rotor_speed_array = np.array([[math.pow(w1, 2)],
																					[math.pow(w2, 2)],
																					[math.pow(w3, 2)],
																					[math.pow(w4, 2)]])
						u_array = np.dot(constant_matrix, rotor_speed_array)

						# Calculate w' values
						short_u_array = np.array([[u_array[1][0]],
																			[u_array[2][0]],
																			[u_array[3][0]]])

						inverse_I_array = np.array([[1.0/Ixx, 0, 0],
																				[0, 1.0/Iyy, 0],
																				[0, 0, 1.0/Izz]])

						omega_x = new_drone_state[9]
						omega_y = new_drone_state[10]
						omega_z = new_drone_state[11]

						Iw_array = np.array([[((Iyy-Izz)/(Ixx))*omega_y*omega_z],
																 [((Izz-Ixx)/(Iyy))*omega_x*omega_z],
																 [((Ixx-Iyy)/(Izz))*omega_x*omega_y]])

						derivative_omega_array = Iw_array + np.dot(inverse_I_array, short_u_array)

						# Update the angular frame velocities
						new_drone_state[9] += derivative_omega_array[0][0]
						new_drone_state[10] += derivative_omega_array[1][0]
						new_drone_state[11] += derivative_omega_array[2][0]

						# Update the omega values
						omega_x = new_drone_state[9]
						omega_y = new_drone_state[10]
						omega_z = new_drone_state[11]

						# Calculate the rpy' values
						omega_array = np.array([[omega_x],
																		[omega_y],
																		[omega_z]])

						r = new_drone_state[3]
						p = new_drone_state[4]
						y = new_drone_state[5]

						rpy_derivative_rotation_array = np.array([[1, math.sin(r) * math.tan(p), math.cos(r) * math.tan(p)],
																											[0, math.cos(r), -1 * math.sin(r)],
																											[0, math.sin(r) * mpmath.sec(p), math.cos(r) * mpmath.sec(p)]])

						rpy_derivative = np.dot(rpy_derivative_rotation_array, omega_array)

						# Update the roll pitch yaw values
						new_drone_state[3] += rpy_derivative[0][0]
						new_drone_state[4] += rpy_derivative[1][0]
						new_drone_state[5] += rpy_derivative[2][0]

						# Update the rpy values
						r = new_drone_state[3]
						p = new_drone_state[4]
						y = new_drone_state[5]

						# Calculate acceleration array
						gravity_array = np.array([[0],
																			[0],
																			[-1 * g]])

						rotation_array = np.array([[math.cos(r)*math.cos(y)*math.sin(p)+math.sin(r)*math.sin(y)],
																			 [math.cos(r)*math.sin(p)*math.sin(y)-math.cos(y)*math.sin(r)],
																			 [math.cos(p)*math.cos(r)]])

						acceleration_array = gravity_array + (1.0/m) * rotation_array * u_array[0]

						# Update the velocity and acceleration of the vehicle
						new_drone_state[6] += acceleration_array[0][0]
						new_drone_state[7] += acceleration_array[1][0]
						new_drone_state[8] += acceleration_array[2][0]

						new_drone_state[0] += new_drone_state[6]
						new_drone_state[1] += new_drone_state[7]
						new_drone_state[2] += new_drone_state[8]

						new_drone_state[12] += 1

						# Check to make sure the drone has not hit the floor
						if new_drone_state[2] > 0:
							# Append the new drone state to the possible positions
							possible_drone_states.put(new_drone_state)

	# We are done with this state, so we can now plot it
	plotting_drone_state.put(np.array(current_processing_state, dtype=float))

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
	x_pos.append(drone_state[0])
	y_pos.append(drone_state[1])
	z_pos.append(drone_state[2])

# Calculate the convex hull
points = np.column_stack((x_pos, y_pos, z_pos))
hull = ConvexHull(points)

print(points)

ax.scatter(x_pos, y_pos, z_pos, c='g', marker='o', label="drone position", s=3)

for s in hull.simplices:
	s = np.append(s, s[0])  # Here we cycle back to the first coordinate
	ax.plot(points[s, 0], points[s, 1], points[s, 2], color='r', linestyle='-', linewidth=0.75)

# Set the labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Add the legend
ax.legend()
plt.show()