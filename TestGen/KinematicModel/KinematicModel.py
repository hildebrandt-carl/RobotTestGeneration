import math
import copy
import mpmath
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# Motor speeds in RPM
w1 = 6000
w2 = 5990
w3 = 6000
w4 = 6010

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
max_rotor_speed = 10000

# Sampling steps
motor_speed_sampling_steps = 3

# Calculate all reachable positions based on this
drone_state = {}
drone_state['position'] = [0, 0, 0]
drone_state['attitude'] = [0, 0, 0]
drone_state['velocity'] = [0, 0, 0]
drone_state['angularv'] = [0, 0, 0]

# The drones new state
new_state = copy.deepcopy(drone_state)

# Calculate the space we will search
search_space = np.linspace(0, max_rotor_speed, motor_speed_sampling_steps)

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

omega_x = new_state['angularv'][0]
omega_y = new_state['angularv'][1]
omega_z = new_state['angularv'][2]

Iw_array = np.array([[((Iyy-Izz)/(Ixx))*omega_y*omega_z],
										 [((Izz-Ixx)/(Iyy))*omega_x*omega_z],
										 [((Ixx-Iyy)/(Izz))*omega_x*omega_y]])

derivative_omega_array = Iw_array + np.dot(inverse_I_array, short_u_array)

# Update the angular frame velocities
new_state['angularv'][0] += derivative_omega_array[0][0]
new_state['angularv'][1] += derivative_omega_array[1][0]
new_state['angularv'][2] += derivative_omega_array[2][0]

# Calculate the rpy' values
omega_array = np.array([[new_state['angularv'][0]],
												[new_state['angularv'][1]],
												[new_state['angularv'][2]]])

r = new_state['attitude'][0]
p = new_state['attitude'][1]
y = new_state['attitude'][2]

rpy_derivative_rotation_array = np.array([[1, math.sin(r) * math.tan(p), math.cos(r) * math.tan(p)],
																					[0, math.cos(r), -1 * math.sin(r)],
																					[0, math.sin(r) * mpmath.sec(p), math.cos(r) * mpmath.sec(p)]])

rpy_derivative = np.dot(rpy_derivative_rotation_array, omega_array)

# Update the roll pitch yaw values
new_state['attitude'][0] += rpy_derivative[0][0]
new_state['attitude'][1] += rpy_derivative[1][0]
new_state['attitude'][2] += rpy_derivative[2][0]

# Update the rpy values
r = new_state['attitude'][0]
p = new_state['attitude'][1]
y = new_state['attitude'][2]

# Calculate acceleration array
gravity_array = np.array([[0],
													[0],
													[-1 * g]])

rotation_array = np.array([[math.cos(r)*math.cos(y)*math.sin(p)+math.sin(r)*math.sin(y)],
													 [math.cos(r)*math.sin(p)*math.sin(y)-math.cos(y)*math.sin(r)],
													 [math.cos(p)*math.cos(r)]])

acceleration_array = gravity_array + (1.0/m) * rotation_array * u_array[0]

# Update the velocity and acceleration of the vehicle
new_state['velocity'][0] += acceleration_array[0][0]
new_state['velocity'][1] += acceleration_array[1][0]
new_state['velocity'][2] += acceleration_array[2][0]

new_state['position'][0] += new_state['velocity'][0]
new_state['position'][1] += new_state['velocity'][1]
new_state['position'][2] += new_state['velocity'][2]

# Create the figure
fig = plt.figure()
ax = Axes3D(fig)

ax.scatter(drone_state['position'][0], drone_state['position'][1], drone_state['position'][2], c='g', marker='o', label="Original position", s=5)
ax.scatter(new_state['position'][0], new_state['position'][1], new_state['position'][2], c='r', marker='o', label="Final position", s=5)

# Set the labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

ax.set_xlim([min(-5, new_state['position'][0]), max(5, new_state['position'][0])])
ax.set_ylim([min(-5, new_state['position'][1]), max(5, new_state['position'][1])])
ax.set_zlim([min(-5, new_state['position'][2]), max(5, new_state['position'][2])])

# Add the legend
ax.legend()
plt.show()
