import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from DroneKinematic import DroneKinematic
from copy import copy

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

# Create the drone kinematic object
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

# Get the current position
init_pos = drone_object.get_position()
drone_obj_copy = copy(drone_object)
# Get the position after 1 time step
first_pos = drone_object.next_state(w1=6000, w2=6000, w3=6000, w4=6000)
# Get the position after 2 time step
sec_pos = drone_object.next_state(w1=6000, w2=6000, w3=6000, w4=6000)

# Now move the second object 1 position
first_pos_copy = drone_obj_copy.next_state(w1=5990, w2=6000, w3=6010, w4=6000)

# Create the figure
fig = plt.figure()
ax = Axes3D(fig)

# Plot the current and new position
ax.scatter(init_pos[0], init_pos[1], init_pos[2], c='g', marker='o', label="Initial position", s=5)
ax.scatter(first_pos[0], first_pos[1], first_pos[2], c='r', marker='o', label="First position", s=5)
ax.scatter(sec_pos[0], sec_pos[1], sec_pos[2], c='b', marker='o', label="Second position", s=5)
ax.scatter(first_pos_copy[0], first_pos_copy[1], first_pos_copy[2], c='k', marker='o', label="Copy Drone", s=5)

# Set the labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Set the axis limit
ax.set_xlim([min(-5, first_pos_copy[0]), max(5, first_pos_copy[0])])
ax.set_ylim([min(-5, sec_pos[1]), max(5, sec_pos[1])])
ax.set_zlim([min(-5, sec_pos[2]), max(5, sec_pos[2])])

# Add the legend
ax.legend()
plt.show()
