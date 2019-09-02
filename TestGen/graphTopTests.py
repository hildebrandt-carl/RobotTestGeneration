import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string


plot_first = 100

all_folders = ["./Results/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_kinematic_angle180/"]

beam_lengths = [10]
depths = [10]
res_numbers = [4]

systems = ["waypoint",
           "constant"]

system_constant_dev = []
system_constant_angles = []
system_constant_magnitudes = []
system_constant_xangles = []
system_constant_yangles = []
system_constant_zangles = []
system_constant_xvel = []
system_constant_yvel = []
system_constant_zvel = []

system_waypoint_dev = []
system_waypoint_angles = []
system_waypoint_magnitudes = []
system_waypoint_xangles = []
system_waypoint_yangles = []
system_waypoint_zangles = []
system_waypoint_xvel = []
system_waypoint_yvel = []
system_waypoint_zvel = []

for system in systems:

    distance_heuristic_box_data = []
    time_heuristic_box_data = []

    for i in range(0, len(all_folders)):

        file_location = all_folders[i]
        analysis_file_names = glob.glob(file_location + "maps/map*/analysis_" + system + ".txt")

        # Make sure we go in order from highest score to lowest score
        total_files = len(analysis_file_names)
        file_counter = 0

        # Used to save all the data
        all_dev_per_waypoint = []
        all_flight_angles = []
        all_flight_magnitudes = []
        all_flight_xangle = []
        all_flight_yangle = []
        all_flight_zangle = []
        all_flight_xvel = []
        all_flight_yvel = []
        all_flight_zvel = []

        for file_counter in range(1, total_files + 1):

            # Create the file name
            flight_log_file = file_location + "maps/map" + str(file_counter) + "/analysis_" + system + ".txt"
            flight_details_file = file_location + "maps/map" + str(file_counter) + "/details.txt"

            # Get the data
            dev_per_waypoint = get_numbers_after_string(file_name=flight_log_file,
                                                        the_string="Average deviation between waypoints:")
            euler_angles = get_numbers_after_string(file_name=flight_details_file,
                                                    the_string="Euler Angles:")
            velocties = get_numbers_after_string(file_name=flight_details_file,
                                                 the_string="Velocity")
            flight_angles = get_numbers_after_string(file_name=flight_details_file,
                                                        the_string="Angles:")
            flight_magnitudes = get_numbers_after_string(file_name=flight_details_file,
                                                         the_string="Out Vector Magnitude:")

            # Get the euler angles
            x_angle = []
            y_angle = []
            z_angle = []
            x_vel = []
            y_vel = []
            z_vel = []
            euler_angles = euler_angles[0]
            velocties = velocties[0]
            while len(euler_angles) > 0:
                # Save the x y and z angles
                x_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                y_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                z_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                # Save the x y and z velocities
                x_vel.append(abs(velocties[0]))
                velocties = velocties[1:]
                y_vel.append(abs(velocties[0]))
                velocties = velocties[1:]
                z_vel.append(abs(velocties[0]))
                velocties = velocties[1:]

            # There is no imcoming velocity on the first waypoint
            flight_magnitudes[0] = flight_magnitudes[0]

            # Save the data
            all_dev_per_waypoint.extend(dev_per_waypoint[0])
            all_flight_angles.extend(flight_angles[0])
            all_flight_magnitudes.extend([0])
            all_flight_magnitudes.extend(flight_magnitudes[0][:-1])
            all_flight_xangle.extend(x_angle)
            all_flight_yangle.extend(y_angle)
            all_flight_zangle.extend(z_angle)
            all_flight_xvel.extend(x_vel)
            all_flight_yvel.extend(y_vel)
            all_flight_zvel.extend(z_vel)

        # Save the data into each respective system
        if system == "constant":
            system_constant_dev.extend(all_dev_per_waypoint)
            system_constant_angles.extend(all_flight_angles)
            system_constant_magnitudes.extend(all_flight_magnitudes)
            system_constant_xangles.extend(all_flight_xangle)
            system_constant_yangles.extend(all_flight_yangle)
            system_constant_zangles.extend(all_flight_zangle)
            system_constant_xvel.extend(all_flight_xvel)
            system_constant_yvel.extend(all_flight_yvel)
            system_constant_zvel.extend(all_flight_zvel)
        elif system == "waypoint":
            system_waypoint_dev.extend(all_dev_per_waypoint)
            system_waypoint_angles.extend(all_flight_angles)
            system_waypoint_magnitudes.extend(all_flight_magnitudes)
            system_waypoint_xangles.extend(all_flight_xangle)
            system_waypoint_yangles.extend(all_flight_yangle)
            system_waypoint_zangles.extend(all_flight_zangle)
            system_waypoint_xvel.extend(all_flight_xvel)
            system_waypoint_yvel.extend(all_flight_yvel)
            system_waypoint_zvel.extend(all_flight_zvel)

# Sort according to score
system_constant_dev, system_constant_angles, system_constant_magnitudes, system_constant_xangles, system_constant_yangles, system_constant_zangles, system_constant_xvel, system_constant_yvel, system_constant_zvel = list(zip(*sorted(zip(system_constant_dev,
                                                                                               system_constant_angles,
                                                                                               system_constant_magnitudes,
                                                                                               system_constant_xangles,
                                                                                               system_constant_yangles,
                                                                                               system_constant_zangles,
                                                                                               system_constant_xvel,
                                                                                               system_constant_yvel,
                                                                                               system_constant_zvel), reverse=True)))
system_waypoint_dev, system_waypoint_angles, system_waypoint_magnitudes, system_waypoint_xangles, system_waypoint_yangles, system_waypoint_zangles, system_waypoint_xvel, system_waypoint_yvel, system_waypoint_zvel = list(zip(*sorted(zip(system_waypoint_dev,
                                                                                               system_waypoint_angles,
                                                                                               system_waypoint_magnitudes,
                                                                                               system_waypoint_xangles,
                                                                                               system_waypoint_yangles,
                                                                                               system_waypoint_zangles,
                                                                                               system_waypoint_xvel,
                                                                                               system_waypoint_yvel,
                                                                                               system_waypoint_zvel), reverse=True)))

# # How many do we want to plot
# number_to_plot = 10
#
# # Plot the data
# plt.figure()
# plt.scatter(system_constant_magnitudes[:number_to_plot], system_constant_angles[:number_to_plot], label="Most Deviation", s=10)
# plt.scatter(system_constant_magnitudes[-number_to_plot:], system_constant_angles[-number_to_plot:], label="Least Deviation", s=10)
# plt.xlabel("Linear Speed")
# plt.ylabel("Angle")
# plt.legend()
# plt.title("Constant Velocity Controller")
# plt.show()


# # Plot the data
# plt.figure()
# plt.scatter(system_waypoint_magnitudes[:number_to_plot], system_waypoint_angles[:number_to_plot], label="Most Deviation", s=10)
# plt.scatter(system_waypoint_magnitudes[-number_to_plot:], system_waypoint_angles[-number_to_plot:], label="Least Deviation", s=10)
# plt.xlabel("Linear Speed")
# plt.ylabel("Angle")
# plt.title("Waypoint Controller")
# plt.show()



# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 10))
ax1.scatter(system_constant_xangles, system_constant_dev, s=10)
ax1.set_xlabel("X Angle")
ax1.set_ylabel("Deviation")
ax1.set_title("Constant Velocity Controller")

ax2.scatter(system_constant_yangles, system_constant_dev, s=10)
ax2.set_xlabel("Y Angle")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Velocity Controller")

ax3.scatter(system_constant_zangles, system_constant_dev, s=10)
ax3.set_xlabel("Z Angle")
ax3.set_ylabel("Deviation")
ax3.set_title("Constant Velocity Controller")
plt.show()





# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 10))
ax1.scatter(system_waypoint_xangles, system_waypoint_dev, s=10)
ax1.set_xlabel("X Angle")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")

ax2.scatter(system_waypoint_yangles, system_waypoint_dev, s=10)
ax2.set_xlabel("Y Angle")
ax2.set_ylabel("Deviation")
ax2.set_title("Waypoint Controller")

ax3.scatter(system_waypoint_zangles, system_waypoint_dev, s=10)
ax3.set_xlabel("Z Angle")
ax3.set_ylabel("Deviation")
ax3.set_title("Waypoint Controller")
plt.show()






# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 10))
ax1.scatter(system_constant_xvel, system_constant_dev, s=10)
ax1.set_xlabel("X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Constant Velocity Controller")

ax2.scatter(system_constant_yvel, system_constant_dev, s=10)
ax2.set_xlabel("Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Velocity Controller")

ax3.scatter(system_constant_zvel, system_constant_dev, s=10)
ax3.set_xlabel("Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Constant Velocity Controller")
plt.show()





# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 10))
ax1.scatter(system_waypoint_xvel, system_waypoint_dev, s=10)
ax1.set_xlabel("X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")

ax2.scatter(system_waypoint_yvel, system_waypoint_dev, s=10)
ax2.set_xlabel("Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Waypoint Controller")

ax3.scatter(system_waypoint_zvel, system_waypoint_dev, s=10)
ax3.set_xlabel("Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Waypoint Controller")
plt.show()
