import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string
from sklearn.linear_model import LinearRegression
from mpl_toolkits.mplot3d import Axes3D

plot_first = 100

all_folders = ["./Results/VariationTest/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_kinematic_angle180/"]

beam_lengths = [10]
depths = [10]
res_numbers = [4]

systems = ["waypoint",
           "constant"]

system_constant_dev = []
system_constant_time = []
system_constant_angles = []
system_constant_magnitudes = []
system_constant_xangles = []
system_constant_yangles = []
system_constant_zangles = []
system_constant_xvel = []
system_constant_yvel = []
system_constant_zvel = []
system_constant_delta_xvel = []
system_constant_delta_yvel = []
system_constant_delta_zvel = []

system_waypoint_dev = []
system_waypoint_time = []
system_waypoint_angles = []
system_waypoint_magnitudes = []
system_waypoint_xangles = []
system_waypoint_yangles = []
system_waypoint_zangles = []
system_waypoint_xvel = []
system_waypoint_yvel = []
system_waypoint_zvel = []
system_waypoint_delta_xvel = []
system_waypoint_delta_yvel = []
system_waypoint_delta_zvel = []

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
        all_time_per_waypoint = []
        all_flight_angles = []
        all_flight_magnitudes = []
        all_flight_xangle = []
        all_flight_yangle = []
        all_flight_zangle = []
        all_flight_xvel = []
        all_flight_yvel = []
        all_flight_zvel = []
        all_flight_delta_xvel = []
        all_flight_delta_yvel = []
        all_flight_delta_zvel = []

        for file_counter in range(1, total_files + 1):

            # Create the file name
            flight_log_file = file_location + "maps/map" + str(file_counter) + "/analysis_" + system + ".txt"
            flight_details_file = file_location + "maps/map" + str(file_counter) + "/details.txt"

            # Get the data
            dev_per_waypoint = get_numbers_after_string(file_name=flight_log_file,
                                                        the_string="Average deviation between waypoints:")
            time_per_waypoint = get_numbers_after_string(file_name=flight_log_file,
                                                         the_string="Time between waypoints:")
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

            # We want out going velocities (remove the first three)
            velocties = velocties[3:]

            while len(euler_angles) > 0:
                # Save the x y and z angles
                x_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                y_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                z_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                # Save the x y and z velocities
                x_vel.append(velocties[0])
                velocties = velocties[1:]
                y_vel.append(velocties[0])
                velocties = velocties[1:]
                z_vel.append(velocties[0])
                velocties = velocties[1:]

            # There is no imcoming velocity on the first waypoint
            flight_magnitudes[0] = flight_magnitudes[0]

            # Save the data
            if len(dev_per_waypoint[0]) != len(x_angle):
                print("Error in: " + str(flight_log_file))
                print(len(dev_per_waypoint[0]))
                print(len(x_angle))

            for asd in dev_per_waypoint[0]:
                if asd > 5:
                    print("Large Deviation Detected!")
                    print(flight_log_file)

            all_dev_per_waypoint.extend(dev_per_waypoint[0])
            all_time_per_waypoint.extend(time_per_waypoint[0])
            all_flight_angles.extend(flight_angles[0])
            all_flight_magnitudes.extend([0])
            all_flight_magnitudes.extend(flight_magnitudes[0][:-1])
            all_flight_xangle.extend(x_angle)
            all_flight_yangle.extend(y_angle)
            all_flight_zangle.extend(z_angle)
            all_flight_xvel.extend([abs(temp) for temp in x_vel])
            all_flight_yvel.extend([abs(temp) for temp in y_vel])
            all_flight_zvel.extend([abs(temp) for temp in z_vel])

            # Save change in velocities
            previous_x = 0
            previous_y = 0
            previous_z = 0
            for change_calc in range(0, len(x_vel)):
                all_flight_delta_xvel.extend([x_vel[change_calc] - previous_x])
                all_flight_delta_yvel.extend([y_vel[change_calc] - previous_y])
                all_flight_delta_zvel.extend([z_vel[change_calc] - previous_z])
                previous_x = x_vel[change_calc]
                previous_y = y_vel[change_calc]
                previous_z = z_vel[change_calc]

        # Save the data into each respective system
        if system == "constant":
            system_constant_dev.extend(all_dev_per_waypoint)
            system_constant_time.extend(all_time_per_waypoint)
            system_constant_angles.extend(all_flight_angles)
            system_constant_magnitudes.extend(all_flight_magnitudes)
            system_constant_xangles.extend(all_flight_xangle)
            system_constant_yangles.extend(all_flight_yangle)
            system_constant_zangles.extend(all_flight_zangle)
            system_constant_xvel.extend(all_flight_xvel)
            system_constant_yvel.extend(all_flight_yvel)
            system_constant_zvel.extend(all_flight_zvel)
            system_constant_delta_xvel.extend(all_flight_delta_xvel)
            system_constant_delta_yvel.extend(all_flight_delta_yvel)
            system_constant_delta_zvel.extend(all_flight_delta_zvel)
        elif system == "waypoint":
            system_waypoint_dev.extend(all_dev_per_waypoint)
            system_waypoint_time.extend(all_time_per_waypoint)
            system_waypoint_angles.extend(all_flight_angles)
            system_waypoint_magnitudes.extend(all_flight_magnitudes)
            system_waypoint_xangles.extend(all_flight_xangle)
            system_waypoint_yangles.extend(all_flight_yangle)
            system_waypoint_zangles.extend(all_flight_zangle)
            system_waypoint_xvel.extend(all_flight_xvel)
            system_waypoint_yvel.extend(all_flight_yvel)
            system_waypoint_zvel.extend(all_flight_zvel)
            system_waypoint_delta_xvel.extend(all_flight_delta_xvel)
            system_waypoint_delta_yvel.extend(all_flight_delta_yvel)
            system_waypoint_delta_zvel.extend(all_flight_delta_zvel)

# Turn all the lists into numpy arrays
system_constant_xangles = np.asarray(system_constant_xangles)
system_constant_yangles = np.asarray(system_constant_yangles)
system_constant_zangles = np.asarray(system_constant_zangles)
system_constant_xvel = np.asarray(system_constant_xvel)
system_constant_yvel = np.asarray(system_constant_yvel)
system_constant_zvel = np.asarray(system_constant_zvel)
system_constant_dev = np.asarray(system_constant_dev)
system_constant_time = np.asarray(system_constant_time)
system_constant_angles = np.asarray(system_constant_angles)
system_constant_magnitudes = np.asarray(system_constant_magnitudes)
system_constant_delta_xvel = np.asarray(system_constant_delta_xvel)
system_constant_delta_yvel = np.asarray(system_constant_delta_yvel)
system_constant_delta_zvel = np.asarray(system_constant_delta_zvel)

system_waypoint_xangles = np.asarray(system_waypoint_xangles)
system_waypoint_yangles = np.asarray(system_waypoint_yangles)
system_waypoint_zangles = np.asarray(system_waypoint_zangles)
system_waypoint_xvel = np.asarray(system_waypoint_xvel)
system_waypoint_yvel = np.asarray(system_waypoint_yvel)
system_waypoint_zvel = np.asarray(system_waypoint_zvel)
system_waypoint_dev = np.asarray(system_waypoint_dev)
system_waypoint_time = np.asarray(system_waypoint_time)
system_waypoint_angles = np.asarray(system_waypoint_angles)
system_waypoint_magnitudes = np.asarray(system_waypoint_magnitudes)
system_waypoint_delta_xvel = np.asarray(system_waypoint_delta_xvel)
system_waypoint_delta_yvel = np.asarray(system_waypoint_delta_yvel)
system_waypoint_delta_zvel = np.asarray(system_waypoint_delta_zvel)

# Sort all the lists according to deviaton
constant_sorted_indicies = system_constant_xangles.argsort()
waypoint_sorted_indicies = system_waypoint_dev.argsort()

# Sort the lists
system_constant_xangles = system_constant_xangles[constant_sorted_indicies]
system_constant_yangles = system_constant_yangles[constant_sorted_indicies]
system_constant_zangles = system_constant_zangles[constant_sorted_indicies]
system_constant_xvel = system_constant_xvel[constant_sorted_indicies]
system_constant_yvel = system_constant_yvel[constant_sorted_indicies]
system_constant_zvel = system_constant_zvel[constant_sorted_indicies]
system_constant_dev = system_constant_dev[constant_sorted_indicies]
system_constant_time = system_constant_time[constant_sorted_indicies]
system_constant_angles = system_constant_angles[constant_sorted_indicies]
system_constant_magnitudes = system_constant_magnitudes[constant_sorted_indicies]
system_constant_delta_xvel = system_constant_delta_xvel[constant_sorted_indicies]
system_constant_delta_yvel = system_constant_delta_yvel[constant_sorted_indicies]
system_constant_delta_zvel = system_constant_delta_zvel[constant_sorted_indicies]


system_waypoint_xangles = system_waypoint_xangles[waypoint_sorted_indicies]
system_waypoint_yangles = system_waypoint_yangles[waypoint_sorted_indicies]
system_waypoint_zangles = system_waypoint_zangles[waypoint_sorted_indicies]
system_waypoint_xvel = system_waypoint_xvel[waypoint_sorted_indicies]
system_waypoint_yvel = system_waypoint_yvel[waypoint_sorted_indicies]
system_waypoint_zvel = system_waypoint_zvel[waypoint_sorted_indicies]
system_waypoint_dev = system_waypoint_dev[waypoint_sorted_indicies]
system_waypoint_time = system_waypoint_time[waypoint_sorted_indicies]
system_waypoint_angles = system_waypoint_angles[waypoint_sorted_indicies]
system_waypoint_magnitudes = system_waypoint_magnitudes[waypoint_sorted_indicies]
system_waypoint_delta_xvel = system_waypoint_delta_xvel[waypoint_sorted_indicies]
system_waypoint_delta_yvel = system_waypoint_delta_yvel[waypoint_sorted_indicies]
system_waypoint_delta_zvel = system_waypoint_delta_zvel[waypoint_sorted_indicies]


# Remove all nan values from the list
nan_values = np.isnan(system_constant_zangles)
system_constant_xangles = system_constant_xangles[~nan_values]
system_constant_yangles = system_constant_yangles[~nan_values]
system_constant_zangles = system_constant_zangles[~nan_values]
system_constant_xvel = system_constant_xvel[~nan_values]
system_constant_yvel = system_constant_yvel[~nan_values]
system_constant_zvel = system_constant_zvel[~nan_values]
system_constant_dev = system_constant_dev[~nan_values]
system_constant_time = system_constant_time[~nan_values]
system_constant_angles = system_constant_angles[~nan_values]
system_constant_magnitudes = system_constant_magnitudes[~nan_values]
system_constant_delta_xvel = system_constant_delta_xvel[~nan_values]
system_constant_delta_yvel = system_constant_delta_yvel[~nan_values]
system_constant_delta_zvel = system_constant_delta_zvel[~nan_values]

nan_values = np.isnan(system_waypoint_zangles)
system_waypoint_xangles = system_waypoint_xangles[~nan_values]
system_waypoint_yangles = system_waypoint_yangles[~nan_values]
system_waypoint_zangles = system_waypoint_zangles[~nan_values]
system_waypoint_xvel = system_waypoint_xvel[~nan_values]
system_waypoint_yvel = system_waypoint_yvel[~nan_values]
system_waypoint_zvel = system_waypoint_zvel[~nan_values]
system_waypoint_dev = system_waypoint_dev[~nan_values]
system_waypoint_time = system_waypoint_time[~nan_values]
system_waypoint_angles = system_waypoint_angles[~nan_values]
system_waypoint_magnitudes = system_waypoint_magnitudes[~nan_values]
system_waypoint_delta_xvel = system_waypoint_delta_xvel[~nan_values]
system_waypoint_delta_yvel = system_waypoint_delta_yvel[~nan_values]
system_waypoint_delta_zvel = system_waypoint_delta_zvel[~nan_values]

# Reshape into a 2D array as linear regresion expects 2D array
system_constant_xangles = system_constant_xangles.reshape(-1, 1)
system_constant_yangles = system_constant_yangles.reshape(-1, 1)
system_constant_zangles = system_constant_zangles.reshape(-1, 1)
system_constant_xvel = system_constant_xvel.reshape(-1, 1)
system_constant_yvel = system_constant_yvel.reshape(-1, 1)
system_constant_zvel = system_constant_zvel.reshape(-1, 1)
system_constant_dev = system_constant_dev.reshape(-1, 1)
system_constant_time = system_constant_time.reshape(-1, 1)
system_constant_angles = system_constant_angles.reshape(-1, 1)
system_constant_magnitudes = system_constant_magnitudes.reshape(-1, 1)
system_constant_delta_xvel = system_constant_delta_xvel.reshape(-1, 1)
system_constant_delta_yvel = system_constant_delta_yvel.reshape(-1, 1)
system_constant_delta_zvel = system_constant_delta_zvel.reshape(-1, 1)

system_waypoint_xangles = system_waypoint_xangles.reshape(-1, 1)
system_waypoint_yangles = system_waypoint_yangles.reshape(-1, 1)
system_waypoint_zangles = system_waypoint_zangles.reshape(-1, 1)
system_waypoint_xvel = system_waypoint_xvel.reshape(-1, 1)
system_waypoint_yvel = system_waypoint_yvel.reshape(-1, 1)
system_waypoint_zvel = system_waypoint_zvel.reshape(-1, 1)
system_waypoint_dev = system_waypoint_dev.reshape(-1, 1)
system_waypoint_time = system_waypoint_time.reshape(-1, 1)
system_waypoint_angles = system_waypoint_angles.reshape(-1, 1)
system_waypoint_magnitudes = system_waypoint_magnitudes.reshape(-1, 1)
system_waypoint_delta_xvel = system_waypoint_delta_xvel.reshape(-1, 1)
system_waypoint_delta_yvel = system_waypoint_delta_yvel.reshape(-1, 1)
system_waypoint_delta_zvel = system_waypoint_delta_zvel.reshape(-1, 1)


# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_constant_xangles, system_constant_dev, s=10)
model1 = LinearRegression()
model1.fit(system_constant_xangles, system_constant_dev)
ax1.plot(system_constant_xangles, model1.predict(system_constant_xangles), c='r')
ax1.set_xlabel("X Angle")
ax1.set_ylabel("Deviation")
ax1.set_title("Constant Velocity Controller")
# ax1.set_xlim([0, 180])
# ax1.set_ylim([0, 7])

ax2.scatter(system_constant_yangles, system_constant_dev, s=10)
model2 = LinearRegression()
model2.fit(system_constant_yangles, system_constant_dev)
ax2.plot(system_constant_yangles, model2.predict(system_constant_yangles), c='r')
ax2.set_xlabel("Y Angle")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Velocity Controller")
# ax2.set_xlim([0, 180])
# ax2.set_ylim([0, 7])

ax3.scatter(system_constant_zangles, system_constant_dev, s=10)
model3 = LinearRegression()
model3.fit(system_constant_zangles, system_constant_dev)
ax3.plot(system_constant_zangles, model3.predict(system_constant_zangles), c='r')
ax3.set_xlabel("Z Angle")
ax3.set_ylabel("Deviation")
ax3.set_title("Constant Velocity Controller")
# ax3.set_xlim([0, 180])
# ax3.set_ylim([0, 7])
plt.show()





# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_waypoint_xangles, system_waypoint_dev, s=10)
model1 = LinearRegression()
model1.fit(system_waypoint_xangles, system_waypoint_dev)
ax1.plot(system_waypoint_xangles, model1.predict(system_waypoint_xangles), c='r')
ax1.set_xlabel("X Angle")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")
# ax1.set_xlim([0, 180])
# ax1.set_ylim([0, 7])


ax2.scatter(system_waypoint_yangles, system_waypoint_dev, s=10)
model2 = LinearRegression()
model2.fit(system_waypoint_yangles, system_waypoint_dev)
ax2.plot(system_waypoint_yangles, model2.predict(system_waypoint_yangles), c='r')
ax2.set_xlabel("Y Angle")
ax2.set_ylabel("Deviation")
ax2.set_title("Waypoint Controller")
# ax2.set_xlim([0, 180])
# ax2.set_ylim([0, 7])


ax3.scatter(system_waypoint_zangles, system_waypoint_dev, s=10)
model3 = LinearRegression()
model3.fit(system_waypoint_zangles, system_waypoint_dev)
ax3.plot(system_waypoint_zangles, model3.predict(system_waypoint_zangles), c='r')
ax3.set_xlabel("Z Angle")
ax3.set_ylabel("Deviation")
ax3.set_title("Waypoint Controller")
# ax3.set_xlim([0, 180])
# ax3.set_ylim([0, 7])
plt.show()






# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_constant_xvel, system_constant_dev, s=10)
model1 = LinearRegression()
model1.fit(system_constant_xvel, system_constant_dev)
ax1.plot(system_constant_xvel, model1.predict(system_constant_xvel), c='r')
ax1.set_xlabel("X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Constant Velocity Controller")
# ax1.set_xlim([0, 20])
# ax1.set_ylim([0, 7])


ax2.scatter(system_constant_yvel, system_constant_dev, s=10)
model2 = LinearRegression()
model2.fit(system_constant_yvel, system_constant_dev)
ax2.plot(system_constant_yvel, model2.predict(system_constant_yvel), c='r')
ax2.set_xlabel("Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Velocity Controller")
# ax2.set_xlim([0, 20])
# ax2.set_ylim([0, 7])


ax3.scatter(system_constant_zvel, system_constant_dev, s=10)
model3 = LinearRegression()
model3.fit(system_constant_zvel, system_constant_dev)
ax3.plot(system_constant_zvel, model3.predict(system_constant_zvel), c='r')
ax3.set_xlabel("Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Constant Velocity Controller")
# ax3.set_xlim([0, 20])
# ax3.set_ylim([0, 7])

plt.show()





# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_waypoint_xvel, system_waypoint_dev, s=10)
model1 = LinearRegression()
model1.fit(system_waypoint_xvel, system_waypoint_dev)
ax1.plot(system_waypoint_xvel, model1.predict(system_waypoint_xvel), c='r')
ax1.set_xlabel("X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")
# ax1.set_xlim([0, 20])
# ax1.set_ylim([0, 7])

ax2.scatter(system_waypoint_yvel, system_waypoint_dev, s=10)
model2 = LinearRegression()
model2.fit(system_waypoint_yvel, system_waypoint_dev)
ax2.plot(system_waypoint_yvel, model2.predict(system_waypoint_yvel), c='r')
ax2.set_xlabel("Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Waypoint Controller")
# ax2.set_xlim([0, 20])
# ax2.set_ylim([0, 7])

ax3.scatter(system_waypoint_zvel, system_waypoint_dev, s=10)
model3 = LinearRegression()
model3.fit(system_waypoint_zvel, system_waypoint_dev)
ax3.plot(system_waypoint_zvel, model3.predict(system_waypoint_zvel), c='r')
ax3.set_xlabel("Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Waypoint Controller")
# ax3.set_xlim([0, 20])
# ax3.set_ylim([0, 7])
plt.show()




















# Plot the data
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
ax1.scatter(system_waypoint_magnitudes, system_waypoint_dev, s=10)
model1 = LinearRegression()
model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
ax1.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax1.set_xlabel("Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")
ax1.set_xlim([0, 25])
ax1.set_ylim([0, 7])

ax2.scatter(system_constant_magnitudes, system_constant_dev, s=10)
model2 = LinearRegression()
model2.fit(system_constant_magnitudes, system_constant_dev)
ax2.plot(system_constant_magnitudes, model2.predict(system_constant_magnitudes), c='r')
ax2.set_xlabel("Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Controller")
ax2.set_xlim([0, 25])
ax2.set_ylim([0, 7])
plt.show()

















# Plot the data
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
ax1.scatter(system_waypoint_angles, system_waypoint_dev, s=10)
model3 = LinearRegression()
model3.fit(system_waypoint_angles, system_waypoint_dev)
ax1.plot(system_waypoint_angles, model3.predict(system_waypoint_angles), c='r')
ax1.set_xlabel("Angle")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")
ax1.set_xlim([0, 180])
ax1.set_ylim([0, 7])

ax2.scatter(system_constant_angles, system_constant_dev, s=10)
model4 = LinearRegression()
model4.fit(system_constant_angles, system_constant_dev)
ax2.plot(system_constant_angles, model4.predict(system_constant_angles), c='r')
ax2.set_xlabel("Angle")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Controller")
ax2.set_xlim([0, 180])
ax2.set_ylim([0, 7])
plt.show()


















# Plot the data
fig = plt.figure()
ax = fig.add_subplot(131, projection='3d')
ax.scatter(system_constant_xangles, system_constant_xvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("X Angle")
ax.set_ylabel("X Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(132, projection='3d')
ax.scatter(system_constant_yangles, system_constant_yvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Y Angle")
ax.set_ylabel("Y Velocity")
ax.set_zlabel("Deviation")
ax.set_title("Constant Velocity Controller")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(133, projection='3d')
ax.scatter(system_constant_zangles, system_constant_zvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Z Angle")
ax.set_ylabel("Z Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])
plt.show()




# Plot the data
fig = plt.figure()
ax = fig.add_subplot(131, projection='3d')
ax.scatter(system_waypoint_xangles, system_waypoint_xvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("X Angle")
ax.set_ylabel("X Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(132, projection='3d')
ax.scatter(system_waypoint_yangles, system_waypoint_yvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Y Angle")
ax.set_ylabel("Y Velocity")
ax.set_zlabel("Deviation")
ax.set_title("Waypoint Controller")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(133, projection='3d')
ax.scatter(system_waypoint_zangles, system_waypoint_zvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Z Angle")
ax.set_ylabel("Z Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])
plt.show()



















# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_constant_delta_xvel, system_constant_dev, s=10)
model1 = LinearRegression()
model1.fit(system_constant_delta_xvel, system_constant_dev)
ax1.plot(system_constant_delta_xvel, model1.predict(system_constant_delta_xvel), c='r')
ax1.set_xlabel("Change in X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Constant Velocity Controller")
# ax1.set_xlim([0, 20])
# ax1.set_ylim([0, 7])


ax2.scatter(system_constant_delta_yvel, system_constant_dev, s=10)
model2 = LinearRegression()
model2.fit(system_constant_delta_yvel, system_constant_dev)
ax2.plot(system_constant_delta_yvel, model2.predict(system_constant_delta_yvel), c='r')
ax2.set_xlabel("Change in Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Velocity Controller")
# ax2.set_xlim([0, 20])
# ax2.set_ylim([0, 7])


ax3.scatter(system_constant_delta_zvel, system_constant_dev, s=10)
model3 = LinearRegression()
model3.fit(system_constant_delta_zvel, system_constant_dev)
ax3.plot(system_constant_delta_zvel, model3.predict(system_constant_delta_zvel), c='r')
ax3.set_xlabel("Change in Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Constant Velocity Controller")
# ax3.set_xlim([0, 20])
# ax3.set_ylim([0, 7])

plt.show()





# Plot the data
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 7))
ax1.scatter(system_waypoint_delta_xvel, system_waypoint_dev, s=10)
model1 = LinearRegression()
model1.fit(system_waypoint_delta_xvel, system_waypoint_dev)
ax1.plot(system_waypoint_delta_xvel, model1.predict(system_waypoint_delta_xvel), c='r')
ax1.set_xlabel("Change in X Velocity")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")
# ax1.set_xlim([0, 20])
# ax1.set_ylim([0, 7])

ax2.scatter(system_waypoint_delta_yvel, system_waypoint_dev, s=10)
model2 = LinearRegression()
model2.fit(system_waypoint_delta_yvel, system_waypoint_dev)
ax2.plot(system_waypoint_delta_yvel, model2.predict(system_waypoint_delta_yvel), c='r')
ax2.set_xlabel("Change in Y Velocity")
ax2.set_ylabel("Deviation")
ax2.set_title("Waypoint Controller")
# ax2.set_xlim([0, 20])
# ax2.set_ylim([0, 7])

ax3.scatter(system_waypoint_delta_zvel, system_waypoint_dev, s=10)
model3 = LinearRegression()
model3.fit(system_waypoint_delta_zvel, system_waypoint_dev)
ax3.plot(system_waypoint_delta_zvel, model3.predict(system_waypoint_delta_zvel), c='r')
ax3.set_xlabel("Change in Z Velocity")
ax3.set_ylabel("Deviation")
ax3.set_title("Waypoint Controller")
# ax3.set_xlim([0, 20])
# ax3.set_ylim([0, 7])
plt.show()





























# Plot the data
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
ax1.scatter(system_waypoint_time, system_waypoint_dev, s=10)
model1 = LinearRegression()
model1.fit(system_waypoint_time, system_waypoint_dev)
ax1.plot(system_waypoint_time, model1.predict(system_waypoint_time), c='r')
ax1.set_xlabel("Time")
ax1.set_ylabel("Deviation")
ax1.set_title("Waypoint Controller")

ax2.scatter(system_constant_time, system_constant_dev, s=10)
model2 = LinearRegression()
model2.fit(system_constant_time, system_constant_dev)
ax2.plot(system_constant_time, model2.predict(system_constant_time), c='r')
ax2.set_xlabel("Time")
ax2.set_ylabel("Deviation")
ax2.set_title("Constant Controller")
plt.show()



















# Plot the data
fig = plt.figure()
ax = fig.add_subplot(131, projection='3d')
ax.scatter(system_constant_xangles, system_constant_xvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("X Angle")
ax.set_ylabel("X Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(132, projection='3d')
ax.scatter(system_constant_yangles, system_constant_yvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Y Angle")
ax.set_ylabel("Y Velocity")
ax.set_zlabel("Deviation")
ax.set_title("Constant Velocity Controller")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(133, projection='3d')
ax.scatter(system_constant_zangles, system_constant_zvel, system_constant_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Z Angle")
ax.set_ylabel("Z Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])
plt.show()



# Plot the data
fig = plt.figure()
ax = fig.add_subplot(131, projection='3d')
ax.scatter(system_waypoint_xangles, system_waypoint_xvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("X Angle")
ax.set_ylabel("X Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(132, projection='3d')
ax.scatter(system_waypoint_yangles, system_waypoint_yvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Y Angle")
ax.set_ylabel("Y Velocity")
ax.set_zlabel("Deviation")
ax.set_title("Waypoint Controller")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])


ax = fig.add_subplot(133, projection='3d')
ax.scatter(system_waypoint_zangles, system_waypoint_zvel, system_waypoint_dev, s=10)
# model1 = LinearRegression()
# model1.fit(system_waypoint_magnitudes, system_waypoint_dev)
# ax.plot(system_waypoint_magnitudes, model1.predict(system_waypoint_magnitudes), c='r')
ax.set_xlabel("Z Angle")
ax.set_ylabel("Z Velocity")
ax.set_zlabel("Deviation")
# ax.set_xlim([0, 25])
# ax.set_ylim([0, 7])
plt.show()