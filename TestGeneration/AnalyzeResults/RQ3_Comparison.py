import glob
import copy

import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from math import sqrt, pow
from processResultsUtils import get_numbers_from_string
from processResultsUtils import lineseg_dist

# # For RQ3
main_folder = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/"

# For outdoor outdoor
# # # For the RQ3 length 10
all_folders = "anafi_learned_run_flown/learned_anafi_sim_ANAFI_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/"

# All the different system types which are generated using the WorldEngineSimulator
system_types = ["anafi_outdoor",
                "anafi_sim"]


# Creat the folder based on the passed arguments
folder = main_folder + all_folders

# Get the total number of files
file_names = glob.glob(folder + "/maps/map*/test.txt")
total_files = len(file_names)

position_outdoor = []
position_goals = []
position_sim = []
plotting_file_name = []
outdoor_deviation = []

# For each of the systems
for file_counter in range(1, total_files + 1):

    for sys in system_types:

        # Get the file name
        file_name = glob.glob(folder + "/maps/map" + str(file_counter) + "/performance_" + str(sys) + ".txt")

        if len(file_name) > 1:
            print(file_name)
            print("error")
            exit()
        else:
            file_name = file_name[0]

        # Work out the folder name by removing the word performance.txt
        folder_name = file_name[:-len("performance_" + sys + ".txt")]

        # Used to store current test information
        current_drone_position = []
        current_elapsed_time = []
        test_waypoints = []
        current_waypoint_position = []
        current_waypoint_time = []
        current_deviation = []
        deviation_per_waypoint = []
        max_deviation_per_waypoint = []
        deviation_squared_per_waypoint = []
        goal_switch_indicies = []
        current_total_time = -1
        total_distance_travelled = 0
        trajectory_distance = 0

        # Initialize a dummy waypoint
        waypoint = [-np.inf, -np.inf, -np.inf]

        # First check if the test had a collision
        collision = False
        first_position_recorded = False
        current_time = 0
        current_distance_to_goal = 0
        file = open(file_name, "r")
        for line in file:
            # Find the goal positions
            if "Current Drone Position" in line:
                first_position_recorded = True
            # Get the time to collision
            if "Elapsed Time" in line and not collision:
                current_time = get_numbers_from_string(line)
            # Get the time to collision
            if "Distance to Goal" in line and not collision:
                current_distance_to_goal = get_numbers_from_string(line)
            if "Current Goal Position" in line and not collision:
                current_goal = get_numbers_from_string(line)

        file.close()

        # If there was a collision don't continue processing this file
        if collision:
            print("Collision occurred please check file!")
            continue

        # Get details from the simulation data
        file = open(file_name, "r")
        for line in file:
            # Find the goal positions
            if "Current Goal Position" in line:
                goal_pos = get_numbers_from_string(line)
                current_waypoint_position.append(goal_pos)
                if goal_pos != waypoint:
                    waypoint = goal_pos
                    test_waypoints.append(waypoint)
            # Find the drone positions
            if "Current Drone Position" in line:
                current_position = get_numbers_from_string(line)
                current_drone_position.append(current_position)
                # Current goal is saved in drone_position_waypoints
            # Get the drones time
            if "Elapsed Time" in line:
                current_time = get_numbers_from_string(line)
                current_elapsed_time.append(current_time)
            # Get the time between goals
            if "Time between goals" in line:
                time = get_numbers_from_string(line)
                current_waypoint_time.append(time)
            # Find the total time
            if "Total Time" in line:
                current_total_time = get_numbers_from_string(line)
        file.close()

        if len(test_waypoints) != 10:
            print(test_waypoints)
            print(file_name)
            print("ERROR")

        # Make sure that the drone_position_waypoints is the same length as the current drone position
        assert(len(current_drone_position) == len(current_waypoint_position))

        # Make sure that each of the arrays are the same length
        # There are cases in which some of the details were printed just after the drone finished
        min_length = min(len(current_drone_position), len(current_elapsed_time))
        current_drone_position = current_drone_position[0:min_length]
        current_elapsed_time = current_elapsed_time[0:min_length]

        # Remove the first time_between_waypoints as this is the time to hit the first waypoint
        current_waypoint_time = current_waypoint_time[1:]
        current_waypoint_time = np.hstack(current_waypoint_time)

        # Keeps track of how many recordings per waypoint there are
        recordings_per_waypoint = 0

        # Get the average distance from the optimal trajectory
        for cur_pos, cur_goal in zip(current_drone_position, current_waypoint_position):
            # There is no previous waypoint before we hit the first waypoint
            prev_goal = None
            # Set the deviation to 0 before we hit the first waypoint
            d = 0

            # Get the index of the current goal
            index = test_waypoints.index(cur_goal)

            # If we have gone past the first waypoint
            if index != 0:
                # Get previous waypoint
                prev_goal = test_waypoints[index-1]

                # Calculate the distance from current position to the line created by the goal
                d = lineseg_dist(p=np.asarray(cur_pos),
                                    a=np.asarray(cur_goal),
                                    b=np.asarray(prev_goal))

            # Save the deviation per waypoint
            if len(deviation_per_waypoint) <= index:
                # Get the average for the previous waypoint
                if len(deviation_per_waypoint) > 0:
                    deviation_per_waypoint[index - 1] = deviation_per_waypoint[index - 1] / float(recordings_per_waypoint)
                    deviation_squared_per_waypoint[index - 1] = deviation_squared_per_waypoint[index - 1] / float(recordings_per_waypoint)

                # Start the new deviation tracker
                deviation_per_waypoint.append(abs(d))
                deviation_squared_per_waypoint.append(abs(d) ** 2)
                max_deviation_per_waypoint.append(abs(d))
                recordings_per_waypoint = 1

                # Save the index as this is when a switch between waypoints happened
                goal_switch_indicies.append(len(current_deviation))
            else:
                deviation_per_waypoint[index] += abs(d)
                deviation_squared_per_waypoint[index] += (abs(d) ** 2)
                max_deviation_per_waypoint[index] = max( max_deviation_per_waypoint[index], abs(d) )
                recordings_per_waypoint += 1

            # Save the smallest distance to the deviation from optimal array
            current_deviation.append(abs(d))

        if sys == system_types[0]:
            max_dev = max(current_deviation)
            # Get the outdoor maximum deviation
            position_outdoor.append(np.array(current_drone_position))
            position_goals.append(np.array(test_waypoints))
            plotting_file_name.append(copy.deepcopy(file_name))
            outdoor_deviation.append(copy.deepcopy(max_dev))
        else:
            position_sim.append(np.array(current_drone_position))

zipped_files = zip(outdoor_deviation, position_outdoor, position_sim, position_goals, plotting_file_name)
outdoor_deviation, position_outdoor, position_sim, position_goals, plotting_file_name = zip(*sorted(zipped_files))













shift_down = 0.85
plt_num = -1

# print("The worst test was:")
# print(plotting_file_name[plt_num])
# # Stack the drone positions and waypoints for plotting
# d_pos_out1 = np.vstack(position_outdoor[plt_num])
# d_pos_sim1 = np.vstack(position_sim[plt_num])
# w_pos1 = np.vstack(position_goals[plt_num])

# # Create a 3D plot of the trajectory and actual path
# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot(w_pos1[:, 0], w_pos1[:, 1], w_pos1[:, 2]-shift_down, color='C0', linewidth=2, linestyle=":", label='Ideal Trajectory')
# ax.scatter(w_pos1[:, 0], w_pos1[:, 1], w_pos1[:, 2]-shift_down, c='C0')
# ax.plot3D(d_pos_out1[:, 0], d_pos_out1[:, 1], d_pos_out1[:, 2]-shift_down, color='C2', linewidth=2, label='Real-World')
# ax.plot3D(d_pos_sim1[:, 0], d_pos_sim1[:, 1], d_pos_sim1[:, 2]-shift_down, color='C1', linewidth=2, label='Simulation')

# ax.set_xlim([0, 30])
# ax.set_ylim([0, -30])
# ax.set_zlim([0, 30])
# ax.set_xlabel('X-Axis(m)')
# ax.set_ylabel('Y-Axis(m)')
# ax.set_zlabel('Z-Axis(m)')
# plt.title("Optimal vs. true trajectory")
# ax.legend()
# plt.show()




























fig = plt.figure(figsize=(20,5))
plt_num = -1
labelsize = 13
headingsize = 15




# Select the plot
ax = fig.add_subplot(1, 4, 1, projection='3d')

# Stack the drone positions and waypoints for plotting
d_pos_out1 = np.vstack(position_outdoor[plt_num])
d_pos_sim1 = np.vstack(position_sim[plt_num])
w_pos1 = np.vstack(position_goals[plt_num])

# Create a 3D plot of the trajectory and actual path
ax.plot(w_pos1[:, 0], -1*w_pos1[:, 1], w_pos1[:, 2]-shift_down, color='black', linewidth=2, linestyle=":", label='Ideal Trajectory')
ax.scatter(w_pos1[:, 0], -1*w_pos1[:, 1], w_pos1[:, 2]-shift_down, c='black')
ax.plot3D(d_pos_out1[:, 0], -1*d_pos_out1[:, 1], d_pos_out1[:, 2]-shift_down, color='C2', linewidth=2, label='Real-World')
ax.plot3D(d_pos_sim1[:, 0], -1*d_pos_sim1[:, 1], d_pos_sim1[:, 2]-shift_down, linestyle="--", color='C1', linewidth=2, label='Simulation')

ax.set_xlim([0, 30])
ax.set_ylim([0, 30])
ax.set_zlim([0, 30])
ax.set_xlabel('X-Axis(m)', fontsize=labelsize)
ax.set_ylabel('Y-Axis(m)', fontsize=labelsize)
ax.set_zlabel('Z-Axis(m)', fontsize=labelsize)
plt.title("3D View", fontsize=headingsize)
plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)

ax.zaxis._axinfo['juggled'] = (1,2,1)




# Select the plot
ax1 = fig.add_subplot(1, 4, 2)
print("The worst test was:")
print(plotting_file_name[plt_num])
# Stack the drone positions and waypoints for plotting
d_pos_out1 = np.vstack(position_outdoor[plt_num])
d_pos_sim1 = np.vstack(position_sim[plt_num])
w_pos1 = np.vstack(position_goals[plt_num])

# Create a 3D plot of the trajectory and actual path
plt.plot(w_pos1[:, 0], -1*w_pos1[:, 1], color='black', linewidth=2, linestyle=":", label='Ideal Trajectory')
plt.scatter(w_pos1[:, 0], -1*w_pos1[:, 1], c='black')
plt.plot(d_pos_out1[:, 0], -1*d_pos_out1[:, 1], color='C2', linewidth=2, label='Real-World')
plt.plot(d_pos_sim1[:, 0], -1*d_pos_sim1[:, 1], color='C1', linewidth=2, linestyle="--", label='Simulation')

plt.xlim([0, 35])
plt.ylim([0, 35])
plt.xlabel('X-Axis(m)', fontsize=labelsize)
plt.ylabel('Y-Axis(m)', fontsize=labelsize)
plt.title("Top View", fontsize=headingsize)
plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)



# Select the plot
plt.subplot(143)

# Stack the drone positions and waypoints for plotting
d_pos_out1 = np.vstack(position_outdoor[plt_num])
d_pos_sim1 = np.vstack(position_sim[plt_num])
w_pos1 = np.vstack(position_goals[plt_num])

# Create a 3D plot of the trajectory and actual path
plt.plot(w_pos1[:, 0], w_pos1[:, 2]-shift_down, color='black', linewidth=2, linestyle=":", label='Ideal Trajectory')
plt.scatter(w_pos1[:, 0], w_pos1[:, 2]-shift_down, c='black')
plt.plot(d_pos_out1[:, 0], d_pos_out1[:, 2]-shift_down, color='C2', linewidth=2, label='Real-World')
plt.plot(d_pos_sim1[:, 0], d_pos_sim1[:, 2]-shift_down, color='C1', linestyle="--", linewidth=2, label='Simulation')

plt.xlim([0, 35])
plt.ylim([0, 35])
plt.xlabel('X-Axis(m)', fontsize=labelsize)
plt.ylabel('Z-Axis(m)', fontsize=labelsize)
plt.title("Side View", fontsize=headingsize)
plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)





# Select the plot
plt.subplot(144)

# Stack the drone positions and waypoints for plotting
d_pos_out1 = np.vstack(position_outdoor[plt_num])
d_pos_sim1 = np.vstack(position_sim[plt_num])
w_pos1 = np.vstack(position_goals[plt_num])

# Create a 3D plot of the trajectory and actual path
plt.plot(-1*w_pos1[:, 1], w_pos1[:, 2]-shift_down, color='black', linewidth=2, linestyle=":", label='Ideal Trajectory')
plt.scatter(-1*w_pos1[:, 1], w_pos1[:, 2]-shift_down, c='black')
plt.plot(-1*d_pos_out1[:, 1], d_pos_out1[:, 2]-shift_down, color='C2', linewidth=2, label='Real-World')
plt.plot(-1*d_pos_sim1[:, 1], d_pos_sim1[:, 2]-shift_down, color='C1', linestyle="--", linewidth=2, label='Simulation')

plt.xlim([0, 35])
plt.ylim([0, 35])
plt.xlabel('Y-Axis(m)', fontsize=labelsize)
plt.ylabel('Z-Axis(m)', fontsize=labelsize)
plt.title("Side View", fontsize=headingsize)
plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)



fig.tight_layout()
plt.show()
