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

# Used to save the worst deviation
worst_deviation = 0
worst_position_outdoor = []
worst_position_goals = []
worst_position_sim = []
worst_file_name = ""
save_next = False

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


        # Save the test with the worst devaition
        if save_next:
            worst_position_sim = copy.deepcopy(current_drone_position)
            save_next = False

        if sys == "anafi_outdoor":
            maximum_deviation = np.sum(current_deviation)

            # If it is the worst save it
            if maximum_deviation > worst_deviation:
                # Save the position
                worst_position_outdoor = copy.deepcopy(current_drone_position)
                worst_position_goals = copy.deepcopy(test_waypoints)
                save_next = True
                worst_deviation = maximum_deviation
                worst_file_name = file_name

print("The worst test was:")
print(worst_file_name)

# Stack the drone positions and waypoints for plotting
d_pos_out = np.vstack(worst_position_outdoor)
d_pos_sim = np.vstack(worst_position_sim)
w_pos = np.vstack(worst_position_goals)

# Create a 3D plot of the trajectory and actual path
fig = plt.figure()
ax = Axes3D(fig)
ax.plot(w_pos[:, 0], w_pos[:, 1], w_pos[:, 2], color='C0', linewidth=2, linestyle=":", label='Ideal Trajectory')
ax.scatter(w_pos[:, 0], w_pos[:, 1], w_pos[:, 2], c='C0')
ax.plot3D(d_pos_out[:, 0], d_pos_out[:, 1], d_pos_out[:, 2], color='C2', linewidth=2, label='Real-World')
ax.plot3D(d_pos_sim[:, 0], d_pos_sim[:, 1], d_pos_sim[:, 2], color='C1', linewidth=2, label='Simulation')

ax.set_xlim([0, 30])
ax.set_ylim([0, -30])
ax.set_zlim([0, 30])
ax.set_xlabel('X-Axis(m)')
ax.set_ylabel('Y-Axis(m)')
ax.set_zlabel('Z-Axis(m)')
plt.title("Optimal vs. true trajectory")
ax.legend()
plt.show()

print("")
print("-------------------------------------------")
print("-----------Completed Processing------------")
print("-------------------------------------------")
print("")
