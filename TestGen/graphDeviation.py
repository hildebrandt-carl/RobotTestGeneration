import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string


# This has the run when all tests are considered
# all_folders = ["./Results/FullRun/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint/",
#                 "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
#                 "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/"]


# This has the tests when only the first 87 are considered
all_folders = ["./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint/",
                "./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
                "./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/"]



beam_lengths = [10]
depths = [10]
res_numbers = [4]

# save_names = ["Random Search",
#               "Random + Max Velocity",
#               "Random + Kinematic",
#               "Score + Kinematic 180",
#               "Score + Kinematic 90"]
#
# tick_names = ["Random Search",
#               "Random + Max Velocity",
#               "Random + Kinematic",
#               "Score + Kinematic 180",
#               "Score + Kinematic 90"]

save_names = ["Random", "Waypoint", "Constant"]

tick_names = ["Random", "Waypoint", "Constant"]

systems = ["waypoint",
           "constant"]

system_constant_time = []
system_constant_distance = []
system_waypoint_time = []
system_waypoint_distance = []
system_constant_maximum_deviation = []
system_waypoint_maximum_deviation = []

for system in systems:

    distance_heuristic_box_data = []
    time_heuristic_box_data = []

    for i in range(0, len(all_folders)):
        folder = all_folders[i]

        scores = []
        average_deviation = []
        total_deviation = []
        average_time = []
        total_time = []
        distance_heuristic = []
        time_heuristic = []
        maximum_deviation = []
        trajectory_length = []

        for depth in depths:
            for beam in beam_lengths:

                file_location = folder
                analysis_file_names = glob.glob(file_location + "maps/map*/analysis_" + system + ".txt")

                # Make sure we go in order from highest score to lowest score
                total_files = len(analysis_file_names)
                file_counter = 0

                for file_counter in range(1, total_files + 1):

                    # Create the file name
                    file_name = file_location + "maps/map" + str(file_counter) + "/analysis_" + system + ".txt"

                    # Get the average and total deviation for that test
                    avg_dev = get_numbers_after_string(file_name=file_name, the_string="Average deviation from optimal trajectory:")
                    tot_dev = get_numbers_after_string(file_name=file_name, the_string="Total deviation from optimal trajectory:")

                    # Get the score for that test
                    scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                    # Get the average and total time for that test
                    avg_time = get_numbers_after_string(file_name=file_name, the_string="Average time between waypoints:")
                    tot_time = get_numbers_after_string(file_name=file_name, the_string="Total time between waypoints:")

                    # Get the maximum deviation from the optimal trajectory
                    max_dev = get_numbers_after_string(file_name=file_name, the_string="Maximum deviation from optimal trajectory:")

                    # Get the total distance travelled and the total trajectory length
                    tot_dist = get_numbers_after_string(file_name=file_name, the_string="Total distance travelled:")
                    traj_len = get_numbers_after_string(file_name=file_name, the_string="Trajectory length:")
                    optimal_distance_heuristic = tot_dist[0][0] / traj_len[0][0]

                    # Get the number of waypoints (This should be equal to the trajectory's optimal time)
                    num_way = get_numbers_after_string(file_name=file_name, the_string="Total waypoints:")
                    optimal_time_heuristic = tot_time[0][0] / num_way[0][0]

                    # Get the trajectory length

                    # Check for any anomalies
                    if max_dev[0][0] > 5 and system == "constant":
                        print("Maximum Deviation over 6.2m (" + str(max_dev[0][0]) + "m): " + str(file_name))

                    # # Check for any anomalies
                    # if tot_dev[0][0] > 450:
                    #     print("Total Deviation over 450m (" + str(tot_dev[0][0]) + "m): " + str(file_name))
                    #
                    # if avg_time[0][0] > 20:
                    #     print("Average time over 20 seconds (" + str(avg_time[0][0]) + "s): " + str(file_name))
                    #
                    # if optimal_distance_heuristic > 2.5:
                    #     print("Optimal distance heuristic over 2.2 (" + str(optimal_distance_heuristic) + "): " + str(file_name))
                    #
                    # if tot_time[0][0] > 40:
                    #     print("Total time over 40 (" + str(tot_time[0][0]) + "): " + str(file_name))

                    # Save the data
                    average_deviation.append(avg_dev[0][0])
                    total_deviation.append(tot_dev[0][0])
                    scores.append(scr[0][0])
                    average_time.append(avg_time[0][0])
                    total_time.append(tot_time[0][0])
                    distance_heuristic.append(optimal_distance_heuristic)
                    time_heuristic.append(optimal_time_heuristic)
                    maximum_deviation.append(max_dev[0][0])
                    trajectory_length.append(traj_len)

        # Save the data into each respective system
        if system == "constant":
            system_constant_distance.append(total_deviation)
            system_constant_time.append(total_time)
            system_constant_maximum_deviation.append(maximum_deviation)
        elif system == "waypoint":
            system_waypoint_distance.append(total_deviation)
            system_waypoint_time.append(total_time)
            system_waypoint_maximum_deviation.append(maximum_deviation)

# Display the compared box plots
def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=2)
    # plt.setp(bp['whiskers'], color=color)
    # plt.setp(bp['caps'], color=color)
    # plt.setp(bp['medians'], color=color)
    # plt.setp(bp['fliers'], color=color)
    # plt.setp(bp['means'], color=color)



fig, ax1 = plt.subplots(1, 1, figsize=(6,6))
bpl = plt.boxplot(system_constant_maximum_deviation, positions=np.array(range(len(system_constant_maximum_deviation)))*2.0-0.4, widths=0.6, showmeans=True)
bpr = plt.boxplot(system_waypoint_maximum_deviation, positions=np.array(range(len(system_waypoint_maximum_deviation)))*2.0+0.4, widths=0.6, showmeans=True)
set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpr, '#2C7BB6')

# draw temporary red and blue lines and use them to create a legend
plt.plot([], c='#D7191C', label='Constant Velocity')
plt.plot([], c='#2C7BB6', label='Waypoint Controller')
plt.legend()
plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=7, rotation=15)
plt.xlim(-2, len(tick_names)*2)
plt.tight_layout()
plt.ylabel("Maximum Deviation")
plt.xlabel("Test Generation Test Set")

print("----------------------------------------------------------")
print("Maximum Deviation Statistics")
print("Random Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[0])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[0])))
print("Waypoint Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[1])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[1])))
print("Constant Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[2])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[2])))
print("----------------------------------------------------------")




fig, ax1 = plt.subplots(1, 1, figsize=(6,6))
bpl = plt.boxplot(system_constant_distance, positions=np.array(range(len(system_constant_distance)))*2.0-0.4, widths=0.6, showmeans=True)
bpr = plt.boxplot(system_waypoint_distance, positions=np.array(range(len(system_waypoint_distance)))*2.0+0.4, widths=0.6, showmeans=True)
set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpr, '#2C7BB6')

# draw temporary red and blue lines and use them to create a legend
plt.plot([], c='#D7191C', label='Constant Velocity')
plt.plot([], c='#2C7BB6', label='Waypoint Controller')
plt.legend()
plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=7, rotation=15)
plt.xlim(-2, len(tick_names)*2)
plt.tight_layout()
plt.ylabel("Accumulated Deviation")
plt.xlabel("Test Generation Test Set")


print("----------------------------------------------------------")
print("Accumulated Deviation Statistics")
print("Random Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[0])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[0])))
print("Waypoint Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[1])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[1])))
print("Constant Test Set:")
print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[2])))
print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[2])))
print("----------------------------------------------------------")

# va = [0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0]
# for t, y in zip(ax1.get_xticklabels(), va):
#     t.set_y(y)


fig1, ax2 = plt.subplots(1, 1, figsize=(6,6))
bpl = plt.boxplot(system_constant_time, positions=np.array(range(len(system_constant_time)))*2.0-0.4, widths=0.6, showmeans=True)
bpr = plt.boxplot(system_waypoint_time, positions=np.array(range(len(system_waypoint_time)))*2.0+0.4, widths=0.6, showmeans=True)
set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpr, '#2C7BB6')

# draw temporary red and blue lines and use them to create a legend
plt.plot([], c='#D7191C', label='Constant Velocity')
plt.plot([], c='#2C7BB6', label='Waypoint Controller')
plt.legend()
plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=7, rotation=15)
plt.xlim(-2, len(tick_names)*2)
plt.tight_layout()
plt.ylabel("Total Time")
plt.xlabel("Test Generation Test Set")


plt.show()