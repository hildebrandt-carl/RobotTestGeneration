import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string


# This has the run when all tests are considered
# all_folders = ["./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
#                "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/",
#                "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint/"]

def add_values(bp, ax, left=False):
    """ This actually adds the numbers to the various points of the boxplots"""
    for element in ['whiskers', 'medians', 'caps']:
        for line in bp[element]:
            # Get the position of the element. y is the label you want
            (x_l, y),(x_r, _) = line.get_xydata()
            # Make sure datapoints exist
            # (I've been working with intervals, should not be problem for this case)
            if not np.isnan(y):
                if left:
                    if element == 'medians':
                        x_line_center = x_l - (x_r - x_l)/2
                    else:
                        x_line_center = x_l - (x_r - x_l)
                    if element == 'whiskers':
                        x_line_center = x_r
                else:
                    x_line_center = x_r
                y_line_center = y  # Since it's a line and it's horisontal
                # overlay the value:  on the line, from center to right
                ax.text(x_line_center, y_line_center, # Position
                        '%.3f' % y, # Value (3f = 3 decimal float)
                        verticalalignment='center', # Centered vertically with line
                        fontsize=10, backgroundcolor="white")

# This has the tests when only the first 87 are considered
# all_folders = ["./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint/",
#                "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
#                "./Results/PolyRunFull/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/"]
#
#
# all_folders = ["./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint/",
#                "./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint/",
#                "./Results/SameNumber/MIT_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant/"]

# all_folders = ["./Results/PolySameTime2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_kinematic_waypoint/",
#                "./Results/PolySameTime2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_waypoint/",
#                "./Results/PolySameTime2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_constant/"]

all_folders = ["./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_kinematic_waypoint/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed10/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed5/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-1/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-2/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-1_minsnap1/",
               "./Results/PolySameTimeFull1/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-1_minsnap2/"]

system_types = ["speed10",
                "speed5",
                "speed-1",
                "speed-2",
                "speed-1_minsnap1",
                "speed-1_minsnap2"]


failed_tests = 0

beam_lengths = [10]
depths = [10]
res_numbers = [4]

system_time = []
system_distance = []
system_maximum_deviation = []

for folder in all_folders:

    for system in system_types:

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
                    if max_dev[0][0] > 30:
                        print("Maximum Deviation over 5m (" + str(max_dev[0][0]) + "m): " + str(file_name))
                        continue

                    if max_dev[0][0] > 11 and system == "speed-1_minsnap1":
                        print("Maximum Deviation over 5m (" + str(max_dev[0][0]) + "m): " + str(file_name))

                    # Count how many minsnap corridor failed
                    if max_dev[0][0] > 12 and system == "speed-1_minsnap2":
                        print("Maximum Deviation over 5m (" + str(max_dev[0][0]) + "m): " + str(file_name))
                        failed_tests += 1
                    else:


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
        system_distance.append(total_deviation)
        system_time.append(total_time)
        system_maximum_deviation.append(maximum_deviation)


print("Failed tests: " + str(failed_tests))


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=2)
    # plt.setp(bp['whiskers'], color=color)
    # plt.setp(bp['caps'], color=color)
    # plt.setp(bp['medians'], color=color)
    # plt.setp(bp['fliers'], color=color)
    # plt.setp(bp['means'], color=color)

tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[0]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[6]], showmeans=True)
add_values(bpl, ax1)
add_values(bpr, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Constant Velocity 10m/s")

tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[1]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[13]], showmeans=True)
add_values(bpr, ax1)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Constant Velocity 5m/s")

tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[2]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[20]], showmeans=True)
add_values(bpr, ax1)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Stable Waypoint Controller")

tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[3]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[27]], showmeans=True)
add_values(bpr, ax1)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Unstable Waypoint Controller")


tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[4]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[34]], showmeans=True)
add_values(bpr, ax1)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Minimum Snap No Corridor")

tick_names = ["Random Test", "Our Technique"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[4]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[41]], showmeans=True)
add_values(bpr, ax1)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Minimum Snap With Corridor")


tick_names = ["Velocity 10m/s", "Velocity 5m/s", "Stable Waypoint", "Unstable Waypoint", "Way Min Snap", "Corridor Min Snap"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
bpl = plt.boxplot([system_maximum_deviation[0]] + [system_maximum_deviation[1]] + [system_maximum_deviation[2]] + [system_maximum_deviation[3]] + [system_maximum_deviation[4]] + [system_maximum_deviation[5]], showmeans=True)
add_values(bpl, ax1)
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
plt.xlabel("Test Technique", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Comparison of Performance")
plt.show()


tick_names = ["Velocity 10m/s", "", "Velocity 5m/s", "", "Stable Waypoint", "", "Unstable Waypoint", "", "Way Min Snap", "", "Corridor Min Snap", ""]
fig1, ax1 = plt.subplots(1, 1, figsize=(13, 10))
bpl = plt.boxplot([system_maximum_deviation[0]] + [system_maximum_deviation[41]] + [system_maximum_deviation[1]] + [system_maximum_deviation[41]] + [system_maximum_deviation[2]] + [system_maximum_deviation[41]] + [system_maximum_deviation[3]] + [system_maximum_deviation[41]] + [system_maximum_deviation[4]] + [system_maximum_deviation[41]] + [system_maximum_deviation[5]] + [system_maximum_deviation[41]], showmeans=True)
bpr = plt.boxplot([system_maximum_deviation[41]] + [system_maximum_deviation[6]] + [system_maximum_deviation[41]] + [system_maximum_deviation[13]] + [system_maximum_deviation[41]] + [system_maximum_deviation[20]] + [system_maximum_deviation[41]] + [system_maximum_deviation[27]] + [system_maximum_deviation[41]] + [system_maximum_deviation[34]] + [system_maximum_deviation[41]] + [system_maximum_deviation[41]], showmeans=True)
add_values(bpl, ax1)
set_box_color(bpr, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpl, '#2C7BB6')
plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend()
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=35)
plt.xlabel("Controller", fontsize=15)
plt.ylabel("Maximum Deviation", fontsize=15)
plt.title("Comparison of Performance")
plt.show()

# [system_maximum_deviation[0]] + [system_maximum_deviation[6]] + [system_maximum_deviation[1]] + [system_maximum_deviation[13]] + [system_maximum_deviation[2]] + [system_maximum_deviation[20] + [system_maximum_deviation[3]] + [system_maximum_deviation[27]] + [system_maximum_deviation[4]] + [system_maximum_deviation[34]] + [system_maximum_deviation[5]] + [system_maximum_deviation[41]]








#
#
#
# # Display the compared box plots

#
# # First plot
# tick_names = ["Random Test", "Our Technique"]
# fig1, ax1 = plt.subplots(1, 1, figsize=(10, 10))
# bpl = plt.boxplot([system_maximum_deviation[0]] + [system_maximum_deviation[4]], showmeans=True)
# add_values(bpl, ax1)
# plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
# plt.xlabel("Test Technique", fontsize=15)
# plt.ylabel("Maximum Deviation", fontsize=15)
# plt.title("Unstable Waypoint Controller")
#
# fig2, ax1 = plt.subplots(1, 1, figsize=(10, 10))
# bpl = plt.boxplot([system_maximum_deviation[1]] + [system_maximum_deviation[9]], showmeans=True)
# add_values(bpl, ax1)
# plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
# plt.xlabel("Test Technique", fontsize=15)
# plt.ylabel("Maximum Deviation", fontsize=15)
# plt.title("Waypoint Controller")
#
# fig3, ax1 = plt.subplots(1, 1, figsize=(10, 10))
# bpl = plt.boxplot([system_maximum_deviation[2]] + [system_maximum_deviation[14]], showmeans=True)
# add_values(bpl, ax1)
# plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
# plt.xlabel("Test Technique", fontsize=15)
# plt.ylabel("Maximum Deviation", fontsize=15)
# plt.title("Constant Speed 5m/s")
#
# fig4, ax1 = plt.subplots(1, 1, figsize=(10, 10))
# bpl = plt.boxplot([system_maximum_deviation[3]] + [system_maximum_deviation[19]], showmeans=True)
# add_values(bpl, ax1)
# plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=12, rotation=0)
# plt.xlabel("Test Technique", fontsize=15)
# plt.ylabel("Maximum Deviation", fontsize=15)
# plt.title("Constant Speed 10m/s")
# plt.show()
#
#
# # for i in range(0, len(system_types)):
# #     bpl = plt.boxplot(system_maximum_deviation[i], positions=np.array(range(len(system_maximum_deviation[i])))*2.0-0.4, widths=0.6, showmeans=True)
# #     add_values(bpl, ax1, left=True)
# #     bpr = plt.boxplot(system_waypoint_maximum_deviation, positions=np.array(range(len(system_waypoint_maximum_deviation)))*2.0+0.4, widths=0.6, showmeans=True)
# #     add_values(bpr, ax1)
# #     set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
# #     set_box_color(bpr, '#2C7BB6')
# # #
# # # draw temporary red and blue lines and use them to create a legend
# # plt.plot([], c='#D7191C', label='Constant Velocity')
# # plt.plot([], c='#2C7BB6', label='Waypoint Controller')
# # plt.legend()
# # plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=12, rotation=15)
# # plt.xlim(-2, len(tick_names)*2)
# # plt.tight_layout()
# #
# #
# #
# # print("----------------------------------------------------------")
# # print("Maximum Deviation Statistics")
# # print("Random Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[0])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[0])))
# # print("Waypoint Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[1])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[1])))
# # print("Constant Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_maximum_deviation[2])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_maximum_deviation[2])))
# # print("----------------------------------------------------------")
# #
# #
# #
# #
# # fig, ax1 = plt.subplots(1, 1, figsize=(10, 10))
# # bpl = plt.boxplot(system_constant_distance, positions=np.array(range(len(system_constant_distance)))*2.0-0.4, widths=0.6, showmeans=True)
# # add_values(bpl, ax1, left=True)
# # bpr = plt.boxplot(system_waypoint_distance, positions=np.array(range(len(system_waypoint_distance)))*2.0+0.4, widths=0.6, showmeans=True)
# # add_values(bpr, ax1)
# # set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
# # set_box_color(bpr, '#2C7BB6')
# #
# # # draw temporary red and blue lines and use them to create a legend
# # plt.plot([], c='#D7191C', label='Constant Velocity')
# # plt.plot([], c='#2C7BB6', label='Waypoint Controller')
# # plt.legend()
# # plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=12, rotation=15)
# # plt.xlim(-2, len(tick_names)*2)
# # plt.tight_layout()
# # plt.ylabel("Accumulated Deviation", fontsize=15)
# # plt.xlabel("Test Set", fontsize=15)
# #
# #
# # print("----------------------------------------------------------")
# # print("Accumulated Deviation Statistics")
# # print("Random Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[0])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[0])))
# # print("Waypoint Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[1])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[1])))
# # print("Constant Test Set:")
# # print("Constant Velocity Controller Mean: " + str(np.mean(system_constant_distance[2])))
# # print("Waypoint Controller Mean: " + str(np.mean(system_waypoint_distance[2])))
# # print("----------------------------------------------------------")
# #
# # # va = [0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0]
# # # for t, y in zip(ax1.get_xticklabels(), va):
# # #     t.set_y(y)
# #
# #
# # fig1, ax2 = plt.subplots(1, 1, figsize=(10, 10))
# # bpl = plt.boxplot(system_constant_time, positions=np.array(range(len(system_constant_time)))*2.0-0.4, widths=0.6, showmeans=True)
# # add_values(bpl, ax2, left=True)
# # bpr = plt.boxplot(system_waypoint_time, positions=np.array(range(len(system_waypoint_time)))*2.0+0.4, widths=0.6, showmeans=True)
# # add_values(bpr, ax2)
# # set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
# # set_box_color(bpr, '#2C7BB6')
# #
# # # draw temporary red and blue lines and use them to create a legend
# # plt.plot([], c='#D7191C', label='Constant Velocity')
# # plt.plot([], c='#2C7BB6', label='Waypoint Controller')
# # plt.legend()
# # plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=12, rotation=15)
# # plt.xlim(-2, len(tick_names)*2)
# # plt.tight_layout()
# # plt.ylabel("Total Time", fontsize=15)
# # plt.xlabel("Test Set", fontsize=15)
# #
# #
# # plt.show()