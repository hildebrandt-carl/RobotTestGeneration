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

all_folders = ["./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_kinematic_waypoint/",
               "./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed10/",
               "./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed5/",
               "./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-1/",
               "./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-2/",
               "./Results/PolySameTimeFull2/MIT_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simtime90_score_speed-1_minsnap1/"]

system_types = ["speed-2",
                "speed-1",
                "speed5",
                "speed10",
                "speed-1_minsnap1"]

ticks = ["Waypoint Unstable", "Waypoint Stable", "Fixed Velocity Slow", "Fixed Velocity Fast", "Min Snap"]


failed_tests = 0

beam_lengths = [10]
depths = [10]
res_numbers = [4]

final_data = []

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

        record = {
            'total_deviation': total_deviation,
            'total_time': total_time,
            'max_deviation': maximum_deviation,
            'test_set': folder,
            'system_type': system,
            'total_deviation': total_deviation,
            'average_deviation': average_deviation,
        }

        final_data.append(record)


print("Failed tests: " + str(failed_tests))


# Create the data from running on the kinematic set
kin_max_dev = []
kin_tot_time = []
kin_tot_dev = []
kin_avg_dev = []
for sys in system_types:
    for item in final_data:
        if "kinematic" in item['test_set'] and sys == item['system_type']:
            kin_max_dev.append(item['max_deviation'])
            kin_tot_time.append(item['total_time'])
            kin_tot_dev.append(item['total_deviation'])
            kin_avg_dev.append(item['average_deviation'])

our_max_dev = []
our_tot_time = []
our_tot_dev = []
our_avg_dev = []
for sys in system_types:
    for item in final_data:
        if (sys in item['test_set']) and (sys == item['system_type']):
            # Check if there is nothing after sys in item['test_set']
            if (len(item['test_set'][item['test_set'].find(sys):-1]) == len(sys)):
                our_max_dev.append(item['max_deviation'])
                our_tot_time.append(item['total_time'])
                our_tot_dev.append(item['total_deviation'])
                our_avg_dev.append(item['average_deviation'])
    print("------------------------------")




def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=3)
    plt.setp(bp['whiskers'], linewidth=2)
    plt.setp(bp['caps'], linewidth=2)
    plt.setp(bp['medians'], linewidth=2)
    # plt.setp(bp['fliers'], linewidth=3)
    # plt.setp(bp['means'], linewidth=3)


fig1, ax1 = plt.subplots(1, 1, figsize=(10, 9))

bpl = plt.boxplot(kin_max_dev, positions=1.5*np.arange(len(kin_max_dev)), showmeans=True)
bpr = plt.boxplot(our_max_dev, positions=1.5*np.arange(len(our_max_dev))+0.6, showmeans=True)

# add_values(bpl, ax1)
# add_values(bpr, ax1)
set_box_color(bpl, '#2C7BB6')
set_box_color(bpr, '#D7191C')

plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend(fontsize=20)

plt.xlim([-0.5, 1.5*len(our_max_dev)-0.5])

plt.yticks(fontsize=15)
plt.xticks(1.5 * np.arange(len(ticks)) + 0.3, ticks, fontsize=15, rotation=10)

plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
plt.ylabel("Maximum Deviation", fontweight='bold', fontsize=20)

# log scale
from matplotlib.ticker import FormatStrFormatter
plt.yscale('log')
plt.tick_params(axis='y', which='minor', labelsize=15)
ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))

plt.show()








fig1, ax1 = plt.subplots(1, 1, figsize=(10, 9))

bpl = plt.boxplot(kin_avg_dev, positions=1.5*np.arange(len(kin_avg_dev)), showmeans=True)
bpr = plt.boxplot(our_avg_dev, positions=1.5*np.arange(len(our_avg_dev))+0.6, showmeans=True)

# add_values(bpl, ax1)
# add_values(bpr, ax1)
set_box_color(bpl, '#2C7BB6')
set_box_color(bpr, '#D7191C')

plt.plot([], c='#D7191C', label='Computed Tests')
plt.plot([], c='#2C7BB6', label='Random Tests')
plt.legend(fontsize=20)

plt.xlim([-0.5, 1.5*len(our_avg_dev)-0.5])

plt.yticks(fontsize=15)
plt.xticks(1.5 * np.arange(len(ticks)) + 0.3, ticks, fontsize=15, rotation=10)

plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
plt.ylabel("Average Deviation", fontweight='bold', fontsize=20)

# log scale
from matplotlib.ticker import FormatStrFormatter
plt.yscale('log')
plt.tick_params(axis='y', which='minor', labelsize=15)
ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))

plt.show()
