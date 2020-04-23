import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string
from matplotlib.ticker import FormatStrFormatter


def annotate_boxplot(bpdict, positions, text, top=False):

    shift = [2.5, 3.5, 0.5, 3]
    counter = 0

    for pos in positions:
        if counter == 0:
            plt.text(x=(pos - 0.1), y=(bpdict['caps'][counter*2].get_ydata()[0] - shift[counter]), s=text, rotation=-90, rotation_mode='anchor', backgroundcolor='white', fontsize=12)
        else:
            plt.text(x=(pos + 0.1), y=(bpdict['caps'][(counter*2)+1].get_ydata()[0] + shift[counter]), s=text, rotation=90, rotation_mode='anchor', backgroundcolor='white', fontsize=12)
        counter += 1


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=3)
    plt.setp(bp['whiskers'], linewidth=2)
    plt.setp(bp['caps'], linewidth=2)
    plt.setp(bp['medians'], linewidth=2)
    # plt.setp(bp['fliers'], linewidth=3)
    plt.setp(bp['means'], linewidth=3)


def add_values(bp, ax, left=False):
    """ This actually adds the numbers to the various points of the boxplots"""
    for element in ['means']:
        for line in bp[element]:
            # Get the position of the element. y is the label you want
            x_r, x_l, y = 0, 0, 0
            if element == "means":
                a = line.get_xydata()[0]
                x_r = a[0]
                y = a[1]
            else:
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
                    x_line_center = x_r + 0.25
                y_line_center = y  # Since it's a line and it's horisontal
                # overlay the value:  on the line, from center to right
                ax.text(x_line_center, y_line_center, # Position
                        '%.2f' % y, # Value (3f = 3 decimal float)
                        verticalalignment='center', # Centered vertically with line
                        fontsize=12,
                        fontweight='bold')

# For RQ2
main_folder = "../FinalResults/"

# For the RQ2 length 10
all_folders = ["initial_run_flown/initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime7200_simtime90_searchtype_kinematic_scoretype_random/",
                "handcrafted_run_flown/handcrafted_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_edge/",
                "handcrafted_run_flown/handcrafted_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_edge90/",
                "handcrafted_run_flown/handcrafted_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_edge180/",
                "learned_run_flown/learned_speed-2_minsnap0_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/",
                "learned_run_flown/learned_speed-1_minsnap0_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/",
                "learned_run_flown/learned_speed2_minsnap0_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/",
                "learned_run_flown/learned_speed5_minsnap0_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/",
                "learned_run_flown/learned_speed10_minsnap0_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/",
                "learned_run_flown/learned_speed-1_minsnap1_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/"]


# All the different system types which are generated using the WorldEngineSimulator
system_types = ["speed-2_minsnap0",
                "speed-1_minsnap0",
                "speed2_minsnap0",
                "speed5_minsnap0",
                "speed-1_minsnap1"]


# System type names
ticks = ["Unstable Waypoint", "Stable Waypoint", "Fixed Velocity Slow", "Fixed Velocity Normal", "Fixed Velocity Fast", "Min Snap"]




failed_tests = 0
beam_lengths = [10]
depths = [10]
res_numbers = [4]

final_data = []

for folder in all_folders:

    for system in system_types:

        scores = []
        average_deviation = []
        travelled_length =[]
        total_deviation = []
        average_time = []
        total_time = []
        distance_heuristic = []
        time_heuristic = []
        maximum_deviation = []
        trajectory_length = []
        avg_vel_heuristic = []
        max_vel_heuristic = []
        avg_acc_heuristic = []
        max_acc_heuristic = []

        for depth in depths:
            for beam in beam_lengths:

                file_location = main_folder + folder
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
                    
                    # Get the average and maximum velocity
                    avg_vel = get_numbers_after_string(file_name=file_name, the_string="Average Velocity:")
                    max_vel = get_numbers_after_string(file_name=file_name, the_string="Maximum Velocity:")

                    # Get the average and maximum acceleration
                    avg_acc = get_numbers_after_string(file_name=file_name, the_string="Average Acceleration:")
                    max_acc = get_numbers_after_string(file_name=file_name, the_string="Maximum Acceleration:")

                    # Get the maximum deviation from the optimal trajectory
                    max_dev = get_numbers_after_string(file_name=file_name, the_string="Maximum deviation from optimal trajectory:")

                    # Get the total distance travelled and the total trajectory length
                    tot_dist = get_numbers_after_string(file_name=file_name, the_string="Total distance travelled:")
                    traj_len = get_numbers_after_string(file_name=file_name, the_string="Trajectory length:")
                    optimal_distance_heuristic = tot_dist[0][0] / traj_len[0][0]

                    # Get the number of waypoints (This should be equal to the trajectory's optimal time)
                    num_way = get_numbers_after_string(file_name=file_name, the_string="Total waypoints:")
                    optimal_time_heuristic = tot_time[0][0] / num_way[0][0]

                    # Check for any anomalies
                    if max_dev[0][0] > 40:
                        print("Maximum Deviation over 40m (" + str(max_dev[0][0]) + "m): " + str(file_name))
                        failed_tests += 1
                        continue

                    elif max_dev[0][0] > 15 and system == "speed-1_minsnap1":
                        print("Maximum Deviation over 15m (" + str(max_dev[0][0]) + "m): " + str(file_name))
                        failed_tests += 1
                        continue

                    # Count how many minsnap corridor failed
                    elif max_acc[0][0] > 30:
                        print("Maximum Acceleration over 30m^2 (" + str(max_acc[0][0]) + "m): " + str(file_name))
                        failed_tests += 1
                        continue

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
                        travelled_length.append(tot_dist[0][0])
                        avg_vel_heuristic.append(avg_vel[0][0])
                        max_vel_heuristic.append(max_vel[0][0])
                        avg_acc_heuristic.append(avg_acc[0][0])
                        max_acc_heuristic.append(max_acc[0][0])

        # Save the data into each respective system

        record = {
            'total_deviation': total_deviation,
            'total_time': total_time,
            'max_deviation': maximum_deviation,
            'test_set': folder,
            'system_type': system,
            'total_deviation': total_deviation,
            'average_deviation': average_deviation,
            'total_travelled_length': travelled_length,
            'avg_velocity': avg_vel_heuristic,
            'max_velocity': max_vel_heuristic,
            'avg_acceleration': avg_acc_heuristic,
            'max_acceleration': max_acc_heuristic,
        }

        final_data.append(record)


print("Failed tests: " + str(failed_tests))















# # RQ2) <------ Log Scale

# # Get the results

# selectedR2_systems = ["speed-2_minsnap0", "speed-1_minsnap0", "speed2_minsnap0", "speed-1_minsnap1"]
# randomscore_results = []
# edgecore_results = []
# edge90score_results = []
# edge180score_results = []
# learnedscore_results = []

# for sys in selectedR2_systems:
#     for item in final_data:
#         if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
#             randomscore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge/" in item['test_set'] and sys == item['system_type']:
#             edgecore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge90/" in item['test_set'] and sys == item['system_type']:
#             edge90score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge180/" in item['test_set'] and sys == item['system_type']:
#             edge180score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_learned/" in item['test_set'] and sys == item['system_type'] and sys in item['test_set']:
#             learnedscore_results.append(item['max_deviation'])



# ticks = ["Unstable Waypoint", "Stable Waypoint", "Fixed Velocity", "Minimum Snap"]
# fig1, ax1 = plt.subplots(1, 1, figsize=(10, 7))

# bp1_pos = positions=4*np.arange(len(randomscore_results))
# bp1 = plt.boxplot(randomscore_results, positions=bp1_pos, showmeans=True)
# annotate_boxplot(bp1, positions=bp1_pos, text="No Scoring")

# bp2_pos = positions=4*np.arange(len(edgecore_results))+0.6
# bp2 = plt.boxplot(edgecore_results, positions=bp2_pos, showmeans=True)
# annotate_boxplot(bp2, positions=bp2_pos, text="High Velocity")

# bp3_pos = positions=4*np.arange(len(edge90score_results))+1.2
# bp3 = plt.boxplot(edge90score_results, positions=bp3_pos, showmeans=True)
# annotate_boxplot(bp3, positions=bp3_pos, text="High Velocity + 90 Deg")

# bp4_pos = positions=4*np.arange(len(edge180score_results))+1.8
# bp4 = plt.boxplot(edge180score_results, positions=bp4_pos, showmeans=True)
# annotate_boxplot(bp4, positions=bp4_pos, text="High Velocity + 180 Deg")

# bp5_pos = positions=4*np.arange(len(learnedscore_results))+2.4
# bp5 = plt.boxplot(learnedscore_results, positions=bp5_pos, showmeans=True)
# annotate_boxplot(bp5, positions=bp5_pos, text="Learned Scoring")

# ax1.grid()
# ax1.grid(which='minor', linestyle='--', linewidth=0.5)

# # add_values(bpl, ax1)
# # add_values(bpr, ax1)
# set_box_color(bp1, 'C0')
# set_box_color(bp2, 'C5')
# set_box_color(bp3, 'C2')
# set_box_color(bp4, 'C6')
# set_box_color(bp5, 'C3')

# # plt.plot([], c='C0', linewidth=3, label='No Scoring')
# # plt.plot([], c='C5', linewidth=3, label='High Velocity')
# # plt.plot([], c='C2', linewidth=3, label='High Velocity + 90 Deg')
# # plt.plot([], c='C6', linewidth=3, label='High Velocity + 180 Deg')
# # plt.plot([], c='C3', linewidth=3, label='Learned Scoring')
# # ax1.legend(loc='upper center', bbox_to_anchor=(0.5, 1.25), ncol=2, fontsize=18)
# # plt.subplots_adjust(top=0.8)

# plt.xlim([-0.5, 4*len(randomscore_results)-1.15])

# plt.yticks(fontsize=15)
# plt.xticks(4 * np.arange(len(ticks)) + 1.2, ticks, fontsize=15, rotation=0)

# plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
# plt.ylabel("Maximum Deviation", fontweight='bold', fontsize=20)
# plt.minorticks_on()
# # log scale
# plt.yscale('log')
# plt.tick_params(axis='y', which='minor', labelsize=15)
# ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
# ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))

# plt.minorticks_on()
# plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
# plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)
# plt.ylim([0,50])
# plt.show()



















# # RQ2) <------ Ratio

# # Get the results

# selectedR2_systems = ["speed-2_minsnap0", "speed-1_minsnap0", "speed2_minsnap0", "speed-1_minsnap1"]
# randomscore_results = []
# edgecore_results = []
# edge90score_results = []
# edge180score_results = []
# learnedscore_results = []

# for sys in selectedR2_systems:
#     for item in final_data:
#         if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
#             randomscore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge/" in item['test_set'] and sys == item['system_type']:
#             edgecore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge90/" in item['test_set'] and sys == item['system_type']:
#             edge90score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge180/" in item['test_set'] and sys == item['system_type']:
#             edge180score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_learned/" in item['test_set'] and sys == item['system_type'] and sys in item['test_set']:
#             learnedscore_results.append(item['max_deviation'])



# # Get the ratio of each
# for i in range(0,4):
#     mean = np.median(randomscore_results[i])
#     print("Median: " + str(mean))

#     randomscore_results[i] = randomscore_results[i] / mean
#     edgecore_results[i] = edgecore_results[i] / mean
#     edge90score_results[i] = edge90score_results[i] / mean
#     edge180score_results[i] = edge180score_results[i] / mean
#     learnedscore_results[i] = learnedscore_results[i] / mean
    
# ticks = ["Unstable Waypoint", "Stable Waypoint", "Fixed Velocity", "Minimum Snap"]
# fig1, ax1 = plt.subplots(1, 1, figsize=(10, 7))

# bp1_pos = positions=4*np.arange(len(randomscore_results))
# bp1 = plt.boxplot(randomscore_results, positions=bp1_pos, showmeans=True)
# annotate_boxplot(bp1, positions=bp1_pos, text="No Scoring")

# bp2_pos = positions=4*np.arange(len(edgecore_results))+0.6
# bp2 = plt.boxplot(edgecore_results, positions=bp2_pos, showmeans=True)
# annotate_boxplot(bp2, positions=bp2_pos, text="High Velocity")

# bp3_pos = positions=4*np.arange(len(edge90score_results))+1.2
# bp3 = plt.boxplot(edge90score_results, positions=bp3_pos, showmeans=True)
# annotate_boxplot(bp3, positions=bp3_pos, text="High Velocity + 90 Deg")

# bp4_pos = positions=4*np.arange(len(edge180score_results))+1.8
# bp4 = plt.boxplot(edge180score_results, positions=bp4_pos, showmeans=True)
# annotate_boxplot(bp4, positions=bp4_pos, text="High Velocity + 180 Deg")

# bp5_pos = positions=4*np.arange(len(learnedscore_results))+2.4
# bp5 = plt.boxplot(learnedscore_results, positions=bp5_pos, showmeans=True)
# annotate_boxplot(bp5, positions=bp5_pos, text="Learned Scoring")

# ax1.grid()
# ax1.grid(which='minor', linestyle='--', linewidth=0.5)

# set_box_color(bp1, 'C0')
# set_box_color(bp2, 'C5')
# set_box_color(bp3, 'C2')
# set_box_color(bp4, 'C6')
# set_box_color(bp5, 'C3')

# plt.xlim([-0.5, 4*len(randomscore_results)-1.15])

# plt.yticks(fontsize=15)
# plt.xticks(4 * np.arange(len(ticks)) + 1.2, ticks, fontsize=15, rotation=0)

# plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
# plt.ylabel("Ratio Improvement", fontweight='bold', fontsize=20)

# # log scale
# # plt.yscale('log')
# plt.tick_params(axis='y', which='minor', labelsize=15)
# # ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
# ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))

# plt.minorticks_on()
# plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
# plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)
# fig1.tight_layout()
# plt.show()


























# # RQ2) <------ Seperate Axes

# selectedR2_systems = ["speed-2_minsnap0", "speed-1_minsnap0", "speed2_minsnap0", "speed-1_minsnap1"]
# randomscore_results = []
# edgecore_results = []
# edge90score_results = []
# edge180score_results = []
# learnedscore_results = []

# for sys in selectedR2_systems:
#     for item in final_data:
#         if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
#             randomscore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge/" in item['test_set'] and sys == item['system_type']:
#             edgecore_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge90/" in item['test_set'] and sys == item['system_type']:
#             edge90score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_edge180/" in item['test_set'] and sys == item['system_type']:
#             edge180score_results.append(item['max_deviation'])

#     for item in final_data:
#         if "scoretype_learned/" in item['test_set'] and sys == item['system_type'] and sys in item['test_set']:
#             learnedscore_results.append(item['max_deviation'])


# fig2, ax2 = plt.subplots(1, 1, figsize=(10, 5))
# # Get the ratio of each
# for i in range(0,4):
#     mean = np.median(randomscore_results[i])
#     print("Median: " + str(mean))

#     randomscore_results[i] = randomscore_results[i] / mean
#     edgecore_results[i] = edgecore_results[i] / mean
#     edge90score_results[i] = edge90score_results[i] / mean
#     edge180score_results[i] = edge180score_results[i] / mean
#     learnedscore_results[i] = learnedscore_results[i] / mean

#     plt.subplot(141 + (i))
#     plt.boxplot([randomscore_results[i], edgecore_results[i], edge90score_results[i], edge180score_results[i], learnedscore_results[i]])
#     plt.minorticks_on()
#     plt.grid(b=True, which='major', linestyle='-', linewidth=0.5)
#     plt.grid(b=True, which='minor', linestyle='--', linewidth=0.5)

#     plt.xlabel(ticks[i])
#     if i == 0:
#         plt.ylabel("Maximum Deviation")

# fig2.tight_layout()
# plt.minorticks_on()
# plt.show()

















# RQ2) <------ Ratio Final

# Get the results

selectedR2_systems = ["speed-2_minsnap0", "speed-1_minsnap0", "speed2_minsnap0", "speed-1_minsnap1"]
randomscore_results = []
edgecore_results = []
edge90score_results = []
edge180score_results = []
learnedscore_results = []

for sys in selectedR2_systems:
    for item in final_data:
        if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
            randomscore_results.append(item['max_deviation'])

    for item in final_data:
        if "scoretype_edge/" in item['test_set'] and sys == item['system_type']:
            edgecore_results.append(item['max_deviation'])

    for item in final_data:
        if "scoretype_edge90/" in item['test_set'] and sys == item['system_type']:
            edge90score_results.append(item['max_deviation'])

    for item in final_data:
        if "scoretype_edge180/" in item['test_set'] and sys == item['system_type']:
            edge180score_results.append(item['max_deviation'])

    for item in final_data:
        if "scoretype_learned/" in item['test_set'] and sys == item['system_type'] and sys in item['test_set']:
            learnedscore_results.append(item['max_deviation'])



# Get the ratio of each
for i in range(0,4):
    mean = np.mean(randomscore_results[i])
    randomscore_results[i] = randomscore_results[i] / mean
    edgecore_results[i] = edgecore_results[i] / mean
    edge90score_results[i] = edge90score_results[i] / mean
    edge180score_results[i] = edge180score_results[i] / mean
    learnedscore_results[i] = learnedscore_results[i] / mean

best_increase_percentage_handcraft = []
increase_percentage_learned = []

for i in range(0, 4):
    mean = np.mean(randomscore_results[i])
    print("Mean Deviation for Random:\t" + str(mean))
    print("Ratio High Velocity:\t\t" + str(np.mean(edgecore_results[i])))
    print("Ratio High Velocity + 90:\t" + str(np.mean(edge90score_results[i])))
    print("Ratio High Velocity + 180:\t" + str(np.mean(edge180score_results[i])))
    print("Ratio Learned Model:\t\t" + str(np.mean(learnedscore_results[i])))
    best_increase_percentage_handcraft.append(max(np.mean(edgecore_results[i]), np.mean(edge90score_results[i]), np.mean(edge180score_results[i])) - 1)
    increase_percentage_learned.append(np.mean(learnedscore_results[i]) - 1)
    print("-------------------------------------------------")

print("Best Handcrafted Increases: " + str(best_increase_percentage_handcraft))
print("Best Learned Increases: " + str(increase_percentage_learned))
print("Overall Handcraft Increase: " + str(np.mean(best_increase_percentage_handcraft) * 100))
print("Overall Learned Increase: " + str(np.mean(increase_percentage_learned) * 100))
    
    
ticks = ["High Velocity", "High Velocity + 90 Deg", "High Velocity + 180 Deg", "Learned"]
fig1, ax1 = plt.subplots(1, 1, figsize=(15, 7))

# Draw the central line
plt.axhline(y=1, color='gray', linestyle='--')

# Seperate the plot
plt.axvline(x=5, color='black', linestyle='-', linewidth=0.5)
plt.axvline(x=10, color='black', linestyle='-', linewidth=0.5)
plt.axvline(x=15, color='black', linestyle='-', linewidth=0.5)

# Plot the box plots
bp2_pos = positions=5*np.arange(len(edgecore_results))+1
bp2 = plt.boxplot(edgecore_results, positions=bp2_pos, showmeans=True)
add_values(bp2, ax1)
# annotate_boxplot(bp2, positions=bp2_pos, text="High Velocity")

bp3_pos = positions=5*np.arange(len(edge90score_results))+2
bp3 = plt.boxplot(edge90score_results, positions=bp3_pos, showmeans=True)
add_values(bp3, ax1)
# annotate_boxplot(bp3, positions=bp3_pos, text="High Velocity + 90 Deg")

bp4_pos = positions=5*np.arange(len(edge180score_results))+3
bp4 = plt.boxplot(edge180score_results, positions=bp4_pos, showmeans=True)
add_values(bp4, ax1)
# annotate_boxplot(bp4, positions=bp4_pos, text="High Velocity + 180 Deg")

bp5_pos = positions=5*np.arange(len(learnedscore_results))+4
bp5 = plt.boxplot(learnedscore_results, positions=bp5_pos, showmeans=True)
add_values(bp5, ax1)
# annotate_boxplot(bp5, positions=bp5_pos, text="Learned Scoring")


# Set the right scale
plt.xlim([0, 5*len(randomscore_results)])

# Change the Y ticks font size
plt.yticks(fontsize=15)

# Plot the minor and major grid for the Y axis
plt.minorticks_on()
plt.grid(b=True, which='major', axis='y', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', axis='y', linestyle='--', linewidth=0.5)

# Add the ticks for X
final_ticks = []
final_ticks += ticks
final_ticks += [""]
final_ticks += ticks
final_ticks += [""]
final_ticks += ticks
final_ticks += [""]
final_ticks += ticks
final_ticks += [""]
plt.xticks(np.arange(len(final_ticks)) + 1, final_ticks, fontsize=12, rotation=15, ha="right", rotation_mode="anchor")
ax1.tick_params(axis='x', which='minor', bottom=False)
ax1.tick_params(axis='x', length=10, which='major')
for a in ax1.xaxis.get_majorticklabels():
    a.set_y(.005)

# Plot the minor and major grid for the X axis
plt.grid(b=True, which='major', axis='x', linestyle='-', linewidth=0.5)
plt.grid(b=False, which='minor', axis='x')

# Add the labels
plt.xlabel("Scoring Model", fontweight='bold', fontsize=20)
plt.ylabel("Max $\mathregular{Dev_{Scoring Model}}$ \ Max $\mathregular{Dev_{No Scoring}}$", fontweight='bold', fontsize=15)

topticks = ["Unstable Waypoint", "Stable Waypoint", "Fixed Velocity", "Minimum Snap"]
new_tick_locations = np.array([2.5, 7.5 , 12.5 ,17.5])
ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
ax2.set_xticks(new_tick_locations)
ax2.set_xticklabels(topticks, fontsize=15)
ax2.tick_params(axis='x', length=0, which='major')
for a in ax2.xaxis.get_majorticklabels():
    a.set_y(a.get_position()[1]-.01)


# Colour the boxes
set_box_color(bp2, 'C5')
set_box_color(bp3, 'C2')
set_box_color(bp4, 'C6')
set_box_color(bp5, 'C3')

# Turn off the bottom minor labels
ax1.tick_params(axis='x', which='minor', bottom=False)

# Display graph
fig1.tight_layout()
plt.show()
