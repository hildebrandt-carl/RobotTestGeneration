import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string



def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=3)
    plt.setp(bp['whiskers'], linewidth=2)
    plt.setp(bp['caps'], linewidth=2)
    plt.setp(bp['medians'], linewidth=2)
    # plt.setp(bp['fliers'], linewidth=3)
    # plt.setp(bp['means'], linewidth=3)


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





# For RQ2
main_folder = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/"

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






# RQ2) <------ Multiple Controllers

selectedR2_systems = ["speed-1_minsnap0", "speed-1_minsnap1"]
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


ticks = ["Stable Waypoint Controller", "Minimum Snap Controller"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 7))

bp1 = plt.boxplot(randomscore_results, positions=4*np.arange(len(randomscore_results)), showmeans=True)
bp2 = plt.boxplot(edgecore_results, positions=4*np.arange(len(edgecore_results))+0.6, showmeans=True)
bp3 = plt.boxplot(edge90score_results, positions=4*np.arange(len(edge90score_results))+1.2, showmeans=True)
bp4 = plt.boxplot(edge180score_results, positions=4*np.arange(len(edge180score_results))+1.8, showmeans=True)
bp5 = plt.boxplot(learnedscore_results, positions=4*np.arange(len(learnedscore_results))+2.4, showmeans=True)

ax1.grid()
ax1.grid(which='minor', linestyle='--', linewidth=0.5)

# add_values(bpl, ax1)
# add_values(bpr, ax1)
set_box_color(bp1, 'C0')
set_box_color(bp2, 'C5')
set_box_color(bp3, 'C2')
set_box_color(bp4, 'C6')
set_box_color(bp5, 'C3')

plt.plot([], c='C0', linewidth=3, label='No Scoring')
plt.plot([], c='C5', linewidth=3, label='High Velocity')
plt.plot([], c='C2', linewidth=3, label='High Velocity + 90 Deg')
plt.plot([], c='C6', linewidth=3, label='High Velocity + 180 Deg')
plt.plot([], c='C3', linewidth=3, label='Learned Scoring')
ax1.legend(loc='upper center', bbox_to_anchor=(0.5, 1.25), ncol=2, fontsize=18)
plt.subplots_adjust(top=0.8)

plt.xlim([-0.5, 4*len(randomscore_results)-1.15])

plt.yticks(fontsize=15)
plt.xticks(4 * np.arange(len(ticks)) + 1.2, ticks, fontsize=15, rotation=0)

plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
plt.ylabel("Maximum Deviation", fontweight='bold', fontsize=20)

# log scale
from matplotlib.ticker import FormatStrFormatter
plt.yscale('log')
plt.tick_params(axis='y', which='minor', labelsize=15)
ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))

plt.show()


# RQ2) <------ Multiple Metrics

selectedR2_systems = ["speed-1_minsnap0"]
randomscore_results = []
edgecore_results = []
edge90score_results = []
edge180score_results = []
learnedscore_results = []

for sys in selectedR2_systems:
    for item in final_data:
        if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
            randomscore_results.append(item['max_deviation'])
            randomscore_results.append(item['max_velocity'])
            randomscore_results.append(item['max_acceleration'])

    for item in final_data:
        if "scoretype_edge/" in item['test_set'] and sys == item['system_type']:
            edgecore_results.append(item['max_deviation'])
            edgecore_results.append(item['max_velocity'])
            edgecore_results.append(item['max_acceleration'])

    for item in final_data:
        if "scoretype_edge90/" in item['test_set'] and sys == item['system_type']:
            edge90score_results.append(item['max_deviation'])
            edge90score_results.append(item['max_velocity'])
            edge90score_results.append(item['max_acceleration'])

    for item in final_data:
        if "scoretype_edge180/" in item['test_set'] and sys == item['system_type']:
            edge180score_results.append(item['max_deviation'])
            edge180score_results.append(item['max_velocity'])
            edge180score_results.append(item['max_acceleration'])

    for item in final_data:
        if "scoretype_learned/" in item['test_set'] and sys == item['system_type'] and sys in item['test_set']:
            learnedscore_results.append(item['max_deviation'])
            learnedscore_results.append(item['max_velocity'])
            learnedscore_results.append(item['max_acceleration'])


ticks = ["Maximum Deviation", "Maximum Velocity", "Maximum Acceleration"]
fig1, ax1 = plt.subplots(1, 1, figsize=(10, 7))

bp1 = plt.boxplot(randomscore_results, positions=3.5*np.arange(len(randomscore_results)), showmeans=True)
bp2 = plt.boxplot(edgecore_results, positions=3.5*np.arange(len(edgecore_results))+0.6, showmeans=True)
bp3 = plt.boxplot(edge90score_results, positions=3.5*np.arange(len(edge90score_results))+1.2, showmeans=True)
bp4 = plt.boxplot(edge180score_results, positions=3.5*np.arange(len(edge180score_results))+1.8, showmeans=True)
bp5 = plt.boxplot(learnedscore_results, positions=3.5*np.arange(len(learnedscore_results))+2.4, showmeans=True)

ax1.grid()
ax1.grid(which='minor', linestyle='--', linewidth=0.5)

# add_values(bpl, ax1)
# add_values(bpr, ax1)
set_box_color(bp1, 'C0')
set_box_color(bp2, 'C5')
set_box_color(bp3, 'C2')
set_box_color(bp4, 'C6')
set_box_color(bp5, 'C3')

plt.plot([], c='C0', linewidth=3, label='No Scoring')
plt.plot([], c='C5', linewidth=3, label='Edge of Reachable Set')
plt.plot([], c='C2', linewidth=3, label='Edge of Reachable Set + 90 Deg')
plt.plot([], c='C6', linewidth=3, label='Edge of Reachable Set + 180 Deg')
plt.plot([], c='C3', linewidth=3, label='Learned Model')
plt.legend(fontsize=18)

plt.xlim([-0.5, 3.5*len(randomscore_results)-0.5])

plt.yticks(fontsize=15)
plt.xticks(3.5 * np.arange(len(ticks)) + 1.2, ticks, fontsize=15, rotation=0)

plt.xlabel("Controller Type", fontweight='bold', fontsize=20)
plt.ylabel("Maximum Deviation", fontweight='bold', fontsize=20)

# log scale
from matplotlib.ticker import FormatStrFormatter
plt.yscale('log')
plt.tick_params(axis='y', which='minor', labelsize=15)
ax1.yaxis.set_minor_formatter(FormatStrFormatter("%.1f"))
ax1.yaxis.set_major_formatter(FormatStrFormatter("%.1f"))
plt.show()


