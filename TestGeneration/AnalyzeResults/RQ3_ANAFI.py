import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string

def annotate_boxplot(bpdict, positions, text, top=False):

    counter = 0
    for pos in positions:
            plt.text(x=(pos + 0.21), y=(bpdict['medians'][(counter*2)].get_ydata()[0]-0.5), s=text, rotation=90, rotation_mode='anchor', backgroundcolor='white', fontsize=12)

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



# # For RQ3
main_folder = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/"

# For outdoor outdoor
# # # For the RQ3 length 10
all_folders = ["initial_run_flown/initial_ANAFI_seed10_length10_nodes250_res4_beamwidth5_totaltime7200_simtime90_searchtype_kinematic_scoretype_random/",
               "anafi_learned_run_flown/learned_anafi_sim_ANAFI_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_learned/"]

# All the different system types which are generated using the WorldEngineSimulator
system_types = ["anafi_sim",
                "anafi_outdoor"]

# System type names
ticks = ["Simulation", "Outdoor"]


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
                        print("Maximum Deviation over 30m (" + str(max_dev[0][0]) + "m): " + str(file_name))
                        failed_tests += 1
                        continue

                    elif max_dev[0][0] > 15 and system == "speed-1_minsnap1":
                        print("Maximum Deviation over 15m (" + str(max_dev[0][0]) + "m): " + str(file_name))
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


# Outdoor!

randomscore_results = []
learnedscore_results = []

for sys in system_types:
    for item in final_data:
        if "scoretype_random/" in item['test_set'] and sys == item['system_type']:
            randomscore_results.append(item['max_deviation'])

        if "scoretype_learned/" in item['test_set'] and sys == item['system_type']:
            learnedscore_results.append(item['max_deviation'])

# Get the ratio of each
for i in range(0,2):
    mean = np.mean(randomscore_results[i])
    print("System: " + str(system_types[i]))
    print("Mean (random score): " + str(mean))
    print("Max (random score): " + str(np.max(randomscore_results[i])))
    print("Mean (learned score): " + str(np.mean(learnedscore_results[i])))
    print("Max (learned score): " + str(np.max(learnedscore_results[i])))
    print("")

    randomscore_results[i] = randomscore_results[i] / mean
    learnedscore_results[i] = learnedscore_results[i] / mean




fig1, ax1 = plt.subplots(1, 1, figsize=(10, 7))
plt.axvline(x=3, color='black', linestyle='-', linewidth=0.5)

bp1_pos = positions=3*np.arange(len(randomscore_results))+1
bp1 = plt.boxplot(randomscore_results, positions=bp1_pos, showmeans=True)
# annotate_boxplot(bp1, positions=bp1_pos, text="No Scoring")

bp2_pos = positions=3*np.arange(len(learnedscore_results))+2
bp2 = plt.boxplot(learnedscore_results, positions=bp2_pos, showmeans=True)
# annotate_boxplot(bp2, positions=bp2_pos, text="Learned Scoring")

# Add colors
set_box_color(bp1, 'C0')
set_box_color(bp2, 'C3')

# Set the right scale
plt.xlim([0, 3*len(randomscore_results)])

# Draw the central line
plt.axhline(y=1, color='gray', linestyle='--')

# Change the Y ticks font size
plt.yticks(fontsize=15)

# Plot the minor and major grid for the Y axis
plt.minorticks_on()
plt.grid(b=True, which='major', axis='y', linestyle='-', linewidth=0.5)
plt.grid(b=True, which='minor', axis='y', linestyle='--', linewidth=0.5)

# Add the ticks for X
ticks = ["No Scoring", "Learned Scoring"]
final_ticks = []
final_ticks += ticks
final_ticks += [""]
final_ticks += ticks
final_ticks += [""]
plt.xticks(np.arange(len(final_ticks)) + 1, final_ticks, fontsize=10, rotation=15, ha="right", rotation_mode="anchor")
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

topticks = ["Simulation", "Real-World"]
new_tick_locations = np.array([1.5, 4.5])
ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
ax2.set_xticks(new_tick_locations)
ax2.set_xticklabels(topticks, fontsize=15)
ax2.tick_params(axis='x', length=0, which='major')
for a in ax2.xaxis.get_majorticklabels():
    a.set_y(a.get_position()[1]-.01)

# Turn off the bottom minor labels
ax1.tick_params(axis='x', which='minor', bottom=False)

# Display graph
fig1.tight_layout()
plt.show()
