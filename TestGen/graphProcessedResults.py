import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string


all_folders = ["./Results/SecondCompleteRun/MIT_seed10_depth10_nodes3200_res4_beamwidth10_searchtime36000_kinematic_angle180/",
               "./Results/SecondCompleteRun/MIT_seed10_depth10_nodes3200_res4_beamwidth10_searchtime36000_score_angle180/",
               "./Results/SecondCompleteRun/MIT_seed10_depth10_nodes3200_res4_beamwidth10_searchtime36000_score_angle135/",
               "./Results/SecondCompleteRun/MIT_seed10_depth10_nodes3200_res4_beamwidth10_searchtime36000_score_angle90/"]


beam_lengths = [10]
depths = [10]
res_numbers = [4]

save_names = ["Random + Kinematic",
              "Score + Kinematic 180",
              "Score + Kinematic 135",
              "Score + Kinematic 90"]

tick_names = ["Random + Kinematic",
              "Score + Kinematic 180",
              "Score + Kinematic 135",
              "Score + Kinematic 90"]


systems = ["waypoint",
           "constant"]

system_constant_time = []
system_constant_distance = []
system_waypoint_time = []
system_waypoint_distance = []
system_constant_maximum_deviation = []
system_waypoint_maximum_deviation = []

for system in systems:

    # # Create the figures and add labels to axis
    # f_average_deviation = plt.figure(1)
    # plt.xlabel("Test Score")
    # plt.ylabel("Average Spacial Deviation")
    # plt.title("System: " + system)
    # f_total_deviation = plt.figure(2)
    # plt.xlabel("Test Score")
    # plt.ylabel("Total Spacial Deviation")
    # plt.title("System: " + system)
    # f_average_time = plt.figure(3)
    # plt.xlabel("Test Score")
    # plt.ylabel("Average Temporal Deviation")
    # plt.title("System: " + system)
    # f_total_time = plt.figure(4)
    # plt.xlabel("Test Score")
    # plt.ylabel("Total Temporal Deviation")
    # plt.title("System: " + system)
    # f_dist_heuristic = plt.figure(5)
    # plt.xlabel("Test Score")
    # plt.ylabel("Distance Heuristic")
    # plt.title("System: " + system)
    # f_time_heuristic = plt.figure(6)
    # plt.xlabel("Test Score")
    # plt.ylabel("Time Heuristic")
    # plt.title("System: " + system)
    # f_dist_heuristic_box = plt.figure(7)
    # plt.xlabel("Test Technique")
    # plt.ylabel("Distance Heuristic")
    # plt.title("System: " + system)
    # f_time_heuristic_box = plt.figure(8)
    # plt.xlabel("Test Technique")
    # plt.ylabel("Time Heuristic")
    # plt.title("System: " + system)

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

        for depth in depths:
            for beam in beam_lengths:

                file_location = folder
                analysis_file_names = glob.glob(file_location + "maps/map*/analysis_" + system + ".txt")

                for file_name in analysis_file_names:
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

                    # Check for any anomalies
                    if tot_dev[0][0] > 450:
                        print("Total Deviation over 450m (" + str(tot_dev[0][0]) + "m): " + str(file_name))

                    if avg_time[0][0] > 20:
                        print("Average time over 20 seconds (" + str(avg_time[0][0]) + "s): " + str(file_name))

                    if optimal_distance_heuristic > 2.5:
                        print("Optimal distance heuristic over 2.2 (" + str(optimal_distance_heuristic) + "): " + str(file_name))

                    # Save the data
                    average_deviation.append(avg_dev[0][0])
                    total_deviation.append(tot_dev[0][0])
                    scores.append(scr[0][0])
                    average_time.append(avg_time[0][0])
                    total_time.append(tot_time[0][0])
                    distance_heuristic.append(optimal_distance_heuristic)
                    time_heuristic.append(optimal_time_heuristic)
                    maximum_deviation.append(max_dev[0][0])

        # Save the time heuristic and distance heuristic
        distance_heuristic_box_data.append(distance_heuristic)
        time_heuristic_box_data.append(time_heuristic)

        # # Plot the data
        # f_average_deviation = plt.figure(1)
        # plt.scatter(scores, average_deviation, s=6, label=save_names[i])
        # plt.title("System: " + system)
        # f_total_deviation = plt.figure(2)
        # plt.scatter(scores, total_deviation, s=6, label=save_names[i])
        # plt.title("System: " + system)
        # f_average_time = plt.figure(3)
        # plt.scatter(scores, average_time, s=6, label=save_names[i])
        # plt.title("System: " + system)
        # f_total_time = plt.figure(4)
        # plt.scatter(scores, total_time, s=6, label=save_names[i])
        # plt.title("System: " + system)
        # f_dist_heuristic = plt.figure(5)
        # plt.scatter(scores, distance_heuristic, s=6, label=save_names[i])
        # plt.title("System: " + system)
        # f_time_heuristic = plt.figure(6)
        # plt.scatter(scores, time_heuristic, s=6, label=save_names[i])

        # Save the data into each respective system
        if system == "constant":
            system_constant_distance.append(total_deviation)
            system_constant_time.append(total_time)
            system_constant_maximum_deviation.append(maximum_deviation)
        elif system == "waypoint":
            system_waypoint_distance.append(total_deviation)
            system_waypoint_time.append(total_time)
            system_waypoint_maximum_deviation.append(maximum_deviation)

    # # Update the figure legends and show them
    # f_average_deviation = plt.figure(1)
    # plt.legend(prop={'size': 6})
    # plt.title("System: " + system)
    # f_total_deviation = plt.figure(2)
    # plt.legend(prop={'size': 6})
    # plt.title("System: " + system)
    # f_average_time = plt.figure(3)
    # plt.legend(prop={'size': 6})
    # plt.title("System: " + system)
    # f_total_time = plt.figure(4)
    # plt.legend(prop={'size': 6})
    # plt.title("System: " + system)
    # f_dist_heuristic = plt.figure(5)
    # plt.legend(prop={'size': 6})
    # f_time_heuristic = plt.figure(6)
    # plt.title("System: " + system)
    # plt.legend(prop={'size': 6})
    # plt.title("System: " + system)

    # f_dist_heuristic_box = plt.figure(7)
    # plt.boxplot(distance_heuristic_box_data)
    # plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=7, rotation=0)
    # plt.title("System: " + system)
    # f_time_heuristic_box = plt.figure(8)
    # plt.boxplot(time_heuristic_box_data)
    # plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=7, rotation=0)
    # plt.title("System: " + system)

    # # Show the figures
    # plt.show()


# Display the compared box plots
def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=2)
    # plt.setp(bp['whiskers'], color=color)
    # plt.setp(bp['caps'], color=color)
    # plt.setp(bp['medians'], color=color)

plt.figure(1)
bpl = plt.boxplot(system_constant_distance, positions=np.array(range(len(system_constant_distance)))*2.0-0.4, sym='', widths=0.6)
bpr = plt.boxplot(system_waypoint_distance, positions=np.array(range(len(system_waypoint_distance)))*2.0+0.4, sym='', widths=0.6)
set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpr, '#2C7BB6')

# draw temporary red and blue lines and use them to create a legend
plt.plot([], c='#D7191C', label='Constant Velocity')
plt.plot([], c='#2C7BB6', label='Waypoint Controller')
plt.legend()
plt.xticks(range(0, len(tick_names) * 2, 2), tick_names, fontsize=7, rotation=15)
plt.xlim(-2, len(tick_names)*2)
plt.tight_layout()
plt.ylabel("Total Deviation")
plt.xlabel("Test Generation Technique")


plt.figure(2)
bpl = plt.boxplot(system_constant_time, positions=np.array(range(len(system_constant_time)))*2.0-0.4, sym='', widths=0.6)
bpr = plt.boxplot(system_waypoint_time, positions=np.array(range(len(system_waypoint_time)))*2.0+0.4, sym='', widths=0.6)
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
plt.xlabel("Test Generation Technique")


# plt.figure(3)
# for system in systems:
#     if system == "constant":
#         data = system_constant_maximum_deviation
#     else:
#         data = system_waypoint_maximum_deviation


#     for i in range(0, len(data)):

#         # Create the x and y axis
#         obstacle_distance = np.linspace(0, 10, 10)
#         number_failures = []

#         for j in obstacle_distance:
#             number_failures.append(sum(data[i] > j))

#         plt.plot(obstacle_distance, number_failures, label=str(system) + ": " + str(tick_names[i]))
        
#     plt.xlabel("Minimum Acceptable Deviation")
#     plt.ylabel("Number of Failing Tests")
#     plt.legend()

plt.show()