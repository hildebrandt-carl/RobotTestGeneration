import numpy as np
import matplotlib.pyplot as plt
import glob
from processResultsUtils import get_numbers_after_string
from processResultsUtils import lineseg_dist


all_folders = ["./Results/FullRun-08-08-19/nodes1000_res2/",
               "./Results/FullRun-08-09-19/nodes1000_res2/",
               "./Results/FullRun-08-09-19/nodes1000_res4/",
               "./Results/FullRun-08-11-19/nodes1000_res2/",
               "./Results/FullRun-08-11-19/nodes1000_res4/",
               "./Results/FullRun-08-15-19/nodes1000_res2/",
               "./Results/FullRun-08-15-19/nodes1000_res4/"]

beam_lengths = [[1, 2, 3, 4, 5, 10],
                [1, 2, 3, 4, 5, 10, 25, 50, 100],
                [1, 3, 5, 25],
                [1, 2, 3, 4, 5, 10, 25, 50, 100],
                [1, 2, 3, 4, 5, 10],
                [1, 2, 3, 4, 5, 10, 25, 50, 100],
                [1, 2, 3, 4, 5, 10, 25, 50, 100]]

save_names = ["OutputV*Angle_Res2_Depth3-10",
              "DeltaV*Angle_Res2_Depth10",
              "DeltaV*Angle_Res4_Depth10",
              "SubVector_Res2_Depth10",
              "SubVector_Res4_Depth10",
              "InputV*Angle_Res2_Depth10",
              "InputV*Angle_Res4_Depth10"]

tick_names = ["OutV*Ang_R2D3-10",
              "DV*Ang_R2D10",
              "DV*Ang_Res4D10",
              "SubVec_Res2D10",
              "SubVec_Res4D10",
              "InV*Ang_Res2D10",
              "InV*Ang_Res4D10"]

res_numbers = [2,
               2,
               4,
               2,
               4,
               2,
               4]


# Create the figures and add labels to axis
f_average_deviation = plt.figure(1)
plt.xlabel("Test Score")
plt.ylabel("Average Spacial Deviation")
f_total_deviation = plt.figure(2)
plt.xlabel("Test Score")
plt.ylabel("Total Spacial Deviation")
f_average_time = plt.figure(3)
plt.xlabel("Test Score")
plt.ylabel("Average Temporal Deviation")
f_total_time = plt.figure(4)
plt.xlabel("Test Score")
plt.ylabel("Total Temporal Deviation")
f_dist_heuristic = plt.figure(5)
plt.xlabel("Test Score")
plt.ylabel("Distance Heuristic")
f_time_heuristic = plt.figure(6)
plt.xlabel("Test Score")
plt.ylabel("Time Heuristic")
f_dist_heuristic_box = plt.figure(7)
plt.xlabel("Test Technique")
plt.ylabel("Distance Heuristic")
f_time_heuristic_box = plt.figure(8)
plt.xlabel("Test Technique")
plt.ylabel("Time Heuristic")

distance_heuristic_box_data = []
time_heuristic_box_data = []

for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    scores = []
    average_deviation = []
    total_deviation = []
    average_time = []
    total_time = []
    distance_heuristic = []
    time_heuristic = []

    for beam in beams:
        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:

            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:
                # Get the average and total deviation for that test
                avg_dev = get_numbers_after_string(file_name=file_name, the_string="Average deviation from optimal trajectory:")
                tot_dev = get_numbers_after_string(file_name=file_name, the_string="Total deviation from optimal trajectory:")

                # Get the score for that test
                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                # Get the average and total time for that test
                avg_time = get_numbers_after_string(file_name=file_name, the_string="Average time between waypoints:")
                tot_time = get_numbers_after_string(file_name=file_name, the_string="Total time between waypoints:")

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

                if optimal_distance_heuristic > 2.2:
                    print("Optimal distance heuristic over 2.2 (" + str(optimal_distance_heuristic) + "): " + str(file_name))

                # Save the data
                average_deviation.append(avg_dev[0][0])
                total_deviation.append(tot_dev[0][0])
                scores.append(scr[0][0])
                average_time.append(avg_time[0][0])
                total_time.append(tot_time[0][0])
                distance_heuristic.append(optimal_distance_heuristic)
                time_heuristic.append(optimal_time_heuristic)

    # Save the time heuristic and distance heuristic
    distance_heuristic_box_data.append(distance_heuristic)
    time_heuristic_box_data.append(time_heuristic)

    # Plot the data
    f_average_deviation = plt.figure(1)
    plt.scatter(scores, average_deviation, s=6, label=save_names[i])
    f_total_deviation = plt.figure(2)
    plt.scatter(scores, total_deviation, s=6, label=save_names[i])
    f_average_time = plt.figure(3)
    plt.scatter(scores, average_time, s=6, label=save_names[i])
    f_total_time = plt.figure(4)
    plt.scatter(scores, total_time, s=6, label=save_names[i])
    f_dist_heuristic = plt.figure(5)
    plt.scatter(scores, distance_heuristic, s=6, label=save_names[i])
    f_time_heuristic = plt.figure(6)
    plt.scatter(scores, time_heuristic, s=6, label=save_names[i])

# Update the figure legends and show them
f_average_deviation = plt.figure(1)
plt.legend(prop={'size': 6})
f_total_deviation = plt.figure(2)
plt.legend(prop={'size': 6})
f_average_time = plt.figure(3)
plt.legend(prop={'size': 6})
f_total_time = plt.figure(4)
plt.legend(prop={'size': 6})
f_dist_heuristic = plt.figure(5)
plt.legend(prop={'size': 6})
f_time_heuristic = plt.figure(6)
plt.legend(prop={'size': 6})

f_dist_heuristic_box = plt.figure(7)
plt.boxplot(distance_heuristic_box_data)
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=7, rotation=45)
f_time_heuristic_box = plt.figure(8)
plt.boxplot(time_heuristic_box_data)
plt.xticks(range(1, len(tick_names) + 1), tick_names, fontsize=7, rotation=45)

# Show the figures
plt.show()