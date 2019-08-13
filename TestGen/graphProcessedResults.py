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
               "./Results/FullRun-08-13-19/nodes1000_res2/",
               "./Results/FullRun-08-13-19/nodes1000_res4/"]

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

res_numbers = [2,
               2,
               4,
               2,
               4,
               2,
               4]

fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    testset_gen_times = []
    testset_total_tests = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:

            file_location = folder
            file_name = "details_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + ".txt"

            # /Results/FullRun-08-08-19/nodes1000_res2/details_seed10_depth8_nodes1000_res2_beamwidth4.txt

            # Get the time to generate that test set
            time = get_numbers_after_string(file_name=file_location+file_name, the_string="Process finished successfully with time:")
            total = get_numbers_after_string(file_name=file_location+file_name, the_string="DATA: Total unique paths found:")

            assert(len(time) == 1)
            assert (len(total) == 1)

            testset_gen_times.append(time[0])
            testset_total_tests.append(total[0])

    plt.scatter(testset_total_tests, testset_gen_times, s=6, label=save_names[i])

plt.xlabel("Number Tests")
plt.ylabel("Test Suite Generation Time (s)")
plt.title("Test Generation Scalability")
plt.legend()
plt.show()


fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    deviations = []
    scores = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:

            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set
                dev = get_numbers_after_string(file_name=file_name, the_string="Average deviation from optimal trajectory:")
                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                deviations.append(dev[0][0])
                if i == 5 or i == 6:
                    scores.append(scr[0][0] / 10)
                else:
                    scores.append(scr[0][0])

    plt.scatter(scores, deviations, s=6, label=save_names[i])


plt.xlabel("Test Score")
plt.ylabel("Average Spacial Deviation")
plt.title("Score Compared to Spacial Deviation")
# plt.legend()
plt.show()



fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    deviations = []
    scores = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:
            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set
                dev = get_numbers_after_string(file_name=file_name, the_string="Total deviation from optimal trajectory:")
                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                if dev[0][0] > 450:
                    print("Total Deviation over 450m (" + str(dev[0][0]) + "m): " + str(file_name))

                deviations.append(dev[0][0])
                if i == 5 or i == 6:
                    scores.append(scr[0][0] / 10)
                else:
                    scores.append(scr[0][0])

    plt.scatter(scores, deviations, s=6, label=save_names[i])


plt.xlabel("Test Score")
plt.ylabel("Total Spacial Deviation")
plt.title("Score Compared to Spacial Deviation")
# plt.legend()
plt.show()






fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    number_waypoints = []
    scores = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:

            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set

                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")
                way = get_numbers_after_string(file_name=file_name, the_string="Total waypoints:")

                number_waypoints.append(way[0][0])
                if i == 5 or i == 6:
                    scores.append(scr[0][0] / 10)
                else:
                    scores.append(scr[0][0])

    plt.scatter(scores, number_waypoints, s=6, label=save_names[i])

plt.xlabel("Test Score")
plt.ylabel("Total Waypoints")
plt.title("Score Compared to Number Waypoints")
plt.legend()
plt.show()









fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    number_waypoints = []
    total_deviation = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:

            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set

                dev = get_numbers_after_string(file_name=file_name, the_string="Total deviation from optimal trajectory:")
                way = get_numbers_after_string(file_name=file_name, the_string="Total waypoints:")

                number_waypoints.append(way[0][0])
                total_deviation.append(dev[0][0])

    plt.scatter(total_deviation, number_waypoints, s=6, label=save_names[i])

plt.xlabel("Total Spacial Deviation")
plt.ylabel("Total Waypoints")
plt.title("Spacial Deviation Compared to Number Waypoints")
plt.legend()
plt.show()







fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    avg_time = []
    scores = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:
            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set
                av = get_numbers_after_string(file_name=file_name, the_string="Average time between waypoints:")
                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                if av[0][0] > 20:
                    print("Average time over 20 seconds: " + str(file_name))

                avg_time.append(av[0][0])
                if i == 5 or i == 6:
                    scores.append(scr[0][0] / 10)
                else:
                    scores.append(scr[0][0])

    plt.scatter(scores, avg_time, s=6, label=save_names[i])


plt.xlabel("Test Score")
plt.ylabel("Average Time Between Waypoints")
plt.title("Score Compared to Average Time Between Waypoints")
# plt.legend()
plt.show()






fig = plt.figure()
for i in range(0, len(all_folders)):
    folder = all_folders[i]
    beams = beam_lengths[i]
    res = res_numbers[i]

    total_time = []
    scores = []

    for beam in beams:

        if i == 0:
            depths = [3, 4, 5, 6, 7, 8, 9, 10]
        else:
            depths = [10]

        for depth in depths:
            file_location = folder + "MIT_seed10_depth" + str(depth) + "_nodes1000_res" + str(res) + "_beamwidth" + str(beam) + "_baseline0/"
            analysis_file_names = glob.glob(file_location + "maps/map*/analysis.txt")

            for file_name in analysis_file_names:

                # Get the time to generate that test set
                tt = get_numbers_after_string(file_name=file_name, the_string="Total time between waypoints:")
                scr = get_numbers_after_string(file_name=file_name, the_string="Path Score:")

                total_time.append(tt[0][0])
                if i == 5 or i == 6:
                    scores.append(scr[0][0] / 10)
                else:
                    scores.append(scr[0][0])

    plt.scatter(scores, total_time, s=6, label=save_names[i])


plt.xlabel("Test Score")
plt.ylabel("Total Time")
plt.title("Score Compared to Total Time")
# plt.legend()
plt.show()