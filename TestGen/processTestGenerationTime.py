import numpy as np
import matplotlib.pyplot as plt
from processResultsUtils import get_numbers_from_string

all_files = ["./Results/PolyRunFull/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_kinematic_waypoint.txt",
             "./Results/PolyRunFull/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_waypoint.txt",
             "./Results/PolyRunFull/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime21600_score_constant.txt"]

all_files = ["./Results/PolySameTime/mit_details_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simulationtime90_kinematic_waypoint.txt",
             "./Results/PolySameTime/mit_details_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simulationtime90_score_waypoint.txt",
             "./Results/PolySameTime/mit_details_seed10_depth10_nodes250_res4_beamwidth10_totaltime28800_simulationtime90_score_constant.txt"]

labels = ["Random Search with Kinematic",
          "Waypoint Score with Kinematic",
          "Constant Score with Kinematic"]

fig = plt.figure()
for i in range(0, len(all_files)):

    file_name = all_files[i]

    with open(file_name, "r") as f:
        lines = f.readlines()

        base_time = 0.0

        time_found = []

        for line in lines:

            if "Search Completed at Time: " in line:
                found_time = np.array(get_numbers_from_string(line)).item()
                time_found.append(found_time)

            if "Time for that run: " in line:
                base_time += np.array(get_numbers_from_string(line)).item()

        plt.plot(time_found, np.arange(0, len(time_found)), label=labels[i])

plt.xlabel('Time')
plt.ylabel('Total Tests')
plt.legend()
plt.show()