import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string

all_files = ["./Results/FullNewRun/mit_details_seed10_depth10_nodes1000_res4_beamwidth10_searchtime600_random.txt",
             "./Results/FullNewRun/mit_details_seed10_depth10_nodes1000_res4_beamwidth10_searchtime600_maxvel.txt",
             "./Results/FullNewRun/mit_details_seed10_depth10_nodes1000_res4_beamwidth10_searchtime600_kinematic.txt",
             "./Results/FullNewRun/mit_details_seed10_depth10_nodes1000_res4_beamwidth10_searchtime600_score.txt"]

save_names = ["Random Search",
              "Random + Max Velocity",
              "Random + Kinematic",
              "Score + Kinematic"]

tick_names = ["Random Search",
              "Random + Max Velocity",
              "Random + Kinematic",
              "Score + Kinematic"]

# Used to save the results for each of the files
all_total_paths = []
all_valid_paths = []
all_processed_paths = []

for f in all_files:
    # Get the total number of paths found
    total_paths = get_numbers_after_string(file_name=f, the_string="Total paths found:")

    # Get the total number of valid paths
    valid_paths = get_numbers_after_string(file_name=f, the_string="Valid paths found:")

    # Get the total number of paths considered
    processed_paths = get_numbers_after_string(file_name=f, the_string="Total paths processed:")
    processed_paths = max(processed_paths)

    all_total_paths.append(total_paths[0][0])
    all_valid_paths.append(valid_paths[0][0])
    all_processed_paths.append(processed_paths[0])

# Create the indices
ind = np.arange(0, len(all_valid_paths), 1)

# Convert the arrays to numpy arrays
all_total_paths = np.array(all_total_paths)
all_valid_paths = np.array(all_valid_paths)
all_processed_paths = np.array(all_processed_paths)

# Plot the data
fig = plt.figure()
scaled_all_processed_paths = all_processed_paths / 100000
plt.bar(ind, scaled_all_processed_paths - (all_total_paths - all_valid_paths), color='C3', width=0.02, label='Dropped Paths', bottom=all_total_paths - all_valid_paths)
plt.bar(ind, all_total_paths - all_valid_paths, color='C1', width=0.6, label='Found Paths', bottom=all_valid_paths)
plt.bar(ind, all_valid_paths, color='C0', width=0.6, label='Valid Paths')
plt.xticks(ind, tick_names, fontsize=7, rotation=0)
plt.ylabel("Number Found")
plt.xlabel("Techniques")
plt.legend(loc="upper right")

# zip joins x and y coordinates in pairs
for x_pos, y_pos, lab in zip(ind, scaled_all_processed_paths, all_processed_paths):
    label = "{:.2f} million".format(lab / 100000.0)
    plt.annotate(label, # this is the text
                 (x_pos, y_pos),# this is the point to label
                 textcoords="offset points",# how to position the text
                 xytext=(0, 10),# distance from text to points (x,y)
                 ha='center') # horizontal alignment can be left, right or center

plt.ylim([0, 150])
plt.show()

