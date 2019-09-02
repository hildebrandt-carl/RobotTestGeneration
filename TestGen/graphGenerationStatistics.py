import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string
import math

all_files = ["./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_random_angle180.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_maxvel_angle180.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_kinematic_angle180.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle30.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle60.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle90.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle120.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle150.txt",
             "./Results/AngleTest250/mit_details_seed10_depth10_nodes250_res4_beamwidth10_searchtime1200_score_angle180.txt"]

save_names = ["R", "M", "K", "30", "60", "90", "120", "150", "180"]

tick_names = ["R", "M", "K", "30", "60", "90", "120", "150", "180"]

# tick_names = ["Random Search",
#               "Random + Max Velocity",
#               "Random + Kinematic",
#               "Score + Kinematic 180",
#               "Score + Kinematic 90"]

# Used to save the results for each of the files
all_total_paths = []
all_valid_paths = []
all_processed_paths = []

bottom_cut = 500
top_cut = 150
largest_point = 2000
top_increments = 500


d = .015

for f in all_files:
    # Get the total number of paths found
    total_paths = get_numbers_after_string(file_name=f, the_string="Total paths found:")

    # Get the total number of valid paths
    valid_paths = get_numbers_after_string(file_name=f, the_string="Valid paths found:")

    # Get the total number of paths considered
    processed_paths = get_numbers_after_string(file_name=f, the_string="Total paths processed:")

    if total_paths == -math.inf:
        all_total_paths.append(-1)
        all_valid_paths.append(-1)
        all_processed_paths.append(-1)
    else:
        #processed_paths = max(processed_paths)
        if len(total_paths) == 0:
            all_total_paths.append(0)
        else:
            all_total_paths.append(total_paths[0][0])
        if len(valid_paths) == 0:
            all_valid_paths.append(0)
        else:
            all_valid_paths.append(valid_paths[0][0])

# Create the indices
ind = np.arange(0, len(all_valid_paths), 1)

# Convert the arrays to numpy arrays
all_total_paths = np.array(all_total_paths)
all_valid_paths = np.array(all_valid_paths)
#all_processed_paths = np.array(all_processed_paths)

# Plot the data
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(6,6))

ax1.spines['bottom'].set_visible(False)
ax1.tick_params(axis='x', which='both', bottom=False)
ax2.spines['top'].set_visible(False)

ax2.set_ylim(0, top_cut)
ax1.set_ylim(bottom_cut, largest_point)

ax1.set_yticks(np.arange(bottom_cut, largest_point+1, top_increments))

bars1 = ax1.bar(ind, all_total_paths - all_valid_paths, color='C1', width=0.6, label='Completed Paths', bottom=all_valid_paths)
bars2 = ax1.bar(ind, all_valid_paths, color='C0', width=0.6, label='Valid Paths')

bars3 = ax2.bar(ind, all_total_paths - all_valid_paths, color='C1', width=0.6, label='Completed Paths', bottom=all_valid_paths)
bars4 = ax2.bar(ind, all_valid_paths, color='C0', width=0.6, label='Valid Paths')

for tick in ax2.get_xticklabels():
    tick.set_rotation(0)


kwargs = dict(transform=ax1.transAxes, color='k', clip_on=False)
ax1.plot((-d, +d), (-d, +d), **kwargs)
ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)
kwargs.update(transform=ax2.transAxes)
ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)
ax2.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)

for tick in ax2.get_xticklabels():
    tick.set_rotation(0)

d = .015
kwargs = dict(transform=ax1.transAxes, color='k', clip_on=False)
ax1.plot((-d, +d), (-d, +d), **kwargs)
ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)
kwargs.update(transform=ax2.transAxes)
ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)
ax2.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)

plt.xticks(ind, tick_names, fontsize=7, rotation=15)

# plt.ylabel("Number Found")
# ax2.ylabel.set_label_coords(1.05, -0.025)
# plt.xlabel("Techniques")

ax2.set_xlabel('Number Found')
ax2.set_ylabel('Techniques')
ax2.yaxis.set_label_coords(0.05, 0.5, transform=fig.transFigure)

for b1, b2 in zip(bars1, bars3):
    posx = b2.get_x() + b2.get_width()/2.
    if b2.get_height() > top_cut:
        ax2.plot((posx-5*d, posx+5*d), (1 - d, 1 + d), color='k', clip_on=False,
                 transform=ax2.get_xaxis_transform())
    if b1.get_height() > bottom_cut:
        ax1.plot((posx-5*d, posx+5*d), (- d, + d), color='k', clip_on=False,
                 transform=ax1.get_xaxis_transform())

for b1, b2 in zip(bars2, bars4):
    posx = b2.get_x() + b2.get_width()/2.
    if b2.get_height() > top_cut:
        ax2.plot((posx-5*d, posx+5*d), (1 - d, 1 + d), color='k', clip_on=False,
                 transform=ax2.get_xaxis_transform())
    if b1.get_height() > bottom_cut:
        ax1.plot((posx-5*d, posx+5*d), (- d, + d), color='k', clip_on=False,
                 transform=ax1.get_xaxis_transform())


# va = [0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, -0.1, 0, 0]
# for t, y in zip(ax2.get_xticklabels(), va):
#     t.set_y(y)

ax1.legend()

plt.tight_layout()
plt.show()