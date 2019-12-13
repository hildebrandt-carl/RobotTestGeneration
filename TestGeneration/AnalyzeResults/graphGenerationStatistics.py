import glob
import matplotlib.pyplot as plt
import numpy as np
from processResultsUtils import get_numbers_after_string
import math

folder = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/initial_run_generation/"

# # Anafi Length 10
# all_files = ["initial_searchtype_random_scoretype_random_dronetype_anafi_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
#             "initial_searchtype_maxvel_scoretype_random_dronetype_anafi_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
#             "initial_searchtype_kinematic_scoretype_random_dronetype_anafi_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt"]

# # Anafi Length 5
# all_files = ["initial_searchtype_random_scoretype_random_dronetype_anafi_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
#             "initial_searchtype_maxvel_scoretype_random_dronetype_anafi_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
#             "initial_searchtype_kinematic_scoretype_random_dronetype_anafi_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt"]

# # MIT Length 10
# all_files = ["initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
#             "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
#             "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt"]

# MIT Length 5
all_files = ["initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt"]

tick_names = ["Random", "Approx Kinematic", "Full Kinematic"]

# Used to save the results for each of the files
explored_trajectories = []
complete_trajectories = []
valid_trajectories = []

for f in all_files:

    # Get the fullname
    fullname = folder + f

    # Get the number of trajectories for this file
    et = get_numbers_after_string(file_name=fullname, the_string="DATA: Total explored trajectories:")
    ct = get_numbers_after_string(file_name=fullname, the_string="DATA: Total complete trajectories:")
    vt = get_numbers_after_string(file_name=fullname, the_string="DATA: Total valid trajectories:")

    # Get the last value
    et = et[-1][0]
    ct = ct[-1][0]
    vt = vt[-1][0]

    # Save the number
    explored_trajectories.append(et)
    complete_trajectories.append(ct)
    valid_trajectories.append(vt)


fig1, ax1 = plt.subplots(1, 1, figsize=(10, 9))

ax1.grid(zorder=-100)
ax1.grid(which='minor', linestyle='--', linewidth=0.5)

barWidth = 0.25
r1 = np.arange(len(explored_trajectories))
r2 = [x + barWidth for x in r1]
r3 = [x + barWidth for x in r2]
plt.bar(r1, explored_trajectories, color='C1', width=barWidth, edgecolor='white', label="Processed Trajectories", hatch='x', zorder=100)
plt.bar(r2, complete_trajectories, color='C0', width=barWidth, edgecolor='white', label="Complete Trajectories", hatch='//', zorder=100)
plt.bar(r3, valid_trajectories, color='C2', width=barWidth, edgecolor='white', label="Valid Trajectories", zorder=100)

plt.xlabel("Technique", fontweight='bold', fontsize=20)
plt.ylabel("Number Trajectories", fontweight='bold', fontsize=20)

plt.xticks(np.arange(len(tick_names)) + barWidth, tick_names, fontsize=20, rotation=0)
plt.yticks(fontsize=15)

plt.yscale('log')

plt.legend(fontsize=20)

plt.show()
















folder = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/initial_run_generation/"


all_files = [["initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_3_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_27.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_4_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_36.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_6_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_54.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_7_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_63.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_8_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_72.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_9_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_81.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_11_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_99.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_12_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_108.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_13_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_117.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_14_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_126.txt",
            "initial_searchtype_random_scoretype_random_dronetype_mit_trajectorylength_15_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_135.txt"],
            ["initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_3_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_27.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_4_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_36.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_6_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_54.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_7_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_63.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_8_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_72.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_9_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_81.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_11_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_99.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_12_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_108.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_13_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_117.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_14_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_126.txt",
            "initial_searchtype_maxvel_scoretype_random_dronetype_mit_trajectorylength_15_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_135.txt"],
            ["initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_3_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_27.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_4_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_36.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_5_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_45.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_6_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_54.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_7_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_63.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_8_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_72.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_9_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_81.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_10_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_90.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_11_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_99.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_12_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_108.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_13_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_117.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_14_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_126.txt",
            "initial_searchtype_kinematic_scoretype_random_dronetype_mit_trajectorylength_15_beamwidth_5_nodes_250_resolution_4_seed_10_totaltime_3600_simulationtime_135.txt"]]

# This will hold the results for all systems
explored_trajectories = []
complete_trajectories = []
valid_trajectories = []

# For each of the systems in all_files
for system_type in all_files:

    # This will hold the data for that specific system
    system_explored_traj = []
    system_complete_traj =[]
    system_valid_traj = []

    # For each file in this system
    for f in system_type:

        # Get the fullname
        fullname = folder + f

        print("Processing: " + str(fullname))

        # Get the number of trajectories for this file
        et = get_numbers_after_string(file_name=fullname, the_string="DATA: Total explored trajectories:")
        ct = get_numbers_after_string(file_name=fullname, the_string="DATA: Total complete trajectories:")
        vt = get_numbers_after_string(file_name=fullname, the_string="DATA: Total valid trajectories:")

        # Get the last value
        et = et[-1][0]
        ct = ct[-1][0]
        vt = vt[-1][0]

        # Save the number
        system_explored_traj.append(et)
        system_complete_traj.append(ct)
        system_valid_traj.append(vt)


    # Save the systems data
    explored_trajectories.append(system_explored_traj)
    complete_trajectories.append(system_complete_traj)
    valid_trajectories.append(system_valid_traj)

# At this point each system random, approx kinematic and kinematic have there data in the correct lists
fig2, ax2 = plt.subplots(1, 1, figsize=(10, 6))

x = np.arange(3, len(valid_trajectories[0]) + 3)
xp = np.linspace(3, 15, 100)

# randp = np.poly1d(np.polyfit(x, valid_trajectories[0], 5))
# plt.plot(xp, randp(xp), linestyle="--", color='C0', linewidth=0.75)
plt.plot(x, valid_trajectories[0], linestyle="--", label="Random", marker="o", markersize=10)

# maxvp = np.poly1d(np.polyfit(x, valid_trajectories[1], 5))
# plt.plot(xp, maxvp(xp), linestyle="--", color='C1', linewidth=0.75)
plt.plot(x, valid_trajectories[1], linestyle="--", label="Max Velocity", marker="^", markersize=10)

# kinp = np.poly1d(np.polyfit(x, valid_trajectories[2], 3))
# plt.plot(xp, kinp(xp), linestyle="--", color='C2', linewidth=0.75)
plt.plot(x, valid_trajectories[2], linestyle="--", label="Kinematic", marker="P", markersize=10)

plt.xlabel("Trajectory Length", fontweight='bold', fontsize=20)
plt.ylabel("Valid Trajectories", fontweight='bold', fontsize=20)

plt.xticks(np.arange(3, len(valid_trajectories[0])+3, 1.0), fontsize=15)
plt.yticks(fontsize=15)
plt.legend(fontsize=20)

plt.grid(which='major', linestyle='-', linewidth=0.5)

plt.show()