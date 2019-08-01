import csv
import os
from PRM import PRM
import numpy as np
import matplotlib.pyplot as plt
from Trajectory import Trajectory
from TrajectoryManager import TrajectoryManager
from FigureManager import FigureManager
from EquivalenceChecker import EquivalenceChecker
from DroneKinematic import DroneKinematic
from RankingSystem import RankingSystem
from enum import Enum
import sys
import argparse
import time

class DroneType(Enum):
    BEBOP = 1
    HECTOR = 2
    MIT = 3

# Get the time of the program start
start_time = time.time()

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--drone',
                    default="bebop",
                    type=str,
                    help='Select drone type (bebop), (hector), (mit)')
parser.add_argument('-x', '--depth',
                    default=5,
                    type=int,
                    help='Total number of state changes allowed per trajectory')
parser.add_argument('-p', '--drop',
                    default=0,
                    type=float,
                    help='Percentage of nodes removed when considering next state')
parser.add_argument('-n', '--nodes',
                    default=50,
                    type=int,
                    help='Number of nodes considered')
parser.add_argument('-r', '--resolution',
                    default=2,
                    type=int,
                    help='Resolution of the sample space')
args = parser.parse_args()

drone = None
save_path = None

if args.drone == "bebop":
    drone = DroneType.BEBOP
    # Save locations
    save_path = "BEBOP_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_drop" + str(int(args.drop * 100)) + "/"
elif args.drone == "hector":
    drone = DroneType.HECTOR
    # Save locations
    save_path = "Results/HECTOR_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_drop" + str(int(args.drop * 100)) + "/"
elif args.drone == "mit":
    drone = DroneType.MIT
    # Save locations
    save_path = "Results/MIT_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_drop" + str(int(args.drop * 100)) + "/"

# Flags
plotting = True

# Test initial conditions
initial_conditions = {"map_x_bounds": [0, 30],
                      "map_y_bounds": [0, 30],
                      "map_z_bounds": [0, 15],
                      "start_point": [1, 1, 1],
                      "end_point": [29, 29, 14]}

# Specified by the tester
human_specified_factors = {"kinematic_sampling_resolution": args.resolution}

# Used to limit our search
traj_search_conditions = {"number_nodes": int(args.nodes),
                          "search_depth": int(args.depth),
                          "drop_rate": float(args.drop)}

# Robot kinematics
robot_kinematics = {}
if drone == DroneType.BEBOP:
    robot_kinematics["m"] = 0.5
    robot_kinematics["d"] = 0.16
    robot_kinematics["kf"] = 6.11e-8
    robot_kinematics["km"] = 1.5e-9
    robot_kinematics["max_rotor_speed"] = 8000
    robot_kinematics["inertial_properties"] = [2.32e-3, 2.32e-3, 4.00e-3]
    robot_kinematics["position"] = initial_conditions["start_point"]
    robot_kinematics["attitude"] = [0, 0, 0]
    robot_kinematics["velocity"] = [0, 0, 0]
    robot_kinematics["angular_velocity"] = [0, 0, 0]

elif drone == DroneType.HECTOR:
    robot_kinematics["m"] = 1.477
    robot_kinematics["d"] = 0.275
    robot_kinematics["kf"] = 4.142069415e-05
    robot_kinematics["km"] = -7.011631909766668e-5
    robot_kinematics["max_rotor_speed"] = 600
    robot_kinematics["inertial_properties"] = [0.01464, 0.01464, 0.02664]
    robot_kinematics["position"] = initial_conditions["start_point"]
    robot_kinematics["attitude"] = [0, 0, 0]
    robot_kinematics["velocity"] = [0, 0, 0]
    robot_kinematics["angular_velocity"] = [0, 0, 0]

elif drone == DroneType.MIT:
    robot_kinematics["m"] = 1.0
    robot_kinematics["d"] = 0.175
    robot_kinematics["kf"] = 1.91e-6
    robot_kinematics["km"] = 2.6e-7
    robot_kinematics["max_rotor_speed"] = 2200.0
    robot_kinematics["inertial_properties"] = [0.0049, 0.0049, 0.0049]
    robot_kinematics["position"] = initial_conditions["start_point"]
    robot_kinematics["attitude"] = [0, 0, 0]
    robot_kinematics["velocity"] = [0, 0, 0]
    robot_kinematics["angular_velocity"] = [0, 0, 0]

# Create the Figure manager
fig_manager = FigureManager(save_path)

# Make sure the directory we want to save our files in is created
fig_manager.create_directory()

# Set the figure managers figure bounds
FigureManager.x_range = {"lower": initial_conditions["map_x_bounds"][0],
                         "upper": initial_conditions["map_x_bounds"][1]}
FigureManager.y_range = {"lower": initial_conditions["map_y_bounds"][0],
                         "upper": initial_conditions["map_y_bounds"][1]}
FigureManager.z_range = {"lower": initial_conditions["map_z_bounds"][0],
                         "upper": initial_conditions["map_z_bounds"][1]}

# Generate the prm map
print("UPDATE: Populating Trajectory Graph")
# Create the PRM object
p = PRM(start_pos=initial_conditions["start_point"],
        end_pos=initial_conditions["end_point"],
        map_x_bounds=initial_conditions["map_x_bounds"],
        map_y_bounds=initial_conditions["map_y_bounds"],
        map_z_bounds=initial_conditions["map_z_bounds"])

# Find valid positions for waypoints
p.populate_with_nodes(num_vertices=traj_search_conditions["number_nodes"])

# # Find possible connections between waypoints based on velocity
# p.populate_with_edges(max_distance=robot_kinematics["max_velocity"],
#                       min_distance=robot_kinematics["min_velocity"])

if plotting:
    # Show the map after the prm construction phase
    print("UPDATE: Displaying Map")
    map_plt = fig_manager.plot_prm_graph(nodes=p.get_vertices(),
                                         edges=p.get_edges(),
                                         figure_size=(10, 10))

    # Display the figure
    fig_manager.display_and_save(fig=map_plt,
                                 save_name='original_map',
                                 only_save=True)

all_paths, rejected_lines = p.find_all_paths_dfs(drone_kinematic_values=robot_kinematics,
                                                 kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"],
                                                 total_waypoints=traj_search_conditions["search_depth"],
                                                 drop_rate=traj_search_conditions["drop_rate"])

# Display the selected paths
if plotting:
    # Show the map after the prm construction phase
    print("UPDATE: Displaying Selected Trajectories")
    map_plt = fig_manager.plot_selected_trajectories(nodes=p.get_vertices(),
                                                     selected_paths=all_paths,
                                                     figure_size=(10, 10))

    # Display the figure
    fig_manager.display_and_save(fig=map_plt,
                                 save_name='selected_trajectories',
                                 only_save=True)


# # Display the rejected paths
# if plotting:
#     # Show the map after the prm construction phase
#     print("UPDATE: Displaying Rejected Trajectories")
#     map_plt = fig_manager.plot_rejected_trajectories(nodes=p.get_vertices(),
#                                                      not_selected_paths=rejected_lines,
#                                                      figure_size=(10, 10))

#     # Display the figure
#     fig_manager.display_and_save(fig=map_plt,
#                                  save_name='rejected_trajectories',
#                                  only_save=False)


# Assert that we have found some paths
assert(len(all_paths) > 0)
print("DATA: Total unique paths found: " + str(len(all_paths)))

# Create a ranking system object
ranking_obj = RankingSystem(paths=all_paths)

# Score each of the paths
ranking_obj.calculate_scores()

# print("\nINFO: Displaying the score")
# print(ranking_obj.get_scores())

# Save the scores
ranking_obj.save_trajectories_according_to_score(folder=save_path)

# Print Completion
print("UPDATE: Completed")
print("DATA: Total time - " + str(time.time() - start_time))






























# print("UPDATE: Finding Possible Paths")
# # Find all paths between start and end position
# all_paths = p.findAllPaths(heading=initial_conditions["robot_heading"],
#                            min_turn_deg=robot_kinematics["min_turn"] - human_specified_factors["epsilon_angle"],
#                            max_turn_deg=robot_kinematics["max_turn"] + human_specified_factors["epsilon_angle"],
#                            depth=traj_search_conditions["search_depth"],
#                            max_traj=traj_search_conditions["max_trajectories"])
#
# # Assert that we have found some paths
# assert(len(all_paths) > 0)
# print("DATA: Total unique paths found: " + str(len(all_paths)))
#
# # Classify if the tests pass or fail
# print("UPDATE: Classifying each path into fail or pass test")
# for path in all_paths:
#     path.check_if_passing(max_turn=robot_kinematics["max_turn"],
#                           min_turn=robot_kinematics["min_turn"])
#
# # Count the number of tests which are passable
# print("UPDATE: Checking how many tests can pass and fail")
# passing_test_count = 0
# for path in all_paths:
#     if path.get_passing():
#         passing_test_count += 1
# print("DATA: Total paths which are passable: " + str(passing_test_count))
# print("DATA: Total paths which should fail: " + str(len(all_paths) - passing_test_count))
#
# # Create a scorer class and score the trajectories
# manager = TrajectoryManager(kinematics=robot_kinematics)
# scorer_success = manager.calculate_score(trajectories=all_paths)
#
# # Make sure all trajectories have scores
# assert(scorer_success == 1)
#
# # Sort the trajectories based on score
# all_paths = manager.sort_by_score(trajectories=all_paths)
#
# # if plotting:
# #     # Display the trajectories
# #     print("UPDATE: Displaying Trajectories Over Each Other")
# #     path_plot = fig_manager.plot_allpaths(trajectories=all_paths,
# #                                       tsuffix="Possible Trajectories",
# #                                       figure_size=(10, 10))
# #     # Display the figure
# #     fig_manager.display_and_save(fig=path_plot,
# #                                  save_name='all_paths')
#
# if plotting:
#     # Get the scores
#     scores = manager.get_all_scores(trajectories=all_paths)
#
#     # Create the plot
#     print("UPDATE: Displaying the scores")
#     score_plt = fig_manager.plot_scores(score_array=scores)
#
#     # Display the figure
#     fig_manager.display_and_save(fig=plt,
#                                  save_name='scores_graph')
#
# # TODO
# # Update this into your Figure manager
# if plotting:
#     # Create a new figure
#     plt.figure()
#
#     # Create the plot
#     plt.hist(x=scores,
#              bins=200,
#              range=(0, 1))
#     plt.title("Histogram of scores")
#     plt.xlabel("Score")
#     plt.ylabel("Total")
#
#     # Display the figure
#     fig_manager.display_and_save(fig=plt,
#                                  save_name='score_hist')
#
#
# # Group the trajectories into different classes
# grouper = EquivalenceChecker(epsilon=human_specified_factors["epsilon_class_equivalence"])
# total_classes = grouper.find_equivalence(all_paths)
#
# print("DATA: Total classes: " + str(total_classes))
#
# if plotting:
#     # Display the classes
#     classes = manager.get_all_classes(trajectories=all_paths)
#
#     # Plot the class histogram
#     print("UPDATE: Displaying the scores")
#     class_plt = fig_manager.plot_class_histogram(class_array=classes)
#
#     # Display the figure
#     fig_manager.display_and_save(fig=class_plt,
#                                  save_name='class_count')
#
# # Get the final path selection
# final_paths = manager.sort_by_class_and_score(trajectories=all_paths)
#
# if plotting:
#     # Plot the trajectories by class
#     y_size = human_specified_factors["selected_per_class"] * 2
#     class_plt = fig_manager.plot_trajectories_by_class(trajectories_with_class=final_paths,
#                                                        number_per_class=human_specified_factors["selected_per_class"],
#                                                        tsuffix="",
#                                                        figure_size=(total_classes, y_size))
#
#     # Display the figure
#     fig_manager.display_and_save(fig=class_plt,
#                                  save_name='class_trajs')
#
# # Create the converter class
# x_bound = initial_conditions["map_x_bounds"][1]
# y_bound = initial_conditions["map_y_bounds"][1]
# converter = UnityConverter(test_dimensions=[x_bound, y_bound],
#                            unity_dimensions=initial_conditions["unity_bounds"])
#
#
# print("UPDATE: Saving the tests to the correct folder")
# # For each test save the test into the correct class folder
# for class_key in final_paths:
#     # Used to keep track of the test counter
#     test_counter = 0
#
#     # For each trajectory in that class
#     for i in range(0, human_specified_factors["selected_per_class"]):
#
#         # If there are enough trajectories in that class
#         if len(final_paths[class_key]) <= i:
#             continue
#
#         # Get the trajectory
#         traj = final_paths[class_key][i]
#
#         # Create the save location
#         save_location = "maps/" + str(class_key) + "/test" + str(test_counter) + "/"
#
#         # Make sure this location has been created
#         if not os.path.isdir(save_path + save_location):
#             # If not create it
#             os.makedirs(save_path + save_location)
#
#         # Convert a trajectory into unity waypoints
#         unity_waypoints = converter.scale_to_unity(trajectory=traj)
#         wall_segments = converter.corridor_generator(unity_waypoints, corridor_gap=5)
#
#         if plotting:
#             corridor_plt = fig_manager.plot_corridor(waypoints=unity_waypoints,
#                                                      wall_segments=wall_segments,
#                                                      unity_dimensions=initial_conditions["unity_bounds"])
#             # Display the figure
#             fig_manager.display_and_save(fig=corridor_plt,
#                                          save_name=str("corridor_plot"),
#                                          save_directory=save_location,
#                                          only_save=False)
#
#             # Save the test to a unity file
#             converter.unity_text_file(waypoints=unity_waypoints,
#                                       corridor=wall_segments,
#                                       save_location=save_path + save_location)
#
#         # Increment the test counter
#         test_counter += 1
