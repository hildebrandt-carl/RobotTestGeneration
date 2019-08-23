import csv
import os

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
                    default="mit",
                    type=str,
                    help='Select drone type (bebop), (hector), (mit)')
parser.add_argument('-t', '--type',
                    default="score",
                    type=str,
                    help='Select search type (random), (maxvel), (kinematic), (score)')
parser.add_argument('-x', '--depth',
                    default=10,
                    type=int,
                    help='Total number of state changes allowed per trajectory')
parser.add_argument('-b', '--beamwidth',
                    default=1,
                    type=int,
                    help='The beam width used in the frontier exploration')
parser.add_argument('-n', '--nodes',
                    default=50,
                    type=int,
                    help='Number of nodes considered')
parser.add_argument('-r', '--resolution',
                    default=4,
                    type=int,
                    help='Resolution of the sample space')
parser.add_argument('-p', '--plotting',
                    action='store_true',
                    help='Save the tests as 3D figures')
parser.add_argument('-g', '--debug',
                    action='store_true',
                    help='Displays each stage during the selection phase')
parser.add_argument('-s', '--seed',
                    default=10,
                    type=int,
                    help='Use to set a seed for the PRM construction phase. Set to 0 for to use time as seed')
parser.add_argument('-i', '--searchtime',
                    default=300,
                    type=int,
                    help='The amount of time allowed for path searching in seconds')
parser.add_argument('-l', '--scoreangle',
                    default=180,
                    type=int,
                    help='The angle you assume to be the best')
args = parser.parse_args()

drone = None
save_path = None

if args.drone == "bebop":
    drone = DroneType.BEBOP
    # Save locations
    save_path = "Results/BEBOP_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_angle" + str(args.scoreangle) +"/"
elif args.drone == "hector":
    drone = DroneType.HECTOR
    # Save locations
    save_path = "Results/HECTOR_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_angle" + str(args.scoreangle) +"/"
elif args.drone == "mit":
    drone = DroneType.MIT
    # Save locations
    save_path = "Results/MIT_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_angle" + str(args.scoreangle) +"/"

# Do you want to plot the figures or not
plotting = False
if args.plotting == True:
    plotting = True

# If we are debugging
if args.debug == True:
    from PRM_Visualize import PRM
else:
    from PRM import PRM

# Test initial conditions
initial_conditions = {"map_x_bounds": [0, 30],
                      "map_y_bounds": [0, 30],
                      "map_z_bounds": [0, 30],
                      "start_point": [0.1, 0.1, 0.1],
                      "end_point": [15, 15, 15]}

# Specified by the tester
human_specified_factors = {"kinematic_sampling_resolution": args.resolution}

# Used to limit our search
traj_search_conditions = {"number_nodes": int(args.nodes),
                          "search_depth": int(args.depth),
                          "beam_width": float(args.beamwidth)}

# Robot kinematics
drone_kinematic = {}
if drone == DroneType.BEBOP:
    drone_kinematic["m"] = 0.71
    drone_kinematic["d"] = 0.16
    drone_kinematic["kf"] = 6.11e-8
    drone_kinematic["km"] = 1.5e-9
    drone_kinematic["max_rotor_speed"] = 8000
    drone_kinematic["inertial_properties"] = [2.32e-3, 2.32e-3, 4.00e-3]
    drone_kinematic["position"] = initial_conditions["start_point"]
    drone_kinematic["attitude"] = [0, 0, 0]
    drone_kinematic["velocity"] = [0, 0, 0]
    drone_kinematic["angular_velocity"] = [0, 0, 0]
    drone_kinematic["maximum_velocity"] = 20 # This take horizontal and vertical into consideration

elif drone == DroneType.HECTOR:
    drone_kinematic["m"] = 1.477
    drone_kinematic["d"] = 0.275
    drone_kinematic["kf"] = 4.142069415e-05
    drone_kinematic["km"] = -7.011631909766668e-5
    drone_kinematic["max_rotor_speed"] = 600
    drone_kinematic["inertial_properties"] = [0.01464, 0.01464, 0.02664]
    drone_kinematic["position"] = initial_conditions["start_point"]
    drone_kinematic["attitude"] = [0, 0, 0]
    drone_kinematic["velocity"] = [0, 0, 0]
    drone_kinematic["angular_velocity"] = [0, 0, 0]
    drone_kinematic["maximum_velocity"] = 15

elif drone == DroneType.MIT:
    drone_kinematic["m"] = 1.0
    drone_kinematic["d"] = 0.16
    drone_kinematic["kf"] = 1.91e-6
    drone_kinematic["km"] = 2.6e-7
    drone_kinematic["max_rotor_speed"] = 2000.0
    drone_kinematic["inertial_properties"] = [0.0049, 0.0049, 0.0049]
    drone_kinematic["position"] = initial_conditions["start_point"]
    drone_kinematic["attitude"] = [0, 0, 0]
    drone_kinematic["velocity"] = [0, 0, 0]
    drone_kinematic["angular_velocity"] = [0, 0, 0]
    drone_kinematic["maximum_velocity"] = 20 # This take horizontal and vertical into consideration

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

# Used to keep track of how many paths were considered
total_paths_considered = 0
valid_paths_considered = 0
current_seed = args.seed
current_search_time = args.searchtime

# While we still have time
while time.time() - start_time < args.searchtime and total_paths_considered <= 0:
    # keep track of when this run started
    run_start_time = time.time()

    print("")
    print('New Run!')

    # Generate the prm map
    print("UPDATE: Populating Trajectory Graph")
    # Create the PRM object
    p = PRM(start_pos=initial_conditions["start_point"],
            end_pos=initial_conditions["end_point"],
            map_x_bounds=initial_conditions["map_x_bounds"],
            map_y_bounds=initial_conditions["map_y_bounds"],
            map_z_bounds=initial_conditions["map_z_bounds"])

    # Find valid positions for waypoints
    p.populate_with_nodes(num_vertices=traj_search_conditions["number_nodes"],
                          input_seed=current_seed)

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

    print("UPDATE: Finding all paths")
    all_paths = None
    # Use random selection of nodes
    if args.type == "random":
        all_paths = p.find_all_paths_random(total_waypoints=traj_search_conditions["search_depth"],
                                            beam_width=traj_search_conditions["beam_width"],
                                            search_time=current_search_time)

    # Use random selection of nodes within max velocity
    elif args.type == "maxvel":
        all_paths = p.find_all_paths_maxvel(max_velocity=drone_kinematic["maximum_velocity"],
                                            total_waypoints=traj_search_conditions["search_depth"],
                                            beam_width=traj_search_conditions["beam_width"],
                                            search_time=current_search_time)


    # Use random selection of nodes within reachable set
    elif args.type == "kinematic":
        all_paths = p.find_all_paths_kinematic(robot_kinematic_model=drone_kinematic,
                                               kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"],
                                               total_waypoints=traj_search_conditions["search_depth"],
                                               beam_width=traj_search_conditions["beam_width"],
                                               search_time=current_search_time)

    # Use scored selection of nodes within reachable set
    elif args.type == "score":
        all_paths = p.find_all_paths_score(robot_kinematic_model=drone_kinematic,
                                           kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"],
                                           total_waypoints=traj_search_conditions["search_depth"],
                                           beam_width=traj_search_conditions["beam_width"],
                                           search_time=current_search_time,
                                           best_angle=args.scoreangle)
    else:
        print("ERROR: Search type not recognized")
        exit()


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

    # Assert that we have found some paths
    print("DATA: Total unique paths found: " + str(len(all_paths)))
    if len(all_paths) > 0:

        # Create a ranking system object
        ranking_obj = RankingSystem()

        # We need to go through all recorded paths and record which are valid and which are not
        valid_paths = ranking_obj.validate_paths(paths=all_paths,
                                                 robot_kinematic_model=drone_kinematic,
                                                 kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"])

        print("Paths considered that run: " + str(len(all_paths)))
        print("Valid considered found run: " + str(len(valid_paths)))
        print("")

        # Display the selected paths
        if plotting:
            # Show the map after the prm construction phase
            print("UPDATE: Displaying Valid Trajectories")
            map_plt = fig_manager.plot_selected_trajectories(nodes=p.get_vertices(),
                                                             selected_paths=valid_paths,
                                                             figure_size=(10, 10))

            # Display the figure
            fig_manager.display_and_save(fig=map_plt,
                                         save_name='valid_trajectories',
                                         only_save=True)

        # Save the scores
        ranking_obj.save_trajectories_according_to_score(paths=valid_paths,
                                                         folder=save_path,
                                                         path_number=valid_paths_considered)

    # Print Completion
    print("DATA: Time for that run: " + str(time.time() - run_start_time))

    # Keep track of the total
    total_paths_considered += len(all_paths)
    valid_paths_considered += len(valid_paths)

    # If we are going to loop again increment the seed and work out what our new search time is:
    current_search_time = current_search_time - (time.time() - run_start_time)
    current_seed += 1

    # Print how much time we have left
    print("DATA: Time left: " + str(current_search_time))

print("")
print("-----------------------------------")
print("DATA: Total time: " + str(time.time() - start_time))
print("Total paths found: " + str(total_paths_considered))
print("Valid paths found: " + str(valid_paths_considered))
print("UPDATE: Completed")




























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
