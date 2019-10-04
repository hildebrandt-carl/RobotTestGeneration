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
                    default=5,
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
parser.add_argument('-u', '--debug',
                    action='store_true',
                    help='Displays each stage during the selection phase')
parser.add_argument('-s', '--seed',
                    default=10,
                    type=int,
                    help='Use to set a seed for the PRM construction phase. Set to 0 for to use time as seed')
parser.add_argument('-i', '--searchtime',
                    default=15,
                    type=int,
                    help='The amount of time allowed for path searching in seconds')
parser.add_argument('-g', '--gentype',
                    default="waypoint",
                    type=str,
                    help='Are you generating tests for waypoint or constant velocity controller')
args = parser.parse_args()

drone = None
save_path = None

# Make sure the correct term is given
assert(args.gentype == "waypoint" or args.gentype == "constant")

if args.drone == "bebop":
    drone = DroneType.BEBOP
    # Save locations
    save_path = "Results/BEBOP_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_" + str(args.gentype) +"/"
elif args.drone == "hector":
    drone = DroneType.HECTOR
    # Save locations
    save_path = "Results/HECTOR_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_" + str(args.gentype) +"/"
elif args.drone == "mit":
    drone = DroneType.MIT
    # Save locations
    save_path = "Results/MIT_seed" + str(args.seed) + "_depth" + str(args.depth) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_searchtime" + str(args.searchtime) + "_" + str(args.type) + "_" + str(args.gentype) +"/"

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

# Create a ranking system object
random_scoring = True
if args.type == "score":
    random_scoring = False
ranking_obj = RankingSystem(random_scoring=random_scoring)

# Used to keep track of how many paths were considered
total_paths_considered = 0
valid_paths_considered = 0
all_valid_paths = []
current_seed = args.seed
current_search_time = args.searchtime

# While we still have time
while time.time() - start_time < args.searchtime:
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
                                           gen_type=args.gentype)
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
    valid_paths = []
    if len(all_paths) > 0:

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

        # Save append the valid paths
        all_valid_paths.extend(valid_paths)

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

# Save the scores
if len(all_valid_paths) > 0:
    ranking_obj.save_trajectories_according_to_score(paths=all_valid_paths,
                                                     folder=save_path,
                                                     gen_type=args.gentype)

print("")
print("-----------------------------------")
print("DATA: Total time: " + str(time.time() - start_time))
print("Total paths found: " + str(total_paths_considered))
print("Valid paths found: " + str(valid_paths_considered))
print("UPDATE: Completed")