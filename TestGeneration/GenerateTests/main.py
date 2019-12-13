import sys
import argparse
import time

import numpy as np

from ValidationSystem import ValidationSystem
from FigureManager import FigureManager
from ScoringSystem import ScoringSystem
from PRM import PRM
from enum import Enum

class DroneType(Enum):
    MIT = 1
    ANAFI = 2

# Get the time of the program start
start_time = time.time()

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--drone',
                    default="mit",
                    type=str,
                    help='Select drone type (mit), (anafi)')
parser.add_argument('-t', '--searchtype',
                    default="maxvel",
                    type=str,
                    help='Select search type (random), (maxvel), (kinematic)')
parser.add_argument('-c', '--scoretype',
                    default="random",
                    type=str,
                    help='Select the scoring type used (random), (edge), (edge90), (edge180) (learned)')
parser.add_argument('-l', '--trajectorylength',
                    default=5,
                    type=int,
                    help='Total number of waypoints per trajectory')
parser.add_argument('-b', '--beamwidth',
                    default=5,
                    type=int,
                    help='The beam width used in the frontier exploration')
parser.add_argument('-n', '--nodes',
                    default=50,
                    type=int,
                    help='Number of nodes in the world which are considered')
parser.add_argument('-r', '--resolution',
                    default=4,
                    type=int,
                    help='Resolution of the sample space for reachable set generation')
parser.add_argument('-p', '--plotting',
                    action='store_true',
                    help='Save each of the tests in a 3D plot')
parser.add_argument('-e', '--seed',
                    default=10,
                    type=int,
                    help='Use to set a seed for the PRM construction phase. Set to 0 for to use time as seed')
parser.add_argument('-o', '--totaltime',
                    default=180,
                    type=int,
                    help='The total amount of time for both searching and executing the tests')
parser.add_argument('-x', '--simulationtime',
                    default=90,
                    type=int,
                    help='The total time executing a test takes. This time is subtracted from the totaltime after a physically valid trajectory is found')
parser.add_argument('-s', '--savename',
                    default="",
                    type=str,
                    help='This name will be appended as a suffix to each of the files saved by test generation')
parser.add_argument('-s', '--modeldirectory',
                    default="",
                    type=str,
                    help='This is the directory of the scoring model you are using if you are using the score type learned')
parser.add_argument('-s', '--modelprefix',
                    default="",
                    type=str,
                    help='This is the name of the scoring model you are using if you are using the score type learned')
args = parser.parse_args()

# Init the drone and save path vairables
drone = None
save_path = None

# Compute the prefix
prefix = ""
if args.savename != "":
    prefix = args.savename + "_"

# Get the type of drone
if args.drone == "mit":
    # Set the drone type and save location
    drone = DroneType.MIT
    save_path = "./Results/" + str(prefix) + "MIT_seed" + str(args.seed) + "_length" + str(args.trajectorylength) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_totaltime" + str(args.totaltime) + "_simtime" + str(args.simulationtime) + "_searchtype_" + str(args.searchtype) + "_scoretype_" + str(args.scoretype) + "/"
elif args.drone == "anafi":
    # Set the drone type and save location
    drone = DroneType.ANAFI
    save_path = "Results/" + str(prefix) + "ANAFI_seed" + str(args.seed) + "_length" + str(args.trajectorylength) + "_nodes" + str(args.nodes) + "_res" + str(args.resolution) + "_beamwidth" + str(args.beamwidth) + "_totaltime" + str(args.totaltime) + "_simtime" + str(args.simulationtime) + "_searchtype_" + str(args.searchtype) + "_scoretype_" + str(args.scoretype) + "/"
else:
    # Exit as we do not know the type of drone
    sys.exit("Error 1: Drone type ''" + str(args.drone) + "'' is unknown, please use either mit or anafi")


# Do you want to plot the figures or not
plotting = False
if args.plotting == True:
    plotting = True

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
                          "search_depth": int(args.trajectorylength),
                          "beam_width": float(args.beamwidth)}

# Robot kinematics
drone_kinematic = {}
if drone == DroneType.MIT:
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
    drone_kinematic["maximum_velocity"] = 20  # This take horizontal and vertical into consideration

elif drone == DroneType.ANAFI:
    drone_kinematic["m"] = 0.8
    drone_kinematic["d"] = 0.20
    drone_kinematic["kf"] = 1.91e-6
    drone_kinematic["km"] = 2.6e-7
    drone_kinematic["max_rotor_speed"] = 2000.0
    drone_kinematic["inertial_properties"] = [2.32e-3, 2.32e-3, 4.00e-3]
    drone_kinematic["position"] = initial_conditions["start_point"]
    drone_kinematic["attitude"] = [0, 0, 0]
    drone_kinematic["velocity"] = [0, 0, 0]
    drone_kinematic["angular_velocity"] = [0, 0, 0]
    drone_kinematic["maximum_velocity"] = 15  # This take horizontal and vertical into consideration


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
scoring_function = ScoringSystem(score_type=args.scoretype, modeldir=args.modeldirectory + args.modelprefix)

# Create a validation system class
validation_function = ValidationSystem()

# Used to keep track of how many paths were considered (Used to answer RQ1)
total_explored_trajectories = 0
total_complete_trajectories = 0
total_valid_trajectories = 0

# Create a list of valid trajectories
all_valid_trajectories = []

# Init variables used for generation
current_seed = args.seed
generation_time_left = args.totaltime - (len(all_valid_trajectories) * args.simulationtime)

# While we still have time
while time.time() - start_time < generation_time_left:
    # keep track of when this run started
    print("DATA: Starting search at time: " + str(time.time() - start_time))
    print("DATA: Generation time left: " + str(generation_time_left))

    print("")
    print('UPDATE: Running search on new world')

    # We need to generate a new prm map
    print("UPDATE: Populating trajectory graph")

    # Create the PRM object
    p = PRM(start_pos=initial_conditions["start_point"],
            end_pos=initial_conditions["end_point"],
            start_time=time.time(),
            map_x_bounds=initial_conditions["map_x_bounds"],
            map_y_bounds=initial_conditions["map_y_bounds"],
            map_z_bounds=initial_conditions["map_z_bounds"])

    # Find valid positions for placement waypoints
    pop_success = p.populate_with_nodes(num_vertices=traj_search_conditions["number_nodes"],
                                        input_seed=current_seed)

    # Check that PRM worked
    if not pop_success:
        sys.exit("Error 5: Populating world using PRM failed")

    # Check if we are plotting. 
    if plotting:
        # Show the world after the prm construction phase
        print("UPDATE: Displaying current world")
        map_plt = fig_manager.plot_prm_graph(nodes=p.get_waypoints(),
                                             edges=[],
                                             figure_size=(10, 10))

        # Save the figure
        fig_manager.display_and_save(fig=map_plt,
                                     save_name='original_world',
                                     only_save=True)

    print("UPDATE: Searching for trajectories in the world")    
    # Create a list of all complete trajectories
    complete_trajectories = None
    number_explored = 0

    # Use random selection of nodes
    if args.searchtype == "random":
        complete_trajectories, number_explored = p.find_all_paths_random(total_waypoints=traj_search_conditions["search_depth"],
                                                                         beam_width=traj_search_conditions["beam_width"],
                                                                         search_time=generation_time_left)

    # Use random selection of nodes within max velocity
    elif args.searchtype == "maxvel":
        complete_trajectories, number_explored = p.find_all_paths_maxvel(max_velocity=drone_kinematic["maximum_velocity"],
                                                                         total_waypoints=traj_search_conditions["search_depth"],
                                                                         beam_width=traj_search_conditions["beam_width"],
                                                                         search_time=generation_time_left)

    # Uses the kinematic model to reject nodes
    elif args.searchtype == "kinematic":
        complete_trajectories, number_explored = p.find_all_paths_kinematic(robot_kinematic_model=drone_kinematic,
                                                                            kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"],
                                                                            total_waypoints=traj_search_conditions["search_depth"],
                                                                            beam_width=traj_search_conditions["beam_width"],
                                                                            search_time=generation_time_left,
                                                                            score_func=scoring_function)
    
    # Otherwise there was an error
    else:
        sys.exit("Error 7: The search type was not recognized")

    # Display the complete trajectories
    if plotting:
        # Show the map after the prm construction phase
        print("UPDATE: Displaying completed trajectories")
        map_plt = fig_manager.plot_selected_trajectories(nodes=p.get_waypoints(),
                                                         selected_paths=complete_trajectories,
                                                         figure_size=(10, 10))

        # Display the figure
        fig_manager.display_and_save(fig=map_plt,
                                     save_name='complete_trajectories',
                                     only_save=True)

    # Assert that we have found some complete trajectories
    print("DATA: Total unique complete trajectories found: " + str(len(complete_trajectories)))

    valid_trajectories = []
    if len(complete_trajectories) > 0:

        # We need to go through all recorded trajectories and record which are valid and which are not
        valid_trajectories = validation_function.validate_trajectories(trajectories=complete_trajectories,
                                                                       robot_kinematic_model=drone_kinematic,
                                                                       kinematic_sample_resolution=human_specified_factors["kinematic_sampling_resolution"])

        print("Total complete trajectories: " + str(len(complete_trajectories)))
        print("Total valid trajectories: " + str(len(valid_trajectories)))
        print("")

        # Display the selected trajectories
        if plotting:
            # Show the map after the prm construction phase
            print("UPDATE: Displaying valid trajectories")
            map_plt = fig_manager.plot_selected_trajectories(nodes=p.get_waypoints(),
                                                             selected_paths=valid_trajectories,
                                                             figure_size=(10, 10))

            # Display the figure
            fig_manager.display_and_save(fig=map_plt,
                                         save_name='valid_trajectories',
                                         only_save=True)

        # Save append the valid trajectories
        all_valid_trajectories.extend(valid_trajectories)

    # Keep track of the total number of explored, complete and valid trajectories
    total_explored_trajectories += number_explored
    total_complete_trajectories += len(complete_trajectories)
    total_valid_trajectories += len(valid_trajectories)

    # If we are going to loop again increment the seed and work out what our new search time is:
    generation_time_left = args.totaltime - (len(all_valid_trajectories) * args.simulationtime)
    current_seed += 1

    # Print Completion
    print("UPDATE: Reseting PRM as trajectories were found")
    print("DATA: Search Completed at Time: " + str(time.time() - start_time))
    print("DATA: Trajectories explored during that search: " + str(number_explored))
    print("DATA: Complete trajectories found during that search: " + str(len(complete_trajectories)))
    print("DATA: Valid trajectories found during that search: " + str(len(valid_trajectories)))
    print("DATA: Total explored trajectories: " + str(total_explored_trajectories))
    print("DATA: Total complete trajectories: " + str(total_complete_trajectories))
    print("DATA: Total valid trajectories: " + str(total_valid_trajectories))
    print("DATA: Estimated simulation time for valid trajectories: " + str(len(all_valid_trajectories) * args.simulationtime))
    print("DATA: Total generation time left: " + str(generation_time_left))

# Save the test based on the score for each of the tests
if len(all_valid_trajectories) > 0:
    validation_function.save_trajectories_according_to_score(trajectories=all_valid_trajectories,
                                                             scoring_function=scoring_function,
                                                             folder=save_path)

print("")
print("-----------------------------------")
print("DATA: Total search time: " + str(time.time() - start_time))
print("DATA: Estimated simulation time: " + str(len(all_valid_trajectories) * args.simulationtime))
print("DATA: Total overall time: " + str(time.time() - start_time + (len(all_valid_trajectories) * args.simulationtime)))
print("DATA: Given time: " + str(args.totaltime))
print("DATA: Total explored trajectories: " + str(total_explored_trajectories))
print("DATA: Total complete trajectories: " + str(total_complete_trajectories))
print("DATA: Total valid trajectories: " + str(total_valid_trajectories))
print("-----------------------------------")
print("UPDATE: Test generation completed")
print("-----------------------------------")




# bebop and hector no longer supported
# if drone == DroneType.BEBOP:
#     drone_kinematic["m"] = 0.71
#     drone_kinematic["d"] = 0.16
#     drone_kinematic["kf"] = 6.11e-8
#     drone_kinematic["km"] = 1.5e-9
#     drone_kinematic["max_rotor_speed"] = 8000
#     drone_kinematic["inertial_properties"] = [2.32e-3, 2.32e-3, 4.00e-3]
#     drone_kinematic["position"] = initial_conditions["start_point"]
#     drone_kinematic["attitude"] = [0, 0, 0]
#     drone_kinematic["velocity"] = [0, 0, 0]
#     drone_kinematic["angular_velocity"] = [0, 0, 0]
#     drone_kinematic["maximum_velocity"] = 20 # This take horizontal and vertical into consideration

# elif drone == DroneType.HECTOR:
#     drone_kinematic["m"] = 1.477
#     drone_kinematic["d"] = 0.275
#     drone_kinematic["kf"] = 4.142069415e-05
#     drone_kinematic["km"] = -7.011631909766668e-5
#     drone_kinematic["max_rotor_speed"] = 600
#     drone_kinematic["inertial_properties"] = [0.01464, 0.01464, 0.02664]
#     drone_kinematic["position"] = initial_conditions["start_point"]
#     drone_kinematic["attitude"] = [0, 0, 0]
#     drone_kinematic["velocity"] = [0, 0, 0]
#     drone_kinematic["angular_velocity"] = [0, 0, 0]
#     drone_kinematic["maximum_velocity"] = 15