import csv
import os
from PRM import prm
import numpy as np
import matplotlib.pyplot as plt
from Trajectory import Trajectory
from TrajectoryManager import TrajectoryManager
from FigureManager import FigureManager
from EquivalenceChecker import EquivalenceChecker
from UnityConverter import UnityConverter

# Flags
plotting = True
plot_maps = True

# Save locations
save_path = "Results/TestGen/run10/"

# Load a map file
map_name = "map1.csv"

# Robot start and end locations
robot_kinematics = {"max_turn": 30,
                    "min_turn": -30,
                    "max_velocity": 5,
                    "min_velocity": 1}

# Test initial conditions
initial_conditions = {"map_x_bounds": [0, 25],
                      "map_y_bounds": [0, 25],
                      "start_point": [1, 13],
                      "end_point": [24, 13],
                      "robot_heading": 0,
                      "unity_bounds": [100, 100]}

# Specified by the tester
human_specified_factors = {"epsilon_angle": 2.5,
                           "epsilon_velocity": 1,
                           "epsilon_class_equivalence": 1,
                           "selected_per_class": 10}

# Used to limit our search
traj_search_conditions = {"number_nodes": 200,
                          "max_trajectories": 3000,
                          "search_depth": 15}

# Give the trajectory class the correct bounds
Trajectory.x_range = {"lower": initial_conditions["map_x_bounds"][0],
                      "upper": initial_conditions["map_x_bounds"][1]}
Trajectory.y_range = {"lower": initial_conditions["map_y_bounds"][0],
                      "upper": initial_conditions["map_y_bounds"][1]}

# Create the Figure manager
fig_manager = FigureManager(save_path)

# Make sure the directory we want to save our files in is created
fig_manager.create_directory()

# Load the map file
print("UPDATE: Loading Map - " + str(map_name))
filename = "maps/" + str(map_name)

with open(filename, 'r') as f:
    reader = csv.reader(f, delimiter=',')
    # get all the rows as a list
    our_map = list(reader)
    # Reverse the map so that the bottom of the map file stays at the bottom
    our_map = list(reversed(our_map))
    # transform data into numpy array
    our_map = np.array(our_map).astype(float)

# Generate the prm map
print("UPDATE: Populating Trajectory Graph")
# Create the PRM object
p = prm(map_in=our_map,
        start_pos=initial_conditions["start_point"],
        end_pos=initial_conditions["end_point"])

# Find valid positions for waypoints
p.findValidPositions(num_vertices=traj_search_conditions["number_nodes"],
                     wall_thresh=0.25)

# Connect waypoints
p.plan(max_distance=robot_kinematics["max_velocity"],
       min_distance=robot_kinematics["min_velocity"])

if plotting:
    # Show the map after the prm construction phase
    print("UPDATE: Displaying Map")
    map_plt = p.getPlot(tsuffix="Graph",
                        figure_size=(10, 10))

    # Display the figure
    fig_manager.display_and_save(fig=map_plt,
                                 save_name='original_map')


print("UPDATE: Finding Possible Paths")
# Find all paths between start and end position
all_paths = p.findAllPaths(heading=initial_conditions["robot_heading"],
                           min_turn_deg=robot_kinematics["min_turn"] - human_specified_factors["epsilon_angle"],
                           max_turn_deg=robot_kinematics["max_turn"] + human_specified_factors["epsilon_angle"],
                           depth=traj_search_conditions["search_depth"],
                           max_traj=traj_search_conditions["max_trajectories"])

# Assert that we have found some paths
assert(len(all_paths) > 0)
print("DATA: Total unique paths found: " + str(len(all_paths)))

# Classify if the tests pass or fail
print("UPDATE: Classifying each path into fail or pass test")
for path in all_paths:
    path.check_if_passing(max_turn=robot_kinematics["max_turn"],
                          min_turn=robot_kinematics["min_turn"])

# Count the number of tests which are passable
print("UPDATE: Checking how many tests can pass and fail")
passing_test_count = 0
for path in all_paths:
    if path.get_passing():
        passing_test_count += 1
print("DATA: Total paths which are passable: " + str(passing_test_count))
print("DATA: Total paths which should fail: " + str(len(all_paths) - passing_test_count))

# Create a scorer class and score the trajectories
manager = TrajectoryManager(kinematics=robot_kinematics)
scorer_success = manager.calculate_score(trajectories=all_paths)

# Make sure all trajectories have scores
assert(scorer_success == 1)

# Sort the trajectories based on score
all_paths = manager.sort_by_score(trajectories=all_paths)

if plotting:
    # Display the trajectories
    print("UPDATE: Displaying Trajectories Over Each Other")
    path_plot = fig_manager.plot_allpaths(trajectories=all_paths,
                                      tsuffix="Possible Trajectories",
                                      figure_size=(10, 10))
    # Display the figure
    fig_manager.display_and_save(fig=path_plot,
                                 save_name='all_paths')

if plotting:
    # Get the scores
    scores = manager.get_all_scores(trajectories=all_paths)

    # Create the plot
    print("UPDATE: Displaying the scores")
    score_plt = fig_manager.plot_scores(score_array=scores)

    # Display the figure
    fig_manager.display_and_save(fig=plt,
                                 save_name='scores_graph')


# Group the trajectories into different classes
grouper = EquivalenceChecker(epsilon=human_specified_factors["epsilon_class_equivalence"])
total_classes = grouper.find_equivalence(all_paths)

print("DATA: Total classes: " + str(total_classes))

if plotting:
    # Display the classes
    classes = manager.get_all_classes(trajectories=all_paths)

    # Plot the class histogram
    print("UPDATE: Displaying the scores")
    class_plt = fig_manager.plot_class_histogram(class_array=classes)

    # Display the figure
    fig_manager.display_and_save(fig=class_plt,
                                 save_name='class_count')

# Get the final path selection
final_paths = manager.sort_by_class_and_score(trajectories=all_paths)

if plotting:
    # Plot the trajectories by class
    y_size = human_specified_factors["selected_per_class"] * 2
    class_plt = fig_manager.plot_trajectories_by_class(trajectories_with_class=final_paths,
                                                       number_per_class=human_specified_factors["selected_per_class"],
                                                       tsuffix="",
                                                       figure_size=(total_classes, y_size))

    # Display the figure
    fig_manager.display_and_save(fig=class_plt,
                                 save_name='class_trajs')

# Create the converter class
x_bound = initial_conditions["map_x_bounds"][1]
y_bound = initial_conditions["map_y_bounds"][1]
converter = UnityConverter(test_dimensions=[x_bound, y_bound],
                           unity_dimensions=initial_conditions["unity_bounds"])


print("UPDATE: Saving the tests to the correct folder")
# For each test save the test into the correct class folder
for class_key in final_paths:
    # Used to keep track of the test counter
    test_counter = 0

    # For each trajectory in that class
    for i in range(0, human_specified_factors["selected_per_class"]):

        # If there are enough trajectories in that class
        if len(final_paths[class_key]) <= i:
            continue

        # Get the trajectory
        traj = final_paths[class_key][i]

        # Create the save location
        save_location = "maps/" + str(class_key) + "/test" + str(test_counter) + "/"

        # Make sure this location has been created
        if not os.path.isdir(save_path + save_location):
            # If not create it
            os.makedirs(save_path + save_location)

        # Convert a trajectory into unity waypoints
        unity_waypoints = converter.scale_to_unity(trajectory=traj)
        wall_segments = converter.corridor_generator(unity_waypoints, corridor_gap=10)

        if plotting:
            corridor_plt = fig_manager.plot_corridor(waypoints=unity_waypoints,
                                                     wall_segments=wall_segments,
                                                     unity_dimensions=initial_conditions["unity_bounds"])
            # Display the figure
            fig_manager.display_and_save(fig=corridor_plt,
                                         save_name=str("corridor_plot"),
                                         save_directory=save_location,
                                         only_save=True)

            # Save the test to a unity file
            converter.unity_text_file(waypoints=unity_waypoints,
                                      corridor=wall_segments,
                                      save_location=save_path + save_location)

        # Increment the test counter
        test_counter += 1
