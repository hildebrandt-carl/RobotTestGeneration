import csv
import math
import os
from PRM import prm
from ConversionUtils import *
import numpy as np
from Radar import get_radar
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import random

# Robot start and end locations
r_start = [1, 13]
r_end = [24, 13]
robot_heading = 0
robot_max_turn = 179
robot_min_turn = -179
delta_angle = 0
plotting = False
plot_maps = False

# Save location
save_path = "Results/TestGen/run2/"
if not os.path.isdir(save_path):
    os.makedirs(save_path)

# Create the directory where the maps will be saved
maps_save_path = save_path + "maps/"
if not os.path.isdir(maps_save_path):
    os.makedirs(maps_save_path)

# The original map
map_name = "map1.csv"

# Load the map file
filename = "maps/" + str(map_name)

print("UPDATE: Loading Map - " + str(map_name))

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
p = prm(map_in=our_map,
        start_pos=r_start,
        end_pos=r_end)
p.findValidPositions(num_vertices=250,
                     wall_thresh=0.25)
p.plan(max_distance=5,
       min_distance=4)

if plotting:
    # Show the map after the prm construction phase
    print("UPDATE: Displaying Map")
    map_plt = p.getPlot(tsuffix="Graph",
                        figure_size=(10, 10))
    map_plt.savefig(save_path + str('00-original_map.png'))
    map_plt.show()


print("UPDATE: Finding Possible Paths")
# Find all paths from vertex (0) with robot heading = 0
all_paths, early_stops = p.findAllPaths(source_index=0,
                                        goal_index=1,
                                        heading=robot_heading,
                                        min_turn_deg=robot_min_turn - delta_angle,
                                        max_turn_deg=robot_max_turn + delta_angle,
                                        depth=8,
                                        max_traj=200000)

print("DATA: Total unique paths found: " + str(len(all_paths)))
print("DATA: Early stopping count: " + str(early_stops))


print("UPDATE: Classifying each path into fail or pass test")
path_class = p.classifyPathsReachability(paths=all_paths,
                                         source_index=0,
                                         heading=robot_heading,
                                         min_turn_deg=robot_min_turn,
                                         max_turn_deg=robot_max_turn,
                                         delta=delta_angle)

print("DATA: Total paths which are passable: " + str(path_class.count(True)))
print("DATA: Total paths which should fail: " + str(path_class.count(False)))

if len(all_paths) <= 0:
    exit()

# Score each of the paths
print("UPDATE: Scoring Each Path")
scores = p.scoreAllPaths(paths=all_paths,
                         source_index=0,
                         heading=robot_heading,
                         min_turn=robot_min_turn,
                         max_turn=robot_max_turn,
                         coverage_segments=20)

print("UPDATE: Normalizing Scores")
scores = p.normalizeScores(scores)

if plotting:
    # Display the trajectories
    print("UPDATE: Displaying Trajectories")
    path_plot = p.getPlot(highlighted_paths=all_paths,
                          path_class=path_class,
                          tsuffix="Possible Trajectories",
                          figure_size=(10, 10),
                          color_map='jet_r')
    path_plot.savefig(save_path + str('01-all_paths.png'))
    path_plot.show()


if plotting:
    # Display the scores
    print("UPDATE: Displaying Scores")
    labels = ['Reachability Set Coverage',
              'Average\nSegment\nLength',
              'Segment\nLength\nVariance',
              'Heading Variance',
              'Path\nLength',
              'Number\nTurns']
    title = "Path Comparison"
    radar_plt = get_radar(case_data=scores,
                          title="Path Comparison",
                          spoke_labels=labels,
                          color_map='jet_r')
    radar_plt.savefig(save_path + str('02-all_paths_score.png'))
    radar_plt.show()


# This will calculate the convex hull of the score
# Convex hell takes in of the shape (#points, #values)
print("UPDATE: Calculating Convex Hull")
hull = ConvexHull(scores)

# This goes through each points along the hull
print("UPDATE: Selecting Tests")
testsAlongHull = []
for simplex in hull.simplices:
    # Simplex is an array which contains the vertices forming a face of the convex hull
    testsAlongHull.extend(list(simplex))

# Get the unique set of tests selected
selected_tests_indices = list(set(testsAlongHull))
print("DATA: Total Tests Found: " + str(len(scores)))
print("DATA: Total Tests Selected: " + str(len(selected_tests_indices)))

# Get the list of selected paths, scores and classifications
selected_tests = np.array(all_paths)[selected_tests_indices]
selected_scores = scores[selected_tests_indices, :]
selected_classification = np.array(path_class)[selected_tests_indices]

# Get the list of unselected paths, scores and unselected tests
all_paths_indices = np.arange(len(all_paths))
not_selected_indices = all_paths_indices[~np.isin(all_paths_indices, selected_tests_indices)]
not_selected_tests = np.array(all_paths)[not_selected_indices]
not_selected_scores = scores[not_selected_indices, :]
not_selected_classifcation = np.array(path_class)[not_selected_indices]

# Generate a list of random indices the same length as selected indices
total_selected_tests = len(selected_tests_indices)
randomly_selected_tests_indices = random.sample(population=list(all_paths_indices),
                                                k=total_selected_tests)
# Get the randomly selected paths
print("UPDATE: Randomly Selecting Tests")
print("UPDATE: Total Randomly Selected Tests: " + str(len(randomly_selected_tests_indices)))
randomly_selected_tests = np.array(all_paths)[randomly_selected_tests_indices]
randomly_selected_scores = scores[randomly_selected_tests_indices, :]
randomly_selected_classification = np.array(path_class)[randomly_selected_tests_indices]


print("UPDATE: Updating Tests")
# Calculate some base differences
metric_lab = ['Reachability Set Coverage\t',
              'Average Segment Length\t\t',
              'Segment Length Variance\t\t',
              'Heading Variance \t\t\t',
              'Path Length\t\t\t\t\t',
              'Number Turns \t\t\t\t']
print("---------------------------------------------------------")
for i in range(0, 6):
    average_selected = np.average(selected_scores[:, i])
    average_rejected = np.average(not_selected_scores[:, i])
    variance_selected = np.var(selected_scores[:, i])
    variance_rejected = np.var(not_selected_scores[:, i])
    print("Data: Average: " + metric_lab[i] + "\t[Selected Tests]: " + str(average_selected))
    print("Data: Variance: " + metric_lab[i] + "\t[Selected Tests]: " + str(variance_selected))
    print("Data: Average: " + metric_lab[i] + "[Not Selected Tests]: " + str(average_rejected))
    print("Data: Variance: " + metric_lab[i] + "[Not Selected Tests]: " + str(variance_rejected))
    print()
print("---------------------------------------------------------")

# Display the selected graphs
if plotting:
    print("UPDATE: Displaying Selected Paths")
    path_plot = p.getPlot(highlighted_paths=selected_tests,
                          path_class=selected_classification,
                          tsuffix="Selected",
                          color_map='jet_r',
                          figure_size=(10, 10))
    path_plot.savefig(save_path + str('03-selected_paths.png'))
    path_plot.show()


# Display the randomly selected paths
if plotting:
    print("UPDATE: Displaying Selected Randomly Paths")
    path_plot = p.getPlot(highlighted_paths=randomly_selected_tests,
                          path_class=randomly_selected_classification,
                          tsuffix="Randomly Selected",
                          color_map='jet_r',
                          figure_size=(10, 10))
    path_plot.savefig(save_path + str('04-randomly_selected_paths.png'))
    path_plot.show()


if plotting:
    # Show the score selection for the selected scores
    print("UPDATE: Displaying Scores Selected Tests")
    radar_plt = get_radar(case_data=selected_scores,
                          title="Selected - Path Comparison",
                          spoke_labels=labels,
                          color_map='jet_r')
    radar_plt.savefig(save_path + str('05-selected_paths_score.png'))
    radar_plt.show()


if plotting:
    # Show the display the not selected graphs
    print("UPDATE: Displaying Not Selected Paths")
    path_plot = p.getPlot(highlighted_paths=not_selected_tests,
                          path_class=not_selected_classifcation,
                          tsuffix="Not Selected",
                          color_map='jet_r',
                          figure_size=(10, 10))
    path_plot.savefig(save_path + str('06-not_selected_paths.png'))
    path_plot.show()


if plotting:
    # Show the scores graph for the not selected scores
    print("UPDATE: Displaying Scores Of Not Selected Tests")
    radar_plt = get_radar(case_data=not_selected_scores,
                          title="Not Selected - Path Comparison",
                          spoke_labels=labels,
                          color_map='jet_r')
    radar_plt.savefig(save_path + str('07-not_selected_paths_score.png'))
    radar_plt.show()


if plotting:
    # Plot all the individual paths
    print("UPDATE: Displaying Selected Paths Individually")
    total_images = (math.ceil(math.sqrt(len(selected_tests)))) ** 2
    selectedpathplot = p.plotTrajectories(selected_tests=selected_tests,
                                          path_class=selected_classification,
                                          path_scores=selected_scores,
                                          total_plots=total_images,
                                          figure_size=(25, 25),
                                          tsuffix="Selected")
    selectedpathplot.savefig(save_path + str('08-selected_paths_ind.png'))
    selectedpathplot.show()

if plotting:
    # Plot all the randomly selected paths individualy
    print("UPDATE: Displaying Randomly Selected Paths Individually")
    total_images = (math.ceil(math.sqrt(len(randomly_selected_tests)))) ** 2
    selectedpathplot = p.plotTrajectories(selected_tests=randomly_selected_tests,
                                          path_class=randomly_selected_classification,
                                          path_scores=randomly_selected_scores,
                                          total_plots=total_images,
                                          figure_size=(25, 25),
                                          tsuffix="Randomly Selected")
    selectedpathplot.savefig(save_path + str('09-randomly_selected_paths_ind.png'))
    selectedpathplot.show()


if plotting:
    # Randomly select 100 tests from the not selected tests and plot them individually
    print("UPDATE: Displaying Not Selected Paths Individually")
    total_images = (math.ceil(math.sqrt(len(not_selected_tests)))) ** 2
    notselectedpathplot = p.plotTrajectories(selected_tests=not_selected_tests,
                                             path_class=not_selected_classifcation,
                                             path_scores=not_selected_scores,
                                             total_plots=total_images,
                                             figure_size=(25, 25),
                                             tsuffix="Not Selected")
    notselectedpathplot.savefig(save_path + str('10-not_selected_paths_ind.png'))
    notselectedpathplot.show()


# Selecting tests based on edge coverage
print("UPDATE: Finding Tests Based on Coverage")
final_tests, indices = p.selectTestsBasedOnCoverage(selected_tests=selected_tests)
final_scores = list(np.array(selected_scores[indices]))
final_classification = list(np.array(selected_classification[indices]))
print("DATA: Total tests selected: " + str(len(final_tests)))
print("DATA: Total tests which are passable: " + str(final_classification.count(True)))
print("DATA: Total tests which should fail: " + str(final_classification.count(False)))

print("UPDATE: Displaying Final Paths")
path_plot = p.getPlot(highlighted_paths=final_tests,
                      path_class=final_classification,
                      tsuffix="Final Paths",
                      color_map='jet_r',
                      figure_size=(10, 10))
path_plot.savefig(save_path + str('11-final_tests.png'))
path_plot.show()

# Display the final test cases found
print("UPDATE: Displaying Finally Selected Trajectories")
# Calculate the total number of images we want
total_images = (math.ceil(math.sqrt(len(final_tests))))**2
finalselectedtestsplot = p.plotTrajectories(selected_tests=final_tests,
                                            path_class=final_classification,
                                            path_scores=final_scores,
                                            total_plots=total_images,
                                            figure_size=(25, 25),
                                            tsuffix="Final Paths")
finalselectedtestsplot.savefig(save_path + str('12-final_tests_ind.png'))
finalselectedtestsplot.show()

# Create the randomly selected tests
path_i = -1
for path in randomly_selected_tests:
    # Increment the path counter
    path_i += 1

    # Create the map directory
    map_folder = "random/maps" + str(path_i) + "/"
    map_full_path = maps_save_path + map_folder
    if not os.path.isdir(map_full_path):
        os.makedirs(map_full_path)

    # Get the waypoints in the path
    waypoints = p.getWaypointsFromPath(path)
    # Create a window map from the trajectory
    window_test = p.windowMapFromWaypoints(waypoints=waypoints,
                                           window_gap=2,
                                           min_wall_distance=2)
    # Create a corridor map from the trajectory
    corridor_test = p.corridorMapFromWaypoints(waypoints=waypoints,
                                               corridor_gap=2)

    # Show the window map
    plt.imshow(window_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, window_test.shape[1] - 1])
    plt.ylim([0, window_test.shape[0] - 1])
    plt.savefig(map_full_path + str('window_test.png'))
    if plot_maps:
        plt.show()

    # Show the corridor map
    plt.imshow(corridor_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, corridor_test.shape[1] - 1])
    plt.ylim([0, corridor_test.shape[0] - 1])
    plt.savefig(map_full_path + str('corridor_test.png'))
    if plot_maps:
        plt.show()

    # Convert the map to a set of walls
    walls_window, ww_fig = pythonMaptoWalls(map=window_test,
                                            distance_threshold=1)
    if plot_maps:
        ww_fig.show()

    walls_corridor, wc_fig = pythonMaptoWalls(map=corridor_test,
                                              distance_threshold=1)
    if plot_maps:
        wc_fig.show()

    # Save this as a unity readable file
    wallsToUnityFile(walls=walls_window,
                     waypoints=waypoints,
                     savename=map_full_path + str("unity_wall"),
                     raining=False,
                     day=True)
    wallsToUnityFile(walls=walls_corridor,
                     waypoints=waypoints,
                     savename=map_full_path + str("unity_corridor"),
                     raining=False,
                     day=True)

    plt.close("all")


# Create the maps for the final tests
path_i = -1
for path in final_tests:
    # Increment the path counter
    path_i += 1

    # Create the map directory
    map_folder = "selected/maps" + str(path_i) + "/"
    map_full_path = maps_save_path + map_folder
    if not os.path.isdir(map_full_path):
        os.makedirs(map_full_path)

    # Get the waypoints in the path
    waypoints = p.getWaypointsFromPath(path)
    # Create a window map from the trajectory
    window_test = p.windowMapFromWaypoints(waypoints=waypoints,
                                           window_gap=2,
                                           min_wall_distance=2)
    # Create a corridor map from the trajectory
    corridor_test = p.corridorMapFromWaypoints(waypoints=waypoints,
                                               corridor_gap=2)

    # Show the window map
    plt.imshow(window_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, window_test.shape[1] - 1])
    plt.ylim([0, window_test.shape[0] - 1])
    plt.savefig(map_full_path + str('window_test.png'))
    if plot_maps:
        plt.show()

    # Show the corridor map
    plt.imshow(corridor_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, corridor_test.shape[1] - 1])
    plt.ylim([0, corridor_test.shape[0] - 1])
    plt.savefig(map_full_path + str('corridor_test.png'))
    if plot_maps:
        plt.show()

    # Convert the map to a set of walls
    walls_window, ww_fig = pythonMaptoWalls(map=window_test,
                                            distance_threshold=1)
    if plot_maps:
        ww_fig.show()

    walls_corridor, wc_fig = pythonMaptoWalls(map=corridor_test,
                                              distance_threshold=1)
    if plot_maps:
        wc_fig.show()

    # Save this as a unity readable file
    wallsToUnityFile(walls=walls_window,
                     waypoints=waypoints,
                     savename=map_full_path + str("unity_wall"),
                     raining=False,
                     day=True)
    wallsToUnityFile(walls=walls_corridor,
                     waypoints=waypoints,
                     savename=map_full_path + str("unity_corridor"),
                     raining=False,
                     day=True)

    plt.close("all")
print("Process Completed")
