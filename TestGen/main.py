import csv
from PRM import prm
from ConversionUtils import *
import numpy as np
from Radar import get_radar
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

# Robot start and end locations
r_start = [1, 8]
r_end = [19, 8]
robot_heading = 0
robot_max_turn = 45
robot_min_turn = -45

# # Get the map
for map_num in range(1, 2):
    # Load the map file
    filename = "maps/map" + str(map_num) + ".csv"
    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get all the rows as a list
        our_map = list(reader)
        # Reverse the map so that the bottom of the map file stays at the bottom
        our_map = list(reversed(our_map))
        # transform data into numpy array
        our_map = np.array(our_map).astype(float)

    # Generate the prm map
    p = prm(map_in=our_map,
            start_pos=r_start,
            end_pos=r_end)
    p.find_valid_positions(num_vertices=100,
                           wall_thresh=0.25)
    p.plan(dist_thresh=4)

    # Show the map after the prm construction phase
    map_plt = p.get_plot()
    map_plt.show()

    # Find all paths from vertex (0) with robot heading = 0
    all_paths = p.findAllPaths(source_index=0,
                               goal_index=1,
                               heading=robot_heading,
                               min_turn=robot_min_turn,
                               max_turn=robot_max_turn)

    print("Total unique paths found: " + str(len(all_paths)))

    if len(all_paths) <= 0:
        exit()

    # Score each of the paths
    scores = p.scoreAllPaths(paths=all_paths,
                             source_index=0,
                             heading=robot_heading,
                             min_turn=robot_min_turn,
                             max_turn=robot_max_turn,
                             coverage_segments=20)

    scores = p.normalizeScores(scores)
    #print(scores)

    # Generate the colors we want to use
    #path_plot = p.get_plot(all_paths, color_map='jet_r')
    #path_plot.show()
    print('---------------------')

    labels = ['Reachability Set Coverage',
              'Average\nSegement\nLength',
              'Segement\nLength\nVariance',
              'Heading Variance',
              'Path\nLength',
              'Number\nTurns']
    title = "Path Comparison"

    # radar_plt = get_radar(case_data=scores,
    #                       title="Path Comparison",
    #                       spoke_labels=labels,
    #                       color_map='jet_r')

    #radar_plt.show()

    # This will calculate the convex hull of the score
    # Convex hell takes in of the shape (#points, #values)
    hull = ConvexHull(scores)

    # This goes through each points along the hull
    testsAlongHull = []
    for simplex in hull.simplices:
        # Simplex is an array which contains the vertices forming a face of the convex hull
        testsAlongHull.extend(list(simplex))

    # Get the unique set of tests selected
    selected_tests_indices = list(set(testsAlongHull))
    print("All Tests Selected: " + str(selected_tests_indices))
    print("Total Tests Found: " + str(len(scores)))
    print("Total Tests Selected: " + str(len(selected_tests_indices)))

    # Get the list of selected paths and scores
    selected_tests = np.array(all_paths)[selected_tests_indices]
    selected_scores = scores[selected_tests_indices, :]

    # Get the list of unselected paths and scores
    all_paths_indices = np.arange(len(all_paths))
    not_selected_indices = all_paths_indices[~np.isin(all_paths_indices, selected_tests_indices)]
    not_selected_tests = np.array(all_paths)[not_selected_indices]
    not_selected_scores = scores[not_selected_indices, :]

    # Calculate some base differences
    metric_lab = ['Reachability Set Coverage\t',
                  'Average Segement Length\t',
                  'Segement Length Variance\t',
                  'Heading Variance\t\t\t',
                  'Path Length\t\t\t\t',
                  'Number Turns\t\t\t\t']
    print("---------------------------------------------------------")
    for i in range(0, 6):
        average_selected = np.average(selected_scores[:, i])
        average_rejected = np.average(not_selected_scores[:, i])
        variance_selected = np.var(selected_scores[:, i])
        variance_rejected = np.var(not_selected_scores[:, i])
        print("Average: " + metric_lab[i] + "\t[Selected Tests]: " + str(average_selected))
        print("Variance: " + metric_lab[i] + "\t[Selected Tests]: " + str(variance_selected))
        print("Average: " + metric_lab[i] + "[Not Selected Tests]: " + str(average_rejected))
        print("Variance: " + metric_lab[i] + "[Not Selected Tests]: " + str(variance_rejected))
        print()
    print("---------------------------------------------------------")

    # Plot the scores
    path_plot = p.get_plot(selected_tests, tsuffix="Selected", color_map='jet_r')
    path_plot.show()
    radar_plt = get_radar(case_data=selected_scores,
                          title="Selected - Path Comparison",
                          spoke_labels=labels,
                          color_map='jet_r')
    radar_plt.show()

    path_plot = p.get_plot(not_selected_tests, tsuffix="Selected", color_map='jet_r')
    path_plot.show()
    radar_plt = get_radar(case_data=not_selected_scores,
                          title="Not Selected - Path Comparison",
                          spoke_labels=labels,
                          color_map='jet_r')
    radar_plt.show()



















    # highest_variance = np.argmax(scores)
    # lowest_variance = np.argmin(scores)
    # highest_variance_path = [all_paths[highest_variance]]
    # lowest_variance_path = [all_paths[lowest_variance]]
    #
    # print("Shortest Line Headings: " + str(p.generateLineHeadings(all_paths[0])))
    # print("Lowest Variance Headings: " + str(p.generateLineHeadings(lowest_variance_path[0])))
    # print("Highest Variance Headings: " + str(p.generateLineHeadings(highest_variance_path[0])))
    #
    # p.plot(all_paths, "All Paths")
    # p.plot([all_paths[0]], "Shortest Path")
    # p.plot(highest_variance_path, "Highest Variance")
    # p.plot(lowest_variance_path, "Lowest Variance")
    #
    # # Generate window map from these plots
    # waypoints = p.getWaypointsFromPath(all_paths[highest_variance])
    # window_test = p.windowMapFromWaypoints(waypoints=waypoints,
    #                                        window_gap=8,
    #                                        min_wall_distance=2)
    #
    # # Generate corridor map from these plots
    # corridor_test = p.corridorMapFromWaypoints(waypoints=waypoints,
    #                                            corridor_gap=2)
    #
    # # Show the map 1
    # plt.imshow(window_test, cmap='binary')
    # plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    # plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    # plt.xlim([0, window_test.shape[1] - 1])
    # plt.ylim([0, window_test.shape[1] - 1])
    # plt.show()
    #
    # # Show the map 2
    # plt.imshow(corridor_test, cmap='binary')
    # plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    # plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    # plt.xlim([0, corridor_test.shape[1] - 1])
    # plt.ylim([0, corridor_test.shape[1] - 1])
    # plt.show()
    #
    # walls1 = pythonMaptoWalls(map=window_test,
    #                           distance_threshold=1)
    # walls2 = pythonMaptoWalls(map=window_test,
    #                           distance_threshold=5)
    # walls3 = pythonMaptoWalls(map=corridor_test,
    #                           distance_threshold=1)
    # walls4 = pythonMaptoWalls(map=corridor_test,
    #                           distance_threshold=5)
    #
    # wallsToUnityFile(walls1, waypoints, "/home/autosoftlab/Desktop/window1")
    # wallsToUnityFile(walls2, waypoints, "/home/autosoftlab/Desktop/window5")
    # wallsToUnityFile(walls3, waypoints, "/home/autosoftlab/Desktop/corridor1")
    # wallsToUnityFile(walls4, waypoints, "/home/autosoftlab/Desktop/corridor5")
    #





