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
    print("UPDATE: Loading Map")
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
    p.findValidPositions(num_vertices=300,
                         wall_thresh=0.25)
    p.plan(dist_thresh=2.5)

    # Show the map after the prm construction phase
    # print("UPDATE: Displaying Map")
    # map_plt = p.getPlot(tsuffix="Graph",
    #                     figure_size=(10, 10))
    # map_plt.show()

    print("UPDATE: Finding Possible Paths")
    # Find all paths from vertex (0) with robot heading = 0
    all_paths = p.findAllPaths(source_index=0,
                               goal_index=1,
                               heading=robot_heading,
                               min_turn=robot_min_turn,
                               max_turn=robot_max_turn,
                               depth=15,
                               max_traj=15000)

    print("DATA: Total unique paths found: " + str(len(all_paths)))

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

    # Display the trajectories
    # print("UPDATE: Displaying Trajectories")
    # path_plot = p.getPlot(highlighted_paths=all_paths,
    #                       tsuffix="Possible Trajectories",
    #                       figure_size=(10, 10),
    #                       color_map='jet_r')
    # path_plot.show()

    # Display the scores
    # print("UPDATE: Displaying Scores")
    # labels = ['Reachability Set Coverage',
    #           'Average\nSegment\nLength',
    #           'Segment\nLength\nVariance',
    #           'Heading Variance',
    #           'Path\nLength',
    #           'Number\nTurns']
    # title = "Path Comparison"
    # radar_plt = get_radar(case_data=scores,
    #                       title="Path Comparison",
    #                       spoke_labels=labels,
    #                       color_map='jet_r')
    # radar_plt.show()

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

    # Get the list of selected paths and scores
    selected_tests = np.array(all_paths)[selected_tests_indices]
    selected_scores = scores[selected_tests_indices, :]

    # Get the list of unselected paths and scores
    all_paths_indices = np.arange(len(all_paths))
    not_selected_indices = all_paths_indices[~np.isin(all_paths_indices, selected_tests_indices)]
    not_selected_tests = np.array(all_paths)[not_selected_indices]
    not_selected_scores = scores[not_selected_indices, :]

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

    # Show the display the not selected graphs
    # print("UPDATE: Displaying Selected Paths")
    # path_plot = p.getPlot(highlighted_paths=selected_tests,
    #                       tsuffix="Selected",
    #                       color_map='jet_r',
    #                       figure_size=(10, 10))
    # path_plot.show()

    # Show the score selection for the selected scores
    # print("UPDATE: Displaying Scores Selected Tests")
    # radar_plt = get_radar(case_data=selected_scores,
    #                       title="Selected - Path Comparison",
    #                       spoke_labels=labels,
    #                       color_map='jet_r')
    # radar_plt.show()

    # Show the display the not selected graphs
    # print("UPDATE: Displaying Not Selected Paths")
    # path_plot = p.getPlot(highlighted_paths=not_selected_tests,
    #                       tsuffix="Not Selected",
    #                       color_map='jet_r',
    #                       figure_size=(10, 10))
    # path_plot.show()

    # Show the scores graph for the not selected scores
    # print("UPDATE: Displaying Scores Of Not Selected Tests")
    # radar_plt = get_radar(case_data=not_selected_scores,
    #                       title="Not Selected - Path Comparison",
    #                       spoke_labels=labels,
    #                       color_map='jet_r')
    # radar_plt.show()

    # Randomly select 100 tests from the selected tests and plot them individually
    # print("UPDATE: Displaying Selected Paths Individually")
    # selectedpathplot = p.plotTrajectories(selected_tests=selected_tests,
    #                                       total_plots=100,
    #                                       figure_size=(25, 25),
    #                                       tsuffix="Selected")
    # selectedpathplot.show()

    # Randomly select 100 tests from the not selected tests and plot them individually
    # print("UPDATE: Displaying Not Selected Paths Individually")
    # notselectedpathplot = p.plotTrajectories(selected_tests=not_selected_tests,
    #                                          total_plots=100,
    #                                          figure_size=(25, 25),
    #                                          tsuffix="Not Selected")
    # notselectedpathplot.show()



    # Selecting tests based on edge coverage
    print("UPDATE: Finding Tests Based on Coverage")
    final_tests = p.selectTestsBasedOnCoverage(selected_tests=selected_tests)
    print("DATA: Total Tests Selected: " + str(len(final_tests)))

    print("UPDATE: Displaying Final Paths")
    path_plot = p.getPlot(highlighted_paths=final_tests,
                          tsuffix="Final Paths",
                          color_map='jet_r',
                          figure_size=(10, 10))
    path_plot.show()


    # Display the final test cases found
    print("UPDATE: Displaying Finally Selected Trajectories")
    finalselectedtestsplot = p.plotTrajectories(selected_tests=final_tests,
                                                total_plots=100,
                                                figure_size=(25, 25),
                                                tsuffix="Final Paths")
    finalselectedtestsplot.show()











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