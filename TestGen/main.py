import csv
from PRM import prm
from ConversionUtils import *
import numpy as np
import matplotlib.pyplot as plt

# Robot start and end locations
r_start = [1, 25]
r_end = [49, 25]

# Get the map
for map_num in range(1, 2):
    # Load the map file
    filename = "maps/map" + str(map_num) + ".csv"
    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        # get all the rows as a list
        our_map = list(reader)
        # transform data into numpy array
        our_map = np.array(our_map).astype(float)

    # Generate the prm map
    p = prm(map_in=our_map,
            start_pos=r_start,
            end_pos=r_end)
    p.find_valid_positions(num_nodes=100,
                           wall_thresh=0.25)
    p.plan(dist_thresh=10)

    # Find paths
    path_details = p.findShortestPath()
    path = path_details['Path']
    print(path)
    edge_headings = p.generateLineHeadings(path)
    paths = [path]
    print("Shortest path: " + str(path))

    # Find all paths
    all_paths = p.findAllPaths()
    print("Total unique paths found: " + str(len(all_paths)))
    scores = p.scoreAllPaths(all_paths)
    highest_variance = np.argmax(scores)
    lowest_variance = np.argmin(scores)
    highest_variance_path = [all_paths[highest_variance]]
    lowest_variance_path = [all_paths[lowest_variance]]

    print("Shortest Line Headings: " + str(p.generateLineHeadings(all_paths[0])))
    print("Lowest Variance Headings: " + str(p.generateLineHeadings(lowest_variance_path[0])))
    print("Highest Variance Headings: " + str(p.generateLineHeadings(highest_variance_path[0])))

    p.plot(all_paths, "All Paths")
    p.plot([all_paths[0]], "Shortest Path")
    p.plot(highest_variance_path, "Highest Variance")
    p.plot(lowest_variance_path, "Lowest Variance")

    # Generate window map from these plots
    waypoints = p.getWaypointsFromPath(all_paths[highest_variance])
    window_test = p.windowMapFromWaypoints(waypoints=waypoints,
                                           window_gap=8,
                                           min_wall_distance=2)

    # Generate corridor map from these plots
    corridor_test = p.corridorMapFromWaypoints(waypoints=waypoints,
                                               corridor_gap=2)

    # Show the map 1
    plt.imshow(window_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, window_test.shape[1] - 1])
    plt.ylim([0, window_test.shape[1] - 1])
    plt.show()

    # Show the map 2
    plt.imshow(corridor_test, cmap='binary')
    plt.scatter(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1])
    plt.plot(np.stack(waypoints)[:, 0], np.stack(waypoints)[:, 1], color='red')
    plt.xlim([0, corridor_test.shape[1] - 1])
    plt.ylim([0, corridor_test.shape[1] - 1])
    plt.show()

    walls1 = pythonMaptoWalls(map=window_test,
                              distance_threshold=1)
    walls2 = pythonMaptoWalls(map=window_test,
                              distance_threshold=5)
    walls3 = pythonMaptoWalls(map=corridor_test,
                              distance_threshold=1)
    walls4 = pythonMaptoWalls(map=corridor_test,
                              distance_threshold=5)

    wallsToUnityFile(walls1, waypoints, "/home/autosoftlab/Desktop/window1")
    wallsToUnityFile(walls2, waypoints, "/home/autosoftlab/Desktop/window5")
    wallsToUnityFile(walls3, waypoints, "/home/autosoftlab/Desktop/corridor1")
    wallsToUnityFile(walls4, waypoints, "/home/autosoftlab/Desktop/corridor5")

