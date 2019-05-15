import csv
from PRM import prm
import numpy as np
import matplotlib.pyplot as plt

# Robot start and end locations
r_start = [1, 10]
r_end = [19, 10]

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
    p = prm(our_map, r_start, r_end)
    p.find_valid_positions(num_nodes=30,
                           wall_thresh=0.25)
    p.plan(dist_thresh=7)

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

    # Generate map from these plots
    waypoints = p.getWaypointsFromPath(all_paths[highest_variance])
    test = p.windowMapFromWaypoints(waypoints=waypoints,
                                    window_gap=1.5)

    plt.imshow(test, cmap='binary')
    plt.xlim([0, test.shape[1] - 1])
    plt.ylim([0, test.shape[1] - 1])
    plt.show()

    
    
    

