import glob
import math
import re
import copy
import argparse

import matplotlib.pyplot as plt
import numpy as np

from processResultsUtils import get_numbers_from_string
from mpl_toolkits.mplot3d import Axes3D
from processResultsUtils import lineseg_dist

# Get the distance between two points
def distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    dz = p1[2] - p2[2]
    return math.sqrt(dx**2 + dy**2 + dz**2)


# Get all occurances of a string in a letter
# Thanks: https://stackoverflow.com/questions/13009675/find-all-the-occurrences-of-a-character-in-a-string
def findOccurrences(s, ch):
    return [i for i, letter in enumerate(s) if letter == ch]


# Rotate a point around the origin in radians
def rotate(point, angle):
    # Get the origin and the point
    ox, oy = (0, 0)
    px, py = point

    # Rotate that point around the origin
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)

    # Return the new point
    return qx, qy


# Match two lines endpoints by rotating the rotation_line
def find_best_rotation(original_line, rotation_line):

    # Get the original line and the new line
    # We are not worried about z right now
    ori_x = original_line[0]
    ori_y = original_line[1]

    line_x = rotation_line[0]
    linx_y = rotation_line[1]

    # We are trying to mimize the distance to the last point
    set_point = (ori_x[-1], ori_y[-1])

    # Rotate the output until it lines up with the test line
    rotate_amount = 0
    min_distance = math.inf

    # Find how much we need to rotate the point
    for r in range(0, 360):

        # Get the lines final point
        final_point = (line_x[-1], linx_y[-1])

        # Rotate that point r radians
        rotated_point = rotate(final_point, math.radians(r))

        # Compute the difference between this point and the final last point
        dist = math.hypot(set_point[0] - rotated_point[0], set_point[1] - rotated_point[1])

        # Save the smallest distance
        if dist < min_distance:
            rotate_amount = r
            min_distance = dist

    print("Optimal Rotation: " + str(rotate_amount) + " degrees")
    return rotate_amount


# Match two lines endpoints by rotating the rotation_line
def rotate_line(rotation_line, rotation):

    # Get the x and y components for this line
    line_x = rotation_line[0]
    line_y = rotation_line[1]

    # This is the data we will return
    rotated_x, rotated_y = [], []

    # Rotate all the points
    for xp, yp in zip(line_x, line_y):
        new_point = rotate((xp, yp), math.radians(rotation))
        rotated_x.append(new_point[0])
        rotated_y.append(new_point[1])

    # Create the final line
    new_line = [rotated_x, rotated_y, rotation_line[2]]
    return new_line


# Parse the input arguments
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--test_directory',
                    type=str,
                    required=True,
                    help='This is the directory where the tests are saved')
parser.add_argument('-d', '--test_type',
                    type=str,
                    default="both",
                    help='Convert either (simulation), (outdoor), (both)')
args = parser.parse_args()

# Define where the tests are stored
file_location = args.test_directory
print("Searching for files in: " + file_location + "/maps/map*/")


all_files = None
# Find all the flight files
if args.test_type == "simulation":
    sim_files = glob.glob(file_location + "/maps/map*/simulation_output.txt")
    all_files = sim_files
elif args.test_type == "outdoor":
    outdoor_files = glob.glob(file_location + "/maps/map*/outdoor_output.txt")
    all_files = outdoor_files
elif args.test_type == "both":
    outdoor_files = glob.glob(file_location + "/maps/map*/outdoor_output.txt")
    sim_files = glob.glob(file_location + "/maps/map*/simulation_output.txt")
    all_files = outdoor_files + sim_files
else:
    print("Unknown test_type")
    exit()

# Used to control plotting
plotting_individual = False
potting_deviation = False
plot_distance_to_goal = False
plot_goals = False

# This saves the data per test
all_data = []

# Go through all the files
for i in range(0, len(all_files)):

    # Load the test
    file_name = all_files[i]

    if plotting_individual:
        fig = plt.figure(i)
        ax = Axes3D(fig)

    # Get the test file it is associated with
    slash_locations = findOccurrences(file_name, '/')
    base_location = file_name[:slash_locations[-1]]
    test_file_name = base_location + '/test.txt'
    print("-------------------------------------------")
    print("Processing: " + str(test_file_name))
    expected_x = []
    expected_y = []
    expected_z = []

    # Used to save this tests information
    data = {}
    data["test_file_name"] = copy.deepcopy(test_file_name)
    data["file_name"] = copy.deepcopy(file_name)

    # Open the file
    file = open(test_file_name, "r")

    # For each line in the file
    for line in file:
        if "G:" in line:
            # Get the goal locations
            result = re.search('[(](.*)[)]', line)
            goal_string = result.group(0)

            # Remove the first character '(' an last charcter ')' from the strong
            goal_string = goal_string[1:-1]

            # Get the goal positions
            goals = goal_string.split(',')

            # Add the goals to the final goal array (Y is inverted in test file)
            expected_x.append(float(goals[0]))
            expected_y.append(float(goals[1]))
            # Add 1 because the drone always starts 1m off the ground
            expected_z.append(float(goals[2]) + 1)

    # Close the file    
    file.close()

    expected_line = [expected_x, expected_y, expected_z]
    # Save the expected behavior
    data["expected"] = copy.deepcopy(expected_line)

    # Plot the expected behavior
    if plotting_individual:
        ax.plot3D(expected_line[0], expected_line[1], expected_line[2], label="Expected Behavior")

    # Print which file is being processed
    print("Processing: " + str(file_name))
    print("")
    

    # Used to save the details from the test
    longitude = []
    latitude = []
    altitude = []
    time = []

    # Open the file
    file = open(file_name, "r")
    
    # For each line in the file
    for line in file:
        if "Longitude" in line:
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                longitude.append(num)
        if "Latitude" in line:
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                latitude.append(num)
        if "Altitude" in line:
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                altitude.append(num)
        if "Time" in line:
            num = get_numbers_from_string(line)[0]
            time.append(num)

    # Close the file    
    file.close()

    # Make sure that there are the same number of longitude, lattitude and altitudes
    assert(len(longitude) == len(latitude) == len(altitude))

    # This is the data containing the raw data
    raw_line = [longitude, latitude, altitude]

    # Compute the change in long, lat and alt
    delta_longitude, delta_latitude, delta_altitude = [], [], []
    test_x_data, test_y_data, test_z_data = [], [], []

    lg_init, lt_init, at_init = longitude[0], latitude[0], altitude[0]
    for j in range(0, len(longitude)):
        # Compute the change in long, late and alt
        delta_longitude.append(longitude[j] - lg_init)
        delta_latitude.append(latitude[j] - lt_init)
        delta_altitude.append(altitude[j] - at_init)

        # Covert to m
        test_x_data.append(delta_longitude[j] * 40075160.0 * math.cos(math.radians(latitude[j])) / 360.0)
        test_y_data.append(delta_latitude[j] * 40008000.0 / 360.0)
        test_z_data.append(delta_altitude[j])

    # Create the line the drone actually flew on
    actual_line = [test_x_data, test_y_data, test_z_data]

    # Find the best rotation to minimize the different between end points
    # This removes the error due to frames. The drones frame of reference is east in simulation
    # However the drones frame of reference when it is outdoors is not east
    rotation = find_best_rotation(expected_line, actual_line)

    # Rotate the line to minimize the ending points
    best_fit_line = rotate_line(actual_line, rotation)

    # Check the shapes of all the data
    print("Expected line shape (goals):\t\t" + str(np.shape(expected_line)))
    print("Raw line shape (raw long lat alt):\t" + str(np.shape(raw_line)))
    print("Actual line shape (raw x y z):\t\t" + str(np.shape(actual_line)))
    print("Best line shape (rotated x y z):\t" + str(np.shape(best_fit_line)))
    print("Time data shape (raw total time):\t" + str(np.shape(time)))
    
    # Convert to numpy array for better indexing
    expected_line = np.array(expected_line)
    raw_line = np.array(raw_line)
    actual_line = np.array(actual_line)
    best_fit_line = np.array(best_fit_line)
    time = np.array(time)

    # Plot the data
    if plotting_individual:
        ax.plot3D(best_fit_line[0, :], best_fit_line[1, :], best_fit_line[2, :], label="Test Data")
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        plt.show()

    print("")
    print("Saving data in standard format")

    # We need to compute what the current goal is for each timestep to get it into the standard format
    current_index = 0
    goal_x, goal_y, goal_z = [], [], []

    # use this for debugging
    if plot_goals:
        plt.figure()

    # For each point in the best_fit_line
    total_goals = np.shape(expected_line)[1]
    current_index = 0
    for g in range(0, total_goals):
    
        # Get the goal 

        goal = expected_line[:, g]
        dis = []    

        # For each goal
        total_points = np.shape(best_fit_line)[1]
        for p in range(current_index, total_points):

            point = best_fit_line[:, p]

            # Compute the distance between the point and the goal
            d = distance(point, goal)
            dis.append(d)

        # Get the index of the minimum
        min_index = np.argmin(dis) + current_index

        # Compute the indices we were going to that goal
        number_indices = min_index - current_index 
        current_index = max(min_index, current_index)

        # Add an extra goal for the last one
        if g == total_goals - 1:
            number_indices += 1

        # use this for debugging
        if plot_goals:
            plt.plot(dis, label="Goal " + str(g))
            plt.xlabel("Point Index")
            plt.ylabel("Distance to Goal")

        if number_indices < 0:
            print("One of your goal was never met.")
        
        # We know that the current goal is up until this index
        for i in range(0, number_indices):
            goal_x.append(goal[0])
            goal_y.append(goal[1])
            goal_z.append(goal[2])

    # use this for debugging
    if plot_goals:
        plt.show()

    # Save the goal line
    goal_line = [goal_x, goal_y, goal_z]
    print("The goal line shape:\t" + str(np.shape(goal_line)))

    # Convert goal_line to numpy array for better indexing
    goal_line = np.array(goal_line)
    final_length = np.shape(goal_line)[1]

    # We now know when the drone reaches the goal. So remove any excess datapoints
    raw_line = raw_line[:, :final_length]
    actual_line = actual_line[:, :final_length]
    best_fit_line = best_fit_line[:, :final_length]
    time = time[:final_length]

    # Save the distance between each goal and each point
    distance_to_goal = []  

    # Compute the distance to goal metric
    total_points = np.shape(best_fit_line)[1]
    for p in range(0, total_points):

        # Get the current goal:
        goal = goal_line[:, p]
        # Get the current position:
        pos = best_fit_line[:, p]

        # Compute the distance
        d = distance(pos, goal)
        distance_to_goal.append(d)

    print("The distance to goal shape:\t" + str(np.shape(distance_to_goal)))
    data["distance_to_goal"] = copy.deepcopy(distance_to_goal)

    if plot_distance_to_goal:
        plt.figure()
        plt.plot(distance_to_goal)
        plt.show()

    print("")
    print("Saving to performance file")

    # Compute the filename
    save_name = None
    if "simulation_output.txt" in file_name:
        save_name = base_location + "/performance_anafi_sim.txt"
    elif "outdoor_output" in file_name:
        save_name = base_location + "/performance_anafi_outdoor.txt"
    else:
        print("What is this file?")
        exit()

    # Open the file for writing
    f = open(save_name, "w")

    # Used to save the time between goals
    goal_start_time = 0
    goal_number = 0
    current_goal = [-1, -1, -1]

    # For each point
    total_points = np.shape(best_fit_line)[1]
    for p in range(0, total_points):

        # Get the current goal:
        goal = goal_line[:, p]
        # Get the current position:
        pos = best_fit_line[:, p]
        # Get the current time
        t = time[p]
        # Get the current distance
        d = distance_to_goal[p]

        # Check if you are at the same goal
        if not (goal==current_goal).all():
            f.write("Goal switch\n")
            f.write("Time between goals: " + str(t - goal_start_time) + "\n")
            f.write("Total Time: " + str(t) + "\n")
            f.write("-------------------------------\n")
            # Save the start time and the goal
            current_goal = goal
            goal_start_time = t
            goal_number += 1

        # Write the details
        f.write("Current Drone Position: " + str(pos[0]) + ", " + str(-1 * pos[1]) + ", " + str(pos[2]) + "\n")
        f.write("Current Goal Position: " + str(goal[0]) + ", " + str(-1 * goal[1]) + ", " + str(goal[2]) + "\n")
        f.write("Distance to Goal: " + str(goal_number) + ": " + str(d) + "\n")
        f.write("Elapsed Time: " + str(t) + "\n")
        f.write("-------------------------------\n")

        # Compute the distance
        d = distance(pos, goal)
        distance_to_goal.append(d)

    f.write("Final Reached\n")
    f.write("Goal switch\n")
    f.write("Time between goals: " + str(t - goal_start_time) + "\n")
    f.write("Total Time: " + str(t) + "\n")
    f.write("-------------------------------\n")

    # Close the file
    f.close()

    print("-------------------------------------------")