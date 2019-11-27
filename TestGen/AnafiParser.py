import glob
import matplotlib.pyplot as plt
import math
from processResultsUtils import get_numbers_from_string
from mpl_toolkits.mplot3d import Axes3D
import re
import copy
from processResultsUtils import lineseg_dist
import numpy as np

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

# Define where the tests are stored
file_location = "../AnafiSimulation/TestingAnafi/test/maps"

# Find all the flight files
analysis_file_names = glob.glob(file_location + "/map1/test.txt")
total_files = len(analysis_file_names)

systems = ["simulation", "outdoor"]

# Used to controll plotting
plotting_individual = False
potting_deviation = False

# List used to save all the results for processing later
all_data = []

# This point is what is rotated to by all others
set_point = None

# Go through all the files
for i in range(0, total_files):

    if plotting_individual:
        fig = plt.figure(i)
        ax = Axes3D(fig)

    # Load the test
    file_name = analysis_file_names[i]
    print("-------------------------------------------")
    print("Processing: " + str(file_name))
    print("-------------------------------------------")
    expected_x = []
    expected_y = []
    expected_z = []

    # Used to save this tests information
    data = {}
    data["filename"] = copy.deepcopy(file_name)

    # Open the file
    file = open(file_name, "r")

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
            expected_z.append(float(goals[2]))

    # Close the file    
    file.close()

    expected_beh = [expected_x, expected_y, expected_z]
    # Save the expected behavior
    data["expected"] = copy.deepcopy(expected_beh)

    # Plot the expected behavior
    if plotting_individual:
        ax.plot3D(expected_beh[0], expected_beh[1], expected_beh[2], label="Expected Behavior")

    # For each system type
    for system in systems:
    
        # Print which file is being processed
        file_name = analysis_file_names[i][:-8] + system + "_output.txt"
        print("-------------------------------------------")
        print("Processing: " + str(file_name))
        print("-------------------------------------------")

        # Used to save the details from the test
        longitude = []
        latitude = []
        altitude = []

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

        # Close the file    
        file.close()

        # Make sure that there are the same number of longitude, lattitude and altitudes
        assert(len(longitude) == len(latitude) == len(altitude))

        # Compute the change in long, lat and alt
        delta_longitude, delta_latitude, delta_altitude = [], [], []
        new_longitude, new_latitude, new_altitude = [], [], []

        lg_init, lt_init, at_init = longitude[0], latitude[0], altitude[0]
        for j in range(0, len(longitude)):
            # Compute the change in long, late and alt
            delta_longitude.append(longitude[j] - lg_init)
            delta_latitude.append(latitude[j] - lt_init)
            delta_altitude.append(altitude[j] - at_init)

            # Covert to m
            new_longitude.append(delta_longitude[j] * 40075160.0 * math.cos(math.radians(latitude[j])) / 360.0)
            new_latitude.append(delta_latitude[j] * 40008000.0 / 360.0)
            new_altitude.append(delta_altitude[j])

        print("Final Longitude: " + str(new_longitude[-1]))
        print("Final Latitude: " + str(new_latitude[-1]))
        print("Final Altitude: " + str(new_altitude[-1]))

        rotated_longitude, rotated_latitude = [], []
        # If this is the first reading. Save the final points which will be used by all others to rotate to
        if set_point == None:
            set_point = (new_longitude[-1], new_latitude[-1])
            rotated_longitude = new_longitude
            rotated_latitude = new_latitude

        # Rotate the outdoor line until it lines up with the simulation line
        else:
            # Used to save the 
            rotate_amount = 0
            min_distance = math.inf
            # Find how much we need to rotate the point
            for r_amount in range(0, 360):
                final_point = (new_longitude[-1], new_latitude[-1])
                rotated_point = rotate(final_point, math.radians(r_amount))
                # Compute the difference between this point and the final last point
                dist = math.hypot(set_point[0] - rotated_point[0], set_point[1] - rotated_point[1])
                # Save the smallest distance
                if dist < min_distance:
                    rotate_amount = r_amount
                    min_distance = dist

            print("Optimal Rotation: " + str(rotate_amount) + " degrees")
            data["rotation"] = rotate_amount


            # Rotate all the points
            for xp, yp in zip(new_longitude, new_latitude):
                new_point = rotate((xp, yp), math.radians(rotate_amount))
                rotated_longitude.append(new_point[0])
                rotated_latitude.append(new_point[1])

        # Save the systems data    
        rotated_data = [rotated_longitude, rotated_latitude, new_altitude]
        data[system] = copy.deepcopy(rotated_data)
        
        # Plot the data
        if plotting_individual:
            ax.plot3D(rotated_data[0], rotated_data[1], rotated_data[2], label=system)
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')

    # Save the data to the all_data array
    all_data.append(data)

    if plotting_individual:
        plt.legend()
        plt.show()

# For each of the data points calculate a distance from optimal line
for d in all_data:
    # For each system type
    for system in systems:
        # Used to store the deviation at each point
        deviation = []

        # For each of the points along the traversed line
        for j in range(0, len(d[system][0])):
            
            # Used to compute the distance
            min_distance = math.inf

            # Get the current position
            cur_pos = [d[system][0][j], d[system][1][j], d[system][2][j]]

            # For each line segment:
            goal = []
            prev_goal = []
            for k in range(0, len(d["expected"][0])):
                print(d["expected"][0])
                cur_goal = [d["expected"][0][k], d["expected"][1][k], d["expected"][2][k]]

                # If its not our first run
                if k != 0:
                    print(cur_pos)
                    print(cur_goal)
                    print(prev_goal)
                    print(k)
                    d = lineseg_dist(p=np.asarray(cur_pos),
                                     a=np.asarray(cur_goal),
                                     b=np.asarray(prev_goal))

                    # Save the closest distance to the expected behavior
                    if d < min_distance:
                        min_distance = d
                
                prev_goal = cur_goal

            # Save the distance to the distance array
            deviation.append(min_distance)

        # Save the devation
        d["deviation"] = deviation

        # Display the deviation
        if potting_deviation:
            fig = plt.figure()
            plt.plot(deviation)
            plt.xlabel("Point Index")
            plt.ylabel("Deviation from Optimal")
            plt.show()