import glob
import matplotlib.pyplot as plt
import math
from processResultsUtils import get_numbers_from_string
from mpl_toolkits.mplot3d import Axes3D
import re
import copy
from processResultsUtils import lineseg_dist
import numpy as np

# Define where the tests are stored
file_location = "../AnafiSimulation/TestingAnafi/Indoor/"

systems = ["simulation"]

# Used to controll plotting
plotting_individual = True
potting_deviation = False

# List used to save all the results for processing later
all_data = []

# This point is what is rotated to by all others
set_point = None


if plotting_individual:
    fig = plt.figure()
    # ax = Axes3D(fig)

# Used to save this tests information
data = {}
data["filename"] = copy.deepcopy("test")

# Load the test
expected_x = [1, 1, 0, 0, 1, 1, 0, 0]
expected_y = [0, 1, 1, 0, 0, 1, 1, 0]
expected_z = [1.25,1.25,1.25,1.25,1.25,1.25,1.25,1.25]

expected_beh = [expected_x, expected_y, expected_z]
# Save the expected behavior
data["expected"] = copy.deepcopy(expected_beh)

# Plot the expected behavior
if plotting_individual:
    plt.plot(expected_beh[0], expected_beh[1], label="Expected Behavior")
    # ax.plot3D(expected_beh[0], expected_beh[1], expected_beh[2], label="Expected Behavior")

# For each system type
for system in systems:

    # Print which file is being processed
    file_name = file_location + system + "_output_nogps_actual.txt"
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
        if "anafi4k.worldPosition.x" in line:
            print(line)
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                longitude.append(num)
        if "anafi4k.worldPosition.y" in line:
            print(line)
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                latitude.append(num)
        if "anafi4k.worldPosition.z" in line:
            print(line)
            num = get_numbers_from_string(line)[0]
            if num - 500 != 0:
                altitude.append(num)

    # Close the file    
    file.close()

    # Make sure that there are the same number of longitude, lattitude and altitudes
    assert(len(longitude) == len(latitude) == len(altitude))

    # Compute the change in long, lat and alt
    new_longitude, new_latitude, new_altitude = [], [], []

    lg_init, lt_init, at_init = longitude[0], latitude[0], altitude[0]
    for j in range(0, len(longitude)):
        # Compute the change in long, late and alt
        new_longitude.append(longitude[j])
        new_latitude.append(latitude[j])
        new_altitude.append(altitude[j])

    print("Final Longitude: " + str(new_longitude[-1]))
    print("Final Latitude: " + str(new_latitude[-1]))
    print("Final Altitude: " + str(new_altitude[-1]))

    # Save the systems data    
    rotated_data = [new_longitude, new_latitude, new_altitude]
    data[system] = copy.deepcopy(rotated_data)
    
    # Plot the data
    if plotting_individual:
        # ax.plot3D(rotated_data[0], rotated_data[1], rotated_data[2], label=system)
        plt.plot(rotated_data[0], rotated_data[1], label=system)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')

# Save the data to the all_data array
all_data.append(data)

if plotting_individual:
    plt.legend()
    plt.show()