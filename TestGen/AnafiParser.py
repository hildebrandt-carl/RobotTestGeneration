import glob
import matplotlib.pyplot as plt
import math
from processResultsUtils import get_numbers_from_string

# Define where the tests are stored
file_location = "../AnafiSimulation/TestingAnafi/test/maps/"

# Find all the flight files
analysis_file_names = glob.glob(file_location + "/map1/test.txt")
total_files = len(analysis_file_names)

# Go through all the files
for i in range(0, total_files):

    systems = ["outdoorNorth","outdoorEast","outdoorSouth","outdoorWest", "simulation"]

    fig = plt.figure(i)

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
            invert = 1
            if system == "outdoor":
                invert = -1
            new_longitude.append(delta_longitude[j] * invert * 40075160.0 * math.cos(math.radians(latitude[j])) / 360.0)
            new_latitude.append(delta_latitude[j] * invert * 40008000.0 / 360.0)
            new_altitude.append(delta_altitude[j])

        print("Final Longitude: " + str(new_longitude[-1]))
        print("Final Latitude: " + str(new_latitude[-1]))
        print("Final Altitude: " + str(new_altitude[-1]))

        plt.plot(new_longitude,new_latitude, label=system)
    
    plt.legend()
    plt.show()