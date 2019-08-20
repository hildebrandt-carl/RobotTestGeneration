import argparse
from math import cos, pi


parser = argparse.ArgumentParser()
parser.add_argument('-a', '--latitude',
                    default=0.0,
                    type=float,
                    help='The initial latitude')
parser.add_argument('-o', '--longitude',
                    default=0.0,
                    type=float,
                    help='The initial longitude')
parser.add_argument('-l', '--altitude',
                    default=0.0,
                    type=float,
                    help='The initial altitude')
parser.add_argument('-f', '--loadfile',
                    default="flightplan_raw.mavlink",
                    type=str,
                    help='The flight plan you want to convert')
parser.add_argument('-s', '--savefile',
                    default="flightplan.mavlink",
                    type=str,
                    help='The save name of the new file')
args = parser.parse_args()

# Open the file and read it
with open(args.loadfile, "r") as f:

    # Create a new file
    new_file = open(args.savefile, "w") 

    # Start with an index of 0
    index = 0

    # Read every line
    for line in f:
        # If the line is a waypoint file
        if line[0] == '0':
            # split the file based on tabs
            values = line.split('\t')

            # Get the new latitude
            current_latitude = float(values[8])

            # Get the new longitude
            current_longitude = float(values[9])

            # Get the new altitude
            current_altitude = float(values[10])

            # Update the co-ordinates
            new_latitude = current_latitude + args.latitude
            new_longitude = (current_longitude / cos(args.latitude * (pi / 180.0))) + args.longitude
            new_altitude = current_altitude + args.altitude

            # Write the new line to the file
            new_line = str(index)
            index += 1
            for i in range(1, 8):
                new_line += '\t' + values[i] 

            # Add the new latitude and longitude to the file
            new_line += '\t' + format(new_latitude, '.9f')
            new_line += '\t' + format(new_longitude, '.9f')
            new_line += '\t' + format(new_altitude, '.6f')

            # Add the last part to the line
            new_line += '\t' + values[11] + "\n"

            # Save the line
            new_file.write(new_line)

        else:
            # Save the line
            new_file.write(line)

# Close the new file 
new_file.close() 