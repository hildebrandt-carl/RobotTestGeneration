from math import pi, cos

class MavlinkConverter:
    def __init__(self, save_directory):
        # Save directory
        self.save_directory = save_directory

    def mavlink_waypoint_file(self, waypoints, save_name):
        file = open(self.save_directory + save_name, "w")

        # The first line of the waypoint file
        file.write('QGC WPL 120')

        # Go through each waypoint
        for waypoint in waypoints:
 
            # Radius of the earth
            r_earth = 6378.137

            # 1 meter in degree
            m = (1.0 / ((2.0 * pi / 360.0) * r_earth)) / 1000

            # Calculate the new latitude point
            new_latitude = (waypoint[1] * m)

            # Calculate the new longitude point
            new_longitude = (waypoint[0] * m) / cos(initial_lat * (pi / 180.0))

            # Calculate the new altitude point
            new_alt = waypoint[2]

            # Save this to the Mavlink file
            initial_string = "\n0\t0\t3\t16\t0.000000\t1.000000\t0.000000\t0.000000\t"
            latitude = format(new_latitude, '.10f')
            longitude = format(new_longitude, '.10f')
            altitude = format(new_alt, '.10f')
            end_string = "1\t"

            # Save it in Mavlink format
            file.write(initial_string + latitude + '\t' + longitude + '\t' + altitude + '\t' + end_string)

        file.close()