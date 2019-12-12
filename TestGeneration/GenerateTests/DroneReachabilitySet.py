import sys

import numpy as np
from scipy.spatial import ConvexHull
from copy import copy

class DroneReachabilitySet:

    def __init__(self, robot_kinematic):
        # Save the initial drones kinematic
        self.initial_drone = robot_kinematic

        # Calculate the convex hull of the points
        self.points = None
        self.hull = None

    # Calculate all possible locations the drone can be in. Return all positions considered. Save the convex hul
    def calculate_reachable_area(self, sample_resolution=5):
        # Get the maximum rotor speed
        max_rotor_speed = self.initial_drone.get_max_rotor_speed()

        # Create a sample of rotor speeds
        sample_space = np.linspace(start=0,
                                   stop=max_rotor_speed,
                                   num=sample_resolution)

        # Get a new state
        current_processing_state = copy(self.initial_drone)

        # Used to keep track of the final drone positions
        possible_drone_positions = []

        # For all combinations of motor speeds (create new states)
        for w1 in sample_space:
            for w2 in sample_space:
                for w3 in sample_space:
                    for w4 in sample_space:

                        # Create a new drone state
                        new_drone_state = copy(current_processing_state)

                        # Update the new drones position
                        new_position = new_drone_state.next_state(w1=w1, w2=w2, w3=w3, w4=w4)

                        # Append the new drone state to the possible positions
                        possible_drone_positions.append(new_position)

        # Convert the positions into three lists
        x_pos = []
        y_pos = []
        z_pos = []
        for pos in possible_drone_positions:
            x_pos.append(pos[0])
            y_pos.append(pos[1])
            z_pos.append(pos[2])

        # Save the points and the convex hull
        self.points = np.column_stack((x_pos, y_pos, z_pos))
        # Convex hull could fail for things like having no points due to flying too close to the gound
        try:
            self.hull = ConvexHull(self.points)
        except:
            self.hull = None

        # Return all possible positions
        return self.points

    # Based on the saved convex hull. Check if a waypoint is inside or not
    def is_in_hull(self, waypoints):
        '''
        Datermine if the list of points P lies inside the hull
        :return: list
        List of boolean where true means that the point is inside the convex hull
        '''
        # If there is no convex hull
        if self.hull is None:
            sys.exit("Error 6: Please call ``calculate_reachable_are'' first to compute a convex hull")

        # https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
        A = self.hull.equations[:, 0:-1]
        b = np.transpose(np.array([self.hull.equations[:, -1]]))
        isInHull = np.all((A @ np.transpose(waypoints)) <= np.tile(-b, (1, len(waypoints))), axis=0)
        return isInHull

    # Get the hull simplices for plotting
    def get_hull_simplices(self):

        # Create lists to save the data
        x_positions = []
        y_positions = []
        z_positions = []

        # For each of the hull simplices
        for s in self.hull.simplices:
            s = np.append(s, s[0])

            # Create the points
            x_positions.append(self.points[s, 0])
            y_positions.append(self.points[s, 1])
            z_positions.append(self.points[s, 2])

        # Return the position
        return x_positions, y_positions, z_positions
