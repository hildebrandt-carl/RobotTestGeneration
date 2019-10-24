import numpy as np

from minsnap_utils import arrangeT
from minsnap_utils import minimum_snap_single_axis_close_form
from minsnap_utils import polys_vals

class MinSnapTrajGen:
    def __init__(self, waypoints, total_time, init_v=np.array([0, 0, 0]), init_a=np.array([0, 0, 0]), end_v=np.array([0, 0, 0]), end_a=np.array([0, 0, 0])):

        # Set total time and waypoints
        self.waypoints = waypoints.T
        self.T = total_time

        # Save the initial state
        self.v0 = init_v
        self.a0 = init_a
        self.v1 = end_v
        self.a1 = end_a

        # Assign times to each waypoint based on distance
        self.ts = arrangeT(self.waypoints, self.T)

        self.n_order = 5

        # Counter
        self.counter = 0

        # Generate the trajectory polynomials
        polys_x = minimum_snap_single_axis_close_form(self.waypoints[0, :], self.ts, self.n_order, self.v0[0], self.a0[0], self.v1[0], self.a1[0])
        polys_y = minimum_snap_single_axis_close_form(self.waypoints[1, :], self.ts, self.n_order, self.v0[1], self.a0[1], self.v1[1], self.a1[1])
        polys_z = minimum_snap_single_axis_close_form(self.waypoints[2, :], self.ts, self.n_order, self.v0[2], self.a0[2], self.v1[2], self.a1[2])

        # Create the trajectories based on a time sample
        self.x_final = np.array([])
        self.y_final = np.array([])
        self.z_final = np.array([])
        self.point_time = np.array([])
        for i in range(0, polys_x.shape[1]):
            tt = np.arange(self.ts[i], self.ts[i+1], 0.01)
            self.x_final = np.concatenate([self.x_final, polys_vals(polys_x, self.ts, tt, 0)])
            self.y_final = np.concatenate([self.y_final, polys_vals(polys_y, self.ts, tt, 0)])
            self.z_final = np.concatenate([self.z_final, polys_vals(polys_z, self.ts, tt, 0)])
            self.point_time = np.concatenate([self.point_time, tt])

    def get_next_point(self):
        # Get the x,y,z co-ordinates
        x = self.x_final[self.counter]
        y = self.y_final[self.counter]
        z = self.z_final[self.counter]
        # Increment the counter
        self.counter += 1

        return [x,y,z]

    def get_all_points(self):
        return self.x_final, self.y_final, self.z_final