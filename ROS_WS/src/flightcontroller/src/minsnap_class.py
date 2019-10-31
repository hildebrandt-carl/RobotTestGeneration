import numpy as np
import matlab.engine
from multiprocessing import Pool

from minsnap_utils import arrangeT
from minsnap_utils import minimum_snap_single_axis_close_form
from minsnap_utils import polys_vals
from minsnap_utils import minimum_snap_single_axis_corridor

class MinSnapTrajGen:
    def __init__(self, wpts, total_time, constraint_type=1, corridor=0.25, init_v=np.array([0, 0, 0]), init_a=np.array([0, 0, 0]), end_v=np.array([0, 0, 0]), end_a=np.array([0, 0, 0])):

        # Set total time and waypoints
        self.waypoints = wpts.T
        self.T = total_time

        # Save the initial state
        self.v0 = init_v
        self.a0 = init_a
        self.v1 = end_v
        self.a1 = end_a

        self.n_order = 5

        # Counter
        self.counter = 0

        # If the constraints are to go through each waypoint
        if constraint_type == 1:
            print("Minimizing Snap - Constraint go through waypoints")
            # Assign times to each waypoint based on distance
            self.ts = arrangeT(self.waypoints, self.T)

            p = Pool(3)
            input1 = self.waypoints[0, :], self.ts, self.n_order, self.v0[0], self.a0[0], self.v1[0], self.a1[0]
            input2 = self.waypoints[1, :], self.ts, self.n_order, self.v0[1], self.a0[1], self.v1[1], self.a1[1]
            input3 = self.waypoints[2, :], self.ts, self.n_order, self.v0[2], self.a0[2], self.v1[2], self.a1[2]
            results = p.map(minimum_snap_single_axis_close_form, [input1, input2, input3])

            polys_x = results[0]
            polys_y = results[1]
            polys_z = results[2]

            # Generate the trajectory polynomials
            # polys_x = minimum_snap_single_axis_close_form(self.waypoints[0, :], self.ts, self.n_order, self.v0[0], self.a0[0], self.v1[0], self.a1[0])
            # polys_y = minimum_snap_single_axis_close_form(self.waypoints[1, :], self.ts, self.n_order, self.v0[1], self.a0[1], self.v1[1], self.a1[1])
            # polys_z = minimum_snap_single_axis_close_form(self.waypoints[2, :], self.ts, self.n_order, self.v0[2], self.a0[2], self.v1[2], self.a1[2])

            # Create the trajectories based on a time sample
            self.x_final = np.array([])
            self.y_final = np.array([])
            self.z_final = np.array([])
            self.point_time = np.array([])
            self.waypoint_index = np.array([])
            # Go through each of the time segments and calculate the trajectory
            for i in range(0, polys_x.shape[1]):
                tt = np.arange(self.ts[i], self.ts[i+1], 0.01)
                # Add the x,y,z (they start at the previous waypoint and end 1 element before the end waypoint)
                self.x_final = np.concatenate([self.x_final, polys_vals(polys_x, self.ts, tt, 0)])
                self.y_final = np.concatenate([self.y_final, polys_vals(polys_y, self.ts, tt, 0)])
                self.z_final = np.concatenate([self.z_final, polys_vals(polys_z, self.ts, tt, 0)])
                self.point_time = np.concatenate([self.point_time, tt])
                wy = np.zeros((1, len(tt))).reshape(-1)
                wy[0] = 1
                self.waypoint_index = np.concatenate([self.waypoint_index, wy])

            # Add the final waypoint
            self.x_final = np.concatenate([self.x_final, np.array([self.waypoints[0][-1]])])
            self.y_final = np.concatenate([self.y_final, np.array([self.waypoints[1][-1]])])
            self.z_final = np.concatenate([self.z_final, np.array([self.waypoints[2][-1]])])
            # Add the final true index
            self.waypoint_index = np.concatenate([self.waypoint_index, np.array([1])])

        elif constraint_type == 2:
            # What radius do you want?
            r = corridor 

            print("Minimizing Snap - Stay less than " + str(r) + "m away from trajectory")
            # Re-sample mid points
            step = r
            new_waypts = np.array(self.waypoints[:, 0]).reshape(3, 1)
            for i in range(0, self.waypoints.shape[1]-1):
                x1 = self.waypoints[0, i]
                y1 = self.waypoints[1, i]
                z1 = self.waypoints[2, i]
                x2 = self.waypoints[0, i+1]
                y2 = self.waypoints[1, i+1]
                z2 = self.waypoints[2, i+1]
                n = int(np.ceil((np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2))/step)+1)
                sample_pts = np.vstack([np.linspace(x1, x2, n), np.linspace(y1, y2, n), np.linspace(z1, z2, n)])
                new_waypts = np.hstack([new_waypts, sample_pts[:, 2:]])

            # Mark the original waypoints
            self.waypoint_index = np.zeros((new_waypts.shape[1], 1))
            for i in range(0, new_waypts.shape[1]):
                for j in range(0, self.waypoints.shape[1]):
                    if (new_waypts[:, i] == self.waypoints[:, j]).all():
                        self.waypoint_index[i] = 1

            self.waypoint_index = self.waypoint_index.reshape(-1)


            self.ts = arrangeT(new_waypts, self.T)

            input1 = new_waypts[0, :], self.ts, self.n_order,  self.v0[0], self.a0[0], self.v1[0], self.a1[0], r, self.waypoint_index
            input2 = new_waypts[1, :], self.ts, self.n_order,  self.v0[0], self.a0[0], self.v1[0], self.a1[0], r, self.waypoint_index
            input3 = new_waypts[2, :], self.ts, self.n_order,  self.v0[0], self.a0[0], self.v1[0], self.a1[0], r, self.waypoint_index

            # Multi-processing
            p = Pool(3)
            results = p.map(minimum_snap_single_axis_corridor, [input1, input2, input3])

            polys_x = results[0]
            polys_y = results[1]
            polys_z = results[2]

            # Serial
            # polys_x = minimum_snap_single_axis_corridor(input1)
            # polys_y = minimum_snap_single_axis_corridor(input2)
            # polys_z = minimum_snap_single_axis_corridor(input3)

            tt = np.arange(0, self.T, 0.01)
            self.x_final = polys_vals(polys_x, self.ts, tt, 0)
            self.y_final = polys_vals(polys_y, self.ts, tt, 0)
            self.z_final = polys_vals(polys_z, self.ts, tt, 0)

            # Calculate which points represent the waypoints by calculating the distance to each point
            self.waypoint_index = np.zeros((len(self.x_final), 1)).reshape(-1)

            # For each waypoint
            for i in range(0, self.waypoints.shape[1]):
                # For each point
                min_dist = np.inf
                min_index = -1
                for j in range(0, len(self.x_final)):
                    # Calculate distance from waypoint to point
                    dist = np.sqrt((self.x_final[j] - self.waypoints[0, i])**2 + (self.y_final[j] - self.waypoints[1, i])**2 + (self.z_final[j] - self.waypoints[2, i])**2)
                    # Save min dist
                    if min_dist >= dist:
                        min_dist = dist
                        min_index = j
                # Set the waypoint
                self.waypoint_index[min_index] = 1


        else:
            print("Error: Unknown constraint type")
            exit()


    def get_next_point(self):
        # If we are requesting more than available
        if self.counter >= len(self.x_final):
            self.counter -= 1

        # Get the x,y,z co-ordinates
        x = self.x_final[self.counter]
        y = self.y_final[self.counter]
        z = self.z_final[self.counter]
        w = self.waypoint_index[self.counter]
        # Increment the counter
        self.counter += 1

        return [x,y,z], w

    def get_all_points(self):
        return self.x_final, self.y_final, self.z_final