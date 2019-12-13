from FigureManager import FigureManager
from UnityConverter import UnityConverter
from math import sqrt, radians, degrees, acos
import numpy as np
import random
import warnings

class ScoringSystem:

    def __init__(self, score_type="", modeldir=""):
        # Set the type of scoring you want to use
        self.scoretype = score_type
        self.modeldir=modeldir

    # Given a set of trajectories compute a score for each one based on a scoring function
    def calculate_scores(self, trajectories):

        poly_features = []
        poly_reg = []

        # Used to save the scores of each trajectory
        traj_scores = []

        # If scoring is random
        if self.scoretype == "random":
            traj_scores = self.random_score(traj=trajectories)

        # If scoring is based on being closer to the edge of the kinematic model
        elif self.scoretype == "edge":
            traj_scores = self.edge_score(traj=trajectories)

        # If scoring is based on being closer to the edge and 90deg from input
        elif self.scoretype == "edge90":
            traj_scores = self.edge_angle_score(traj=trajectories, angle=90)

        # If scoring is based on being closer to the edge and 180dge from input
        elif self.scoretype == "edge180":
            traj_scores = self.edge_angle_score(traj=trajectories, angle=180)

        # If scoring is learned
        elif self.scoretype == "learned":
            traj_scores = self.learned_score(traj=trajectories)

        # Otherwise we don't know the scoring function
        else:
            sys.exit("Error 8: The scoring function was not recognized")

        # Confirm that each trajectory has a score
        assert (len(traj_scores) == len(trajectories))

        # Return the scores
        return traj_scores

    # Defines a random scoring function.
    def random_score(self, traj=None):
        # Define the scores we want to return
        trajectory_scores = []

        # For each trajectory
        for t in traj:

            # Create a list to hold the point scores
            waypoint_trajectory_score = []

            # For each waypoint in a trajectory
            for w in t:
                
                # Generate a random score for both the linear and angular component
                s = random.uniform(0, 1)

                # Save the score for each waypoint
                waypoint_trajectory_score.append(s)

            # Save the scores
            trajectory_scores.append(sum(waypoint_trajectory_score))

        return trajectory_scores

    # Defines a edge scoring function
    def edge_score(self, traj=None):
        # Define the scores we want to return
        trajectory_scores = []

        # For each trajectory
        for t in traj:

            # Used to save the waypoint details
            velocities = []
            max_velocities = []

            # For each waypoint in a trajectory
            for w in t:

                # Get the current and max velocity
                max_velocities.append(w.get_maximum_velocity())
                cur_vel = w.get_velocity()
                velocities.append(sqrt(cur_vel[0] ** 2 + cur_vel[1] ** 2 + cur_vel[2] ** 2))
                
            # Used to save a score at each point
            waypoint_trajectory_score = []

            # Go through each of the waypoints and compute a score
            for i in range(0, len(velocities) - 1) :
                
                # get the outwards velocity
                out_vel = velocities[i + 1]

                # Score is computed based on how close to the edge it was
                waypoint_trajectory_score.append(out_vel/ max_velocities[i])

            # Save the scores
            trajectory_scores.append(sum(waypoint_trajectory_score))

        # Return the score
        return trajectory_scores

    # Defines an edge scoring function which considers angles
    def edge_angle_score(self, traj=None, angle=None):
        # Define the scores we want to return
        trajectory_scores = []

        # For each trajectory
        for t in traj:

            # Used to save the waypoint details
            velocities = []
            max_velocities = []

            # For each waypoint in a trajectory
            for w in t:

                # Get the current and max velocity
                max_velocities.append(w.get_maximum_velocity())
                velocities.append(w.get_velocity())
                
            # Used to save a score at each point
            waypoint_trajectory_score = []

            # Go through each of the waypoints and compute a score
            for i in range(0, len(velocities) - 1) :
                
                # get the outwards velocity
                in_vec = velocities[i]
                if np.all(in_vec==0):
                    in_vec = np.array([0, 0, 0.5])
                out_vec = velocities[i + 1]

                # Score is computed based on how close to the edge it was
                out_mag = sqrt(out_vec[0] ** 2 + out_vec[1] ** 2 + out_vec[2] ** 2)
                vel_score = out_mag / max_velocities[i]

                # Get a score for the angle
                ang = self.angle_between_vectors(in_vec, out_vec)
                ang_score = 1 - abs(ang - radians(angle)) / abs(radians(angle))

                # Save the score as the velocity score / angular score
                waypoint_trajectory_score.append(vel_score + ang_score)

            # Save the scores
            trajectory_scores.append(sum(waypoint_trajectory_score))

        # Return the score
        return trajectory_scores

    ## Defines a learned scoring function
    def learned_score(self, traj=None):
        # Load in your feature vector and polynomial regression model
        poly_features = np.load(self.modeldir + "_poly_features.npy", allow_pickle=True).item()
        poly_reg = np.load(self.modeldir + "_regression_model.npy", allow_pickle=True).item()

        # Used to save the trajectory score
        trajectory_scores = []

        # For each trajectory
        for t in traj:
            # We are going to convert each trajectory into a set of velocities, maximum velocities  and positions
            maximum_vel_magnitude = []
            velocities = []
            positions = []

            # For each kinematic in the trajectory
            for k in t:
                # Get the current position
                positions.append(k.get_position())

                # Get the final distance flown in 1 time step
                current_velocity = k.get_velocity()
                velocities.append(current_velocity)

                # Get the maximum velocity you could reach leaving that point
                maximum_vel_magnitude.append(k.get_maximum_velocity())

            waypoint_score = []

            # Calculate the score at each waypoint
            for score_counter in range(0, len(velocities) - 1):
                # Get the vectors going into and out of a waypoint
                in_vec = velocities[score_counter]
                out_vec = velocities[score_counter + 1]

                # If we are on the first point there so set the vectors such that the score is 0.5
                prev_largest_mag = None
                if score_counter == 0:
                    prev_largest_mag = 1
                    in_vec = [0, 0, 0.5]
                else:
                    # Get the largest magnitude at that waypoint
                    prev_largest_mag = maximum_vel_magnitude[score_counter - 1]

                # Calculate the angle
                euler_angles = self.euler_angles_between_vectors(in_vec, out_vec)

                # Used to create our sample input
                vel_x_out = out_vec[0]
                vel_y_out = out_vec[1]
                vel_z_out = out_vec[2]
                vel_x_in = in_vec[0]
                vel_y_in = in_vec[1]
                vel_z_in = in_vec[2]
                ang_x = euler_angles[0]
                ang_y = euler_angles[1]
                ang_z = euler_angles[2]

                # Create the sample data
                sample_data = np.array([ang_x, ang_y, ang_z, vel_x_out, vel_y_out, vel_z_out, vel_x_in, vel_y_in, vel_z_in]).reshape(1, -1)
                sample_data_poly = poly_features.fit_transform(sample_data)

                # Predict what this sample would do
                poly_predict = poly_reg.predict(sample_data_poly)

                # Save the final score
                waypoint_score.append(poly_predict.item())

            # Save the scores
            scores.append(sum(waypoint_score))

        # Return the score
        return trajectory_scores

    # Calculate the x,y and z angle between two vectors
    def euler_angles_between_vectors(self, a, b):
        axy = np.array([a[0], a[1]])
        bxy = np.array([b[0], b[1]])
        ayz = np.array([a[1], a[2]])
        byz = np.array([b[1], b[2]])
        axz = np.array([a[0], a[2]])
        bxz = np.array([b[0], b[2]])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            xangle = acos(np.dot(ayz, byz) / (np.linalg.norm(ayz) * np.linalg.norm(byz)))
            yangle = acos(np.dot(axz, bxz) / (np.linalg.norm(axz) * np.linalg.norm(bxz)))
            zangle = acos(np.dot(axy, bxy) / (np.linalg.norm(axy) * np.linalg.norm(bxy)))

        # Check if any of the angles is NaN. This happens when you have a 0 vector
        if np.isnan((xangle)):
            xangle = 0
        if np.isnan((yangle)):
            yangle = 0
        if np.isnan((zangle)):
            zangle = 0

        return [degrees(xangle), degrees(yangle), degrees(zangle)]

    # Calculate the magnitude ofa  vector
    def get_magnitude_vector(self, v1):
        mag = sqrt(pow(v1[0], 2) + pow(v1[1], 2) + pow(v1[2], 2))
        return mag

    # Returns a single angle in radians between vectors 'v1' and 'v2'
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    def angle_between_vectors(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # Returns the unit vector of the vector
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python/13849249#13849249
    def unit_vector(self, v):
        return v / np.linalg.norm(v)

    # Expects angles in degrees
    def calculate_angular_score(self, current_angle, best_angle, larger):
        if larger and current_angle > best_angle:
            return 1.0
        else:
            return 1.0 - min(abs(current_angle - best_angle) / 90.0, 1.0)
