import numpy as np
from math import sin, cos, tan, pow, inf, sqrt
from mpmath import sec

class DroneKinematic:

    def __init__(self, mass, arm_length, thrust_constant, moment_constant, max_rotor_speed, inertial_properties, position=[0, 0, 0], attitude=[0, 0, 0], velocity=[0, 0, 0], angular_vel=[0, 0, 0]):
        # Keep track of the drones current position
        self.position = np.array(position, dtype=float)
        # Keep track of the drones current attitude
        self.attitude = np.array(attitude, dtype=float)
        # Keep track of the drones current velocity
        self.velocity = np.array(velocity, dtype=float)
        # Keep track of the drones current angular velocity
        self.angular_velocity = np.array(angular_vel, dtype=float)

        # Inertial properties
        self.Ixx = inertial_properties[0]
        self.Iyy = inertial_properties[1]
        self.Izz = inertial_properties[2]

        # Vehicle Properties
        self.d = arm_length
        self.m = mass

        # Motor Characteristics
        self.kf = thrust_constant
        self.km = moment_constant
        self.max_rotor_speed = max_rotor_speed

        # Physics properties
        self.g = 9.8

        # Keep track of how many time steps
        self.time_steps = 0

        # Keeps track of the largest possible velocity
        self.maximum_velocity = -1 * inf

    # Update the copy operator to make a copy of all the parameters
    def __copy__(self):
        # Create a new object
        new_ob = type(self)(mass=self.m,
                            arm_length=self.d,
                            thrust_constant=self.kf,
                            moment_constant=self.km,
                            max_rotor_speed=self.max_rotor_speed,
                            inertial_properties=[self.Ixx, self.Iyy, self.Izz],
                            position=self.get_position(),
                            attitude=self.get_attitude(),
                            velocity=self.get_velocity(),
                            angular_vel=self.get_angular_velocity())

        # Update the time of the new object
        new_ob.set_time_step(self.get_time_step())

        # Return the new object
        return new_ob

    # Return the position
    def get_position(self):
        return np.copy(self.position)

    # Return the attitude
    def get_attitude(self):
        return np.copy(self.attitude)

    # Return the velocity
    def get_velocity(self):
        return np.copy(self.velocity)

    # Return the angular velocity
    def get_angular_velocity(self):
        return np.copy(self.angular_velocity)

    # Return the time
    def get_time_step(self):
        return self.time_steps

    # Return the maximum velocity
    def get_maximum_velocity(self):
        return self.maximum_velocity

    # Return the max rotor speed
    def get_max_rotor_speed(self):
        return self.max_rotor_speed

    # Set the position
    def set_position(self, position):
        np.copyto(self.position, position)

    # Set the attitude
    def set_attitude(self, attitude):
        np.copyto(self.attitude, attitude)

    # Set the velocity
    def set_velocity(self, velocity):
        np.copyto(self.velocity, velocity)

    # Set the angular velocity
    def set_angular_velocity(self, angular_vel):
        np.copyto(self.angular_velocity, angular_vel)

    # Set the time step
    def set_time_step(self, time_in):
        self.time_steps = time_in

    # Calculate where the drone will be in the next time step
    def next_state(self, w1, w2, w3, w4):
        # Calculate u values
        constant_matrix = np.array([[self.kf, self.kf, self.kf, self.kf],
                                    [0, self.d * self.kf, 0, -self.d * self.kf],
                                    [-self.d * self.kf, 0, self.d * self.kf, 0],
                                    [self.km, -self.km, self.km, -self.km]])
        rotor_speed_array = np.array([[pow(w1, 2)],
                                      [pow(w2, 2)],
                                      [pow(w3, 2)],
                                      [pow(w4, 2)]])
        u_array = np.dot(constant_matrix, rotor_speed_array)

        # Calculate w' values
        short_u_array = np.array([[u_array[1][0]],
                                  [u_array[2][0]],
                                  [u_array[3][0]]])

        inverse_I_array = np.array([[1.0 / self.Ixx, 0, 0],
                                    [0, 1.0 / self.Iyy, 0],
                                    [0, 0, 1.0 / self.Izz]])

        omega_x = self.angular_velocity[0]
        omega_y = self.angular_velocity[1]
        omega_z = self.angular_velocity[2]

        Iw_array = np.array([[((self.Iyy - self.Izz) / (self.Ixx)) * omega_y * omega_z],
                             [((self.Izz - self.Ixx) / (self.Iyy)) * omega_x * omega_z],
                             [((self.Ixx - self.Iyy) / (self.Izz)) * omega_x * omega_y]])

        derivative_omega_array = Iw_array + np.dot(inverse_I_array, short_u_array)

        # Update the angular frame velocities
        self.angular_velocity[0] += derivative_omega_array[0][0]
        self.angular_velocity[1] += derivative_omega_array[1][0]
        self.angular_velocity[2] += derivative_omega_array[2][0]

        # update the omega values
        omega_x = self.angular_velocity[0]
        omega_y = self.angular_velocity[1]
        omega_z = self.angular_velocity[2]

        # Calculate the rpy' values
        omega_array = np.array([[omega_x],
                                [omega_y],
                                [omega_z]])

        r = self.attitude[0]
        p = self.attitude[1]
        y = self.attitude[2]

        rpy_derivative_rotation_array = np.array([[1, sin(r) * tan(p), cos(r) * tan(p)],
                                                  [0, cos(r), -1 * sin(r)],
                                                  [0, sin(r) * sec(p), cos(r) * sec(p)]])

        rpy_derivative = np.dot(rpy_derivative_rotation_array, omega_array)

        # Update the roll pitch yaw values
        self.attitude[0] += rpy_derivative[0][0]
        self.attitude[1] += rpy_derivative[1][0]
        self.attitude[2] += rpy_derivative[2][0]

        # Update the rpy values
        r = self.attitude[0]
        p = self.attitude[1]
        y = self.attitude[2]

        # Calculate acceleration array
        gravity_array = np.array([[0],
                                  [0],
                                  [-1 * self.g]])

        rotation_array = np.array([[cos(r) * cos(y) * sin(p) + sin(r) * sin(y)],
                                   [cos(r) * sin(p) * sin(y) - cos(y) * sin(r)],
                                   [cos(p) * cos(r)]])

        acceleration_array = gravity_array + (1.0 / self.m) * rotation_array * u_array[0]

        # Update the velocity and acceleration of the vehicle
        self.velocity[0] += acceleration_array[0][0]
        self.velocity[1] += acceleration_array[1][0]
        self.velocity[2] += acceleration_array[2][0]

        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]
        self.position[2] += self.velocity[2]

        # Update what time step this drone is at
        self.time_steps += 1

        # Return the position
        return np.copy(self.position)

    # Calculate the maximum velocity based on a set of samples from the reachability set
    def calculate_maximum_velocity(self, sample_points):
        # For each of the sample points
        for point in sample_points:
            # Calculate the distance
            dx = self.position[0] - point[0]
            dy = self.position[1] - point[1]
            dz = self.position[2] - point[2]
            distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

            # Save the largest velocity (the distance is the velocity because v=d/t and in our case t=1)
            if self.maximum_velocity < distance:
                self.maximum_velocity = distance

        return self.maximum_velocity
