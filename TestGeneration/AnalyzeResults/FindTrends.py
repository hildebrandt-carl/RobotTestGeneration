import glob
import copy
import os
import numpy as np
import matplotlib.pyplot as plt
from processResultsUtils import get_numbers_after_string
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split


all_folders = ["initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_random",
                "initial_MIT_seed10_length5_nodes250_res4_beamwidth5_totaltime3600_simtime45_searchtype_kinematic_scoretype_random"]

all_folders = ["initial_MIT_seed10_length10_nodes250_res4_beamwidth5_totaltime3600_simtime90_searchtype_kinematic_scoretype_random"]

system_types = ["speed-2_minsnap0",
                "speed-1_minsnap0",
                "speed2_minsnap0",
                "speed5_minsnap0",
                "speed10_minsnap0",
                "speed-1_minsnap1"]

folder_home = "/home/autosoftlab/Desktop/RobotTestGeneration/TestGeneration/FinalResults/initial_run_flown/"

for f in all_folders:

    for system in system_types:

        folder = folder_home + f
        analysis_file_names = glob.glob(folder + "/maps/map*/analysis_" + system + ".txt")
        # Make sure we go in order from highest score to lowest score
        total_files = len(analysis_file_names)

        system_dev = []
        system_xangles = []
        system_yangles = []
        system_zangles = []
        system_xvel_out = []
        system_yvel_out = []
        system_zvel_out = []
        system_xvel_in = []
        system_yvel_in = []
        system_zvel_in = []

        file_counter = 0
        for file_counter in range(1, total_files + 1):

            # Create the file name
            flight_log_file = folder + "/maps/map" + str(file_counter) + "/analysis_" + system + ".txt"
            flight_details_file = folder + "/maps/map" + str(file_counter) + "/details.txt"

            # Get the data
            dev_per_waypoint = get_numbers_after_string(file_name=flight_log_file,
                                                        the_string="Maximum deviation between waypoints:")
            time_per_waypoint = get_numbers_after_string(file_name=flight_log_file,
                                                         the_string="Time between waypoints:")
            euler_angles = get_numbers_after_string(file_name=flight_details_file,
                                                    the_string="Euler Angles:")
            velocty = get_numbers_after_string(file_name=flight_details_file,
                                               the_string="Velocity")
            flight_angles = get_numbers_after_string(file_name=flight_details_file,
                                                     the_string="Angles:")
            flight_magnitudes = get_numbers_after_string(file_name=flight_details_file,
                                                         the_string="Out Vector Magnitude:")


            # Count how many minsnap corridor failed
            if dev_per_waypoint[0][0] > 12 and system == "speed-1_minsnap2":
                print("Maximum Deviation over 5m (" + str(dev_per_waypoint[0][0]) + "m): " + str(flight_log_file))
                continue

            # Get the euler angles
            x_angle = []
            y_angle = []
            z_angle = []
            x_vel_out = []
            y_vel_out = []
            z_vel_out = []
            x_vel_in = []
            y_vel_in = []
            z_vel_in = []
            euler_angles = euler_angles[0]
            velocties_all = velocty[0]

            # We want out going velocities (remove the first three)
            velocties_out = copy.deepcopy(velocties_all[3:])
            velocties_in = copy.deepcopy(velocties_all[:-3])

            # Remove the first euler angle as that is with regards to it hoevering
            euler_angles = euler_angles[3:]
            print(euler_angles)

            assert (len(velocties_out) == len(velocties_in))

            # Save the angles and velocities to correct array
            while len(euler_angles) > 0:
                # Save the x y and z angles
                x_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                y_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                z_angle.append(euler_angles[0])
                euler_angles = euler_angles[1:]
                # Save the x y and z velocities
                x_vel_out.append(velocties_out[0])
                velocties_out = velocties_out[1:]
                y_vel_out.append(velocties_out[0])
                velocties_out = velocties_out[1:]
                z_vel_out.append(velocties_out[0])
                velocties_out = velocties_out[1:]
                # Save the x y and z velocities
                x_vel_in.append(velocties_in[0])
                velocties_in = velocties_in[1:]
                y_vel_in.append(velocties_in[0])
                velocties_in = velocties_in[1:]
                z_vel_in.append(velocties_in[0])
                velocties_in = velocties_in[1:]

            # Check if there is any errors
            e1 = len(dev_per_waypoint[0]) != len(x_angle)
            e2 = len(dev_per_waypoint[0]) != len(y_angle)
            e3 = len(dev_per_waypoint[0]) != len(z_angle)
            e4 = len(dev_per_waypoint[0]) != len(x_vel_out)
            e5 = len(dev_per_waypoint[0]) != len(y_vel_out)
            e6 = len(dev_per_waypoint[0]) != len(z_vel_out)
            e7 = len(dev_per_waypoint[0]) != len(x_vel_in)
            e8 = len(dev_per_waypoint[0]) != len(y_vel_in)
            e9 = len(dev_per_waypoint[0]) != len(z_vel_in)

            if e1 or e2 or e3 or e4 or e5 or e6 or e7 or e8 or e9:
                print("Error in: " + str(flight_log_file))
                print(len(dev_per_waypoint[0]))
                print(len(x_angle))
                continue

            print(flight_details_file)
            print("x angle")
            print(x_angle)
            print("y angle")
            print(y_angle)
            print("z angle")
            print(z_angle)
            print("x vel out")
            print(x_vel_out)
            print("y vel out")
            print(y_vel_out)
            print("z vel out")
            print(z_vel_out)
            print("x vel in")
            print(x_vel_in)
            print("y vel in")
            print(y_vel_in)
            print("z vel in")
            print(z_vel_in)
            exit()

            system_dev.extend(dev_per_waypoint[0])
            system_xangles.extend(x_angle)
            system_yangles.extend(y_angle)
            system_zangles.extend(z_angle)
            system_xvel_out.extend([(temp) for temp in x_vel_out])
            system_yvel_out.extend([(temp) for temp in y_vel_out])
            system_zvel_out.extend([(temp) for temp in z_vel_out])
            system_xvel_in.extend([(temp) for temp in x_vel_in])
            system_yvel_in.extend([(temp) for temp in y_vel_in])
            system_zvel_in.extend([(temp) for temp in z_vel_in])

        # Turn all the lists into numpy arrays
        system_xangles = np.asarray(system_xangles)
        system_yangles = np.asarray(system_yangles)
        system_zangles = np.asarray(system_zangles)
        system_xvel_out = np.asarray(system_xvel_out)
        system_yvel_out = np.asarray(system_yvel_out)
        system_zvel_out = np.asarray(system_zvel_out)
        system_xvel_in = np.asarray(system_xvel_in)
        system_yvel_in = np.asarray(system_yvel_in)
        system_zvel_in = np.asarray(system_zvel_in)
        system_dev = np.asarray(system_dev)

        

        # Remove all nan values from the list
        nan_values = np.isnan(system_zangles)
        system_xangles = system_xangles[~nan_values]
        system_yangles = system_yangles[~nan_values]
        system_zangles = system_zangles[~nan_values]
        system_xvel_out = system_xvel_out[~nan_values]
        system_yvel_out = system_yvel_out[~nan_values]
        system_zvel_out = system_zvel_out[~nan_values]
        system_xvel_in = system_xvel_in[~nan_values]
        system_yvel_in = system_yvel_in[~nan_values]
        system_zvel_in = system_zvel_in[~nan_values]
        system_dev = system_dev[~nan_values]

        # Reshape into a 2D array as linear regresion expects 2D array
        system_xangles = system_xangles.reshape(-1, 1)
        system_yangles = system_yangles.reshape(-1, 1)
        system_zangles = system_zangles.reshape(-1, 1)
        system_xvel_out = system_xvel_out.reshape(-1, 1)
        system_yvel_out = system_yvel_out.reshape(-1, 1)
        system_zvel_out = system_zvel_out.reshape(-1, 1)
        system_xvel_in = system_xvel_in.reshape(-1, 1)
        system_yvel_in = system_yvel_in.reshape(-1, 1)
        system_zvel_in = system_zvel_in.reshape(-1, 1)
        system_dev = system_dev.reshape(-1, 1)

        print("")
        print("")
        print("Fitting the following to a multi linear regression problem")
        print("Training data consists of:")
        print("X Angle")
        print("Y Angle")
        print("Z Angle")
        print("X Velocity Out")
        print("Y Velocity Out")
        print("Z Velocity Out")
        print("X Velocity In")
        print("Y Velocity In")
        print("Z Velocity In")
        print("The label is the absolute deviation")

        x_data = []
        y_data = []

        # Build the training data
        for i in range(0, len(system_dev)):
            # Create the training point
            train = [system_xangles[i][0], system_yangles[i][0], system_zangles[i][0],
                     system_xvel_out[i][0], system_yvel_out[i][0], system_zvel_out[i][0],
                     system_xvel_in[i][0], system_yvel_in[i][0], system_zvel_in[i][0]]

            # Save the data
            x_data.append(train)
            y_data.append(system_dev[i])

        print("Linear Regression")
        print("")
        print("")
        print("Controller: " + str(system))
        print("Coe: XAng, YAng, ZAng, XVel Out, YVel Out, ZVel Out, XVel In, YVel In, ZVel In")
        x_data, y_data = np.array(x_data), np.array(y_data)
        model = LinearRegression().fit(x_data, y_data)
        r_sq = model.score(x_data, y_data)
        print('coefficient of determination:', r_sq)
        print('intercept:', model.intercept_)
        print('slope:', model.coef_)
        y_pred = model.predict(x_data)


        print("Polynomial Regression")
        print("Finding the best degree")

        x_train, x_test, y_train, y_test = train_test_split(x_data, y_data, test_size=0.3)

        rmses = []
        degrees = np.arange(1, 5)
        min_rmse, min_deg_con = 1e10, 0

        for deg in degrees:

            # Train features
            poly_features = PolynomialFeatures(degree=deg, include_bias=False)
            x_poly_train = poly_features.fit_transform(x_train)

            # Linear regression
            poly_reg = LinearRegression()
            poly_reg.fit(x_poly_train, y_train)

            # Compare with test data
            x_poly_test = poly_features.fit_transform(x_test)
            poly_predict = poly_reg.predict(x_poly_test)
            poly_mse = mean_squared_error(y_test, poly_predict)
            poly_rmse = np.sqrt(poly_mse)
            rmses.append(poly_rmse)

            # Cross-validation of degree
            if min_rmse > poly_rmse:
                min_rmse = poly_rmse
                min_deg_con = deg

        # Plot and present results
        print('Best degree {} with RMSE {}'.format(min_deg_con, min_rmse))

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(degrees, rmses)
        ax.set_yscale('log')
        ax.set_xlabel('Degree')
        ax.set_ylabel('RMSE')
        plt.title("Best degree for: " + str(system))
        plt.show()


        # Train features
        poly_features = PolynomialFeatures(degree=min_deg_con, include_bias=False)
        x_poly_train = poly_features.fit_transform(x_data)

        # Linear regression
        poly_reg = LinearRegression()
        model = poly_reg.fit(x_poly_train, y_data)
        r_sq = model.score(x_poly_train, y_data)
        print("Constant Controller")
        print('coefficient of determination:', r_sq)
        print("Degree: " + str(min_deg_con))
        print("Coefficients shape: " + str(np.shape(poly_reg.coef_)))
        print("Intercepts: " + str(poly_reg.intercept_))
        print("Saving the intercept and the coefficients")

        try:
            os.mkdir("Models")
        except:
            pass
        np.save("Models/poly_features_" + str(system), poly_features)
        np.save("Models/regression_model_" + str(system), poly_reg)