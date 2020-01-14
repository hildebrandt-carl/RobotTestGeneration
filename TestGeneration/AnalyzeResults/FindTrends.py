import glob
import copy
import os
import argparse

import numpy as np

from processResultsUtils import get_numbers_after_string
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error
from sklearn.linear_model import RidgeCV
from sklearn.model_selection import KFold


# Parse the input arguments
parser = argparse.ArgumentParser()
parser.add_argument('-m', '--maindirectory',
                    type=str,
                    required=True,
                    help='This is the full directory where all the results are saved')
parser.add_argument('-e', '--searchtype',
                    type=str,
                    required=True,
                    help='Select search type (random), (maxvel), (kinematic)')
parser.add_argument('-c', '--scoretype',
                    type=str,
                    required=True,
                    help='Select score type used (random), (edge), (edge90), (edge180) (learned)')
parser.add_argument('-p', '--fileprefix',
                    type=str,
                    required=True,
                    help='What was the save prefix for the test runs')
parser.add_argument('-l', '--trajectorylength',
                    type=str,
                    required=True,
                    help='Please give the trajectory length for the system')
parser.add_argument('-t', '--searchtime',
                    type=int,
                    required=True,
                    help='Please give the searchtime used for the system')
parser.add_argument('-s', '--saveprefix',
                    type=str,
                    required=True,
                    help='Please give the model a save prefix')
parser.add_argument('-d', '--dronetype',
                    type=str,
                    default="MIT",
                    help='List the dronetype (MIT), (ANAFI)')
args = parser.parse_args()

# Creat3 the folder based on the passed arguments
simtime = str(int(args.trajectorylength) * 9)
folder = args.fileprefix + "_" + str(args.dronetype) + "_seed10_length" + args.trajectorylength + "_nodes250_res4_beamwidth5_totaltime" + str(args.searchtime) + "_simtime" + simtime + "_searchtype_" + args.searchtype + "_scoretype_" + args.scoretype

# All the different system types which are generated using the WorldEngineSimulator
system_types = []
if args.dronetype == "MIT":
    system_types = ["speed-2_minsnap0",
                    "speed-1_minsnap0",
                    "speed2_minsnap0",
                    "speed5_minsnap0",
                    "speed10_minsnap0",
                    "speed-1_minsnap1"]
elif args.dronetype == "ANAFI":
    system_types = ["anafi_sim"]

# Get the data from the analysis files from execution
folder = args.maindirectory + folder
for system in system_types:

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
    system_mainangle = []

    if total_files == 0:
        print("No Files Found in: " + str(folder + "/maps/map*/analysis_" + system + ".txt"))
        exit()

    total_files = 20

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
                                                    the_string="Angles:",
                                                    start=True)
        flight_magnitudes = get_numbers_after_string(file_name=flight_details_file,
                                                        the_string="Out Vector Magnitude:")

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
        main_angle = []

        euler_angles = euler_angles[0]
        velocties_all = velocty[0]

        # We want out going velocities (remove the first three)
        velocties_out = copy.deepcopy(velocties_all[3:])
        velocties_in = copy.deepcopy(velocties_all[:-3])
        euler_angles = copy.deepcopy(euler_angles[3:])
        flight_angles = copy.deepcopy(flight_angles[0][1:])

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
            main_angle.append(flight_angles[0])
            flight_angles = flight_angles[1:]

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

        system_dev.extend(dev_per_waypoint[0])
        system_xangles.extend(x_angle)
        system_yangles.extend(y_angle)
        system_zangles.extend(z_angle)
        system_mainangle.extend(main_angle)
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
    system_mainangle = np.asarray(system_mainangle)
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
    system_mainangle = system_mainangle[~nan_values]
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
    system_mainangle = system_mainangle.reshape(-1, 1)
    system_dev = system_dev.reshape(-1, 1)

    # Init the data
    x_data = []
    y_data = []

    # Build the training data
    for i in range(0, len(system_dev)):
        # Create the training point
        train = [system_xvel_out[i][0], system_yvel_out[i][0], system_zvel_out[i][0],
            system_xvel_in[i][0], system_yvel_in[i][0], system_zvel_in[i][0]]   

        # Save the data
        x_data.append(train)
        y_data.append(system_dev[i])

    # Convert the data to np arrays
    x_data = np.array(x_data)
    y_data = np.array(y_data).reshape(-1)

    # Define the number of folds
    k = 10

    # K-Fold Cross Validation
    kf = KFold(n_splits=k, random_state=None, shuffle=False)

    # Print the system we are working on
    print(system)

    # Use this to keep track of the best model
    best_poly_feature = None
    best_ridgecv = None
    lowest_val_loss = np.inf
    best_deg = 0
    best_score = 0

    # Find the best degree
    for deg in range(1, 5):
        # init the validation loss
        validation_loss = 0

        # PolynomialFeatures (prepreprocessing)
        poly = PolynomialFeatures(degree=deg)
        poly_x_data = poly.fit_transform(x_data)

        # Get training and testing data from kfold
        for train_index, test_index in kf.split(x_data):

            # Get the new data split
            X_train, X_test = poly_x_data[train_index], poly_x_data[test_index]
            y_train, y_test = y_data[train_index], y_data[test_index]

            # Fit a ridge regression on the training data
            regressor = RidgeCV()
            regressor.fit(X_train, y_train)

            # Get the predictions for the test
            y_predict = regressor.predict(X_test)

            # Get the mean squared error
            mse = mean_squared_error(y_test, y_predict)
            
            # Add the MSE to the validation loss
            validation_loss += mse

        # Average the validation loss
        validation_loss /= k

        # Train on all data
        regressor = RidgeCV()
        regressor.fit(poly_x_data, y_data)
        score = regressor.score(poly_x_data, y_data)

        # Save the best model and poly features
        if lowest_val_loss > validation_loss:
            best_poly_feature = copy.deepcopy(poly)
            best_ridgecv = copy.deepcopy(regressor)
            lowest_val_loss = validation_loss
            best_deg = deg
            best_score = score

        # Print out the validation loss for this degree
        print("Validation loss (degree=" + str(deg) + "): " + str(validation_loss) + "- score: " + str(score))

    print("Best Degree: " + str(best_deg) + " - Validation Loss: " + str(lowest_val_loss) + " - Score: " + str(best_score))
    print("Saving Model")

    print("-----------------------------------------------------------------------------")
    
    if not os.path.exists("Models"):
        os.mkdir("Models")

    np.save("Models/" + args.saveprefix + "_" + str(system) + "_poly_features", best_poly_feature)
    np.save("Models/" + args.saveprefix + "_" + str(system) + "_ridgecv_model", best_ridgecv)
