import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob


file_location = "Run-07-29-19/BEBOP_depth6_nodes500_drop90/"
file_names = glob.glob(file_location + "maps/map*/performance.txt")


# https://stackoverflow.com/questions/56463412/distance-from-a-point-to-a-line-segment-in-3d-python
def get_numbers_from_string(string_var):
    # Split by space
    space_list = string_var.split(" ")
    
    # Saves numbers
    final_numbers = []

    # Try convert each word to a number
    for s in space_list:
        # Remove comma
        s = s.strip(",")
        try:
            number = float(s)
            final_numbers.append(number)
        except:
            pass

    return final_numbers


def lineseg_dist(p, a, b):

    # normalized tangent vector
    d = np.divide(b - a, np.linalg.norm(b - a))

    # signed parallel distance components
    s = np.dot(a - p, d)
    t = np.dot(p - b, d)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, 0])

    # perpendicular distance component
    c = np.cross(p - a, d)

    return np.hypot(h, np.linalg.norm(c))


test_total_times = []
test_average_deviation = []
test_average_waypoint_time = []
test_worst_waypoint_time = []
test_worst_deviation = []
test_scores = []
collision_filenames = []
failed_scores = []
failed_times_before_collision = []
failed_distance_to_goal = []
failed_current_goal = []

file_counter = 0
for file_name in file_names:
    # keep track of the file percentage complete
    file_counter += 1
    percentage_complete = round((float(file_counter) / len(file_names)) * 100 , 2)
    print("-------------------------------------------")
    print("Processing: " + str(file_name) + " = " + str(percentage_complete) + "%")
    print("-------------------------------------------")

    # Work out the folder name by removing the word performance.txt
    folder_name = file_name[:-15]

    # Used to store current test information
    current_drone_position = []
    current_elapsed_time = []
    current_waypoint_positions = []
    current_waypoint_time = []
    current_deviation = []
    current_total_time = -1

    # Initialize a dummy waypoint
    waypoint = [-np.inf, -np.inf, -np.inf]

    # First check if the test had a collision
    collision = False
    first_position_recorded = False
    current_time = 0
    current_distance_to_goal = 0
    file = open(file_name, "r")
    for line in file:
        # Find the goal positions
        if "Current Drone Position" in line:
            first_position_recorded = True
        # Check if a collision occured
        if ("Collision" in line) and first_position_recorded and not collision:
            collision_filenames.append(file_name)
            collision = True

            # Save the current time
            failed_times_before_collision.append(current_time[0])
            failed_distance_to_goal.append(current_distance_to_goal[0])
            failed_current_goal.append(current_goal)

            # Get the score of the test which failed
            new_file_name = folder_name + "details.txt"
            file = open(new_file_name, "r")
            for line in file:
                # Find the path score
                if "Path Score: " in line:
                    path_score = get_numbers_from_string(line)
                    # Save the test score
                    failed_scores.append(path_score[0])
            file.close()

        # Get the time to collision
        if "Elapsed Time" in line and not collision:
            current_time = get_numbers_from_string(line)
        # Get the time to collision
        if "Distance to Goal" in line and not collision:
            current_distance_to_goal = get_numbers_from_string(line)
        if "Current Goal Position" in line and not collision:
            current_goal = get_numbers_from_string(line)

    file.close()

    # If there was a collision dont continue processing this file
    if collision:
        print("Collision occurred please check file!")
        continue

    # Get details from the simulation data
    file = open(file_name, "r")
    for line in file:
        # Find the goal positions
        if "Current Goal Position" in line:
            goal_pos = get_numbers_from_string(line)
            if goal_pos != waypoint:
                waypoint = goal_pos
                current_waypoint_positions.append(waypoint)
        # Find the drone positions
        if "Current Drone Position" in line:
            current_position = get_numbers_from_string(line)
            current_drone_position.append(current_position)
        # Get the drones time
        if "Elapsed Time" in line:
            current_time = get_numbers_from_string(line)
            current_elapsed_time.append(current_time)
        # Get the time between goals
        if "Time between goals" in line:
            time = get_numbers_from_string(line)
            current_waypoint_time.append(time)
        # Find the total time
        if "Total Time" in line:
            current_total_time = get_numbers_from_string(line)
    file.close()

    # Make sure that each of the arrays are the same length
    # There are cases in which some of the details were printed just after the drone finished
    min_length = min(len(current_drone_position), len(current_elapsed_time))
    current_drone_position = current_drone_position[0:min_length]
    current_elapsed_time = current_elapsed_time[0:min_length]

    # Get details from the original test specifications
    new_file_name = folder_name + "details.txt"
    path_score = -1
    file = open(new_file_name, "r")
    for line in file:
        # Find the path score
        if "Path Score: " in line:
            path_score = get_numbers_from_string(line)
            # Save the test score
            test_scores.append(path_score[0])
    file.close()

    # Remove the first time_between_waypoints as this is the time to hit the first waypoint
    current_waypoint_time = current_waypoint_time[1:]
    current_waypoint_time = np.hstack(current_waypoint_time)

    # Append the average total time
    test_total_times.append(current_total_time[0])

    # Get the average distance from the optimal trajectory
    for p1i in current_drone_position:
        smallest_distance = np.inf
        # For each line segment along the optimal trajectory
        for p2i, p3i in zip(current_waypoint_positions, current_waypoint_positions[1:]):

            # Convert the lists to arrays
            p1 = np.asarray(p1i)
            p2 = np.asarray(p2i)
            p3 = np.asarray(p3i)

            # TODO: Need to check this is right
            d = lineseg_dist(p1, p2, p3)

            if d < smallest_distance:
                smallest_distance = d

        # Save the smallest distance to the deviation from optimal array
        current_deviation.append(smallest_distance)

    # Calculate the average deviation from optimal trajectory
    average_current_deviation = sum(current_deviation) / len(current_deviation)
    test_average_deviation.append(average_current_deviation)

    # Calculate the average time between waypoints
    average_time_between_waypoints = sum(current_waypoint_time) / len(current_waypoint_time)
    test_average_waypoint_time.append(average_time_between_waypoints)

    # Save the worst time and deviation
    test_worst_waypoint_time.append(max(current_waypoint_time))
    test_worst_deviation.append(max(current_deviation))

    # Save the details of that test into the correct folder
    file = open(folder_name + "analysis.txt", "w")
    file.write("Path Score: " + str(path_score) + "\n")
    file.write("Time between waypoints: " + str(current_waypoint_time) + "\n")
    file.write("Average time between waypoints: " + str(average_time_between_waypoints) + "\n")
    file.write("Total time between waypoints: " + str(sum(current_waypoint_time)) + "\n")
    file.write("Recorded total time: " + str(current_total_time) + "\n")
    file.write("Deviation from optimal trajectory: " + str(current_deviation) + "\n")
    file.write("Average deviation from optimal trajectory: " + str(average_current_deviation) + "\n")
    file.close()

    print("Path Score: " + str(path_score))
    print("Time between waypoints: " + str(current_waypoint_time))
    print("Average time between waypoints: " + str(average_time_between_waypoints))
    print("Total time between waypoints: " + str(sum(current_waypoint_time)))
    print("Recorded total time: " + str(current_total_time))
    print("Deviation from optimal trajectory: " + str(current_deviation))
    print("Average deviation from optimal trajectory: " + str(average_current_deviation))
    print("")

    # Stack the drone positions and waypoints for plotting
    d_pos = np.vstack(current_drone_position)
    w_pos = np.vstack(current_waypoint_positions)

    # Create a 3D plot of the trajectory and actual path
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(d_pos[:, 0], d_pos[:, 1], d_pos[:, 2], color='green', linestyle=":", linewidth=0.75, label='Drone Position')
    ax.plot(w_pos[:, 0], w_pos[:, 1], w_pos[:,2], color='red', linestyle=":", linewidth=0.75, label='Ideal Trajectory')
    ax.scatter(w_pos[:, 0], w_pos[:, 1], w_pos[:,2], c='red', label='Waypoints')
    ax.set_xlim([0, 30])
    ax.set_ylim([0, -30])
    ax.set_zlim([0, 15])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.title("Optimal vs. true trajectory")
    ax.legend()
    plt.savefig(folder_name + 'flight_comparison.png')
    plt.close()

    # Plot the trajectory deviation
    fig = plt.figure()
    plt.plot(current_elapsed_time, current_deviation)
    plt.ylim([0, 5])
    plt.xlabel("Time")
    plt.ylabel("Deviation")
    plt.title("Deviation from optimal trajectory")
    plt.savefig(folder_name + 'deviation.png')
    plt.close()

print("")
print("-------------------------------------------")
print("-----------Completed Processing------------")
print("-------------------------------------------")
print("")



print("Collisions in files:")
print("")

for col in range(0, len(collision_filenames)):
    print("File: " + str(collision_filenames[col]))
    print("Score: " + str(failed_scores[col]))
    print("Time before collision: " + str(failed_times_before_collision[col]))
    print("Distance to goal before collision: " + str(failed_distance_to_goal[col]))
    print("Current goal before fail: " + str(failed_current_goal[col]))
    print("")

print("")
print("Failed Tests")
print("Total collision occurred: " + str(len(collision_filenames)))

if len(failed_scores) > 0:
    average_fail_score = sum(failed_scores) / len(failed_scores)
    print("Average score of test which failed: " + str(average_fail_score))

    average_fail_distance_to_goal = sum(failed_distance_to_goal) / len(failed_distance_to_goal)
    print("Average distance to goal of test which failed: " + str(average_fail_distance_to_goal))
else:
    print("No tests failed")

print("")
print("Passed Tests")
average_score = sum(test_scores)/len(test_scores)
print("Average score: " + str(average_score))

average_total_time = sum(test_total_times)/len(test_total_times)
print("Average total time: " + str(average_total_time))

average_time_between_waypoints = sum(test_average_waypoint_time)/len(test_average_waypoint_time)
print("Average time between waypoints: " + str(average_time_between_waypoints))

average_deviation = sum(test_average_deviation)/len(test_average_deviation)
print("Average Deviation from Optimal: " + str(average_deviation))

worst_totaltime_index = test_total_times.index(max(test_total_times))
best_totaltime_index = test_total_times.index(min(test_total_times))
print("Worst Total Time: " + str(test_total_times[worst_totaltime_index]) + " test " + str(file_names[worst_totaltime_index]))
print("Best Total Time: " + str(test_total_times[best_totaltime_index]) + " test " + str(file_names[best_totaltime_index]))

worst_waypointtime_index = test_average_waypoint_time.index(max(test_average_waypoint_time))
best_waypointtime_index = test_average_waypoint_time.index(min(test_average_waypoint_time))
print("Worst average time between waypoints: " + str(test_average_waypoint_time[worst_waypointtime_index]) + " test " + str(file_names[worst_waypointtime_index]))
print("Best average time between waypoints: " + str(test_average_waypoint_time[best_waypointtime_index]) + " test " + str(file_names[best_waypointtime_index]))

worst_deviation_index = test_average_deviation.index(max(test_average_deviation))
best_deviation_index = test_average_deviation.index(min(test_average_deviation))
print("Worst Deviation from Optimal: " + str(test_average_deviation[worst_deviation_index]) + " test " + str(file_names[worst_deviation_index]))
print("Least Deviation from Optimal: " + str(test_average_deviation[best_deviation_index]) + " test " + str(file_names[best_deviation_index]))

# Sort the scores and reorder the test arrays for plotting:
sorted_indicies = np.argsort(test_scores)
sorted_indicies = sorted_indicies[::-1]

sorted_scores = np.array(test_scores)[sorted_indicies]
sorted_average_deviation = np.array(test_average_deviation)[sorted_indicies]
sorted_average_time = np.array(test_total_times)[sorted_indicies]
sorted_worst_deviation = np.array(test_worst_deviation)[sorted_indicies]
sorted_worst_time = np.array(test_worst_waypoint_time)[sorted_indicies]

# X axis
x_axis = np.arange(len(sorted_scores))

fig = plt.figure()
plt.scatter(x_axis, sorted_scores, label="Test Scores")
plt.xlabel("Test")
plt.ylabel("Score")
plt.legend()
plt.show()

fig = plt.figure()
plt.scatter(x_axis, sorted_average_deviation, label="Average Deviation")
plt.xlabel("Test")
plt.ylabel("Meters(m)")
plt.legend()
plt.show()

fig = plt.figure()
plt.scatter(x_axis, sorted_average_time, label="Test Total Times")
plt.xlabel("Test")
plt.ylabel("Time(s)")
plt.legend()
plt.show()

fig = plt.figure()
plt.scatter(x_axis, sorted_worst_deviation, label="Worst Deviation")
plt.xlabel("Test")
plt.ylabel("Meters(m)")
plt.legend()
plt.show()

fig = plt.figure()
plt.scatter(x_axis, sorted_worst_time, label="Worst Time Between Waypoints")
plt.xlabel("Test")
plt.ylabel("Time(s)")
plt.legend()
plt.show()