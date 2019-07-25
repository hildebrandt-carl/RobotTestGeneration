import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob


file_location = "BEBOP_Waypoint3_single_without_rviz/"
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


total_times = []
all_time_between_waypoints = []
all_average_deviation = []
all_average_time_between_waypoints = []


for file_name in file_names:

    print("Processing: " + str(file_name))

    drone_positions = []
    drone_elapsed_time = []
    final_waypoints = []
    time_between_wayponts = []
    dev_from_optimal = []
    total_time = -1

    waypoint = [-10, -10, -10]

    file = open(file_name, "r")
    for line in file:
        # Find the goal positions
        if "Current Goal Position" in line:
            goal_pos = get_numbers_from_string(line)
            if goal_pos != waypoint:
                waypoint = goal_pos
                final_waypoints.append(waypoint)
        # Find the drone positions
        if "Current Drone Position" in line:
            current_position = get_numbers_from_string(line)
            drone_positions.append(current_position)
        # Find the drone positions
        if "Elapsed Time" in line:
            current_time = get_numbers_from_string(line)
            drone_elapsed_time.append(current_time)
        # Find the drone positions
        if "Time between goals" in line:
            time = get_numbers_from_string(line)
            time_between_wayponts.append(time)

        # Find the total time
        if "Total Time" in line:
            total_time = get_numbers_from_string(line)

    # Remove the first time_between_waypoints as this is the time to hit the first waypoint
    time_between_wayponts = time_between_wayponts[1:]
    time_between_wayponts = np.hstack(time_between_wayponts)

    # Append the individual times between waypoints
    all_time_between_waypoints.append(time_between_wayponts)

    # Append the average total time
    total_times.append(total_time[0])

    # Get the average distance from the optimal trajectory
    for p1i in drone_positions:
        smallest_distance = np.inf
        # For each line segment along the optimal trajectory
        for p2i, p3i in zip(final_waypoints, final_waypoints[1:]):

            # Convert the lists to arrays
            p1 = np.asarray(p1i)
            p2 = np.asarray(p2i)
            p3 = np.asarray(p3i)

            # TODO: Need to check this is right
            d = lineseg_dist(p1, p2, p3)

            if d < smallest_distance:
                smallest_distance = d

        # Save the smallest distance to the deviation from optimal array
        dev_from_optimal.append(smallest_distance)

    # Calculate the average deviation from optimal trajectory
    average_dev_from_optimal = sum(dev_from_optimal) / len(dev_from_optimal)
    all_average_deviation.append(average_dev_from_optimal)

    # Calculate the average time between waypoints
    average_time_between_waypoints = sum(time_between_wayponts) / len(time_between_wayponts)
    all_average_time_between_waypoints.append(average_time_between_waypoints)


    if max(time_between_wayponts) > 6:
        print(time_between_wayponts)
        print("Total time between waypoints: " + str(sum(time_between_wayponts)))
        print("Recorded total time: " + str(total_time))
        print("Deviation from optimal trajectory: " + str(average_dev_from_optimal))
        print("Average time between waypoints: " + str(average_time_between_waypoints))
        print("Maximum time between waypoints: " + str(max(time_between_wayponts)))
        print("")

        drone_positions = np.vstack(drone_positions)
        final_waypoints = np.vstack(final_waypoints)

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot3D(drone_positions[:,0], drone_positions[:,1], drone_positions[:,2], color='green', linestyle=":", linewidth=0.75, label='Drone Position')
        ax.plot(final_waypoints[:,0], final_waypoints[:,1], final_waypoints[:,2], color='red', linestyle=":", linewidth=0.75, label='Ideal Trajectory')
        ax.scatter(final_waypoints[:,0], final_waypoints[:,1], final_waypoints[:,2], c='red', label='Waypoints')

        ax.set_xlim([0, 30])
        ax.set_ylim([0, -30])
        ax.set_zlim([0, 15])
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.legend()
        plt.show()

        fig = plt.figure()
        plt.plot(drone_elapsed_time, dev_from_optimal)
        plt.ylim([0, 5])
        plt.xlabel("Time")
        plt.ylabel("Deviation from Optimal")
        plt.show()



print("")
print("-----------------------------------")
print("")

average_total_time = sum(total_times)/len(total_times)
print("Average total time: " + str(average_total_time))

average_time_between_waypoints = sum(all_average_time_between_waypoints)/len(all_average_time_between_waypoints)
print("Average time between waypoints: " + str(average_time_between_waypoints))

overall_average_deviation = sum(all_average_deviation)/len(all_average_deviation)
print("Average Deviation from Optimal: " + str(overall_average_deviation))

worst_totaltime_index = total_times.index(max(total_times))
best_totaltime_index = total_times.index(min(total_times))
print("Worst Total Time: " + str(total_times[worst_totaltime_index]) + " test " + str(file_names[worst_totaltime_index]))
print("Best Total Time: " + str(total_times[best_totaltime_index]) + " test " + str(file_names[best_totaltime_index]))

worst_waypointtime_index = all_average_time_between_waypoints.index(max(all_average_time_between_waypoints))
best_waypointtime_index = all_average_time_between_waypoints.index(min(all_average_time_between_waypoints))
print("Worst average time between waypoints: " + str(all_average_time_between_waypoints[worst_waypointtime_index]) + " test " + str(file_names[worst_waypointtime_index]))
print("Best average time between waypoints: " + str(all_average_time_between_waypoints[best_waypointtime_index]) + " test " + str(file_names[best_waypointtime_index]))

worst_deviation_index = all_average_deviation.index(max(all_average_deviation))
best_deviation_index = all_average_deviation.index(min(all_average_deviation))
print("Worst Deviation from Optimal: " + str(all_average_deviation[worst_deviation_index]) + " test " + str(file_names[worst_deviation_index]))
print("Least Deviation from Optimal: " + str(all_average_deviation[best_deviation_index]) + " test " + str(file_names[best_deviation_index]))
