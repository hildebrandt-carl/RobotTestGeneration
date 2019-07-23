import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob


file_location = "TestRunning/Results/"
file_names = glob.glob(file_location + "*.txt")

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


total_times = []
all_time_between_waypoints = []


for file_name in file_names:

    print("Proecssing: " + str(file_name))

    drone_positions = []
    final_waypoints = []
    time_between_wayponts = []
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

    if max(time_between_wayponts) > 4.5:
        print(time_between_wayponts)
        print("Total time between waypoints: " + str(sum(time_between_wayponts)))
        print("Recorded total time: " + str(total_time))
        print("")

        drone_positions = np.vstack(drone_positions)
        final_waypoints = np.vstack(final_waypoints)

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot3D(drone_positions[:,0], drone_positions[:,1], drone_positions[:,2], color='green', linestyle=":", linewidth=0.75, label='Drone Position')
        ax.plot(final_waypoints[:,0], final_waypoints[:,1], final_waypoints[:,2], color='red', linestyle=":", linewidth=0.75, label='Ideal Trajectory')
        ax.scatter(final_waypoints[:,0], final_waypoints[:,1], final_waypoints[:,2], c='red', label='Waypoints')

        ax.set_xlim([-2, 30])
        ax.set_ylim([-15, 15])
        ax.set_zlim([0, 15])
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.legend()
        plt.show()

average_total_time = sum(total_times)/len(total_times)
print("Average total time: " + str(average_total_time))

all_time_between_waypoints = np.hstack(all_time_between_waypoints)
average_time_between_waypoints = sum(all_time_between_waypoints)/len(all_time_between_waypoints)
print("Average time between waypoints: " + str(average_time_between_waypoints))