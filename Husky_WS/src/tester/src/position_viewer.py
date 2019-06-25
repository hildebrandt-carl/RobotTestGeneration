#!/usr/bin/env python

import rospy
import math
import copy
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry

class Viewer():
    def __init__(self):
        # Start the viewer class
        rospy.loginfo('Starting position viewer!')

        # Used to save the true robots position
        self.true_pos_x = 0
        self.true_pos_y = 0

        # Used to save the robots odom position
        self.pos_x = 0
        self.pos_y = 0

        # Set the rate
        self.rate = 3

        # Subscribe to the odometry data
        self.odom_sub = rospy.Subscriber(name='odometry/filtered',
                                         data_class=Odometry,
                                         callback=self.getOdom,
                                         queue_size = 1)

        # Subscribe to the ground truth data
        self.truth_sub = rospy.Subscriber(name='ground_truth/state',
                                          data_class=Odometry,
                                          callback=self.getGroundTruth,
                                          queue_size = 1)

        # Get the parameter
        self.save_name = rospy.get_param("/savename")
        rospy.loginfo("Saving Data to: " + str(self.save_name) + "_odom.png")

        # Run the viewer class
        self.run()

    # Subscriber callback for the odometry
    def getOdom(self, data):
        # Get the robots position
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y

    # Subscriber callback for the ground truth
    def getGroundTruth(self, data):
        # Get the robots position
        self.true_pos_x = data.pose.pose.position.x
        self.true_pos_y = data.pose.pose.position.y


    # This loop runs and records data and then displays it
    def run(self):
        # Variables to determine when the test starts and ends
        started = False
        finished = False
        plotted = False

        # Used for the scatter plot
        ground_x = []
        ground_y = []
        odom_x = []
        odom_y = []
        rec_time = []

        # Used to count the frames
        count = 0

        # Set the rate
        rate = rospy.Rate(self.rate)

        # While we have no shutdown
        while not rospy.is_shutdown():

            # If the robot has not started check to see if its moved
            if not started:
                dx = (0 - self.pos_x)
                dy = (0 - self.pos_y)
                if math.sqrt(dx**2 + dy**2) > 0.25:
                    started = True
                    rospy.loginfo("Test Started")
            # The robot has started
            else:
                # Check if the robot has finished ends at position (10,0)
                if not finished:
                    dx = (10 - self.pos_x)
                    dy = (0 - self.pos_y)
                    if math.sqrt(dx**2 + dy**2) < 0.25:
                        finished = True
                        rospy.loginfo("Test Ended")

            # If the robot is currently moving
            if started and not finished:

                # Increment the frame count by t = 1/rate
                count += 1.0/self.rate

                # Copy the data to remove the chance of having corrupt data
                g_x = copy.copy(self.true_pos_x)
                g_y = copy.copy(self.true_pos_y)
                o_x = copy.copy(self.pos_x)
                o_y = copy.copy(self.pos_y)

                # Append the data for plotting
                ground_x.append(g_x)
                ground_y.append(g_y)
                odom_x.append(o_x)
                odom_y.append(o_y)

                # Save the time for each recording
                rec_time.append(count)

                rospy.loginfo("Recording Data")

            # Plotting once the robot has finished
            if finished and not plotted:
                # Show the robots position
                fig = plt.figure(1)
                marker_size=15
                cmwin = plt.get_cmap("winter")
                cmhot = plt.get_cmap("autumn")
                plt.plot(ground_x, ground_y, color='red', marker='o', linestyle='-', markersize=3,  label="ground truth position")
                plt.plot(odom_x, odom_y, color='green', marker='o', linestyle='-', markersize=3, label="odom position")
                #plt.scatter(scatter_x, scatter_z, c=scatter_y, cmap=cmhot, marker='.',edgecolors="face")
                plt.title("Position Over Time")
                plt.xlabel("X position")
                plt.ylabel("Y position")
                plt.xlim(-2, 12)
                plt.ylim(-7, 7)
                plt.legend()

                # plt.show()
                plt.savefig('/home/autosoftlab/Desktop/' + str(self.save_name) + str("_odom.png"))

                # Shutdown as there is nothing left to do
                rospy.signal_shutdown("Data collection done")

            # Sleep
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('position_viwer_node')
    try:
        viewer = Viewer()
    except rospy.ROSInterruptException:
        pass
