#!/usr/bin/env python
import rospy
import math
import time
import re

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

class GoalTester():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.goal_pub = rospy.Publisher("uav/input/position", Vector3, queue_size=1)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)

    # Variable to set the rate
    self.rate = 2

    # File location of the goals
    file_location = "/home/autosoftlab/Desktop/RobotTestGeneration/Unity/Build/unity_corridor.txt"

    # Create a goal number
    self.goal_number = 0

    # Init the drone position
    self.drone_pos = Vector3(0, 0, 0)

    # Init the goal positions
    self.goal_positions = []

    # The distance which a goal is accepted
    self.acceptance_distance = 10

    # Load the goal positions
    self.LoadGoalPositions(file_location)

    # Sleep to make sure the robot is ready to go
    time.sleep(5)

    # Run the communication node
    self.ControlLoop()


  # This loads the goals positions from a file
  def LoadGoalPositions(self, filename):
    # Load the file and save the data
    with open(filename, "r") as f:
      filedata = f.readlines()
    
    # For each line in the data
    for line in filedata:
      # Get the first character
      initial_char = line[0]
      # If the first character is a G save the location
      if initial_char == "G":
        # Get the goal locations
        result = re.search('[(](.*)[)]', line)
        goal_string = result.group(0)

        # Remove the first character '(' an last charcter ')' from the strong
        goal_string = goal_string[1:-1]

        # Get the goal positions
        goals = goal_string.split(',')

        # Add the goals to the final goal array (Y is inverted)
        self.goal_positions.append(Vector3(x=float(goals[0]), 
                                           y=-1*float(goals[1]),
                                           z=float(goals[2])))


  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # While running
    while not rospy.is_shutdown():
      # Check if we have listed enough goals
      if self.goal_number < len(self.goal_positions):

        # Publish the goal
        current_goal = self.goal_positions[self.goal_number]
        print(current_goal)
        self.goal_pub.publish(current_goal)

        # Calculated the distance to the goal
        distance_to_goal = self.distance(current_goal, self.drone_pos)
        print("Current Goal: x-" + str(current_goal.x) + "  y-" + str(current_goal.y) + "  z-" + str(current_goal.z))
        print("Current Pos : x-" + str(self.drone_pos.x) + "  y-" + str(self.drone_pos.y) + "  z-" + str(self.drone_pos.z))
        print("Distance: " + str(distance_to_goal))
        print("----------------------------")
      
        if distance_to_goal < self.acceptance_distance:
          self.goal_number += 1
      else:      
        print("COMPLETED")

      # Sleep any excress time
      rate.sleep()


  # Call back to get the gps data
  def get_gps(self, msg):
    x_pos = msg.position.x
    y_pos = msg.position.y
    z_pos = msg.position.z
    self.drone_pos = Vector3(x_pos, y_pos, z_pos)


  # Call back to get the position setpoints
  def set_pos(self, msg):
    self.x_setpoint = msg.x
    self.y_setpoint = msg.y
    self.z_setpoint = msg.z

  
  # Return the distance between two Vector3 points
  def distance(self, pos1, pos2):
    # Calculate the squared difference
    x_sqr = (pos1.x - pos2.x)**2
    y_sqr = (pos1.y - pos2.y)**2
    z_sqr = (pos1.z - pos2.z)**2

    # Calculate the euclidance distance
    distance = math.sqrt(x_sqr + y_sqr + z_sqr)

    # Return the distance
    return distance
  

  # On collsion reset the goal number
  def collision_callback(self, msg):
    self.goal_number = 0


  def shutdown_sequence(self):
    # Close the socket
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('goal_tester_node')
  try:
    goaltester = GoalTester()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()