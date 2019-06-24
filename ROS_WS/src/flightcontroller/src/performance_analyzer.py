#!/usr/bin/env python
import rospy
import math
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

class PerformanceTester():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)

    # Variable to set the rate
    self.rate = 2

    # Open a file writer to save the information
    file_location = "/home/autosoftlab/Desktop/RobotTestGeneration/Unity/Build/performance.txt"
    self.filehandler = open(file_location, "w") 
    
    # Create a collision number
    self.collision_number = 0

    # Init the drone position
    self.drone_pos = Vector3(0, 0, 0)

    # Init the goal positions
    self.goal_position = Vector3(0, 0, 0)
    self.goal_counter = 0

    # Used to record the time between goals:
    self.start_time = time.time()

    # Run the communication node
    self.ProcessLoop()

  # This is the main loop of this class
  def ProcessLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # While running
    while not rospy.is_shutdown():
      # Report the distance to the goal
      distance = self.calculate_distance(self.goal_position, self.drone_pos)
      self.filehandler.write("Distance to Goal " + str(self.goal_counter) + ":" + str(distance) + "\n")

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
    # Get the goal position
    x_pos = msg.x
    y_pos = msg.y
    z_pos = msg.z
    new_goal_position = Vector3(x_pos, y_pos, z_pos)

    # Check if the goal has changed
    if self.goal_position != new_goal_position:
      # Update the time to get to that goal
      end_time = time.time()
      self.filehandler.write("Time between goals: " + str(end_time - self.start_time) + "\n")
      # Update the number of goals gone through
      self.goal_counter += 1
      # Save the new goal position
      self.goal_position = new_goal_position
      # Reset the time between goals
      self.start_time = time.time()
    
  # Return the distance between two Vector3 points
  def calculate_distance(self, pos1, pos2):
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
    self.collision_number += 1
    self.filehandler.write("Collision\n") 


  def shutdown_sequence(self):
    # Close the file handler
    self.filehandler.close()
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('performance_tester_node')
  try:
    performancetester = PerformanceTester()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()