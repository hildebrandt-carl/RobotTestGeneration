#!/usr/bin/env python
import rospy
import math
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock


class PerformanceTester():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 10.0
    self.dt = 1.0 / self.rate

    # Checks to see if the simulation has started
    self.started = False

    # Getting the save file parameters
    save_location = rospy.get_param("/performance_tester_node/save_location")
    save_name = rospy.get_param("/performance_tester_node/save_name")

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": save_location - " + str(save_location))
    rospy.loginfo(str(rospy.get_name()) + ": save_name - " + str(save_name))

    # File location of the goals
    file_location = save_location + "/" + save_name

    # Checks to see if the first waypoint has been crossed
    self.first_waypoint = False

    # Open a file writer to save the information
    self.filehandler = open(file_location, "w") 
    
    # Create a collision number
    self.collision_number = 0

    # Init the drone position
    self.drone_pos = Vector3(0, 0, 0)

    # Init the goal positions
    self.goal_position = Vector3(0, 0, 0)
    self.goal_counter = 0

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()

    # Used to record the time between goals:
    self.start_time = rospy.Time().to_sec()
    
    # Used to record test time
    self.total_time = 0
    self.test_start_time = rospy.Time().to_sec()

    # Create the subscribers and publishers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

    # Run the communication node
    self.ProcessLoop()

  # This is the main loop of this class
  def ProcessLoop(self):
    # Set the rate
    rate = rospy.Rate(1000)

    # While running
    while not rospy.is_shutdown():

      # If the test has started
      if self.started and self.first_waypoint:
        # Report the distance to the goal
        distance = self.calculate_distance(self.goal_position, self.drone_pos)
        self.filehandler.write("Current Drone Position: " + str(self.drone_pos.x) + ", " + str(self.drone_pos.y) + ", " + str(self.drone_pos.z) + "\n")
        self.filehandler.write("Current Goal Position: " + str(self.goal_position.x) + ", " + str(self.goal_position.y) + ", " + str(self.goal_position.z) + "\n")
        self.filehandler.write("Distance to Goal: " + str(self.goal_counter) + ": " + str(distance) + "\n")
        self.filehandler.write("Elapsed Time: " + str(self.current_time.to_sec() - self.test_start_time) + "\n")
        self.filehandler.write("-------------------------------\n")
      else:
        rospy.loginfo(str(rospy.get_name()) + ": Waiting for test to begin")
        # Reset the time
        self.start_time = self.current_time.to_sec()
        self.test_start_time = self.current_time.to_sec()

      # While we are waiting for our rate
      while self.current_time.to_sec() - self.prev_time_check.to_sec() < self.dt:
        # Sleep any excess time
        rate.sleep()
        # Check if ROS has shut down
        if rospy.is_shutdown():
          break

      # Save the start of the new loop
      self.prev_time_check = self.current_time
    
    # Close the file
    time.sleep(1)
    self.filehandler.close()

  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock

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
    print(str(self.goal_position))

    # Check if the goal has changed
    if self.goal_position != new_goal_position and self.started == True:
      self.filehandler.write("Goal switch\n")
      # Update the time to get to that goal
      end_time = self.current_time.to_sec()
      self.filehandler.write("Time between goals: " + str(end_time - self.start_time) + "\n")
      # Update the total time
      self.total_time += (end_time - self.start_time)
      # Print the total time
      self.filehandler.write("Total Time: " + str(self.total_time) + "\n")
      self.filehandler.write("-------------------------------\n")
      # Update the number of goals gone through
      self.goal_counter += 1
      # Save the new goal position
      self.goal_position = new_goal_position
      # Reset the time between goals
      self.start_time = self.current_time.to_sec()
      # Set the flag that you have gone through the first waypoint
      self.first_waypoint = True
    
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

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Performance Analyzers Started")
    self.started = True

  def shutdown_sequence(self):
    # Final goal reached
    self.filehandler.write("Final Reached\n")
    self.filehandler.write("Goal switch\n")
    # Save the final data
    end_time = self.current_time.to_sec()
    self.filehandler.write("Time between goals: " + str(end_time - self.start_time) + "\n")
    # Update the total time
    self.total_time += (end_time - self.start_time)
    # Print the total time
    self.filehandler.write("Total Time: " + str(self.total_time) + "\n")
    self.filehandler.write("-------------------------------\n")

    # Close the file handler
    self.started = False
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('performance_tester_node')
  try:
    performancetester = PerformanceTester()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()