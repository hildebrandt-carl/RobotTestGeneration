#!/usr/bin/env python
import rospy
import numpy as np
from scipy.ndimage.interpolation import shift

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock


class VelocityCalculator():
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Save the GPS Messages
    self.x_prev = 0
    self.y_prev = 0
    self.z_prev = 0
    self.previous_time = rospy.Time()

    # Checks to see if the simulation has started
    self.started = False

    # Moving average
    self.window_size = 50
    self.windowx = np.zeros(self.window_size)
    self.windowy = np.zeros(self.window_size)
    self.windowz = np.zeros(self.window_size)

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()

    # Create the subscribers and publishers
    self.vel_pub = rospy.Publisher('/uav/sensors/velocity', Vector3, queue_size=1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

    # Run the communication node
    self.Process()

  # This is the main loop of this class
  def Process(self):
    # Set the rate
    rate = rospy.Rate(1000)

    # While running
    while not rospy.is_shutdown():

      if self.started:
        pass

      # While we are waiting for our rate
      while self.current_time.to_sec() - self.prev_time_check.to_sec() < self.dt:
        # Sleep any excess time
        rate.sleep()
        # Check if ROS has shut down
        if rospy.is_shutdown():
          break
        
      # Save the start of the new loop
      self.prev_time_check = self.current_time


  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock

  # Reject any outliers greater than 2 standard deviations away
  def reject_outliers(self, data, m=2):
    return data[abs(data - np.mean(data)) <= m * np.std(data)]

  # Call back to get the GPS data
  def get_gps(self, gps_msg):
    # Get the GPS data
    x = gps_msg.position.x
    y = gps_msg.position.y
    z = gps_msg.position.z

    # Calculate dt
    dt = self.current_time.to_sec() - self.previous_time.to_sec()
    
    # Calculate the current velocity
    if dt != 0:
      xvel = (x - self.x_prev)/dt
      yvel = (y - self.y_prev)/dt
      zvel = (z - self.z_prev)/dt

      # Push the velocity into the moving window
      self.windowx = shift(self.windowx, -1, cval=xvel)
      self.windowy = shift(self.windowy, -1, cval=yvel)
      self.windowz = shift(self.windowz, -1, cval=zvel)
      print(self.windowx)
      print(xvel)

      # Publish the velocity (Y axis is inverted)
      veldata = Vector3(np.mean(self.reject_outliers(self.windowx)),-1 * np.mean(self.reject_outliers(self.windowy)),np.mean(self.reject_outliers(self.windowz)))
      print(veldata)
      self.vel_pub.publish(veldata)

      # print("------------------------")
      # print("xvel: " + str(xvel))
      # print("yvel: " + str(yvel))
      # print("zvel: " + str(zvel))
      # print("x: " + str(x))
      # print("y: " + str(y))
      # print("z: " + str(z))
      # print("prev x: " + str(self.x_prev))
      # print("prev y: " + str(self.y_prev))
      # print("prev z: " + str(self.z_prev))
      # print("time: " + str(self.current_time.to_sec()))
      # print("prev time: " + str(self.previous_time.to_sec()))

    # Save the previous values
    self.x_prev = x
    self.y_prev = y
    self.z_prev = z
    self.previous_time = self.current_time


    

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Velocity Calculator Starting")
    self.started = True

  # On collision reset everything
  def collision_callback(self, msg):
    self.x = 0
    self.y = 0
    self.z = 0
    self.x_prev = 0
    self.y_prev = 0
    self.z_prev = 0

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

def main():
  rospy.init_node('vel_calculator')
  try:
    calc = VelocityCalculator()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()