#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import Vector3
from mav_msgs.msg import RateThrust
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class MinimumSnapController():

  def __init__(self):

    # Run the shutdown sequence on shutdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 1000.0
    self.dt = 1.0 / self.rate

    # Checks to see if the simulation has started
    self.started = False

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()

    # Publishers and Subscribers
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
    self.order_pub = rospy.Publisher('/order', String, queue_size=10)

    # Run the communication node
    self.ControlLoop()

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Keep track how many loops have happend
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():

      if self.started:

        # Create the message we are going to send
        msg = RateThrust()
        msg.header.stamp = rospy.get_rostime()

        # Publish the message
        order_string = "Minimum Snap Controller: %s" % rospy.get_time()
        self.order_pub.publish(order_string)

      # While we are waiting for our rate
      while self.current_time.to_sec() - self.prev_time_check.to_sec() < self.dt:
        # Sleep any excess time
        rate.sleep()
        # Check if ROS has shut down
        if rospy.is_shutdown():
          break

      # Save the start of the new loop
      self.prev_time_check = self.current_time
      loop_counter += 1


  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Minimum Snap Controller Starting")
    self.started = True
  
  # On collsion reset the PID's
  def collision_callback(self, msg):
    pass

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('minsnap_controller_node')
  try:
    minsnap = MinimumSnapController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()