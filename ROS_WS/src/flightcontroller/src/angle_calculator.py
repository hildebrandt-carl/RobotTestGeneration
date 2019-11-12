#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock


class AngleCalculator():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Checks to see if the simulation has started
    self.started = False

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()

    # Create the subscribers and publishers
    self.att_pub = rospy.Publisher('/uav/sensors/attitude', Vector3, queue_size=1)
    self.imu_sub = rospy.Subscriber("/uav/sensors/filtered_imu", Imu, self.imu_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

    # Run the node
    self.Run()


  # This is the main loop of this class
  def Run(self):
     # Set the rate
    rate = rospy.Rate(1000)

    # While running
    while not rospy.is_shutdown():

      # If the test has started
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

  # Call back to get the GPS data
  def imu_callback(self, gps_msg):
    # Get the quarternion message
    quarternion_pose = (gps_msg.orientation.x,gps_msg.orientation.y,gps_msg.orientation.z,gps_msg.orientation.w)
    # Convert quaternion to euler
    euler = euler_from_quaternion(quarternion_pose)
    # Publish the Euler Angle
    msg = Vector3(euler[0], euler[1], euler[2])
    self.att_pub.publish(msg)

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Angle Calculator Starting")
    self.started = True

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node("Angle_Calculator_Node")
  try:
    calc = AngleCalculator()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()