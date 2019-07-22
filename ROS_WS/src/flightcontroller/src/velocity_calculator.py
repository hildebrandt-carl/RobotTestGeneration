#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty


class VelocityCalculator():
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.vel_pub = rospy.Publisher('/uav/sensors/velocity', Vector3, queue_size=1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)

    # Set the rate
    self.rate = 10

    # Save the GPS Messages
    self.x = 0
    self.y = 0
    self.z = 0
    self.x_prev = 0
    self.y_prev = 0
    self.z_prev = 0

    # Checks to see if the simulation has started
    self.started = False

    # Run the communication node
    self.Process()

  # This is the main loop of this class
  def Process(self):
    # Set the rate
    rate = rospy.Rate(self.rate)
    dt = 1.0/self.rate

    # While running
    while not rospy.is_shutdown():

      if self.started:

        # Calculate the current velocity
        xvel = (self.x - self.x_prev)/dt
        yvel = (self.y - self.y_prev)/dt
        zvel = (self.z - self.z_prev)/dt

        # Save the previous values
        self.x_prev = self.x
        self.y_prev = self.y
        self.z_prev = self.z

        # Publish the velocity (Y axis is inverted)
        veldata = Vector3(xvel, -1 * yvel, zvel)
        self.vel_pub.publish(veldata)

      # Sleep any excess time
      rate.sleep()

  # Call back to get the GPS data
  def get_gps(self, gps_msg):
    # Get the GPS data
    self.x = gps_msg.position.x
    self.y = gps_msg.position.y
    self.z = gps_msg.position.z

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