#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from pid_class import PID


class PositionController():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.vel_set_sub = rospy.Publisher('/uav/input/velocity', Vector3, queue_size=1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)

    # Variable to set the rate
    self.rate = 5.0

    # Getting the PID parameters
    gains = rospy.get_param('/position_controller_node/gains', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp, Ki, Kd = gains['p'], gains['i'], gains['d']

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.pos_x_PID = PID(Kp, Ki, Kd, self.rate)
    self.pos_y_PID = PID(Kp, Ki, Kd, self.rate)
    self.pos_z_PID = PID(Kp, Ki, Kd, self.rate)

    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 0

    # Create the current output readings
    self.x_pos = 0
    self.y_pos = 0
    self.z_pos = 0

    # Checks to see if the simulation has started
    self.started = False

    # Run the communication node
    self.ControlLoop()


  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # While running
    while not rospy.is_shutdown():

      if self.started:

        # Use a PID to calculate the velocity you want
        x_proportion = self.pos_x_PID.get_output(self.x_setpoint, self.x_pos)
        y_proportion = self.pos_y_PID.get_output(self.y_setpoint, self.y_pos)
        z_proportion = self.pos_z_PID.get_output(self.z_setpoint, self.z_pos)

        total = abs(x_proportion) + abs(y_proportion) + abs(z_proportion)

        # If we are not moving yet
        if total == 0:
          total = 1

        # Set the velocity to 10 m/s
        velocity_total = 10.0 # m/s

        # Cacluate the velocity in the x and y direction
        x_vel = velocity_total * (x_proportion/total)
        y_vel = velocity_total * (y_proportion/total)
        z_vel = velocity_total * (z_proportion/total)

        # Create and publish the data
        velocity = Vector3(x_vel, -1* y_vel, z_vel)
        self.vel_set_sub.publish(velocity)

      # Sleep any excess time
      rate.sleep()


  # Call back to get the gps data
  def get_gps(self, msg):
    self.x_pos = msg.position.x
    self.y_pos = msg.position.y
    self.z_pos = msg.position.z


  # Call back to get the position setpoints
  def set_pos(self, msg):
    self.x_setpoint = msg.x
    self.y_setpoint = msg.y
    self.z_setpoint = msg.z

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Position Controller")
    self.started = True

  # On collsion reset the PID's
  def collision_callback(self, msg):
    self.pos_x_PID.remove_buildup()
    self.pos_y_PID.remove_buildup()
    self.pos_z_PID.remove_buildup()

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('position_controller_node')
  try:
    poscon = PositionController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
