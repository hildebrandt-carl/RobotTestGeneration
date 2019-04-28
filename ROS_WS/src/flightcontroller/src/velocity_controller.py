#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID

class VelocityController():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.vel_read_sub = rospy.Subscriber("/uav/sensors/velocity", Vector3, self.get_vel)
    self.vel_set_sub = rospy.Subscriber('/uav/input/velocity', Vector3, self.set_vel)
    self.att_pub = rospy.Publisher("/uav/input/attitude", Vector3, queue_size=1)
    self.thrust_pub = rospy.Publisher('/uav/input/thrust', Float64, queue_size=1)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)

    # Variable to set the rate
    self.rate = 10.0

    # Getting the PID parameters
    gains = rospy.get_param('/velocity_controller_node/gains', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp, Ki, Kd = gains['p'], gains['i'], gains['d']

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.vel_x_PID = PID(Kp, Ki, Kd, self.rate)
    self.vel_y_PID = PID(Kp, Ki, Kd, self.rate)
    self.vel_z_PID = PID(Kp, Ki, Kd, self.rate)

    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 0

    # Create the current output readings
    self.x_vel = 0
    self.y_vel = 0
    self.z_vel = 0

    # Run the communication node
    self.ControlLoop()


  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # Data we will be publishing
    z_output = Float64()
    z_output.data = 0

    # While running
    while not rospy.is_shutdown():

      # Use a PID to calculate the angle you want to hold and thrust you want
      x_output = self.vel_x_PID.get_output(self.x_setpoint, self.x_vel)
      y_output = self.vel_y_PID.get_output(self.y_setpoint, self.y_vel)
      z_output.data = self.vel_z_PID.get_output(self.z_setpoint, self.z_vel) + 9.8

      # Create and publish the data
      attitude = Vector3(x_output, y_output, 1.6)
      self.att_pub.publish(attitude) 
      self.thrust_pub.publish(z_output) 

      # Sleep any excress time
      rate.sleep()


  # Call back to get the velocity data
  def get_vel(self, vel_msg):
    self.x_vel = vel_msg.x
    self.y_vel = vel_msg.y
    self.z_vel = vel_msg.z


  # Call back to get the velocity setpoints
  def set_vel(self, vel_msg):
    self.x_setpoint = vel_msg.x
    self.y_setpoint = vel_msg.y
    self.z_setpoint = vel_msg.z

    
  # On collsion reset the PID's
  def collision_callback(self):
    self.vel_x_PID.remove_buildup()
    self.vel_y_PID.remove_buildup()
    self.vel_z_PID.remove_buildup()


  def shutdown_sequence(self):
    # Close the socket
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('velocity_controller')
  try:
    velcon = VelocityController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()