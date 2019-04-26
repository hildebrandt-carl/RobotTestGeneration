#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class VelocityController():
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.vel_sub = rospy.Subscriber("/uav/sensors/velocity", Vector3, self.get_vel)
    self.vel_set_sub = rospy.Subscriber('/uav/input/velocityx', Float64, self.set_x)
    self.xangle_pub = rospy.Publisher('/uav/input/pitch_angle', Float64, queue_size=1)
    self.yangle_pub = rospy.Publisher('/uav/input/roll_angle', Float64, queue_size=1)
    self.zangle_pub = rospy.Publisher('/uav/input/yaw_rate', Float64, queue_size=1)
    self.thrust_pub = rospy.Publisher('/uav/input/thrust', Float64, queue_size=1)

    # Variable to set the rate
    self.rate = 10.0

    # Getting the PID parameters
    gains = rospy.get_param('/velocity_controller_node/gains', {'p_x': 0.01, 'i_x': 0.0, 'd_x': 0.0, 'p_y': 0.01, 'i_y': 0, 'd_y': 0, 'p_z': 0.01, 'i_z': 0, 'd_z': 0})
    self.Kp_x, self.Ki_x, self.Kd_x = gains['p_x'], gains['i_x'], gains['d_x']
    self.Kp_y, self.Ki_y, self.Kd_y = gains['p_y'], gains['i_y'], gains['d_y']
    self.Kp_z, self.Ki_z, self.Kd_z = gains['p_z'], gains['i_z'], gains['d_z']

    # Variables used for the controller
    self.integral_x = 0.0
    self.previous_error_x = 0.0
    self.integral_y = 0.0
    self.previous_error_y = 0.0
    self.integral_z = 0.0
    self.previous_error_z = 0.0

    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 0

    # Get the Velocity Messages
    self.velocity = Vector3(0,0,0)

    # Run the communication node
    self.ControlLoop()

  def set_x(self, msg):
    self.x_setpoint = msg.data

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # Data we will be publishing
    x_output = Float64()
    x_output.data = 0
    y_output = Float64()
    y_output.data = 0
    z_output = Float64()
    z_output.data = 0
    h_output = Float64()
    h_output.data = 0

    # While running
    while not rospy.is_shutdown():

      # Use a PID to calculate the angle you want to hold
      # X Velocity Controller
      error_x = self.x_setpoint - self.velocity.x
      self.integral_x = self.integral_x + error_x * dt
      derivative_x = (error_x - self.previous_error_x)/dt
      x_output.data = self.Kp_x*error_x + self.Ki_x*self.integral_x + self.Kd_x*derivative_x
      self.previous_error_x = error_x 

      # Y Velocity Controller
      error_y = self.y_setpoint - self.velocity.y
      self.integral_y = self.integral_y + error_y * dt
      derivative_y = (error_y - self.previous_error_y)/dt
      y_output.data = self.Kp_y*error_y + self.Ki_y*self.integral_y + self.Kd_y*derivative_y
      self.previous_error_y = error_y 

      # Z Velocity Controller
      error_z = self.z_setpoint - self.velocity.z
      self.integral_z = self.integral_z + error_z * dt
      derivative_z = (error_z - self.previous_error_z)/dt
      h_output.data = self.Kp_z*error_z + self.Ki_z*self.integral_z + self.Kd_z*derivative_z
      self.previous_error_z = error_z 

      # Publish the angles
      self.xangle_pub.publish(x_output)
      self.yangle_pub.publish(y_output)
      self.zangle_pub.publish(z_output)
      #self.thrust_pub.publish(h_output)

      # Sleep any excress time
      rate.sleep()

  # Call back to get the GPS data
  def get_vel(self, vel_msg):
    # Save the drones position (NED - ENU)
    self.velocity = vel_msg

  def shutdown_sequence(self):
    # Close the socket
    pass

def main():
  rospy.init_node('velocity_controller')
  try:
    velcon = VelocityController()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()