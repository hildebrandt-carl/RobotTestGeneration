#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Vector3
from mav_msgs.msg import RateThrust
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock


class AngleController():

  def __init__(self):

    # Run the shutdown sequence on shutdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Publishers and Subscribers
    self.rate_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
    self.imu_sub = rospy.Subscriber("/uav/sensors/attitude", Vector3, self.euler_angle_callback)
    self.att_sub = rospy.Subscriber("/uav/input/attitude", Vector3, self.attitude_set_callback)
    self.thrust_sub = rospy.Subscriber("/uav/input/thrust", Float64, self.thrust_callback)
    self.col_pub = rospy.Subscriber('/uav/collision', Empty, self.collision_callback)
    self.shutdown_sub = rospy.Subscriber('/test/completed', Empty, self.completed_callback)
    self.navigation_start = rospy.Subscriber('/test/started', Empty, self.start_callback)
    self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)

    # Getting the PID parameters
    gains = rospy.get_param('/angle_controller_node/gains', {'p': 0.1, 'i': 0, 'd': 0})
    Kp, Ki, Kd = gains['p'], gains['i'], gains['d']

    # Variable to set the rate
    self.rate = 20.0

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.rollPID = PID(Kp, Ki, Kd, self.rate)
    self.pitchPID = PID(Kp, Ki, Kd, self.rate)
    self.yawPID = PID(Kp, Ki, Kd, self.rate)

    # Create the setpoints
    self.roll_setpoint = 0
    self.pitch_setpoint = 0
    self.yaw_setpoint = 1.57
    self.thrust_setpoint = 0

    # Create the current output readings
    self.roll_reading = 0
    self.pitch_reading = 0
    self.yaw_reading = 0

    # Checks to see if the simulation has started
    self.started = False

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()
    self.process_loop = False

    # Run the communication node
    self.ControlLoop()


  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(100)

    # Calculate the time between intervals
    dt = 1.0/self.rate

    # While running
    while not rospy.is_shutdown():


      # Use a PID to calculate the rates you want to publish to maintain an angle
      roll_output = self.rollPID.get_output(self.roll_setpoint, self.roll_reading)
      pitch_output = self.pitchPID.get_output(self.pitch_setpoint, self.pitch_reading)
      yaw_output = self.yawPID.get_output(self.yaw_setpoint, self.yaw_reading)

      # Publish the rate message
      msg = RateThrust()
      msg.header.stamp = rospy.get_rostime()
      if self.started:
        print(str(rospy.get_name()) + " " + str(self.current_time))
        msg.thrust = Vector3(0,0,self.thrust_setpoint)
      else:
        msg.thrust = Vector3(0,0,0)
      msg.angular_rates = Vector3(roll_output,pitch_output,0)
      self.rate_pub.publish(msg)

      while self.process_loop == False:
        # Sleep any excess time
        rate.sleep()

      self.process_loop = False

  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock
    # If we should rerun the control loop
    if self.current_time.to_sec() - self.prev_time_check.to_sec() > self.rate:
      # Reset the previous time
      self.prev_time_check = self.current_time
      # Run a process loop
      self.process_loop = True

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

	# Save the new attitude readings
  def euler_angle_callback(self, msg):
    self.roll_reading = msg.x
    self.pitch_reading = msg.y
    self.yaw_reading = msg.z

	# Save the new attitude setpoints
  def attitude_set_callback(self, msg):
    # Dont allow angles greater than 0.5 for x and y
    self.roll_setpoint = max(min(msg.x, 0.5),-0.5)
    self.pitch_setpoint = max(min(msg.y, 0.5),-0.5)
    self.yaw_setpoint = msg.z

	# Save the new thrust setpoints
  def thrust_callback(self, msg):
    self.thrust_setpoint = msg.data

  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Angle Controller Starting")
    self.started = True
  
  # On collsion reset the PID's
  def collision_callback(self, msg):
    self.rollPID.remove_buildup()
    self.pitchPID.remove_buildup()
    self.yawPID.remove_buildup()

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('angle_controller')
  try:
    angcon = AngleController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()