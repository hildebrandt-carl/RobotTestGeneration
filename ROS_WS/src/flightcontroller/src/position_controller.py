#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class PositionController():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Getting the PID parameters
    stable_gains = rospy.get_param('/position_controller_node/gains/stable/', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp_s, Ki_s, Kd_s = stable_gains['p'], stable_gains['i'], stable_gains['d']

    unstable_gains = rospy.get_param('/position_controller_node/gains/unstable/', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp_u, Ki_u, Kd_u = unstable_gains['p'], unstable_gains['i'], unstable_gains['d']

    # Getting the save file parameters
    save_location = rospy.get_param("/position_controller_node/save_location")
    save_name = rospy.get_param("/position_controller_node/save_name")

    # Getting the requested speed
    self.speed = rospy.get_param('/position_controller_node/speed')

    # If the speed is set to unstable waypoint
    Kp, Ki, Kd = 0, 0, 0
    if self.speed == -2:
      Kp = Kp_u
      Ki = Ki_u
      Kd = Kd_u 
    else:
      Kp = Kp_s
      Ki = Ki_s
      Kd = Kd_s

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))
    rospy.loginfo(str(rospy.get_name()) + ": speed - " + str(self.speed))
    rospy.loginfo(str(rospy.get_name()) + ": save_location - " + str(save_location))
    rospy.loginfo(str(rospy.get_name()) + ": save_name - " + str(save_name))

    # File location of the goals
    file_location = save_location + "/" + save_name

    # Open a file writer to save the information
    self.filehandler = open(file_location, "w") 

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

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()

    # Checks to see if the simulation has started
    self.started = False

    # Create the subscribers and publishers
    self.vel_set_sub = rospy.Publisher('/uav/input/velocity', Vector3, queue_size=1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", Pose, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)
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
    rate = rospy.Rate(1000)

    # Keep track how many loops have happend
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():

      if self.started:

        # Use a PID to calculate the velocity you want
        x_proportion = self.pos_x_PID.get_output(self.x_setpoint, self.x_pos)
        y_proportion = self.pos_y_PID.get_output(self.y_setpoint, self.y_pos)
        z_proportion = self.pos_z_PID.get_output(self.z_setpoint, self.z_pos)

        # Initialize the components of the vector
        x_vel = 0
        y_vel = 0
        z_vel = 0

        # If we are in waypoint mode
        if self.speed < 0:
          # Set the velocity based on distance
          x_vel = x_proportion
          y_vel = y_proportion
          z_vel = z_proportion

        # If we are in constant speed mode
        else:
          total = abs(x_proportion) + abs(y_proportion) + abs(z_proportion)

          # If we are not moving yet
          if total == 0:
            total = 1

          # Set the velocity to in m/s
          velocity_total = self.speed # m/s

          # Calculate the velocity in the x and y direction
          x_vel = velocity_total * (x_proportion/total)
          y_vel = velocity_total * (y_proportion/total)
          z_vel = velocity_total * (z_proportion/total)

        # Create and publish the data
        velocity = Vector3(x_vel, -1* y_vel, z_vel)
        self.vel_set_sub.publish(velocity)
        order_string = "position controller: %s" % rospy.get_time()
        self.order_pub.publish(order_string)

        # Logging
        self.filehandler.write("-----------------------\n")
        self.filehandler.write("Loop Counter: " + str(loop_counter) + "\n")
        self.filehandler.write("Publishing Time: " + str(self.current_time.to_sec()) + "\n")
        self.filehandler.write("Velocity X: " + str(velocity.x) + "\n")
        self.filehandler.write("Velocity Y: " + str(velocity.y) + "\n")
        self.filehandler.write("Velocity Z: " + str(velocity.z) + "\n")

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

    # Close the file
    time.sleep(1)
    self.filehandler.close()

  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock

  # Call back to get the gps data
  def get_gps(self, msg):
    self.x_pos = msg.position.x
    self.y_pos = msg.position.y
    self.z_pos = msg.position.z

  # Call back to get the position setpoints
  def set_pos(self, msg):
    # If our set point changes reset the PID build up
    if self.x_setpoint != msg.x:
      self.pos_x_PID.remove_buildup()
      self.pos_y_PID.remove_buildup()
      self.pos_z_PID.remove_buildup()

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

	# Called on ROS shutdown
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
