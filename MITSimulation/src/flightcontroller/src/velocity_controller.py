#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class VelocityController():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Getting the PID parameters
    gains = rospy.get_param('/velocity_controller_node/gains', {'p_xy': 1, 'i_xy': 0.0, 'd_xy': 0.0, 'p_z': 1, 'i_z': 0.0, 'd_z': 0.0})
    Kp_xy, Ki_xy, Kd_xy = gains['p_xy'], gains['i_xy'], gains['d_xy']
    Kp_z, Ki_z, Kd_z = gains['p_z'], gains['i_z'], gains['d_z']

    # Getting the save file parameters
    save_location = rospy.get_param("/velocity_controller_node/save_location")
    save_name = rospy.get_param("/velocity_controller_node/save_name")

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p_xy - " + str(Kp_xy))
    rospy.loginfo(str(rospy.get_name()) + ": i_xy - " + str(Ki_xy))
    rospy.loginfo(str(rospy.get_name()) + ": d_xy - " + str(Kd_xy))
    rospy.loginfo(str(rospy.get_name()) + ": p_z - " + str(Kp_z))
    rospy.loginfo(str(rospy.get_name()) + ": i_z - " + str(Ki_z))
    rospy.loginfo(str(rospy.get_name()) + ": d_z - " + str(Kd_z))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))
    rospy.loginfo(str(rospy.get_name()) + ": save_location - " + str(save_location))
    rospy.loginfo(str(rospy.get_name()) + ": save_name - " + str(save_name))

    # File location of the goals
    file_location = save_location + "/" + save_name

    # Open a file writer to save the information
    self.filehandler = open(file_location, "w") 

    # Creating the PID's
    self.vel_x_PID = PID(Kp_xy, Ki_xy, Kd_xy, self.rate)
    self.vel_y_PID = PID(Kp_xy, Ki_xy, Kd_xy, self.rate)
    self.vel_z_PID = PID(Kp_z, Ki_z, Kd_z, self.rate)
    
    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 0

    # Create the current output readings
    self.x_vel = 0
    self.y_vel = 0
    self.z_vel = 0

    # Checks to see if the simulation has started
    self.started = False

    # Used to save the clock
    self.current_time = rospy.Time()
    self.prev_time_check = rospy.Time()
    

    # Create the subscribers and publishers
    self.vel_read_sub = rospy.Subscriber("/uav/sensors/velocity", Vector3, self.get_vel)
    self.vel_set_sub = rospy.Subscriber('/uav/input/velocity', Vector3, self.set_vel)
    self.att_pub = rospy.Publisher("/uav/input/attitude", Vector3, queue_size=1)
    self.thrust_pub = rospy.Publisher('/uav/input/thrust', Float64, queue_size=1)
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

    # Data we will be publishing
    z_output = Float64()
    z_output.data = 0

    # While running
    while not rospy.is_shutdown():

      if self.started:

        # Use a PID to calculate the angle you want to hold and thrust you want
        x_output = self.vel_x_PID.get_output(self.x_setpoint, self.x_vel)
        y_output = self.vel_y_PID.get_output(self.y_setpoint, self.y_vel)
        z_output.data = self.vel_z_PID.get_output(self.z_setpoint, self.z_vel) + 9.8

        # Create and publish the data (0 yaw)
        attitude = Vector3(y_output, x_output, -1.57)
        self.att_pub.publish(attitude) 
        self.thrust_pub.publish(z_output)
        order_string = "Velocity Controller: %s" % rospy.get_time()
        self.order_pub.publish(order_string) 

        # Limit the thrust to the drone
        if z_output.data < 0:
          z_output.data = 0
        if z_output.data > 20:
          z_output.data = 20

        # Logging
        self.filehandler.write("-----------------------\n")
        self.filehandler.write("Loop Counter: " + str(loop_counter) + "\n")
        self.filehandler.write("Publishing Time: " + str(self.current_time.to_sec()) + "\n")
        self.filehandler.write("Thrust Z: " + str(z_output) + "\n")
        self.filehandler.write("Attitude X: " + str(attitude.x) + "\n")
        self.filehandler.write("Attitude Y: " + str(attitude.y) + "\n")
        self.filehandler.write("Attitude Z: " + str(attitude.z) + "\n")

        # print("-----------------------\n")

        # print("Loop Counter: " + str(loop_counter))
        # print("Publishing Time: " + str(self.current_time.to_sec()))
        # print("Z Setpoint: " + str(self.z_setpoint))
        # print("Z Velocity: " + str(self.z_vel))
        # print("Thrust Z: " + str(z_output))
        # print("Attitude X: " + str(attitude.x))
        # print("Attitude Y: " + str(attitude.y))
        # print("Attitude Z: " + str(attitude.z))

      # While we are waiting for our rate
      while self.current_time.to_sec() - self.prev_time_check.to_sec() < self.dt:
        # Sleep any excess time
        rate.sleep()
        # Check if ROS has shut down
        if rospy.is_shutdown():
          break

        
      # print("Current Time: " + str(self.current_time.to_sec()))
      # print("Previous Time: " + str(self.prev_time_check.to_sec()))
      # print("Time Difference: " + str(self.current_time.to_sec() - self.prev_time_check.to_sec()))

      # Save the start of the new loop
      self.prev_time_check = self.current_time
      loop_counter += 1

    # Close the file
    time.sleep(1)
    self.filehandler.close()

      
  # Used to save the time
  def clock_callback(self, clock_msg):
    self.current_time = clock_msg.clock

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


  # Called when the navigation is started
  def start_callback(self, msg):
    # Set the started flag to true
    rospy.loginfo(str(rospy.get_name()) + ": Velocity Controller Starting")
    self.started = True
    
  # On collsion reset the PID's
  def collision_callback(self, msgs):
    self.vel_x_PID.remove_buildup()
    self.vel_y_PID.remove_buildup()
    self.vel_z_PID.remove_buildup()

  # Called when the navigation is completed
  def completed_callback(self, msg):
    # Shutdown as there is nothing left to do
    rospy.signal_shutdown("Navigation Complete")

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('velocity_covelocity_controller_nodentroller')
  try:
    velcon = VelocityController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()