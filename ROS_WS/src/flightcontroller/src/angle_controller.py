#!/usr/bin/env python
import rospy
import enum
import time
import numpy as np
from geometry_msgs.msg import Vector3
from mav_msgs.msg import RateThrust
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

def shutdown_sequence():
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

def imu_callback(msg):
    global quarternion_pose
    quarternion_pose = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

def roll_callback(msg):
    global roll
    roll = msg.data

def yaw_callback(msg):
    global yaw
    yaw = msg.data

def pitch_callback(msg):
    global pitch
    pitch = msg.data

def thrust_callback(msg):
    global thrust
    thrust = msg.data

if __name__=="__main__":
    global pitch
    global roll
    global thrust
    global yaw
    global quarternion_pose

    # Initilizing the global variables
    quarternion_pose = (0,0,0,0)
    roll = 0 
    pitch = 0
    thrust = 0
    yaw = 0

    # Start the ros node
    rospy.init_node('angle_controller')
    rospy.on_shutdown(shutdown_sequence)

    # Publishers
    drone_command = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)

    # Subscribers
    rospy.Subscriber("/uav/sensors/filtered_imu", Imu, imu_callback)
    rospy.Subscriber("/uav/input/roll_angle", Float64, roll_callback)
    rospy.Subscriber("/uav/input/pitch_angle", Float64, pitch_callback)
    rospy.Subscriber("/uav/input/yaw_rate", Float64, yaw_callback)
    rospy.Subscriber("/uav/input/thrust", Float64, thrust_callback)

    # Getting the PID parameters
    gains = rospy.get_param('/angle_controller_node/gains', {'p_z': 22.0, 'i_z': 1.0, 'd_z': 7.0, 'p_pr': 0.1, 'i_pr': 0, 'd_pr': 0})
    Kp_z, Ki_z, Kd_z = gains['p_z'], gains['i_z'], gains['d_z']
    Kp_pr, Ki_pr, Kd_pr = gains['p_pr'], gains['i_pr'], gains['d_pr']

    # Getting other parameters
    height = rospy.get_param('/angle_controller_node/initial_height', 1)

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Lauching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p_z - " + str(Kp_z))
    rospy.loginfo(str(rospy.get_name()) + ": i_z - " + str(Ki_z))
    rospy.loginfo(str(rospy.get_name()) + ": d_z - " + str(Kd_z))
    rospy.loginfo(str(rospy.get_name()) + ": p_pr - " + str(Kp_pr))
    rospy.loginfo(str(rospy.get_name()) + ": i_pr - " + str(Ki_pr))
    rospy.loginfo(str(rospy.get_name()) + ": d_pr - " + str(Kd_pr))
    rospy.loginfo(str(rospy.get_name()) + ": initial_height - " + str(height))

    # Setting the rate
    setRate = 20
    rate = rospy.Rate(setRate)

    # Wait for a range reading to initilize the height
    time.sleep(0.2)

    # PID variables for pitch/roll
    previous_error_p = 0.0
    integral_p = 0.0
    previous_error_r = 0.0
    integral_r = 0.0
    dt = 1.0/setRate

    while not rospy.is_shutdown():
        thrustOutput = 0
        pitchOutput = 0
        rollOutput = 0

        # Get the euler angle from the pose
        euler = euler_from_quaternion(quarternion_pose)
        roll_reading = euler[0]
        pitch_reading = euler[1]
        yaw_reading = euler[2]
        
        # Calculate a pitch command using pitch PID
        error_p = pitch - pitch_reading
        integral_p = integral_p + error_p * dt
        derivative_p = (error_p - previous_error_p)/dt
        pitchOutput = Kp_pr*error_p + Ki_pr*integral_p + Kd_pr*derivative_p
        previous_error_p = error_p 

        # Calculate a roll command using pitch PID
        error_r = roll - roll_reading 
        integral_r = integral_r + error_r * dt
        derivative_r = (error_r - previous_error_r)/dt
        rollOutput = Kp_pr*error_r + Ki_pr*integral_r + Kd_pr*derivative_r
        previous_error_r = error_r 

        # Create the thrust message
        msg = RateThrust()
        msg.header.stamp = rospy.get_rostime()
        msg.thrust = Vector3(0,0,thrust)
        msg.angular_rates = Vector3(rollOutput,pitchOutput,yaw)

        drone_command.publish(msg)
rate.sleep()