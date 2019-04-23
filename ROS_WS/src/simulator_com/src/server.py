#!/usr/bin/env python
import rospy
import socket
import json
from math import degrees 
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

# Global variables
global sock

# On shutdown run this command
def shutdown_sequence():
    global sock
    # Do something
    sock.close

def callback(data):
    global sock

    # Convert angle to euler angle
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    print(pitch)

    msg = {}
    msg['ID'] = 2
    msg['Name'] = "ABC"
    msg['Position'] = [-1 * data.position.y, data.position.z, data.position.x]
    msg['Rotation'] = [degrees(pitch), degrees(yaw), -1 * degrees(roll)]
    
    serialized_msg = json.dumps(msg)

    sock.sendall(serialized_msg.encode("utf-8"))
    
def listener():
    global sock
    rospy.init_node('connection_node')

    # On shutdown run the following function
    rospy.on_shutdown(shutdown_sequence)

    rospy.Subscriber("uav/sensors/gps", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    global sock

    host, port = "127.0.0.1", 25001

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))

        listener()

    except:
        print("ERROR Could not connect")

    