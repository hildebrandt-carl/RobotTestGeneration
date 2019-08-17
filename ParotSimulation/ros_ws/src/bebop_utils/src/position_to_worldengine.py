#!/usr/bin/env python
import rospy
import socket
import json
from math import degrees 
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

class Connection():
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.col_pub = rospy.Publisher('/uav/collision', Empty, queue_size=1)
    self.gps_sub = rospy.Subscriber("unity_connection/drone_position", Pose, self.get_position)

    # Save the GPS Messages
    self.x = 0
    self.y = 0
    self.z = 0
    self.roll = 0
    self.pitch = 0
    self.yaw = 0
    self.ResetFlag = False

    # Create the connection
    self.host = "127.0.0.1"
    self.port = 25001
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the socket
    self.sock.connect((self.host, self.port))

    # Run the communication node
    self.communicate()

  # This is the main loop of this class
  def communicate(self):
    # Set the rate
    rate = rospy.Rate(100)

    # Pose message for unity
    pose = {}

    # While running
    while not rospy.is_shutdown():

      # Create a pose message to Unity
      pose['Position'] = [self.x, self.y, self.z]
      pose['Rotation'] = [self.roll, self.pitch, self.yaw]
      pose['Reset'] = self.ResetFlag

      # Serialize and send the pose
      serialized_pose = json.dumps(pose)
      self.sock.sendall(serialized_pose.encode("utf-8"))

      # Wait to receive if the drone has collided
      serialized_rec_data = self.sock.recv(1024).decode("utf-8")
      rec_data = json.loads(serialized_rec_data)

      # If there is a collision
      if(rec_data['Collision'] == True):
        self.col_pub.publish(Empty())
        self.ResetFlag = True
      else:
        self.ResetFlag = False

      # Sleep any excress time
      rate.sleep()

  # Call back to get the GPS data
  def get_position(self, msg):
    # Save the drones position (NED - ENU)
    self.x = -1 * msg.position.y
    self.y = msg.position.z
    self.z = msg.position.x

    # Convert pose from quaternion to euler
    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (r, p, y) = euler_from_quaternion(orientation_list)
    
    # Save the attitude 
    # TODO: This still needs to be done properly
    self.roll = degrees(p)
    self.pitch = -1 * degrees(y)
    self.yaw = -1 * degrees(r)

  def shutdown_sequence(self):
    # Close the socket
    self.sock.close
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

def main():
  rospy.init_node('position_to_worldengine')
  try:
    conn = Connection()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()