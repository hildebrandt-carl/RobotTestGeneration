#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class ViconToPosition():
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Create the subscribers and publishers
    self.vicon_subscriber = rospy.Subscriber('/vicon/PARROT/PARROT', TransformStamped, self.vicon_position)
    self.pose_publisher = rospy.Publisher("unity_connection/drone_position", Pose, queue_size=1)

    # Set the rate
    self.rate = 100

    # Run the node
    self.run()

  # This loop simply keeps the node alive
  def run(self):
      
      # Set the rate
      rate = rospy.Rate(self.rate)

      # While we have no shutdown
      while not rospy.is_shutdown():

          # Sleep
          rate.sleep()

  # Call back for the vicon messages
  def vicon_position(self, msg):
        # Setup Twist Publisher 
        newposition = Pose()

        newposition.position.x = msg.transform.translation.y
        newposition.position.y = -1 * msg.transform.translation.x
        newposition.position.z = msg.transform.translation.z

        # Convert pose from quaternion to euler
        orientation_list = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        (r, p, y) = euler_from_quaternion(orientation_list)

        quat = quaternion_from_euler(p, -1* r, y)

        newposition.orientation.x = quat[0]
        newposition.orientation.y = quat[1]
        newposition.orientation.z = quat[2]
        newposition.orientation.w = quat[3]
        
        # Publish the message
        self.pose_publisher.publish(newposition)

  def shutdown_sequence(self):
    # Close the socket
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

def main():
  rospy.init_node('vicon_to_position')
  try:
    converter = ViconToPosition()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
  main()