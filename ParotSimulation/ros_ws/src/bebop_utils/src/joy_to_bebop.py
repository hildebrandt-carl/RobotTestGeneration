#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
	

class JoyConnection():
    def __init__(self):
        # Start the viewer class
        rospy.loginfo('Starting joy connection node')

        # Setup the rate
        self.rate = 10

        # setup joy topic subscription
        self.joy_subscriber = rospy.Subscriber(name='joy',
                                               data_class=Joy,
                                               callback=self.joy_callback,
                                               queue_size = 10)

        # setup publisher
        self.twist_publisher = rospy.Publisher(name='/bebop/cmd_vel/',
                                               data_class=Twist,
                                               queue_size = 10)

        # setup publishers
        self.twist_publisher = rospy.Publisher(name='/bebop/cmd_vel/',
                                               data_class=Twist,
                                               queue_size = 10)
        self.takeoff_publisher = rospy.Publisher(name='/bebop/takeoff',
                                                 data_class=Empty,
                                                 queue_size=1)
        self.land_publisher = rospy.Publisher(name='/bebop/land',
                                              data_class=Empty,
                                              queue_size=1)
    
        # Run the joy connection class
        self.run()
 

	
    def joy_callback(self, data):

        # Check if you want to launch the drone
        if data.buttons[0] == 1:
            self.takeoff_publisher.publish(Empty())

        # Check if we want to land the drone
        if data.buttons[3] == 1:
            self.land_publisher.publish(Empty())

        # Setup Twist Publisher 
        msg = Twist()

        # Map the linear velocites to the correct data types
        msg.linear.x = data.axes[1]
        msg.linear.y = data.axes[0]
        msg.linear.z = data.axes[4]
        
        # Map the angular velocities to the correct data types
        msg.angular.x = 0
        msg.angular.z = 0
        msg.angular.z = data.axes[3]
        
        # Publish the message
        self.twist_publisher.publish(msg)


    # This loop simply keeps the node alive
    def run(self):
        
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While we have no shutdown
        while not rospy.is_shutdown():

            # Sleep
            rate.sleep()


if __name__ == '__main__':
    # initialize node
    rospy.init_node('joy_to_bebop', anonymous=True)
    try:
        connection = JoyConnection()
    except rospy.ROSInterruptException:
        pass