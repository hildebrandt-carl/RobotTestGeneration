#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
    // Sleep to allow all other threads to start up
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    std::cout<<"STARTING THE TESTER THREAD"<<std::endl;

    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle n;
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    // Set a loop rate of 1
    ros::Rate loop_rate(0.1);

    // Create the goal message
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 10;
  	goal.pose.position.y = 0;
  	goal.pose.position.z = 0;

    goal.pose.orientation.x = 0;
  	goal.pose.orientation.y = 0;
  	goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    goal.header.frame_id = "odom";

    // Send the message
    while (ros::ok())
    {
        // Send the goal message
        goal_pub.publish(goal);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
