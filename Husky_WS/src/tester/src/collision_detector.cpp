#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ContactsState.h"

void chatterCallback(const gazebo_msgs::ContactsStateConstPtr& cs)
{
  std_msgs::String name1;
  std_msgs::String name2;

  for(unsigned int i = 0; i < cs->states.size(); i++) 
  {
    name1.data = cs->states[i].collision1_name;
    name2.data = cs->states[i].collision2_name;

    //int num_elements = my_sizeof(cs->states)/my_sizeof(cs->states[0])
    if ((name1.data.find("box") != std::string::npos) && (name2.data.find("wheel") != std::string::npos))
    {
      std::cout<<"Collision Occurred"<<std::endl;
    }

    if ((name1.data.find("wheel") != std::string::npos) && (name2.data.find("box") != std::string::npos))
    {
      std::cout<<"Collision Occurred"<<std::endl;
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/box_collision", 1000, chatterCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}