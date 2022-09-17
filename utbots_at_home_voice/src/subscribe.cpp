#include "ros/ros.h"
#include "std_msgs/String.h"

void subCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Recebido: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exemplo_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/Tablet/voice", 1000, subCallback);
// "chatter" 
  ros::spin();

  return 0;
}
