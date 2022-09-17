#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "jsk_gui_msgs/VoiceMessage.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exemplo_publisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
//  ros::Publisher pub = n.advertise<jsk_gui_msgs::VoiceMessage>("/Tablet/voice", 1000); 
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "mensagem " << count;
    msg.data = ss.str();

    ROS_INFO("[Enviado] %s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}
