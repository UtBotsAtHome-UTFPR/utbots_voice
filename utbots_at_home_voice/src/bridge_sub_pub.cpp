#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jsk_gui_msgs/VoiceMessage.h"

#include <sstream>

ros::Publisher pub;




void subCallback(const jsk_gui_msgs::VoiceMessage::ConstPtr& msg)
{
long int received_msgs = 0;
long int i = 0;
  
//  ros::NodeHandle r;
//  ros::Publisher pub = r.advertise<std_msgs::String>("text_recognized", 1000);

  
  
  received_msgs = msg->texts.size(); 
  
  if(received_msgs>0)
  {
//    char const *c = msg->texts[i].c_str();
//    std_msgs::String msgString;
//    strcpy(msgString.data, c);
	ROS_INFO("Publicando em text_recognized");
	std_msgs::String msgString;
	std::stringstream ss;
	ss << msg->texts[0].c_str();
	msgString.data = ss.str();
	pub.publish(msgString);
  }
  
  ROS_INFO("Num de Frases Recebidas: [%ld]", received_msgs);
  while(received_msgs>0)
	{ 
	ROS_INFO("Frase [%ld]", i);
	ROS_INFO(" : [%s]", msg->texts[i].c_str());
	i++;
	received_msgs--;
	}
//        ROS_INFO("[Enviado] %s", msg->texts[0].c_str());

//    	pub.publish(msgString);

//    ros::spinOnce();
	
}
/*
void subCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Recebido: [%s]", msg->data.c_str());
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_subscriber_and_publisher");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/Tablet/voice", 1000, subCallback);
//  ros::Subscriber sub = n.subscribe("/voice", 1000, subCallback);
// "chatter" 
  
  ros::NodeHandle r;
  pub = r.advertise<std_msgs::String>("text_recognized", 1000);  
  
  ros::spin();

  return 0;
}



