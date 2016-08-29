/*
 * Disclaimer Note:
 * Code was taken from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string> 

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  std_msgs::String msg;
  msg.data = std::string("Hello World");

  std::cout << "Topic: /chatter" << std::endl;
  std::cout << "Sending data.." << std::endl;

  while(ros::ok()){
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
