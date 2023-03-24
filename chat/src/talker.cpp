#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "my_common.cpp"

class Talker : private Chat<int>
{
public:
  Talker() : Chat("chatter", "reply")
  {
  }

  void run()
  {
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
      this->sender(count++);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void handler(const std_msgs::String::ConstPtr &msg)
  {
    ROS_ERROR("Reply is: [%s]", msg->data.c_str());
  }

  void sender(const int num)
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << num;
    msg.data = ss.str();

    ROS_INFO("Talker sender: %s", msg.data.c_str());

    this->pub.publish(msg);
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  Talker tlkr;
  tlkr.run();

  return 0;
}
