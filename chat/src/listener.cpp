#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"

#include "my_common.cpp"

class Listener : private Chat<const char *>
{
public:
  Listener() : Chat("reply", "chatter") {}

  void run()
  {
    ros::spin();
  }

  void handler(const std_msgs::String::ConstPtr &msg)
  {
    ROS_ERROR("I heard: [%s]", msg->data.c_str());
    sender(msg->data.c_str());
    ros::spinOnce();
  }

  void sender(const char *received)
  {
    std_msgs::String reply;

    std::stringstream ss;
    ss << "hello world " << received;
    reply.data = ss.str();

    this->pub.publish(reply);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  Listener lstr;
  lstr.run();

  return 0;
}
