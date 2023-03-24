#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

template <typename T>
class Chat
{
protected:
  ros::NodeHandle node;
  ros::Publisher pub;
  ros::Subscriber sub;

public:
  Chat() = delete;
  Chat(Chat &) = delete;
  Chat(const std::string &pub_topic_name, const std::string &sub_topic_name)
  {
    sub = node.subscribe(sub_topic_name, 1000, &Chat::handler, this);
    pub = node.advertise<std_msgs::String>(pub_topic_name, 1000);
  }

  virtual void run() = 0;
  virtual void handler(const std_msgs::String::ConstPtr &msg) = 0;
  virtual void sender(const T data) = 0;
};