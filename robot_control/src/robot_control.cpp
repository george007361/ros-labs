#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

enum ObstaclePosition
{
  ON_LEFT,
  ON_RIGHT,
  ON_FRONT,
  NONE
};

ros::Publisher pub;

int obstacle_position;

void scanCallback(const sensor_msgs::LaserScan &msg)
{
  size_t middle_idx = msg.ranges.size() / 2;
  double min_distance = 0.5;
  bool obstacle_left = false;
  bool obstacle_right = false;

  for (int i = 0; i < middle_idx; ++i)
  {
    if (msg.ranges[i] < min_distance)
    {
      obstacle_left = true;
      break;
    }
  }

  for (int i = middle_idx; i < msg.ranges.size(); ++i)
  {
    if (msg.ranges[i] < min_distance)
    {
      obstacle_right = true;
      break;
    }
  }

  if (!obstacle_left && !obstacle_right)
  {
    ROS_INFO("Obstacle not detected");
    obstacle_position = ObstaclePosition::NONE;
    return;
  }

  if (obstacle_left && obstacle_right)
  {
    ROS_INFO("Obstacle detected in front");
    obstacle_position = ObstaclePosition::ON_FRONT;
    return;
  }

  if (obstacle_left)
  {
    ROS_INFO("Obstacle detected on the left");
    obstacle_position = ObstaclePosition::ON_LEFT;
    return;
  }

  if (obstacle_right)
  {
    ROS_INFO("Obstacle detected on the right");
    obstacle_position = ObstaclePosition::ON_RIGHT;
    return;
  }
}

void poseCallback(const nav_msgs::Odometry &msg)
{
  ROS_DEBUG_STREAM("Pose msg: x = " << msg.pose.pose.position.x << " y = " << msg.pose.pose.position.y << " theta = " << 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
}

void timerCallback(const ros::TimerEvent &ev)
{
  geometry_msgs::Twist cmd;

  switch (obstacle_position)
  {
  default:
  {
    ROS_ERROR_STREAM("Idk where is robot");
    cmd.linear = geometry_msgs::Vector3();
    cmd.angular = geometry_msgs::Vector3();
    break;
  }
  case ObstaclePosition::NONE:
  {
    cmd.linear.x = 1;
    cmd.angular.z = 0;
    break;
  }

  case ObstaclePosition::ON_LEFT:
  {
    cmd.linear.x = 0;
    cmd.angular.z = 0.1;
    break;
  }
  case ObstaclePosition::ON_RIGHT:
  {
    cmd.linear.x = 0;
    cmd.angular.z = -0.1;
    break;
  }

  case ObstaclePosition::ON_FRONT:
  {
    cmd.linear.x = 0;
    cmd.angular.z = 0.1;
    break;
  }
  }

  pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;
  obstacle_position = ObstaclePosition::NONE;
  ros::Subscriber laser_sub = n.subscribe("base_scan", 1, scanCallback);
  ros::Subscriber pose_sub = n.subscribe("base_pose_ground_truth", 1, poseCallback);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.001), timerCallback);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::spin();

  return 0;
}
