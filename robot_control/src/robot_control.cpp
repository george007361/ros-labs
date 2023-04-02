#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

bool obstacle = false;
ros::Publisher pub;

void laserCallback(const sensor_msgs::LaserScan &msg)
{
  ROS_DEBUG_STREAM("Laser msg: " << msg.scan_time);

  const double kMinRange = 0.5;
  obstacle = false;

  for (size_t i = 0; i < msg.ranges.size(); i++)
  {
    if (msg.ranges[i] < kMinRange)
    {
      obstacle = true;
      break;
    }
  }
}

void poseCallback(const nav_msgs::Odometry &msg)
{
  ROS_DEBUG_STREAM("Pose msg: x = " << msg.pose.pose.position.x << " y = " << msg.pose.pose.position.y << " theta = " << 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
}

void timerCallback(const ros::TimerEvent &)
{
  static int counter = 0;
  counter++;
  geometry_msgs::Twist cmd;
  if (!obstacle)
  {
    cmd.linear.x = 1;
  }
  else
  {
    cmd.linear.x = 0;
    cmd.angular.z = 0.5;
  }

  pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("base_scan", 1, laserCallback);
  ros::Subscriber pose_sub = n.subscribe("base_pose_ground_truth", 1, poseCallback);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.001), timerCallback);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::spin();

  return 0;
}
