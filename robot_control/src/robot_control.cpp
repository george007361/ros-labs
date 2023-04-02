#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class Obstacles
{

public:
  enum Positions
  {
    LEFT,
    RIGHT,
    FRONT
  };

  typedef std::vector<bool> ObstaclesVector;
  // friend ObstaclesVector operator-(const ObstaclesVector &a, ObstaclesVector &b)
  // {
  //   if (a.size() != b.size())
  //     throw("a.size() != b.size()");
  //   ObstaclesVector c(a.size());
  //   for (size_t i = 0; i < a.size(); ++i)
  //     c[i] = a[i] - b[i];
  //   return c;
  // }

private:
  ObstaclesVector obstacles;
  double min_dist;

public:
  Obstacles(double min_dist = 0.5) : min_dist(min_dist)
  {
    for (size_t i = 0; i < 3; i++)
    {
      obstacles.push_back(false);
    }
  }

  ObstaclesVector get_obstacles()
  {
    return obstacles;
  }

  bool is_obstacle()
  {
    for (size_t i = 0; i < 3; i++)
    {
      if (obstacles[i])
        return true;
    }
    return false;
  }

  bool check_obstacle(int pos)
  {
    if (pos < 0 || pos > 2)
    {
      return false;
    }

    return obstacles[pos];
  }

  static bool check_obstacle(const ObstaclesVector &st, int pos)
  {
    if (pos < 0 || pos > 2)
    {
      return false;
    }

    return st[pos];
  }

  void calc_obstales(const sensor_msgs::LaserScan &msg)
  {
    size_t step = msg.ranges.size() / 5;

    size_t from, to;
    // Right

    from = 0 * step;
    to = 1 * step;
    obstacles[Positions::RIGHT] = check_sector(msg, from, to);

    // Forw

    from = 2 * step;
    to = 3 * step;
    obstacles[Positions::FRONT] = check_sector(msg, from, to);

    // Left

    from = 4 * step;
    to = 5 * step;
    obstacles[Positions::RIGHT] = check_sector(msg, from, to);
  }

private:
  bool check_sector(const sensor_msgs::LaserScan &msg, const int from, const int to)
  {
    for (int i = from; i < to; ++i)
    {
      if (msg.ranges[i] < min_dist)
      {
        return true;
      }
    }
    return false;
  }
};

class RobotControl
{

private:
  int ang_speed;
  int lin_speed;
  Obstacles obstacles;
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber laser_sub;
  ros::Timer timer;

public:
  enum Rotate
  {
    RIGHT = 1,
    LEFT = -1,
    STOP = 0
  };

public:
  RobotControl(const int ang_speed = 1, const int lin_speed = 1, const int min_dist = 1) : ang_speed(ang_speed), lin_speed(lin_speed), obstacles(min_dist)
  {
    this->laser_sub = this->n.subscribe("base_scan", 1, &Obstacles::calc_obstales, &this->obstacles);
    this->pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    this->timer = this->n.createTimer(ros::Rate(1000), &RobotControl::logic, this);
  }

  void start()
  {
    ros::spin();
  }

private:
  void logic(const ros::TimerEvent &ev)
  {
    if (obstacles.check_obstacle(Obstacles::Positions::FRONT))
    {
      stop();
      rotate(Rotate::LEFT);
    }
    else
    {
      go();
    }
  }
  void rotate(int dir)
  {
    geometry_msgs::Twist cmd;
    cmd.angular.z = this->ang_speed * dir;
    this->pub.publish(cmd);
  }

  void go()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = this->lin_speed;
    this->pub.publish(cmd);
  }

  void stop()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    this->pub.publish(cmd);
  }

private:
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");

  RobotControl rob;
  rob.start();

  return 0;
}
