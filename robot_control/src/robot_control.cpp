#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class Command : public geometry_msgs::Twist
{
public:
  enum Rotate
  {
    RIGHT = -1,
    LEFT = 1,
    STOP = 0
  };

public:
  Command() = default;
  void rotate(const int dir, const int ang_speed = 1)
  {
    this->angular.z = ang_speed * dir;
  }

  void go(const int lin_speed = 1)
  {
    this->linear.x = lin_speed;
  }

  void stop()
  {
    this->linear.x = 0;
  }
};

class Obstacles
{

public:
  enum Positions
  {
    LEFT,
    FRONT,
    RIGHT
  };

  typedef std::vector<bool> ObstaclesVector;

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
    obstacles[Positions::LEFT] = check_sector(msg, from, to);
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
    Command cmd;
    if (obstacles.check_obstacle(Obstacles::Positions::FRONT))
    {
      cmd.stop();
      cmd.rotate(Command::Rotate::LEFT, ang_speed);
    }
    else
    {
      cmd.go(lin_speed);
    }

    send_command(cmd);
  }

private:
  void send_command(const Command &cmd)
  {
    pub.publish((geometry_msgs::Twist)cmd);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");

  RobotControl rob;
  rob.start();

  return 0;
}
