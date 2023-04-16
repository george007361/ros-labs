#include "wall_follower.h"
#include "math.h"
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

void WallFollower::setLaserData(const std::vector<float> &data)
// void WallFollower::laserCallback(const sensor_msgs::LaserScan& msg)
{

    // проверим нет ли вблизи робота препятствия
    const double kMinObstacleDistance = 0.3;
    for (size_t i = 0; i < data.size(); i++)
    {
        if (data[i] < kMinObstacleDistance)
        {
            this->obstacle = true;
            ROS_INFO_STREAM("OBSTACLE!!!");
            break;
        }
    }
}

void WallFollower::setRobotPose(double x, double y, double theta)
// void WallFollower::poseCallback(const nav_msgs::Odometry &msg)
{
    ROS_DEBUG_STREAM("Pose msg: x = " << x << " y = " << y << " theta = " << theta); // 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
    // обновляем переменные класса, отвечающие за положение робота
    this->x = x;
    this->y = y;
    this->theta = theta; // 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

double WallFollower::cross_track_err_line()
{
    return line_y - y;
}

double WallFollower::cross_track_err_circle()
{
    double dx = cx - x;
    double dy = cy - y;
    double e = sqrt(dx * dx + dy * dy) - R;
    return e;
}

double WallFollower::cross_track_err_figure()
{
    double dx;
    double dy;
    double e;
    if (x < -cx)
    {
        dx = -cx - x;
        dy = y;
        e = sqrt(dx * dx + dy * dy) - R;
    }
    else if (x > cx)
    {
        dx = cx - x;
        dy = y;
        e = sqrt(dx * dx + dy * dy) - R;
    }
    else if (y > 0)
    {
        e = -R + y;
    }
    else
    {
        e = -R - y;
    }
    return e;
}

// void WallFollower::publish_error(double e)
// {
//     std_msgs::Float64 err;
//     err.data = e;
//     err_pub.publish(err);
// }

void WallFollower::getControl(double &v, double &w)
{
    if (!this->obstacle)
    {
        //  вычислим текущую ошибку управления
        double err = cross_track_err_figure();

        //  публикация текущей ошибки
        // publish_error(err);

        //  интегрируем ошибку
        int_error += err;

        //  диффференцируем ошибку
        double diff_error = err - old_error;

        //   запоминаем значение ошибки для следующего момента времени
        old_error = err;

        v = task_vel;

        //  ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
        w = prop_factor * err + int_factor * int_error + diff_error * diff_factor;

        ROS_DEBUG_STREAM("error = " << err << " cmd v=" << v << " w = " << w);
    }
}

WallFollower::WallFollower(double line_y,
                           double cx,
                           double cy,
                           double R,
                           double task_vel,
                           double prop_factor,
                           double int_factor,
                           double diff_factor,
                           double min_obstacle_range)
{
    ROS_INFO_STREAM("WallFollower initialisation");

    //  читаем параметры
    this->line_y = line_y;                         // node.param("line_y", -10.0);
    this->cx = cx;                                 // node.param("cx", 6);
    this->cy = cy;                                 // node.param("cy", 0);
    this->R = R;                                   // node.param("R", 6);
    this->task_vel = task_vel;                     // node.param("task_vel", 1.0);
    this->prop_factor = prop_factor;               //  node.param("prop_factor", 0.1);
    this->int_factor = int_factor;                 // node.param("int_factor", 0.0);
    this->diff_factor = diff_factor;               // node.param("diff_factor", 0.0);
    this->min_obstacle_range = min_obstacle_range; // node.param("min_obstacle_range", 1.0);

    this->obstacle = false;
    this->int_error = this->old_error = 0;

    // double dt = node.param("dt", 0.1);

    //  подписываемся на необъодимые данные
    // laser_sub = node.subscribe("/scan", 100, &WallFollower::laserCallback, this);
    // pose_sub = node.subscribe("/base_pose_ground_truth", 100, &WallFollower::poseCallback, this);
    // timer1 = node.createTimer(ros::Duration(dt), &WallFollower::timerCallback, this);
    // cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    // err_pub = node.advertise<std_msgs::Float64>("/err", 100);
}