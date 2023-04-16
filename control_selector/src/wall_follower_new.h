#ifndef WALL_FOLLOWER_NEW
#define WALL_FOLLOWER_NEW

#include <ros/ros.h>
#include "control.h"

class WallFollowerNew : public Control
{
private:
    const double wall_range = 0.5;
    const double K_A = -20;
    const double K_R= -1;
    double range_eps = 0;
    double angle_eps = 0;

public:
    //установка данных лазера
    //WallFollower():Control{} {};
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота
    void setRobotPose(double x, double y, double theta) override {};

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override {return "WallFollower";}

    
};

#endif // WALL_FOLLOWER_NEW