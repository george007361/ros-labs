#ifndef WALL_FOLLOWER
#define WALL_FOLLOWER

#include "control.h"

class WallFollower : public Control
{
public:
    WallFollower(double line_y = -10,
                           double cx = 6,
                           double cy = 0,
                           double R = 6,
                           double task_vel = 1,
                           double prop_factor = 0.1,
                           double int_factor = 0,
                           double diff_factor = 0,
                           double min_obstacle_range = 1);
    // секция приватных функций
private:
    // функция вычисления ошибки управления для движения вдоль прямой
    double cross_track_err_line();

    // функция вычисления ошибки управления для движения вдоль окружности
    double cross_track_err_circle();
    double cross_track_err_figure();

    /**
     * Функция, которая будет вызвана
     * при получении данных от лазерного дальномера
     */
    // void laserCallback(const sensor_msgs::LaserScan& msg);
    void setLaserData(const std::vector<float> &data);

    /**
     * Функция, которая будет вызвана при
     * получении сообщения с текущем положением робота
     */
    // void poseCallback(const nav_msgs::Odometry& msg);
    void setRobotPose(double x, double y, double theta) override;

    void getControl(double &v, double &w) override;
    std::string getName() override {
        return "Wall follower"; };

private:
    // функция публикации ошибки
    // void publish_error(double e);

private:
    // заданная координата линии, вдоль которой должен двигаться робот
    double line_y;
    double cx, cy, R;

    // заданная скорость движения
    double task_vel;

    // пропрциональный коэффициент регулятора обратной связи
    double prop_factor;

    // интегральный коэффициент регулятора
    double int_factor;

    // дифференциальный коэффициент регулятора
    double diff_factor;

    // интеграл ошибки
    double int_error;

    // старое значение ошибки
    double old_error;

    // минимально допустимое значение расстояния до препятствия
    double min_obstacle_range;

    // флаг наличия препятствия
    bool obstacle;

    // положение робота
    double x, y, theta;

    //  публикатор текущей ошибки управления
    // ros::Publisher err_pub;
};

#endif // WALL_FOLLOWER